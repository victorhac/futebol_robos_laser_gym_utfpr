from communication.receiver.grsim_receiver import GrSimReceiver
from communication.receiver.ssl_vision_receiver import SSLVisionReceiver
from communication.sender.grsim_sender import GrSimSender
from configuration.configuration import Configuration
from domain.field import Field
from threads.thread_common_objects import ThreadCommonObjects
from communication.protobuf.ssl_gc_referee_message_pb2 import Referee
import time
from utils.geometry_utils import GeometryUtils
from utils.motion_utils import MotionUtils

class Executor:
    def __init__(self):
        self.configuration = Configuration.get_object()
        self.field = Field()

        # if self.configuration.environment_mode == "REAL":
        #     self.receiver = SSLVisionReceiver(self.field)
        #     self.sender = RosSender()
        # else:
        self.receiver = GrSimReceiver(self.field)
        self.sender = GrSimSender()
        self.last_state = Referee.Command.PREPARE_KICKOFF_YELLOW
        self.goalkeeper_penalty_flag = True
        self.time_saved = False
        self.last_ball_pos_saved = False

    def stop_robot(self, robot, ball_position, is_left_team):
        robotPosition = robot.get_position_tuple() 
        
        if(GeometryUtils.isClose(robotPosition, ball_position,self.configuration.stop_distance_to_ball)):
            direction = 1
            if(ball_position[1] != robotPosition[1]): 
                direction = ball_position[1] - robotPosition[1] / abs(ball_position[1] - robotPosition[1])

                if ball_position[1] + (0.5 * direction) > abs(self.configuration.field_width/2):
                    direction = -direction
            
            targetPosition = (0, ball_position[1] + (0.5 * direction))

            return MotionUtils.goToPoint(robot, targetPosition, is_left_team)
        else:
            return 0, 0, 0
        
    def get_ball_moved(self, position1, position2):
        return (GeometryUtils.distance(position1, position2)) > 0.05
    
    def get_time_passed(self, initial_time):
        return time.time() - initial_time 

    def can_run_after_normal_start(self, initial_time):
        return self.get_time_passed(initial_time) > 10 


    def main(self):
        while True:
            is_left_team = self.configuration.get_is_left_team()

            #message = ThreadCommonObjects.get_gc_to_executor_message()
            message = Referee()
            message.command = Referee.Command.NORMAL_START
            #PREPARE_PENALTY_BLUE
            #NORMAL_START
            #PREPARE_KICKOFF_BLUE
            self.receiver.update()

            atacker_id = self.configuration.team_roles_attacker_id
            defensor_id = self.configuration.team_roles_defensor_id
            goalkeeper_id = self.configuration.team_roles_goalkeeper_id

            atacker = self.field.robots[atacker_id]
            defensor = self.field.robots[defensor_id]
            goalkeeper = self.field.robots[goalkeeper_id]
            ball = self.field.ball
            
            if message.command == Referee.Command.HALT:

                self.sender.transmit_robot(atacker_id, 0, 0)
                self.sender.transmit_robot(defensor_id, 0, 0)
                self.sender.transmit_robot(goalkeeper_id, 0, 0)

            elif message.command == Referee.Command.STOP:
                
                ball_position = ball.get_position_tuple()
                leftMotorSpeed, rightMotorSpeed, error =self.stop_robot(atacker, ball_position, is_left_team)
                self.sender.transmit_robot(atacker_id, leftMotorSpeed, rightMotorSpeed)      

                leftMotorSpeed, rightMotorSpeed, error = self.stop_robot(defensor, ball_position, is_left_team)
                self.sender.transmit_robot(defensor_id, leftMotorSpeed, rightMotorSpeed)      

                self.sender.transmit_robot(goalkeeper_id, 0, 0)    

                print(f"ataque --> {atacker.position}")  
                print(f"bola --> {ball.position}")  
            
            elif message.command == Referee.Command.NORMAL_START:

                if(not self.last_ball_pos_saved):
                    last_ball_position = ball.get_position_tuple()
                    self.last_ball_pos_saved = True
                if(not self.time_saved):
                    timmer = time.time()
                    self.time_saved = True
            
                if self.can_run_after_normal_start(timmer) or \
                    ((self.last_state == Referee.Command.PREPARE_KICKOFF_YELLOW or self.last_state == Referee.Command.PREPARE_KICKOFF_BLUE) and \
                        self.get_ball_moved(ball.get_position_tuple(), last_ball_position)):
                #############################################------ESTRATEGIA------####################################################
                    pass

                else:
                    pass




                            
            
            elif message.command == Referee.Command.PREPARE_KICKOFF_YELLOW:

                self.last_state = Referee.Command.PREPARE_KICKOFF_YELLOW
                atk_target_position = (self.configuration.kickoff_position_left_team_attacker_x, self.configuration.kickoff_position_left_team_attacker_y)
                gk_target_position = self.configuration.kickoff_position_left_team_goalkeeper_first_x, self.configuration.kickoff_position_left_team_goalkeeper_first_y
                def_target_position = self.configuration.kickoff_position_left_team_defensor_x, self.configuration.kickoff_position_left_team_defensor_y

                if(not GeometryUtils.isClose(atacker.get_position_tuple(), atk_target_position, 0.3)):
                    leftMotorSpeed, rightMotorSpeed, error = MotionUtils.goToPoint(atacker, (atk_target_position), is_left_team)
                    self.sender.transmit_robot(atacker_id, leftMotorSpeed, rightMotorSpeed)
                else:
                    self.sender.transmit_robot(atacker_id, 0, 0)
                if(not GeometryUtils.isClose(defensor.get_position_tuple(), def_target_position, 0.3)):

                    leftMotorSpeed, rightMotorSpeed, error = MotionUtils.goToPoint(defensor, (def_target_position), is_left_team)
                    self.sender.transmit_robot(defensor_id, leftMotorSpeed, rightMotorSpeed)
                else:
                    self.sender.transmit_robot(defensor_id, 0, 0)
                if(not GeometryUtils.isClose(goalkeeper.get_position_tuple(), gk_target_position, 0.3)):

                    leftMotorSpeed, rightMotorSpeed, error = MotionUtils.goToPoint(goalkeeper, (gk_target_position), is_left_team)
                    self.sender.transmit_robot(goalkeeper_id, leftMotorSpeed, rightMotorSpeed)
                else:
                    self.sender.transmit_robot(goalkeeper_id, 0, 0)
                    
            elif message.command == Referee.Command.PREPARE_KICKOFF_BLUE:
                self.last_state = Referee.Command.PREPARE_KICKOFF_BLUE
                atk_target_position = (self.configuration.kickoff_position_left_team_attacker_x, self.configuration.kickoff_position_left_team_attacker_y)
                gk_target_position = self.configuration.kickoff_position_left_team_goalkeeper_first_x, self.configuration.kickoff_position_left_team_goalkeeper_first_y
                def_target_position = self.configuration.kickoff_position_left_team_defensor_x, self.configuration.kickoff_position_left_team_defensor_y

                if(not GeometryUtils.isClose(atacker.get_position_tuple(), atk_target_position, 0.3)):
                    leftMotorSpeed, rightMotorSpeed, error = MotionUtils.goToPoint(atacker, (atk_target_position), is_left_team)
                    self.sender.transmit_robot(atacker_id, leftMotorSpeed, rightMotorSpeed)
                else:
                    self.sender.transmit_robot(atacker_id, 0, 0)
                if(not GeometryUtils.isClose(defensor.get_position_tuple(), def_target_position, 0.3)):

                    leftMotorSpeed, rightMotorSpeed, error = MotionUtils.goToPoint(defensor, (def_target_position), is_left_team)
                    self.sender.transmit_robot(defensor_id, leftMotorSpeed, rightMotorSpeed)
                else:
                    self.sender.transmit_robot(defensor_id, 0, 0)

                if(not GeometryUtils.isClose(goalkeeper.get_position_tuple(), gk_target_position, 0.3)):
                    leftMotorSpeed, rightMotorSpeed, error = MotionUtils.goToPoint(goalkeeper, (gk_target_position), is_left_team)
                    self.sender.transmit_robot(goalkeeper_id, leftMotorSpeed, rightMotorSpeed)
                
                else:
                    self.sender.transmit_robot(goalkeeper_id, 0, 0)

            elif message.command == Referee.Command.PREPARE_PENALTY_BLUE or message.command == Referee.Command.PREPARE_PENALTY_YELLOW:


                self.last_state = message.command
                
                gk_target_position = self.configuration.get_kickoff_goalkeeper_first_position()

                if(message.command == Referee.Command.PREPARE_PENALTY_BLUE):
                    if(not self.configuration.team_is_yellow_team):
                        atk_target_position = (0.750, 0)
                        def_target_position = self.configuration.get_kickoff_defensor_position()
                    else:
                        atk_target_position = (0.3, 0.5)
                        def_target_position = (0.3, -0.5)


                if(message.command == Referee.Command.PREPARE_PENALTY_YELLOW):
                    if(self.configuration.team_is_yellow_team):
                        atk_target_position = (0.750, 0)
                        def_target_position = self.configuration.get_kickoff_defensor_position()
                    else:
                        atk_target_position = (0.3, 0.5)
                        def_target_position = (0.3, -0.5)


                if(not GeometryUtils.isClose(atacker.get_position_tuple(), atk_target_position, 0.3)):

                    leftMotorSpeed, rightMotorSpeed, error = MotionUtils.goToPoint(atacker, atk_target_position, is_left_team)
                    self.sender.transmit_robot(atacker_id, leftMotorSpeed, rightMotorSpeed)
                else:
                    self.sender.transmit_robot(atacker_id, 0, 0)

                if(not GeometryUtils.isClose(goalkeeper.get_position_tuple(), gk_target_position, 0.3) and not self.goalkeeper_penalty_flag):

                    leftMotorSpeed, rightMotorSpeed, error = MotionUtils.goToPoint(goalkeeper, (gk_target_position), is_left_team)
                    self.sender.transmit_robot(goalkeeper_id, leftMotorSpeed, rightMotorSpeed)
                else:
                    self.sender.transmit_robot(goalkeeper_id, 0, 0)

                if(not GeometryUtils.isClose(defensor.get_position_tuple(), def_target_position, 0.3)):

                    leftMotorSpeed, rightMotorSpeed, error = MotionUtils.goToPoint(defensor, (def_target_position), is_left_team)
                    self.sender.transmit_robot(defensor_id, leftMotorSpeed, rightMotorSpeed)
                else:
                    self.sender.transmit_robot(defensor_id, 0, 0)






    def strategy(self, is_left_team):
        atacker_id = self.configuration.team_roles_attacker_id
        defensor_id = self.configuration.team_roles_defensor_id
        goalkeeper_id = self.configuration.team_roles_goalkeeper_i
        atacker = self.field.robots[atacker_id]
        defensor = self.field.robots[defensor_id]
        goalkeeper = self.field.robots[goalkeeper_id]
        ball = self.field.ball
        if(ball.position.x > 0):
            """--------------------------------------------------------ATACANTE CHUTA BOLA AO GOL--------------------------------------------------------"""
            if MotionUtils.between(ball.position.x, 1.0, -self.configuration.kickoff_position_left_team_goalkeeper_first_x) and \
                  MotionUtils.between(ball.position.y, self.configuration.field_goal_width,  -self.configuration.field_goal_width) and \
                    MotionUtils.between(atacker.position.y, self.configuration.field_goal_width,  -self.configuration.field_goal_width):
                
                #go on direction
                position = atacker.position
                robotAngle = position.theta
                targetPosition = (ball.position.x, ball.position.y + 0.15)

                MotionUtils.goToPoint(atacker, targetPosition, is_left_team)


        

            # // The yellow team may take a direct free kick.
            # DIRECT_FREE_YELLOW = 8;
            # // The blue team may take a direct free kick.
            # DIRECT_FREE_BLUE = 9;
            # // The yellow team may take an indirect free kick.
            # INDIRECT_FREE_YELLOW = 10 [deprecated = true];
            # // The blue team may take an indirect free kick.
            # INDIRECT_FREE_BLUE = 11 [deprecated = true];
            # // The yellow team is currently in a timeout.
            # TIMEOUT_YELLOW = 12;
            # // The blue team is currently in a timeout.
            # TIMEOUT_BLUE = 13;
            # // The yellow team just scored a goal.
            # // For information only.
            # // Deprecated: Use the score field from the team infos instead. That way, you can also detect revoked goals.
            # GOAL_YELLOW = 14 [deprecated = true];
            # // The blue team just scored a goal. See also GOAL_YELLOW.
            # GOAL_BLUE = 15 [deprecated = true];
            # // Equivalent to STOP, but the yellow team must pick up the ball and
            # // drop it in the Designated Position.
            # BALL_PLACEMENT_YELLOW = 16;
            # // Equivalent to STOP, but the blue team must pick up the ball and drop
            # // it in the Designated Position.
            # BALL_PLACEMENT_BLUE = 17;





def main():
    executor = Executor()
    executor.main()

if __name__ == "__main__":
    main()