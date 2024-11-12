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
        self.last_state = Referee.Command.PREPARE_KICKOFF_BLUE

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

    def main(self):
        while True:
            is_left_team = self.configuration.team_is_yellow_left_team == self.configuration.team_is_yellow_team

            #message = ThreadCommonObjects.get_gc_to_executor_message()
            message = Referee()
            message.command = Referee.Command.PREPARE_KICKOFF_BLUE
            #NORMAL_START
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
                if (self.configuration.team_is_yellow_team and self.last_state == Referee.Command.PREPARE_KICKOFF_YELLOW) or \
                    (not self.configuration.team_is_yellow_team and self.last_state == Referee.Command.PREPARE_KICKOFF_BLUE):
                    
                    target_position = 0.0, 0.0  

                    if(not GeometryUtils.isClose(atacker.get_position_tuple(), target_position, 0.2)):
                        leftMotorSpeed, rightMotorSpeed, error = MotionUtils.goToPoint(atacker, target_position, is_left_team)
                        self.sender.transmit_robot(atacker_id, leftMotorSpeed, rightMotorSpeed)
                    else:
                        self.sender.transmit_robot(atacker_id, 0, 0)

            elif message.command == Referee.Command.PREPARE_KICKOFF_YELLOW:

                self.last_state = Referee.Command.PREPARE_KICKOFF_YELLOW
                atk_target_position = (self.configuration.kickoff_position_left_team_attacker_x, self.configuration.kickoff_position_left_team_attacker_y)
                gk_target_position = self.configuration.kickoff_position_left_team_goalkeeper_x, self.configuration.kickoff_position_left_team_goalkeeper_y
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
                if(not GeometryUtils.isClose(defensor.get_position_tuple(), def_target_position, 0.3)):

                    leftMotorSpeed, rightMotorSpeed, error = MotionUtils.goToPoint(goalkeeper, (gk_target_position), is_left_team)
                    self.sender.transmit_robot(goalkeeper_id, leftMotorSpeed, rightMotorSpeed)
                else:
                    self.sender.transmit_robot(goalkeeper_id, 0, 0)
                    
            elif message.command == Referee.Command.PREPARE_KICKOFF_BLUE:

                self.last_state = Referee.Command.PREPARE_KICKOFF_BLUE
                atk_target_position = (self.configuration.kickoff_position_left_team_attacker_x, self.configuration.kickoff_position_left_team_attacker_y)
                gk_target_position = self.configuration.kickoff_position_left_team_goalkeeper_x, self.configuration.kickoff_position_left_team_goalkeeper_y
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
                if(not GeometryUtils.isClose(defensor.get_position_tuple(), def_target_position, 0.3)):

                    leftMotorSpeed, rightMotorSpeed, error = MotionUtils.goToPoint(goalkeeper, (gk_target_position), is_left_team)
                    self.sender.transmit_robot(goalkeeper_id, leftMotorSpeed, rightMotorSpeed)
                else:
                    self.sender.transmit_robot(goalkeeper_id, 0, 0)







def main():
    executor = Executor()
    executor.main()

if __name__ == "__main__":
    main()