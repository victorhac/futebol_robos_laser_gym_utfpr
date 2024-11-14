from configuration.configuration import Configuration
from domain.entity import Entity
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

        self.set_receiver_and_sender()

        self.last_state = Referee.Command.PREPARE_PENALTY_BLUE
        self.current_state = None
        self.goalkeeper_penalty_flag = True
        self.time_saved = False
        self.last_ball_pos_saved = False
        self.flag_defensor_orbit = False

        self.is_left_team = True
        self.attacker_id = self.configuration.team_roles_attacker_id
        self.defensor_id = self.configuration.team_roles_defensor_id
        self.goalkeeper_id = self.configuration.team_roles_goalkeeper_id

        self.attacker: Entity = None
        self.defensor: Entity = None
        self.goalkeeper: Entity = None
        self.ball: Entity = None

        self.message = None

        self.errors = {
            0: 0,
            1: 0,
            2: 0
        }

    def set_receiver_and_sender(self):
        if self.configuration.environment_mode == "SIMULATION":
            from communication.receiver.grsim_receiver import GrSimReceiver
            from communication.sender.grsim_sender import GrSimSender

            self.receiver = GrSimReceiver(self.field)
            self.sender = GrSimSender()
        else:
            from communication.sender.ros_sender import RosSender

            if self.configuration.receive_data_from_remote:
                from communication.receiver.remote_computer_receiver import RemoteComputerReceiver

                self.receiver = RemoteComputerReceiver(self.field)
            else:
                from communication.receiver.ssl_vision_receiver import SSLVisionReceiver

                self.receiver = SSLVisionReceiver(self.field)

            self.sender = RosSender()

    def stop_robot(
        self,
        robot: Entity,
        ball_position: 'tuple[float, float]'
    ):
        robotPosition = robot.get_position_tuple() 
        
        if GeometryUtils.is_close(robotPosition, ball_position,self.configuration.stop_distance_to_ball):
            direction = 1
            if(ball_position[1] != robotPosition[1]): 
                direction = ball_position[1] - robotPosition[1] / abs(ball_position[1] - robotPosition[1])

                if ball_position[1] + (0.5 * direction) > abs(self.configuration.field_width/2):
                    direction = -direction
            
            targetPosition = (0, ball_position[1] + (0.5 * direction))

            return MotionUtils.go_to_point(robot, targetPosition, self.is_left_team)
        else:
            return 0, 0, 0
        
    def get_ball_moved(self, position1, position2):
        return (GeometryUtils.distance(position1, position2)) > 0.05
    
    def get_time_passed(self, initial_time):
        return time.time() - initial_time 

    def can_run_after_normal_start(self, initial_time):
        return self.get_time_passed(initial_time) > 10
    
    def set_iteration_variables(self):
        self.is_left_team = self.configuration.get_is_left_team()

        #self.message = ThreadCommonObjects.get_gc_to_executor_message()
        self.message = Referee()
        self.message.command = Referee.Command.NORMAL_START

        if self.current_state != self.message.command:
            self.last_state = self.current_state
            self.current_state = self.message.command

        #PREPARE_PENALTY_YELLOW
        #NORMAL_START

        self.receiver.update()

        self.attacker_id = self.configuration.team_roles_attacker_id
        self.defensor_id = self.configuration.team_roles_defensor_id
        self.goalkeeper_id = self.configuration.team_roles_goalkeeper_id

        self.attacker = self.field.robots[self.attacker_id]
        self.defensor = self.field.robots[self.defensor_id]
        self.goalkeeper = self.field.robots[self.goalkeeper_id]
        self.ball = self.field.ball

    def halt(self):
        self.sender.transmit_robot(self.attacker_id, 0, 0)
        self.sender.transmit_robot(self.defensor_id, 0, 0)
        self.sender.transmit_robot(self.goalkeeper_id, 0, 0)

    def get_id_by_name(self, role: str):
        configuration = self.configuration
        if role == "attacker":
            return self.errors[configuration.team_roles_attacker_id]
        elif role == "defensor":
            return self.errors[configuration.team_roles_defensor_id]
        elif role == "goalkeeper":
            return self.errors[configuration.team_roles_goalkeeper_id]
        return 0

    def stop(self):
        ball_position = self.ball.get_position_tuple()
        left_motor_speed, right_motor_speed, self.get = self.stop_robot(self.attacker, ball_position)
        self.sender.transmit_robot(self.attacker_id, left_motor_speed, right_motor_speed)      

        left_motor_speed, right_motor_speed, self.errors[self.get_id_by_name("attacker")] = self.stop_robot(
            self.defensor,
            ball_position
        )

        self.sender.transmit_robot(self.defensor_id, left_motor_speed, right_motor_speed)      
        self.sender.transmit_robot(self.goalkeeper_id, 0, 0)

    def normal_start(self):
        if not self.last_ball_pos_saved:
            self.last_ball_position = self.ball.get_position_tuple()
            self.last_ball_pos_saved = True
        if not self.time_saved:
            self.timmer = time.time()
            self.time_saved = True


        is_prepare_kickoff = self.last_state == Referee.Command.PREPARE_KICKOFF_YELLOW or\
            self.last_state == Referee.Command.PREPARE_KICKOFF_BLUE
        
        ball_moved = self.get_ball_moved(self.ball.get_position_tuple(), self.last_ball_position)
        
        if self.can_run_after_normal_start(self.timmer) or (is_prepare_kickoff and ball_moved):
            attacker_target_position = self.ball.get_position_tuple()
            print("estrategia")
            left_motor_speed, right_motor_speed, self.errors[self.get_id_by_name("attacker")] = MotionUtils.go_to_point(
                self.attacker,
                (attacker_target_position[0] + 0.2, attacker_target_position[1]),
                self.is_left_team
            )
            
            self.sender.transmit_robot(self.attacker_id, left_motor_speed, right_motor_speed)
            
            if self.ball.position.x < 0:
                defensor_target_position = self.ball.get_position_tuple()

                left_motor_speed, right_motor_speed, self.errors[self.get_id_by_name("defensor")] = MotionUtils.go_to_point(
                    self.defensor,
                    (defensor_target_position[0] + 0.2, defensor_target_position[1]),
                    self.is_left_team
                )
            else: 
                targetPosition = (-self.configuration.field_length/2, 0.0)
                if(GeometryUtils.is_close(targetPosition, self.defensor.get_position_tuple(),self.configuration.field_goalkeeper_area_radius)):
                    left_motor_speed, right_motor_speed, self.errors[self.get_id_by_name("defensor")] = MotionUtils.go_to_point(
                        self.defensor,
                        0.0,
                        self.is_left_team)
                    
                else:
                    left_motor_speed, right_motor_speed, self.errors[self.get_id_by_name("defensor")] = MotionUtils.go_to_point(
                        self.defensor,
                        self.ball.get_position_tuple(),
                        self.is_left_team)
                    
                    self.sender.transmit_robot(self.defensor_id, left_motor_speed, right_motor_speed)

            if(abs(self.ball.position.y) > 0.800):
                ballY = 0.800 * self.ball.position.y / abs(self.ball.position.y)
            else:
                ballY = self.ball.position.y    
            goalkeeper_target_position = (-1.9, ballY)

            if(not GeometryUtils.is_close(self.goalkeeper.get_position_tuple(), goalkeeper_target_position, 0.3)):
                left_motor_speed, right_motor_speed, self.errors[self.get_id_by_name("goalkeeper")]= MotionUtils.go_to_point(self.goalkeeper, goalkeeper_target_position, self.is_left_team)
            else:
                left_motor_speed, right_motor_speed = 0, 0
                
            self.sender.transmit_robot(self.goalkeeper_id, left_motor_speed, right_motor_speed)  



            self.sender.transmit_robot(self.defensor_id, left_motor_speed, right_motor_speed)
        else:
            if self.configuration.team_is_yellow_left_team:
                is_team_prepare_penalty = self.last_state == Referee.Command.PREPARE_PENALTY_YELLOW
            else:
                is_team_prepare_penalty = self.last_state == Referee.Command.PREPARE_PENALTY_BLUE
                        
            is_last_state_prepare_penalty = self.last_state == Referee.Command.PREPARE_PENALTY_BLUE or\
                self.last_state == Referee.Command.PREPARE_PENALTY_YELLOW
                
            if is_team_prepare_penalty:
                targetPosition = self.configuration.get_normal_start_after_penalty_attacker_position()
                if(GeometryUtils.is_close(targetPosition, self.attacker.get_position_tuple(),0.2)):
                    self.halt()
                else:
                    left_motor_speed, right_motor_speed, self.errors[self.get_id_by_name("attacker")] = MotionUtils.go_to_point(
                        self.attacker,
                        self.ball.get_position_tuple(),
                        self.is_left_team)
                    
                    self.sender.transmit_robot(self.attacker_id, left_motor_speed, right_motor_speed)
    
            elif is_last_state_prepare_penalty:
                if abs(self.ball.position.y) > 0.800:
                    ballY = 0.800 * self.ball.position.y / abs(self.ball.position.y)
                else:
                    ballY = self.ball.position.y

                goalkeeper_target_position = (-1.9, ballY)

                if not GeometryUtils.is_close(self.goalkeeper.get_position_tuple(), goalkeeper_target_position, 0.3):
                    left_motor_speed, right_motor_speed, self.errors[self.get_id_by_name("defensor")] = MotionUtils.go_to_point(
                        self.goalkeeper,
                        goalkeeper_target_position,
                        self.is_left_team,
                        self.errors[self.get_id_by_name("defensor")])
                else:
                    left_motor_speed, right_motor_speed, self.errors[self.get_id_by_name("defensor")] = 0, 0, 0
            else:
                print("naoépenalti")

                pass

    def prepare_kickoff_team(self):
        positions = self.configuration.get_prepare_kickoff_team_positions()
        self.go_to_positions(positions)

    def prepare_kickoff_foe_team(self):
        positions = self.configuration.get_prepare_kickoff_foe_team_positions()
        self.go_to_positions(positions)

    def go_to_positions(self, positions: 'dict[int, tuple[float, float]]'):
        tolerance = 0.3
        for item in positions:
            robot = self.field.robots[item]

            if not GeometryUtils.is_close(robot.get_position_tuple(), positions[item], tolerance):
                left_motor_speed, right_motor_speed, self.errors[item] = MotionUtils.go_to_point(
                    robot,
                    positions[item],
                    self.is_left_team,
                    self.errors[item])
                
                self.sender.transmit_robot(item, left_motor_speed, right_motor_speed)
            else:
                self.sender.transmit_robot(item, 0, 0)

    def prepare_kickoff_yellow(self):
        if self.configuration.team_is_yellow_team:
            self.prepare_kickoff_team()
        else:
            self.prepare_kickoff_foe_team()

    def prepare_kickoff_blue(self):
        if self.configuration.team_is_yellow_team:
            self.prepare_kickoff_foe_team()
        else:
            self.prepare_kickoff_team()

    def prepare_penalty_team(self):
        positions = self.configuration.get_prepare_penalty_team_positions()
        self.go_to_positions(positions)

    def prepare_penalty_foe_team(self):
        positions = self.configuration.get_prepare_penalty_foe_team_positions()
        self.go_to_positions(positions)

    def prepare_penalty_blue(self):
        if self.configuration.team_is_yellow_team:
            self.prepare_penalty_foe_team()
        else:
            self.prepare_penalty_team()

    def prepare_penalty_yellow(self):
        if self.configuration.team_is_yellow_team:
            self.prepare_penalty_team()
        else:
            self.prepare_penalty_foe_team()

    def transmit_robot_go_to_point(
        self,
        robot_id: int,
        target_position: 'tuple[float, float]',
        role_name: str
    ):
        left_motor_speed, right_motor_speed, self.errors[self.get_id_by_name(role_name)] = MotionUtils.go_to_point(
            self.attacker,
            target_position,
            self.is_left_team,
            self.errors[self.get_id_by_name(role_name)]
        )

        self.sender.transmit_robot(robot_id, left_motor_speed, right_motor_speed)

    def direct_free_team(self):
        attacker_target_position_x, attacker_target_position_y = self.ball.get_position_tuple()
        attacker_target_position_x -= 0.3

        attacker_target_position = (attacker_target_position_x, attacker_target_position_y)

        self.transmit_robot_go_to_point(self.attacker_id, attacker_target_position, "attacker")

    def direct_free_foe_team(self):
        pass

    def direct_free_yellow(self):
        if self.configuration.team_is_yellow_team:
            self.direct_free_team()
        else:
            self.direct_free_foe_team()

    def direct_free_blue(self):
        if self.configuration.team_is_yellow_team:
            self.direct_free_foe_team()
        else:
            self.direct_free_team()

    def strategy(self):
        pass

    def main(self):
        while True:
            self.set_iteration_variables()

            message = self.message

            if message.command == Referee.Command.HALT:
                self.halt()
            elif message.command == Referee.Command.STOP:
                self.stop()
            elif message.command == Referee.Command.NORMAL_START:
                self.normal_start()
            elif message.command == Referee.Command.PREPARE_KICKOFF_YELLOW:
                self.prepare_kickoff_yellow()
            elif message.command == Referee.Command.PREPARE_KICKOFF_BLUE:
                self.prepare_kickoff_blue()
            elif message.command == Referee.Command.PREPARE_PENALTY_BLUE:
                self.prepare_penalty_blue()
            elif message.command == Referee.Command.PREPARE_PENALTY_YELLOW:
                self.prepare_penalty_yellow()
            elif message.command == Referee.Command.DIRECT_FREE_YELLOW:
                self.direct_free_yellow()
            elif message.command == Referee.Command.DIRECT_FREE_BLUE:
                self.direct_free_blue()


    # def strategy(self, is_left_team):
    #     attacker_id = self.configuration.team_roles_attacker_id
    #     defensor_id = self.configuration.team_roles_defensor_id
    #     goalkeeper_id = self.configuration.team_roles_goalkeeper_id
    #     attacker = self.field.robots[attacker_id]
    #     defensor = self.field.robots[defensor_id]
    #     goalkeeper = self.field.robots[goalkeeper_id]
    #     ball = self.field.ball
    #     cont = 0
    #     if(ball.position.x > 0):
    #         """--------------------------------------------------------ATACANTE CHUTA BOLA AO GOL--------------------------------------------------------"""
    #         if GeometryUtils.between(ball.position.x, 0.25, 1.6) and \
    #               GeometryUtils.between(ball.position.y, -self.configuration.field_goal_width,  self.configuration.field_goal_width):
    #             #go on direction
    #             targetPosition = ball.get_position_tuple()

    #             left_motor_speed, right_motor_speed, error = MotionUtils.go_to_point(attacker, (targetPosition[0] + 0.2, targetPosition[1]), is_left_team)
    #             self.sender.transmit_robot(attacker_id, left_motor_speed, right_motor_speed)
    #         else:
    #             atk_target_position = (0,0)

    #             left_motor_speed, right_motor_speed, error = MotionUtils.go_to_point(attacker, atk_target_position, is_left_team, -1)
    #             self.sender.transmit_robot(attacker_id, left_motor_speed, right_motor_speed)
                
    #             left_motor_speed, right_motor_speed, error = MotionUtils.go_to_point(defensor, ball.get_position_tuple(), is_left_team)
    #             self.sender.transmit_robot(defensor_id, left_motor_speed, right_motor_speed)  
            
    #         if(ball.position.x > 0.4):
    #             targetPosition = (-1.2, ball.position.y)

    #             left_motor_speed, right_motor_speed, error = MotionUtils.go_to_point(defensor, targetPosition, is_left_team)

    #             self.sender.transmit_robot(defensor_id, left_motor_speed, right_motor_speed)  

    #     else:
    #         atk_target_position = (0.5, ball.position.y)
    #         if(not GeometryUtils.is_close(attacker.get_position_tuple(), atk_target_position, 0.3)):
    #             left_motor_speed, right_motor_speed, error = MotionUtils.go_to_point(attacker, atk_target_position, is_left_team)
    #         else:
    #             left_motor_speed, right_motor_speed, error = MotionUtils.FaceDirection(attacker, atk_target_position, is_left_team)

    #         self.sender.transmit_robot(attacker_id, left_motor_speed, right_motor_speed) 

    #         ball_position = ball.get_position_tuple()



    #         if(ball.position.x > defensor.position.x):
    #             defensor_target_position = GeometryUtils.directionalVector(defensor.get_position_tuple(), (ball_position[0] + 0.1, ball_position[1]) )

    #             left_motor_speed, right_motor_speed, error = MotionUtils.go_to_point(defensor, defensor_target_position, is_left_team)

    #         else:
    #             defensor_target_position = (self.configuration.kickoff_position_left_team_goalkeeper_first_x, ball.position.y)
    #             if(not GeometryUtils.is_close(defensor.get_position_tuple(), defensor_target_position, is_left_team) and not self.flag_defensor_orbit):
    #                 left_motor_speed, right_motor_speed, error = MotionUtils.go_to_point(defensor, defensor_target_position, is_left_team)
    #             else:
    #                 self.flag_defensor_orbit = True
                
    #             defensor_target_position = ball.get_position_tuple()
    #             if(not GeometryUtils.is_close(defensor.get_position_tuple(), defensor_target_position, is_left_team)and self.flag_defensor_orbit):
    #                 left_motor_speed, right_motor_speed, error = MotionUtils.go_to_point(defensor, defensor_target_position, is_left_team)
    #             else:
    #                 self.sender.transmit_robot(defensor_id, 0, 0)
    #                 self.flag_defensor_orbit = False

                
    #         self.sender.transmit_robot(defensor_id, left_motor_speed, right_motor_speed)  
              
        # if(abs(ball.position.y) > 0.800):
        #     ballY = 0.800 * ball.position.y / abs(ball.position.y)
        # else:
        #     ballY = ball.position.y    
        # goalkeeper_target_position = (-1.9, ballY)

        # if(not GeometryUtils.is_close(goalkeeper.get_position_tuple(), goalkeeper_target_position, 0.3)):
        #     left_motor_speed, right_motor_speed, error = MotionUtils.go_to_point(goalkeeper, goalkeeper_target_position, is_left_team)
        # else:
        #     left_motor_speed, right_motor_speed, error = 0, 0, 0
            
        # self.sender.transmit_robot(goalkeeper_id, left_motor_speed, right_motor_speed)  

            

            
                

        

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