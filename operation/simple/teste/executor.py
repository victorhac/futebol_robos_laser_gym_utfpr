import math
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

        self.last_state = None
        self.current_state = None
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
                from communication.receiver.ssl_vision_remote_receiver import SSLVisionRemoteReceiver

                self.receiver = SSLVisionRemoteReceiver(self.field)
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
    
    def set_iteration_variables(self):
        self.is_left_team = self.configuration.get_is_left_team()

        self.message = ThreadCommonObjects.get_gc_to_executor_message()

        if self.current_state != self.message.command:
            self.last_state = self.current_state
            self.current_state = self.message.command
            self.command_initial_time = time.time()

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
            return configuration.team_roles_attacker_id
        elif role == "defensor":
            return configuration.team_roles_defensor_id
        elif role == "goalkeeper":
            return configuration.team_roles_goalkeeper_id
        return 0
    
    def stop_defensor(self):
        self.sender.transmit_robot(self.defensor_id, 0, 0)

    def stop_goalkeeper(self):
        self.sender.transmit_robot(self.goalkeeper_id, 0, 0)

    def stop_attacker(self):
        self.sender.transmit_robot(self.attacker_id, 0, 0)

    def stop(self):
        ball_position = self.ball.get_position_tuple()
        left_motor_speed, right_motor_speed, self.get = self.stop_robot(self.attacker, ball_position)
        self.sender.transmit_robot(self.attacker_id, left_motor_speed, right_motor_speed)      

        left_motor_speed, right_motor_speed, self.errors[self.get_id_by_name("defensor")] = self.stop_robot(
            self.defensor,
            ball_position
        )

        self.sender.transmit_robot(self.defensor_id, left_motor_speed, right_motor_speed)      
        self.sender.transmit_robot(self.goalkeeper_id, 0, 0)

    def normal_start_after_prepare_kickoff_team(self):
        self.strategy()

    def normal_start_after_prepare_kickoff_foe_team(self):
        self.strategy()

    def normal_start_after_prepare_kickoff(self):
        if self.configuration.team_is_yellow_team:
            if self.last_state == Referee.Command.PREPARE_KICKOFF_YELLOW:
                self.normal_start_after_prepare_kickoff_team()
            else:
                self.normal_start_after_prepare_kickoff_foe_team()
        else:
            if self.last_state == Referee.Command.PREPARE_KICKOFF_BLUE:
                self.normal_start_after_prepare_kickoff_team()
            else:
                self.normal_start_after_prepare_kickoff_foe_team()

    def normal_start_after_prepare_penalty_team(self):
        self.attacker_strategy()
        self.stop_defensor()
        self.stop_goalkeeper()

    def normal_start_after_prepare_penalty_foe_team(self):
        self.attacker_strategy()
        self.stop_defensor()
        self.stop_goalkeeper()

    def normal_start_after_prepare_penalty(self):
        if self.configuration.team_is_yellow_team:
            if self.last_state == Referee.Command.PREPARE_PENALTY_YELLOW:
                self.normal_start_after_prepare_penalty_team()
            else:
                self.normal_start_after_prepare_penalty_foe_team()
        else:
            if self.last_state == Referee.Command.PREPARE_PENALTY_BLUE:
                self.normal_start_after_prepare_penalty_team()
            else:
                self.normal_start_after_prepare_penalty_foe_team()

    def normal_start_after_direct_free_team(self):
        self.strategy()

    def normal_start_after_direct_free_foe_team(self):
        self.strategy()

    def normal_start_after_direct_free(self):
        if self.configuration.team_is_yellow_team:
            if self.last_state == Referee.Command.DIRECT_FREE_YELLOW:
                self.normal_start_after_direct_free_team()
            else:
                self.normal_start_after_direct_free_foe_team()
        else:
            if self.last_state == Referee.Command.DIRECT_FREE_BLUE:
                self.normal_start_after_direct_free_team()
            else:
                self.normal_start_after_direct_free_foe_team()
    
    def normal_start(self):
        if not self.last_ball_pos_saved:
            self.last_ball_position = self.ball.get_position_tuple()
            self.last_ball_pos_saved = True

        is_last_state_prepare_kickoff = self.last_state == Referee.Command.PREPARE_KICKOFF_YELLOW or\
            self.last_state == Referee.Command.PREPARE_KICKOFF_BLUE
        
        is_last_state_prepare_penalty = self.last_state == Referee.Command.PREPARE_PENALTY_YELLOW or\
            self.last_state == Referee.Command.PREPARE_PENALTY_BLUE
        
        is_last_state_direct_free = self.last_state == Referee.Command.DIRECT_FREE_YELLOW or\
            self.last_state == Referee.Command.DIRECT_FREE_BLUE
        
        if is_last_state_prepare_penalty:
            self.normal_start_after_prepare_penalty()
        elif is_last_state_prepare_kickoff:
            self.normal_start_after_prepare_kickoff()
        elif is_last_state_direct_free:
            self.normal_start_after_direct_free()
        else:
            self.strategy()
                    
    def prepare_kickoff_team(self):
        positions = self.configuration.get_prepare_kickoff_team_positions()
        self.go_to_positions(positions)

    def prepare_kickoff_foe_team(self):
        positions = self.configuration.get_prepare_kickoff_foe_team_positions()
        self.go_to_positions(positions)

    def go_to_positions(self, positions: 'dict[int, tuple[float, float]]'):
        tolerance = 0.2
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

    def get_angle_to_goal(self):
        bx, by = self.ball.get_position_tuple()

        goal_x = self.configuration.field_length / 2
        
        goal_center_y = 0

        direction_to_goal_x = goal_x - bx
        direction_to_goal_y = goal_center_y - by
        
        return math.atan2(direction_to_goal_y, direction_to_goal_x)

    def is_aligned_with_goal(self, margin=0.2):
        bx, by = self.ball.get_position_tuple()

        goal_x = self.configuration.field_length / 2
        
        goal_center_y = 0

        direction_to_goal_x = goal_x - bx
        direction_to_goal_y = goal_center_y - by
        
        angle_to_goal = math.atan2(direction_to_goal_y, direction_to_goal_x)

        angle_diff = abs(GeometryUtils.smallestAngleDiff(self.attacker.position.theta, angle_to_goal))

        if not angle_diff <= margin:
            return False
        
        angle_robot_to_ball = GeometryUtils.calculate_slope(
            self.ball.get_position_tuple(),
            self.attacker.get_position_tuple()
        )

        if angle_robot_to_ball is None:
            return False

        is_robot_right_to_ball_to_goal_line = abs(
            GeometryUtils.smallestAngleDiff(
                self.attacker.position.theta,
                angle_robot_to_ball
                )
            ) < margin

        if not is_robot_right_to_ball_to_goal_line:
            return False
        
        return self.attacker.position.x < self.ball.position.x
    
    def is_defensor_aligned_with_ball(self, margin=0.2):        
        angle_robot_to_ball = GeometryUtils.calculate_slope(
            self.ball.get_position_tuple(),
            self.defensor.get_position_tuple()
        )

        if angle_robot_to_ball is None:
            return False

        is_robot_right_to_ball_to_goal_line = abs(
            GeometryUtils.smallestAngleDiff(
                self.defensor.position.theta,
                angle_robot_to_ball
                )
            ) < margin

        if not is_robot_right_to_ball_to_goal_line:
            return False
        
        return self.defensor.position.x < self.ball.position.x

    def is_attacker_almost_in_line_ball_to_goal_line(self):
        return self.is_aligned_with_goal()
    
    def try_attacker_spin(self):
        goal_position = (self.configuration.field_length / 2, 0)

        if not GeometryUtils.is_close(
            goal_position,
            self.attacker.get_position_tuple(),
            self.configuration.strategy_attacker_spin_radius_to_spin
        ):
            return None
        
        spin_clockwise = self.ball.position.y > 0

        if spin_clockwise:
            return 30, -30
        else:
            return -30, 30
        
    def is_close_to_own_goal(self, robot_id: int):
        tolerance = 0.3
        robot_position = self.field.robots[robot_id].position
        return robot_position.x < -1.75 + tolerance and abs(robot_position.y) < 0.675 + tolerance
        
    def is_close_to_foe_goal(self, robot_id: int):
        tolerance = 0.3
        robot_position = self.field.robots[robot_id].position
        return robot_position.x > 1.75 - tolerance and abs(robot_position.y) < 0.675 + tolerance

    def attacker_strategy(self):
        is_close_to_foe_goal = self.is_close_to_foe_goal(self.attacker_id)

        is_in_defense_area = self.attacker.position.x < self.configuration.strategy_defensor_defense_line_x

        must_go_to_center = is_in_defense_area or is_close_to_foe_goal

        if self.is_attacker_almost_in_line_ball_to_goal_line() and not must_go_to_center:
            spin_speeds = self.try_attacker_spin()

            if spin_speeds is None:
                left_motor_speed, right_motor_speed = 30, 30
            else:
                left_motor_speed, right_motor_speed = spin_speeds
        else:
            if must_go_to_center:
                attacker_target_position = (0, 0)
            else:
                attacker_target_position = self.ball.get_position_tuple()
            
            left_motor_speed, right_motor_speed, self.errors[self.get_id_by_name("attacker")] =\
                MotionUtils.go_to_point(
                    self.attacker,
                    attacker_target_position,
                    self.is_left_team,
                    self.errors[self.get_id_by_name("attacker")])
            
        self.sender.transmit_robot(self.attacker_id, left_motor_speed, right_motor_speed)

        self.last_attacker_strategy_speeds = left_motor_speed, right_motor_speed
        
    def defensor_strategy(self):
        defense_line_x = self.configuration.strategy_defensor_defense_line_x

        ball_goal_position_interest_point = GeometryUtils.closest_point_on_segment(
            self.defensor.get_position_tuple(),
            self.ball.get_position_tuple(),
            (-self.configuration.field_length / 2, 0)
        )

        is_close_to_own_goal = self.is_close_to_own_goal(self.defensor_id)

        is_ball_in_attack_area = self.ball.position.x > defense_line_x

        if is_close_to_own_goal:
            defensor_target_position = (0.0,0.0)
        elif is_ball_in_attack_area:
            defensor_target_position = ball_goal_position_interest_point

        if is_close_to_own_goal or is_ball_in_attack_area:
            left_motor_speed, right_motor_speed, self.errors[self.get_id_by_name("defensor")] = MotionUtils.go_to_point(
                self.defensor,
                defensor_target_position,
                self.is_left_team,
                self.errors[self.get_id_by_name("defensor")])
        else:
            if self.is_defensor_aligned_with_ball():
                left_motor_speed, right_motor_speed = 30, 30
            else:
                left_motor_speed, right_motor_speed, self.errors[self.get_id_by_name("defensor")] = MotionUtils.go_to_point(
                    self.defensor,
                    self.ball.get_position_tuple(),
                    self.is_left_team,
                    self.errors[self.get_id_by_name("defensor")])
        
        self.sender.transmit_robot(self.defensor_id, left_motor_speed, right_motor_speed)

    def goalkeeper_strategy(self):
        mid_goal_position = (-self.configuration.field_length / 2 + 0.2, 0)

        target_position_x, target_position_y = mid_goal_position

        ball_position = self.ball.position

        ball_close_to_goal_area_tolerance = 0

        is_ball_inside_goal_area = ball_position.x <= -1.75 + ball_close_to_goal_area_tolerance and\
            abs(ball_position.y) <= 0.675 + ball_close_to_goal_area_tolerance

        if is_ball_inside_goal_area:
            target_position_x, target_position_y = self.ball.get_position_tuple()

        self.transmit_robot_go_to_point(
            self.goalkeeper_id,
            (target_position_x, target_position_y)
        )

    def transmit_robot_go_to_point(
        self,
        robot_id: int,
        target_position: 'tuple[float, float]'
    ):
        left_motor_speed, right_motor_speed, self.errors[robot_id] = MotionUtils.go_to_point(
            self.field.robots[robot_id],
            target_position,
            self.is_left_team,
            self.errors[robot_id]
        )

        self.sender.transmit_robot(robot_id, left_motor_speed, right_motor_speed)

    def direct_free_team(self):
        self.stop_defensor()

        if GeometryUtils.is_close(
            self.attacker.get_position_tuple(),
            self.ball.get_position_tuple(),
            0.2
        ):
            self.stop_attacker()
        else:
            self.transmit_robot_go_to_point(self.attacker_id, self.ball.get_position_tuple())

    def direct_free_foe_team(self):
        ball_goal_position_interest_point = GeometryUtils.closest_point_on_segment(
            self.defensor.get_position_tuple(),
            self.ball.get_position_tuple(),
            (-self.configuration.field_length / 2, 0)
        )

        if GeometryUtils.is_close(
            self.defensor.get_position_tuple(),
            ball_goal_position_interest_point,
            0.2
        ):
            self.stop_defensor()
        else:
            self.transmit_robot_go_to_point(self.defensor_id, ball_goal_position_interest_point)

        if GeometryUtils.is_close(
            self.attacker.get_position_tuple(),
            ball_goal_position_interest_point,
            0.2
        ):
            self.stop_attacker()
        else:
            self.transmit_robot_go_to_point(self.attacker_id, ball_goal_position_interest_point)

    def direct_free_yellow(self):
        if self.configuration.team_is_yellow_team:
            self.direct_free_team()
        else:
            self.direct_free_foe_team()

    def direct_free_blue(self):
        if not self.configuration.team_is_yellow_team:
            self.direct_free_team()
        else:
            self.direct_free_foe_team()

    def strategy(self):
        self.defensor_strategy()
        self.attacker_strategy()
        self.goalkeeper_strategy()

    def main(self):
        while True:
            self.set_iteration_variables()

            message = self.message

            if not message:
                time.sleep(2)
                return

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
            elif message.command == Referee.Command.TIMEOUT_BLUE or message.command == Referee.Command.TIMEOUT_YELLOW:
                self.halt()
            else:
                self.strategy()

def main():
    executor = Executor()
    executor.main()

if __name__ == "__main__":
    main()