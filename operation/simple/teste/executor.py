import math
from configuration.configuration import Configuration
from domain.entity import Entity
from domain.field import Field
from threads.thread_common_objects import ThreadCommonObjects
from communication.protobuf.ssl_gc_referee_message_pb2 import Referee
import time
from utils.geometry_utils import GeometryUtils
from utils.motion_utils import MotionUtils
import math

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
        from communication.receiver.grsim_receiver import GrSimReceiver
        from communication.sender.grsim_sender import GrSimSender

        self.receiver = GrSimReceiver(self.field)
        self.sender = GrSimSender()


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

    def get_id_by_name(self, role: str):
        configuration = self.configuration
        if role == "attacker":
            return configuration.team_roles_attacker_id
        elif role == "defensor":
            return configuration.team_roles_defensor_id
        elif role == "goalkeeper":
            return configuration.team_roles_goalkeeper_id
        return 0
    
    def GoToOminiPoint(self, position, id):
        robot = self.field.robots[id]
        direct_vector = (position[0] - robot.position.x , position[1] - robot.position.y)
        norma = math.sqrt(pow(direct_vector[0], 2) + pow(direct_vector[1], 2))
        direct_vector = direct_vector[0]/ norma, direct_vector[1]/norma
        targetAngle = math.arctan(direct_vector[1] / direct_vector[0])


 
    def main(self):
        while True:
            id = 0
            robot = self.field.robots[id]
            print(robot.position.theta)
            self.set_iteration_variables()

            left_down = 0 
            left_up = 0
            right_down = 0
            right_up = 0

            Omini_left = left_down, left_up
            Omini_right = right_down, right_up

            self.sender.transmit_robot(0, Omini_left, Omini_right)

           

def main():
    executor = Executor()
    executor.main()

if __name__ == "__main__":
    main()