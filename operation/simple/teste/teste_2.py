from communication.receiver.remote_computer_receiver import RemoteComputerReceiver
from communication.receiver.ssl_vision_receiver import SSLVisionReceiver
from communication.sender.ros_sender import RosSender
from configuration.configuration import Configuration
from domain.field import Field
from utils.motion_utils import MotionUtils


def main():
    field = Field()
    receiver = RemoteComputerReceiver(field)
    sender = RosSender()
    configuration = Configuration.get_object()

    error = 0

    while True:
        receiver.update()

        ball = field.ball
        attacker = field.robots[configuration.team_roles_attacker_id]

        print('attacker', attacker.position)
        print('ball', ball.position)

        leftMotorSpeed, rightMotorSpeed, error = MotionUtils.goToPoint(attacker, ball.get_position_tuple(), configuration.get_is_left_team(), error)
        sender.transmit_robot(configuration.team_roles_attacker_id, -leftMotorSpeed, -rightMotorSpeed)

if __name__ == "__main__":
    main()