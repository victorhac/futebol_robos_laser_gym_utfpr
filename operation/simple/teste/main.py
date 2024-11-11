
from communication.receiver.ssl_vision_receiver import SSLVisionReceiver
from configuration.configuration import Configuration
from communication.receiver.grsim_receiver import GrSimReceiver
from communication.sender.grsim_sender import GrSimSender
from domain.field import Field
# from communication.sender.ros_sender import RosSender
import time

configuration = Configuration.get_object()

def main():
    field = Field()

    receiver = GrSimReceiver(field)
    sender = GrSimSender()

    # if configuration.mode == "MANUAL":
    #     receiver = GrSimReceiver(field)
    #     sender = GrSimSender()
    # else:
    #     receiver = SSLVisionReceiver(field)
    #     sender = RosSender()

    while True:
        # sender.transmit_robot(0, 0, 0)
        receiver.update()

        sender.transmit_robot(0, 10, 10)

if __name__ == "__main__":
    main()