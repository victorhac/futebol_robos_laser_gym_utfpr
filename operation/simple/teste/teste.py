from communication.receiver.grsim_receiver import GrSimReceiver
from communication.sender.grsim_sender import GrSimSender
from domain.field import Field


def main():
    sender = GrSimSender()
    field = Field()
    receiver = GrSimReceiver(field)
    while True:
        sender.transmit_robot(0, 10, 10)
        receiver.update()
    

if __name__ == '__main__':
    main()