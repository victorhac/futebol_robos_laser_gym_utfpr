from communication.receiver.ssl_vision_receiver import SSLVisionReceiver
from domain.field import Field


def main():
    field = Field()
    receiver = SSLVisionReceiver(field)

    while True:
        receiver.update()
        print(field)


if __name__ == '__main__':
    main()