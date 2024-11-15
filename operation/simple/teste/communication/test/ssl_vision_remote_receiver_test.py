from communication.receiver.ssl_vision_remote_receiver import SSLVisionRemoteReceiver
from domain.field import Field

if __name__ == "__main__":
    field = Field()
    receiver = SSLVisionRemoteReceiver(field)

    while True:
        receiver.update()
        print(field.ball.position)

