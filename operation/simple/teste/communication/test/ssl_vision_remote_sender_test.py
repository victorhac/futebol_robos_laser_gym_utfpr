from communication.sender.ssl_vision.ssl_vision_remote_sender import SSLVisionRemoteSender
import pickle

def send_message(sender: SSLVisionRemoteSender):
    sender.ssl_vision.update()
    message = pickle.dumps(sender.field)
    print(sender.field)
    sender.client.send(message)

if __name__ == '__main__':
    sender = SSLVisionRemoteSender()
    while True:
        try:
            sender.connect()
        except Exception as e:
            print(f"SSLVision: Error while connecting: {e}")
            exit(1)

        while True:
            try:
                send_message(sender)
            except Exception as e:
                print(f"SSLVision: Unexpected error: {e}")
                sender.close_socket()
                break