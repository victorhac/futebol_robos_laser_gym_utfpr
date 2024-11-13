from communication.sender.ssl_vision.ssl_vision_remote_sender import SSLVisionRemoteSender


class Teste:
    def __init__(self):
        self.ssl_vision_remote_sender = SSLVisionRemoteSender()

    def main(self):
        self.ssl_vision_remote_sender.main()

def main():
    teste = Teste()
    teste.main()

if __name__ == '__main__':
    main()