from communication.receiver.grsim_receiver import GrSimReceiver
from domain.field import Field


class Teste:
    def __init__(self) -> None:
        self.field = Field()
        self.receiver = GrSimReceiver(self.field)

    def main(self):
        while True:
            self.receiver.update()


if __name__ == "__main__":
    teste = Teste()
    teste.main()