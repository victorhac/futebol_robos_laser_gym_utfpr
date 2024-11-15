import pickle
from communication.sender.game_controller.game_controller_remote_sender import GameControllerRemoteSender
from domain.referee_message_domain import RefereeMessageDomain

def send_message(sender: GameControllerRemoteSender):
    game_controller_message, error = sender.ssl_referee_client.consume()
    if not error:
        message = pickle.dumps(RefereeMessageDomain(game_controller_message.command))
        print(game_controller_message.command)
        sender.client.send(message)

if __name__ == '__main__':
    sender = GameControllerRemoteSender()

    try:
        sender.connect()
    except Exception as e:
        print(f"GameController: Error while connecting: {e}")
        exit(1)

    while True:
        try:
            send_message(sender)
        except Exception as e:
            print(f"GameController: Erro ao enviar mensagem: {e}")
            print("GameController: Tentando reconectar...")
            sender.connect()