import socket
import pickle
from communication.receiver.ssl_vision_receiver import SSLVisionReceiver
from configuration.configuration import Configuration
from domain.field import Field

configuration = Configuration.get_object()

field = Field()

ssl_vision = SSLVisionReceiver(field)

client = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

server_address = configuration.remote_computer_bluetooth_address
channel = configuration.remote_computer_bluetooth_channel

try:
    client.connect((server_address, channel))
    print(f"Conectado ao servidor Bluetooth {server_address} no canal {channel}")
except Exception as e:
    print(f"Erro ao conectar: {e}")
    client.close()
    exit(1)

while True:
    try:
        ssl_vision.update()
        message = pickle.dumps(field)
        client.send(message)
    except Exception as e:
        print(f"Erro ao enviar mensagem: {e}")
        break

client.close()