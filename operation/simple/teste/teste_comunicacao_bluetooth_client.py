import socket
import time
import pickle
from domain.field import Field

client = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

server_address = "5C:CD:5B:40:9B:7D"
channel = 4

try:
    client.connect((server_address, channel))
    print(f"Conectado ao servidor Bluetooth {server_address} no canal {channel}")
except Exception as e:
    print(f"Erro ao conectar: {e}")
    client.close()
    exit(1)

field_data = Field()
message = pickle.dumps(field_data)

while True:
    try:
        client.send(message)
        print("Mensagem enviada:", message)
        time.sleep(1)
    except Exception as e:
        print(f"Erro ao enviar mensagem: {e}")
        break

client.close()