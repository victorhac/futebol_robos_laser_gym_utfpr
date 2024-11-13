import socket

from domain.field import Field

server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

server.bind(("5C:CD:5B:40:9B:7D", 4))

server.listen(1)

client, addr = server.accept()

print(f"Conexão recebida de {addr}")

while True:
    try:
        data = client.recv(1024)
        if not data:
            break
        field = Field()

        field.load_from_dict(data)

        print(f"Recebido: {field}")
    except Exception as e:
        print(f"Erro ao receber dados: {e}")
        break

client.close()
server.close()