import socket
import time

client = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

client.bind(("64:32:A8:A9:1", 4))

message = "Teste"

while True:
    try:
        client.send(message.encode("utf-8"))
        time.sleep(1)
    except:
        break

client.close()