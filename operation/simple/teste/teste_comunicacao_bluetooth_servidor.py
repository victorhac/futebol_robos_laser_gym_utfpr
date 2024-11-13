import socket

server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

server.bind(("64:32:A8:A9:1", 4))

client, addr = server.accept()

while True:
    try:
        data = client.recv(1024)
    except:
        break

client.close()
server.close()