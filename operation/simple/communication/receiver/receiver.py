import socket
import struct

import messages_robocup_ssl_detection_pb2 as detection
import messages_robocup_ssl_wrapper_pb2 as wrapper
import messages_robocup_ssl_geometry_pb2 as geometry

# Endereço e porta para escutar
MCAST_GRP = '224.5.23.2'
MCAST_PORT = 10006

# Criar um socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

# Permitir reuso de endereço
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Bind ao grupo de multicast
sock.bind((MCAST_GRP, MCAST_PORT))

# Informar ao kernel para se juntar ao grupo multicast
mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

def receber_mensagem(data):
    
    packet = wrapper.SSL_WrapperPacket()
    packet.ParseFromString(data)

    
    
    if(packet.HasField('detection')):
        detection = packet.detection
        return detection
def dataLoop():

    while True:
        data, addr = sock.recvfrom(8192)  # Aumentar o tamanho do buffer, se necessário
        print(f"Data received from {addr}: {data}")  # Verifique os dados brutos recebidos
        # x:-1125 +1125
        # y: -750 +750
        dataFinal = receber_mensagem(data);  
        return dataFinal    
    