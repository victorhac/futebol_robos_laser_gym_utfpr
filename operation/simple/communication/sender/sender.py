import socket
import struct
import messages_detection_pb2 as seu_arquivo_pb2

# Endereço e porta de multicast para enviar a mensagem
MCAST_GRP = '224.5.23.2'
MCAST_PORT = 10005

def criar_e_serializar_mensagem():
    # Criar uma mensagem TrackedFrame
    frame = seu_arquivo_pb2.TrackedFrame()
    frame.frame_number = 12345
    frame.timestamp = 1628503947.123456

    # Adicionar uma bola ao frame
    bola = frame.balls.add()
    bola.pos.x = 1.0
    bola.pos.y = 2.0
    bola.pos.z = 0.5
    bola.visibility = 0.8

    # Adicionar um robô ao frame
    robo = frame.robots.add()
    robo.robot_id.id = 7
    robo.robot_id.team_color = seu_arquivo_pb2.TEAM_COLOR_YELLOW
    robo.pos.x = -1.5
    robo.pos.y = 3.2
    robo.orientation = 1.57
    robo.visibility = 0.9

    # Serializar a mensagem em um formato binário
    mensagem_serializada = frame.SerializeToString()
    return mensagem_serializada

def enviar_mensagem(mensagem_serializada):
    # Criar um socket UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    
    # Configurar TTL para 1, para garantir que a mensagem não vá além da rede local
    ttl = struct.pack('b', 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

    # Enviar a mensagem serializada para o grupo multicast
    sock.sendto(mensagem_serializada, (MCAST_GRP, MCAST_PORT))

    print(f"Mensagem enviada para {MCAST_GRP}:{MCAST_PORT}")

# Cria e serializa a mensagem
mensagem_serializada = criar_e_serializar_mensagem()

# Envia a mensagem serializada
enviar_mensagem(mensagem_serializada)
