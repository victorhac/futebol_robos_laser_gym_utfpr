import socket
import struct

import communication.receiver.messages_robocup_ssl_detection_pb2 as detection
import communication.receiver.messages_robocup_ssl_wrapper_pb2 as wrapper
import communication.receiver.messages_robocup_ssl_geometry_pb2 as geometry



class Receiver: 
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
        # Suponha que 'data' seja uma mensagem serializada
        
        packet = wrapper.SSL_WrapperPacket()
        packet.ParseFromString(data)

        
        
        if(packet.HasField('detection')):
            detection = packet.detection
            print(f"tenho detection: {detection}\n ")
            return detection
            
        else:
            detection = packet.detection
            print(f"Pacotes não recebidos de forma adequada: {detection}\n ")
            return detection
        
    def dataFinal(self):
    # Loop para receber mensagens

        data, addr = self.sock.recvfrom(8192)  # Aumentar o tamanho do buffer, se necessário
        dataFinal = self.receber_mensagem(data) 
        print(f"tenho detection: {dataFinal}\n ")

        return dataFinal


        # PLANO CARTESIANO DO SSL-VISION PARA AS ATUAIS CONFIG
            # x:-1125 +1125
            # y: -750 +750  
        # RETORNO DO SSL-VISION COM 1 BOLA E 1 ROBO
            # frame_number: 1556
            # t_capture: 1725281975.524639
            # t_sent: 1725281975.527526
            # camera_id: 0
            # balls {
                # confidence: 0.8302321
                # area: 101
                # x: 290.6947
                # y: -70.93996
                # pixel_x: 373.35645
                # pixel_y: 262.0792
                # }
            # robots_yellow {
                # confidence: 0.95025337
                # robot_id: 2
                # x: -655.1742
                # y: 349.62317
                # orientation: 1.2120297
                # pixel_x: 122.76159
                # pixel_y: 141.08609
                # height: 140.0
                # }
            # t_capture_camera: 152364816000.0


    

    