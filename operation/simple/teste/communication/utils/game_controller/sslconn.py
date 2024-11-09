import struct
import base64
from google.protobuf import message
from google.protobuf.internal.encoder import _VarintBytes
from google.protobuf import text_format
from io import BufferedReader
import socket
from google.protobuf.internal.decoder import _DecodeVarint32

def read_data_length(reader: BufferedReader) -> int:
    length_bytes = reader.read(4)
    if len(length_bytes) < 4:
        raise ValueError("Failed to read 4 bytes for data length header")
    return  _DecodeVarint32(length_bytes, 0)[0]

def send_message(conn: socket.socket, message_proto: message.Message) -> None:
    size = message_proto.ByteSize()
    conn.sendall(_VarintBytes(size))
    conn.sendall(message_proto.SerializeToString())

def receive(reader: BufferedReader) -> bytes:
    buf = reader.read1()
    msg_len, new_pos = _DecodeVarint32(buf, 0)
    return buf[new_pos:new_pos+msg_len]

def unmarshal(data: bytes, message_proto: message.Message) -> None:
    try:
        message_proto.ParseFromString(data)
    except message.DecodeError as e:
        encoded_message = base64.b64encode(data).decode('utf-8')
        raise ValueError(f"Could not unmarshal data: {encoded_message}") from e

def receive_message(reader: BufferedReader, message_proto: message.Message) -> None:
    data = receive(reader)
    unmarshal(data, message_proto)