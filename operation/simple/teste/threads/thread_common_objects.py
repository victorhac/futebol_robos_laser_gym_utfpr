import threading
from communication.protobuf.ssl_gc_referee_message_pb2 import Referee

class ThreadCommonObjects:
    gc_to_executor_message = Referee()
    lock = threading.Lock()

    @staticmethod
    def get_gc_to_executor_message():
        with ThreadCommonObjects.lock:
            return ThreadCommonObjects.gc_to_executor_message
        
    @staticmethod
    def set_gc_to_executor_message(value):
        with ThreadCommonObjects.lock:
            ThreadCommonObjects.gc_to_executor_message = value