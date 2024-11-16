from communication.protobuf.ssl_gc_referee_message_pb2 import Referee
import threading

from executor import Executor
from threads.thread_common_objects import ThreadCommonObjects

def main():
    executor = Executor()

    executor_thread = threading.Thread(target=executor.main)
    executor_thread.start()

    while True:
        command = int(input("insert referee command: "))
        referee_message = Referee(command=command)
        ThreadCommonObjects.set_gc_to_executor_message(referee_message)

if __name__ == '__main__':
    main()