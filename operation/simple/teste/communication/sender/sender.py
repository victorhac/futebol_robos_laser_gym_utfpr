from abc import abstractmethod


class Sender:
    @abstractmethod
    def transmit_robot(self, robot_id, left_speed, right_speed):
        pass