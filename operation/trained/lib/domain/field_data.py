from .robot import Robot
from .ball import Ball

class FieldData:
    def __init__(self):
        self.robots = [Robot() for i in range(3)]
        self.foes = [Robot() for i in range(3)]
        self.ball = Ball()

    def __str__(self):
        msg = f'BALL\n{self.ball}'
        for i in range(3):
            msg += f'\nROBOT_{i}\n{self.robots[i]}'
        for i in range(3):
            msg += f'\nFOE_{i}\n{self.foes[i]}'
        return msg

    def __repr__(self):
        return f'FieldData({self})'