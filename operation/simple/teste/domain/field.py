from domain.entity import Entity

class Field:
    def __init__(self):
        self.robots = [Entity() for i in range(3)]
        self.foes = [Entity() for i in range(3)]
        self.ball = Entity()

    def __str__(self):
        msg = f'BALL\n{self.ball}'
        for i in range(3):
            msg += f'\nROBOT_{i}\n{self.robots[i]}'
        for i in range(3):
            msg += f'\nFOE_{i}\n{self.foes[i]}'
        return msg

    def __repr__(self):
        return f'FieldData({self})'
