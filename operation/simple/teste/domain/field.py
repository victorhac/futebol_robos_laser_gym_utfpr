from domain.entity import Entity

class Field:
    def __init__(self):
        self.robots = {
            0: Entity(),
            1: Entity(),
            2: Entity()
        }
        self.foes = {
            0: Entity(),
            1: Entity(),
            2: Entity()
        }
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
