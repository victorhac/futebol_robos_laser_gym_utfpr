from domain.entity import Entity

class Field:
    def __init__(
        self,
        number_of_robots=3,
        number_of_foes=3
    ):
        self.robots = [Entity() for i in range(number_of_robots)]
        self.foes = [Entity() for i in range(number_of_foes)]
        self.ball = Entity()

    def __str__(self):
        msg = f'BALL\n{self.ball}'
        for i in range(3):
            msg += f'\nROBOT_{i}\n{self.robots[i]}'
        for i in range(3):
            msg += f'\nFOE_{i}\n{self.foes[i]}'
        return msg

    def __repr__(self):
        return f'Field({self})'