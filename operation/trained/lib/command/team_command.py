from .robot_command import RobotCommand

class TeamCommand(object):
    def __init__(self):
        self.commands = [RobotCommand() for i in range(3)]

    def reset(self):
        for cmd in self.commands:
            cmd.left_speed = 0
            cmd.right_speed = 0

    def __str__(self):
        msg = f'\nTEAM COMMAND:\n'
        for i in range(3):
            msg += f'ROBOT_{i}\n{self.commands[i]}'
        return msg

    def __repr__(self):
        return f'TeamCommand({self})'