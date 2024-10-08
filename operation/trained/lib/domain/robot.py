from .pose_2d import Pose2D

class Robot():
    def __init__(self):
        self.position = Pose2D()
        self.velocity = Pose2D()

    def __str__(self):
        msg = (
            f'Position: {self.position}\n'
            f'Velocity: {self.velocity}\n'
        )
        return msg

    def __repr__(self):
        return f'Robot({self})'
    
    def get_position_tuple(self):
        return (self.position.x, self.position.y)