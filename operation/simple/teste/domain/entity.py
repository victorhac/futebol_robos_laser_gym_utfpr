from domain.pose_2d import Pose2D

class Entity:
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
        return f'EntityData({self})'
    
    def get_position_tuple(self):
        return (self.position.x, self.position.y)
    
    def update(self, entity: 'Entity'):
        self.position.update(entity.position)
        self.velocity.update(entity.velocity)