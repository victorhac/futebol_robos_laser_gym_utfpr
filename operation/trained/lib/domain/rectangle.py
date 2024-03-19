# TODO: achar nome melhor
class Rectangle:
    def __init__(
        self,
        center: tuple[float, float],
        width: float,
        height: float,
        angle: float
    ):
        self.center = center
        self.width = width
        self.height = height
        self.angle = angle