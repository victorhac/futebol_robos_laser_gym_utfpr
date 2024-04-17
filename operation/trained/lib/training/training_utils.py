import math
from ..domain.robot import Robot
from ..geometry.geometry_utils import GeometryUtils
from ..domain.ball import Ball

import numpy as np

class TrainingUtils():
    @staticmethod
    def rSpeed(
        ballPast: Ball,
        ballCurrent: Ball,
        goalPosition: tuple[float, float]
    ):
        v = TrainingUtils.v(ballPast, ballCurrent, goalPosition)
        return GeometryUtils.clip(v, -1, 1)
    
    @staticmethod
    def rDist(
        robots: list[Robot],
        ball: Ball
    ):
        minDistance = 0
        for robot in robots:
            distance = GeometryUtils.distance((robot.position.x, robot.position.y), (ball.position.x, ball.position.y))
            if minDistance == 0 or distance < minDistance:
                minDistance = distance

        return -1 if minDistance >= 1 else -minDistance
    
    @staticmethod
    def psi(
        aPosition: tuple[float, float],
        bPosition: tuple[float, float],
        cPosition: tuple[float, float]
    ):
        ba = np.array([bPosition[0] - aPosition[0], bPosition[1] - aPosition[1]])
        bc = np.array([bPosition[0] - cPosition[0], bPosition[1] - cPosition[1]])

        dot_product = np.dot(ba, bc)
    
        magnitude_ba = np.linalg.norm(ba)
        magnitude_bc = np.linalg.norm(bc)

        angle_rad = np.arccos(dot_product / (magnitude_ba * magnitude_bc))

        angle_deg = np.degrees(angle_rad)
        
        return angle_deg
    
    @staticmethod
    def rPos(
        aPosition: tuple[float, float],
        bPosition: tuple[float, float],
        cPosition: tuple[float, float]
    ):
        psi = TrainingUtils.psi(aPosition, bPosition, cPosition)
        
        return psi / math.pi - 1.0
    
    @staticmethod
    def rOfe(
        robot: Robot,
        ball: Ball,
        goalOpponentPosition: tuple[float, float]
    ):
        aPosition = (robot.position.x, robot.position.y)
        bPosition = (ball.position.x, ball.position.y)
        cPosition = goalOpponentPosition

        rPos = TrainingUtils.rPos(aPosition, bPosition, cPosition)

        return GeometryUtils.clip(rPos, -1, 0)
    
    @staticmethod
    def rDef(
        robot: Robot,
        ball: Ball,
        ownGoalPosition: tuple[float, float]
    ):
        aPosition = (robot.position.x, robot.position.y)
        bPosition = (ball.position.x, ball.position.y)
        cPosition = ownGoalPosition

        rPos = TrainingUtils.rPos(aPosition, bPosition, cPosition)
        
        return GeometryUtils.clip(rPos, -1, 0)


    # TODO: colocar constantes em configuration.json
    @staticmethod
    def v(
        ballPast: Ball,
        ballCurrent: Ball,
        goalPosition: tuple[float, float]
    ):
        ballPastPosition = (ballPast.position.x, ballPast.position.y)
        ballCurrentPosition = (ballCurrent.position.x, ballCurrent.position.y)
        return (GeometryUtils.distance(ballPastPosition, goalPosition) - \
            GeometryUtils.distance(ballCurrentPosition, goalPosition) - 0.05) / 0.14
    
    @staticmethod
    def rewardAttack(
        robotId: int,
        robots: list[Robot],
        ballPast: Ball,
        ballCurrent: Ball,
        goalPosition: tuple[float, float]
    ):
        rSpeed = TrainingUtils.rSpeed(ballPast, ballCurrent, goalPosition)
        rDist = TrainingUtils.rDist(robots, ballCurrent)
        rOfe = TrainingUtils.rOfe(robots[robotId], ballCurrent, goalPosition)

        return 0.7 * rSpeed + 0.15 * rDist + 0.15 * rOfe
    
    @staticmethod
    def rewardDefense(
        robotId: int,
        robots: list[Robot],
        ballPast: Ball,
        ballCurrent: Ball,
        goalPosition: tuple[float, float],
        ownGoalPosition: tuple[float, float]
    ):
        rSpeed = TrainingUtils.rSpeed(ballPast, ballCurrent, goalPosition)
        rDist = TrainingUtils.rDist(robots, ballCurrent)
        rDef = TrainingUtils.rDef(robots[robotId], ballCurrent, ownGoalPosition)

        return 0.7 * rSpeed + 0.15 * rDist + 0.15 * rDef
    
    @staticmethod
    def rewardGoal(
        isGoalMade: bool,
        startTime: float,
        endTime: float
    ):
        return (10 if isGoalMade else -10) * (endTime - startTime) / endTime
