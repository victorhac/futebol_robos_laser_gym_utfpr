from ..domain.field_data import FieldData
from ..domain.robot import Robot
from ..domain.ball import Ball
from ..geometry.geometry_utils import GeometryUtils

from rsoccer_gym.Entities import Robot as RSoccerRobot, Ball as RSoccerBall

class RSoccerHelper:
    @staticmethod
    def getCorrectedAngle(angle: float):
        angleRadians = GeometryUtils.convertAngleToRadians(angle)
        return GeometryUtils.normalizeInPI(angleRadians)

    @staticmethod
    def toRobot(rSoccerRobot: RSoccerRobot):
        robot = Robot()

        robot.position.x = rSoccerRobot.x
        robot.position.y = rSoccerRobot.y
        robot.position.theta = RSoccerHelper.getCorrectedAngle(rSoccerRobot.theta)

        robot.velocity.x = rSoccerRobot.v_x
        robot.velocity.y = rSoccerRobot.v_y
        robot.velocity.theta = RSoccerHelper.getCorrectedAngle(rSoccerRobot.v_theta)
        
        return robot
    
    @staticmethod
    def toBall(rSoccerBall: RSoccerBall):
        ball = Ball()

        ball.position.x = rSoccerBall.x
        ball.position.y = rSoccerBall.y
        ball.velocity.x = rSoccerBall.v_x
        ball.velocity.y = rSoccerBall.v_y

        return ball
    
    @staticmethod
    def getRSoccerRobotAction(
        id: int,
        isYellowTeam: bool,
        leftMotorSpeed: float,
        rightMotorSpeed: float
    ):
        return RSoccerRobot(
            yellow=isYellowTeam,
            id=id,
            v_wheel0=leftMotorSpeed,
            v_wheel1=rightMotorSpeed
        )
    
    @staticmethod
    def getFieldDatas(nextState, isYellowTeam: bool):
        fieldData = FieldData()
        opponentFieldData = FieldData()

        ball = RSoccerHelper.toBall(nextState[0])

        blueTeam = []
        yellowTeam = []

        for i in range(1, len(nextState)):
            robot = nextState[i]
            if robot.yellow is not None:
                if robot.yellow:
                    yellowTeam.append(RSoccerHelper.toRobot(robot))
                else:
                    blueTeam.append(RSoccerHelper.toRobot(robot))

        fieldData.ball = ball
        opponentFieldData.ball = ball

        if isYellowTeam:
            fieldData.robots = yellowTeam
            fieldData.foes = blueTeam

            opponentFieldData.robots = blueTeam
            opponentFieldData.foes = yellowTeam
        else:
            fieldData.robots = blueTeam
            fieldData.foes = yellowTeam

            opponentFieldData.robots = yellowTeam
            opponentFieldData.foes = blueTeam

        return fieldData, opponentFieldData
