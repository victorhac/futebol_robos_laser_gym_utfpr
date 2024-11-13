import time
import threading
import math
# from lib.comm.vision import ProtoVision
from lib.helpers.robot_helper import RobotHelper
from communication.receiver.vision import SSLVisionReceiver as ProtoVision
from lib.core.data import EntityData, FieldData
from lib.comm.control import ProtoControl
from lib.geometry.geometry import Geometry

from lib.helpers.configuration_helper import ConfigurationHelper
from lib.motion.motion import Motion
from lib.helpers.field_helper import FieldHelper
from lib.helpers.firasim_helper import FIRASimHelper


CONFIGURATION = ConfigurationHelper.getConfiguration()
    
IS_YELLOW_TEAM = CONFIGURATION["team"]["is-yellow-team"]
IS_YELLOW_LEFT_TEAM = CONFIGURATION["team"]["is-yellow-left-team"]

RUN_BOTH_TEAMS = False
TOLERANCE = 0.038 #tolerancia para ir até o ponto, necessario pois muitas vezes o while ficava infinito pois a nao havia precisao para chegar exatamente ao ponto desejado
IS_LEFT_TEAM = FieldHelper.isLeftTeam(IS_YELLOW_TEAM, IS_YELLOW_LEFT_TEAM)

CONTROL_IP = CONFIGURATION["FIRASim"]["control"]["ip"]
CONTROL_PORT = CONFIGURATION["FIRASim"]["control"]["port"]

GOAL_WIDTH = CONFIGURATION["field"]["goal"]["width"]
FIELD_LENGTH = CONFIGURATION["field"]["length"]

GOAL_LINE_DISTANCE_TO_CENTER = 0.75
DEFENSE_LINE_DISTANCE_TO_GOAL = 0.3

FIELD_WIDTH = 0.650
ATAKER_ID = 0
DEFENSER_ID = 1
SWAP = 0
ORIGIN = [-FIELD_WIDTH, 0]

GOALKEEPER_ROBOT_ID = 2

def yGoalValue():
    global fsimcontroler
    fieldData = fsimcontroler[0]
    ball = fieldData.ball
    if(ball.position.y > 0.200):
        return 0.180
    elif(ball.position.y < -0.200):
        return -0.180
    else:
        return ball.position.y
                

def spinIfCloseToBall(
    id: int,
    ):
    global fsimcontroler
    fieldData = fsimcontroler[0]
    vision = fsimcontroler[1]
    teamControl = fsimcontroler[2]
    vision.update()
    robot = fieldData.robots[id]

    ball = vision.field_data.ball
    position = (robot.position.x, robot.position.y)
    targetPosition = (ball.position.x, ball.position.y)

    if Geometry.isClose( (position[0], position[1]), FIRASimHelper.normalizePosition(targetPosition[0], targetPosition[1], IS_LEFT_TEAM), 0.3):

        (leftSpeed, rightSpeed) = Motion.spin(True, 30)

        teamControl.transmit_robot(id, leftSpeed, rightSpeed)

        time.sleep(0.5)

    teamControl.transmit_robot(id, 0, 0)    
        
def between(value, lower, upper):
    return lower <= value <= upper

def placeRobot(
    id: int,
    targetPosition: 'tuple[float, float]',
):
    global fsimcontroler
    fieldData = fsimcontroler[0]
    vision = fsimcontroler[1]
    teamControl = fsimcontroler[2]

    robot = fieldData.robots[id]
    vision.update()
    
    isClose = Geometry.isClose((robot.position.x, robot.position.y), targetPosition, TOLERANCE)

    while not isClose :

        (leftSpeed, rightSpeed, error) = Motion.goToPoint(id, targetPosition,fieldData, IS_LEFT_TEAM)

        teamControl.transmit_robot(id, leftSpeed, rightSpeed)

        vision.update()
        isClose = Geometry.isClose((robot.position.x, robot.position.y), targetPosition, TOLERANCE)
    
def followBallY(
    id: int,
    targetPosition: 'tuple[float, float]',
):
    global fsimcontroler
    fieldData = fsimcontroler[0]
    vision = fsimcontroler[1]
    teamControl = fsimcontroler[2]

    robot = fieldData.robots[id]
    error = 0
    vision.update()
    ball = fieldData.ball
    target = targetPosition
    isClose = Geometry.isClose((robot.position.x, robot.position.y), targetPosition, TOLERANCE)

    while not isClose :
        vision.update()
        target = targetPosition [0] , ball.position.y

        (leftSpeed, rightSpeed, error) = Motion.goToPoint(robot, target, IS_LEFT_TEAM, error)

        teamControl.transmit_robot(id, leftSpeed, rightSpeed)


        vision.update()
        isClose = Geometry.isClose((robot.position.x, robot.position.y), target, TOLERANCE)

def goToPoint(id):
    global fsimcontroler
    fieldData = fsimcontroler[0]
    teamControl = fsimcontroler[2]
    
    ball = fieldData.ball
    atacker = fieldData.robots[id]
    
    ballPosition = ball.position.x, ball.position.y
    atackerPosition = atacker.position.x, atacker.position.y
    ball = fieldData.ball
    atacker = fieldData.robots[id]
    
    position = atacker.position
    robotAngle = position.theta
    targetPosition = (ball.position.x, ball.position.y)

    xTarget, yTarget = FIRASimHelper.normalizePosition(targetPosition[0], targetPosition[1], True)

    angleToTarget = math.atan2(yTarget - position.y, xTarget - position.x)

    error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)

    if abs(error) > math.pi / 2.0 + math.pi / 20.0:
        reversed = True
        robotAngle = Geometry.normalizeInPI(robotAngle + math.pi)
        error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)
    else:
        reversed = False

    kP = 20
    kD = 0.1
    motorSpeed = (kP * error) + (kD * (error - 0))

    baseSpeed = ConfigurationHelper.getBaseSpeed()

    motorSpeed = RobotHelper.truncateMotorSpeed(motorSpeed, baseSpeed)

    leftMotorSpeed, rightMotorSpeed = Motion._getSpeeds(motorSpeed, baseSpeed, reversed)

    teamControl.transmit_robot(id, -leftMotorSpeed, -rightMotorSpeed)
    
def atackerPlayerThread(
    fieldData: FieldData,
    vision: ProtoVision,
    teamControl: ProtoControl
):
    global fsimcontroler

    ball = fieldData.ball
    atacker = fieldData.robots[2]
    goalkeeper = fieldData.robots[1]
    Defenser = fieldData.robots[0]
    ballPosition = (ball.position.x, ball.position.y)
    atacker.position.x, atacker.position.y
    lastError = 0
    cont = 0
    contdef = 0
    
    while True:
        vision.update()
        ball = fieldData.ball
        atacker = fieldData.robots[2]
        goalkeeper = fieldData.robots[1]    
        Defenser = fieldData.robots[0] 

        if(IS_LEFT_TEAM):
            #bola de cara pro gol
            if(ball.position.x > 0):
                """--------------------------------------------------------ATACANTE CHUTA BOLA AO GOL--------------------------------------------------------"""
                if between(ball.position.x, 0.4, 1.0) and between(ball.position.y, -0.3, 0.3) and between(atacker.position.y, -0.3, 0.3):
                    #go on direction
                    position = atacker.position
                    robotAngle = position.theta
                    targetPosition = (ball.position.x, ball.position.y)

                    xTarget, yTarget = FIRASimHelper.normalizePosition(targetPosition[0], targetPosition[1]+0.125, True)

                    angleToTarget = math.atan2(yTarget - position.y, xTarget - position.x)

                    error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)

                    if abs(error) > math.pi / 2.0 + math.pi / 20.0:
                        reversed = True
                        robotAngle = Geometry.normalizeInPI(robotAngle + math.pi)
                        error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)
                    else:
                        reversed = False

                    kP = 20
                    kD = 0.1    
                    motorSpeed = (kP * error) + (kD * (error - 0))

                    baseSpeed = ConfigurationHelper.getBaseSpeed()

                    motorSpeed = RobotHelper.truncateMotorSpeed(motorSpeed, baseSpeed)

                    leftMotorSpeed, rightMotorSpeed = Motion._getSpeeds(motorSpeed, baseSpeed, reversed)

                    teamControl.transmit_robot(2, leftMotorSpeed, rightMotorSpeed)
                else:
                    raio = 0.10002
                vision.update()

                ball = fieldData.ball
                atacker = fieldData.robots[2]

                center = ball.position.x, ball.position.y
                
                # Compute the initial angle in radians
                theta = (math.atan2(center[1] - atacker.position.y, center[0] - atacker.position.x)) + 614 * cont 
                x_d = center[0] + raio * math.cos(theta)
                y_d = center[1] + raio * math.sin(theta)
                
                orientation =1
                
                """------------------------------------COMEÇO DO DELIRIO------------------------------------"""
                if(atacker.position.x > -0.3 and ball.position.x > 0):                    

                    
                    if(not Geometry.isClose(center, (atacker.position.x, atacker.position.y), 0.2)):
                        
                        center = ball.position.x, ball.position.y
                        x_d = center[0] + raio * math.cos(theta)
                        y_d = center[1] + raio * math.sin(theta)
                    desired_position = (x_d, y_d)
                    
                    if(ball.position.y < 0):
                        orientation = -1
                        
                    else:
                        orientation = 1
                        
                    """---------------------------------------------GO TO ORBIT POINT---------------------------------------------"""  
                    
                    position = atacker.position
                    
                    positionX = position.x
                    positionY = position.y
                    atackerAngle = position.theta

                    xTarget, yTarget = FIRASimHelper.normalizePosition(desired_position[0], desired_position[1] + (orientation *0.25), True)

                    # Add a phase shift to the angle calculation for orbital motion
                    angleToTarget = math.atan2(yTarget - positionY, xTarget - positionX) + (orientation * -math.pi/ 24 )

                    error = Geometry.smallestAngleDiff(angleToTarget, atackerAngle)

                    if abs(error) > math.pi / 2.0 + math.pi / 20.0:
                        reversed = True
                        atackerAngle = Geometry.normalizeInPI(atackerAngle + math.pi)
                        error = Geometry.smallestAngleDiff(angleToTarget, atackerAngle)
                    else:
                        reversed = False

                    kP = 20
                    kD = 0.1

                    motorSpeed = (kP * error) + (kD * (error - lastError))

                    baseSpeed = ConfigurationHelper.getBaseSpeed()

                    motorSpeed = RobotHelper.truncateMotorSpeed(motorSpeed, baseSpeed)

                    leftMotorSpeed, rightMotorSpeed = Motion._getSpeeds(motorSpeed, baseSpeed, reversed)
                    
                    """---------------------------------------------FiM DE GOTOORBPONT---------------------------------------------"""  
                    teamControl.transmit_robot(2, leftMotorSpeed, rightMotorSpeed)        
                    cont += 1
                    lastError = error
                else:
                    cont = 0
                    
                """////////////////////////////////////////////////////////////////////////////////////////////////////////////////////"""
                
                """---------------------------------------------DEFENSOR SEGUE A BOLA EM Y---------------------------------------------"""
                    
                position = Defenser.position
                robotAngle = position.theta
                targetPosition = (-0.7, ball.position.y)

                xTarget, yTarget = FIRASimHelper.normalizePosition(targetPosition[0], targetPosition[1], True)

                angleToTarget = math.atan2(yTarget - position.y, xTarget - position.x)

                error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)

                if abs(error) > math.pi / 2.0 + math.pi / 20.0:
                    reversed = True
                    robotAngle = Geometry.normalizeInPI(robotAngle + math.pi)
                    error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)
                else:
                    reversed = False

                kP = 20
                kD = 0.1    
                motorSpeed = (kP * error) + (kD * (error - 0))

                baseSpeed = ConfigurationHelper.getBaseSpeed()

                motorSpeed = RobotHelper.truncateMotorSpeed(motorSpeed, baseSpeed)

                leftMotorSpeed, rightMotorSpeed = Motion._getSpeeds(motorSpeed, baseSpeed, reversed)

                teamControl.transmit_robot(0, leftMotorSpeed, rightMotorSpeed)  
            else: 
                """//////////////////////////////////////////////////////////////////////////////////////////////////////////////"""
                
                """------------------------------------------ATACANTE SEGUE A BOLA EM Y------------------------------------------"""
                #go on direction
                position = atacker.position
                robotAngle = position.theta
                targetPosition = (0.3, ball.position.y)

                xTarget, yTarget = FIRASimHelper.normalizePosition(targetPosition[0], targetPosition[1], True)

                angleToTarget = math.atan2(yTarget - position.y, xTarget - position.x)

                error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)

                if abs(error) > math.pi / 2.0 + math.pi / 20.0:
                    reversed = True
                    robotAngle = Geometry.normalizeInPI(robotAngle + math.pi)
                    error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)
                else:
                    reversed = False

                kP = 20
                kD = 0.1    
                motorSpeed = (kP * error) + (kD * (error - 0))

                baseSpeed = ConfigurationHelper.getBaseSpeed()

                motorSpeed = RobotHelper.truncateMotorSpeed(motorSpeed, baseSpeed)

                leftMotorSpeed, rightMotorSpeed = Motion._getSpeeds(motorSpeed, baseSpeed, reversed)

                teamControl.transmit_robot(2, leftMotorSpeed, rightMotorSpeed)            
                """///////////////////////////////////////////////////////////////////////////////////////////////////////"""
                
                """---------------------------DEFENSOR CHUTA A BOLA PRA FORA DO CAMPO DE DEFESA---------------------------"""     
                if(ball.position.x > Defenser.position.x):
                    position = Defenser.position
                    robotAngle = position.theta
                    targetPosition = (ball.position.x, ball.position.y)

                    xTarget, yTarget = FIRASimHelper.normalizePosition(targetPosition[0], targetPosition[1] + 0.125, True)

                    angleToTarget = math.atan2(yTarget - position.y, xTarget - position.x)

                    error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)

                    if abs(error) > math.pi / 2.0 + math.pi / 20.0:
                        reversed = True
                        robotAngle = Geometry.normalizeInPI(robotAngle + math.pi)
                        error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)
                    else:
                        reversed = False

                    kP = 20
                    kD = 0.1    
                    motorSpeed = (kP * error) + (kD * (error - 0))

                    baseSpeed = ConfigurationHelper.getBaseSpeed()

                    motorSpeed = RobotHelper.truncateMotorSpeed(motorSpeed, baseSpeed)

                    leftMotorSpeed, rightMotorSpeed = Motion._getSpeeds(motorSpeed, baseSpeed, reversed)

                    teamControl.transmit_robot(0, leftMotorSpeed, rightMotorSpeed) 
                    
                else:
                    raio = 0.12

                    ball = fieldData.ball
                    
                    center = ball.position.x, ball.position.y

                    # Compute the initial angle in radians
                    theta = (math.atan2(center[1] - Defenser.position.y, center[0] - Defenser.position.x)) + 0.0654 * contdef
                    
                    x_d = center[0] + raio * math.cos(theta)
                    y_d = center[1] + raio * math.sin(theta)
                    
                    orientation =1
                    
                    """------------------------------------COMEÇO DO DELIRIO------------------------------------"""
                    if Defenser.position.x > (ball.position.x - 0.2) and ball.position.x < 0.2: 
                    

                        vision.update()
                        ball = fieldData.ball
                        Defenser = fieldData.robots[0]
                        
                        if(not Geometry.isClose(center, (Defenser.position.x, Defenser.position.y), 0.1)):
                            
                            center = ball.position.x, ball.position.y
                            x_d = center[0] + raio * math.cos(theta)
                            y_d = center[1] + raio * math.sin(theta)
                            
                        desired_position = (x_d, y_d)
                        
                        if(ball.position.y < 0):
                            orientation = -1
                            
                        else:
                            orientation = 1
                            
                        """---------------------------------------------GO TO ORBIT POINT---------------------------------------------"""  
                        
                        position = Defenser.position
                        
                        positionX = position.x
                        positionY = position.y
                        DefenserAngle = position.theta

                        xTarget, yTarget = FIRASimHelper.normalizePosition(desired_position[0], desired_position[1] + (orientation *0.28), True)

                        # Add a phase shift to the angle calculation for orbital motion
                        angleToTarget = math.atan2(yTarget - positionY, xTarget - positionX) + (orientation * -math.pi/ 25 )

                        error = Geometry.smallestAngleDiff(angleToTarget, DefenserAngle)

                        if abs(error) > math.pi / 2.0 + math.pi / 20.0:
                            reversed = True
                            DefenserAngle = Geometry.normalizeInPI(DefenserAngle + math.pi)
                            error = Geometry.smallestAngleDiff(angleToTarget, DefenserAngle)
                        else:
                            reversed = False

                        kP = 20
                        kD = 0.1

                        motorSpeed = (kP * error) + (kD * (error - lastError))

                        baseSpeed = ConfigurationHelper.getBaseSpeed()

                        motorSpeed = RobotHelper.truncateMotorSpeed(motorSpeed, baseSpeed)

                        leftMotorSpeed, rightMotorSpeed = Motion._getSpeeds(motorSpeed, baseSpeed, reversed)
                        
                        """---------------------------------------------FiM DE GOTOORBPONT---------------------------------------------"""  
                        teamControl.transmit_robot(0, leftMotorSpeed, rightMotorSpeed)        

                        lastError = error  
                    else:
                        contdef = 0
            
            """/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////""" 
                 
            """-------------------------------------------------------GOLEIRO-------------------------------------------------------"""
            position = goalkeeper.position

            robotAngle = position.theta
            if(abs(ball.position.y) > 0.300):
                ballY = 0.300
            else:
                ballY = ball.position.y        
            
            targetPosition = (-1.175, ballY)

            xTarget, yTarget = FIRASimHelper.normalizePosition(targetPosition[0], targetPosition[1], True)

            angleToTarget = math.atan2(yTarget - position.y, xTarget - position.x)

            error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)

            if abs(error) > math.pi / 2.0 + math.pi / 20.0:
                reversed = True
                robotAngle = Geometry.normalizeInPI(robotAngle + math.pi)
                error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)
            else:
                reversed = False

            kP = 20
            kD = 0.1    
            motorSpeed = (kP * error) + (kD * (error - 0))

            baseSpeed = ConfigurationHelper.getBaseSpeed()

            motorSpeed = RobotHelper.truncateMotorSpeed(motorSpeed, baseSpeed)

            leftMotorSpeed, rightMotorSpeed = Motion._getSpeeds(motorSpeed, baseSpeed, reversed)

            teamControl.transmit_robot(1, leftMotorSpeed, rightMotorSpeed)  
        

def createTeam(isYellowTeam, threads):
    global fsimcontroler
    fieldData = FieldData()

    vision = ProtoVision(
        team_color_yellow=isYellowTeam, 
        field_data=fieldData)
    
    teamControl = ProtoControl(
        team_color_yellow= isYellowTeam, 
        control_ip=CONTROL_IP, 
        control_port=CONTROL_PORT)

    fsimcontroler = (fieldData, vision, teamControl)
    
    atackerThread = threading.Thread(target=atackerPlayerThread, args=fsimcontroler)

    threads.append(atackerThread)
    
    atackerThread.start()
    
def main():

    threads = []

    createTeam(IS_YELLOW_TEAM, threads)

    if RUN_BOTH_TEAMS:
        createTeam(not IS_YELLOW_TEAM, threads)

    for thread in threads:
        thread.join()

if __name__ == '__main__':
    main()
