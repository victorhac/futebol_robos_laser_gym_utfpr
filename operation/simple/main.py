import time
import threading

# from lib.comm.vision import ProtoVision
from communication.receiver.vision import SSLVisionReceiver as ProtoVision
from lib.core.data import EntityData, FieldData
from lib.comm.control import ProtoControl
from lib.geometry.geometry import Geometry

from lib.helpers.configuration_helper import ConfigurationHelper
from lib.motion.motion import Motion
from lib.helpers.field_helper import FieldHelper
from lib.helpers.firasim_helper import FIRASimHelper

from communication.receiver.receiver import Receiver

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

        (leftSpeed, rightSpeed, error) = Motion.goToPoint(robot, targetPosition, IS_LEFT_TEAM)

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
            
    
def atackerPlayerThread(
    fieldData: FieldData,
    vision: ProtoVision,
    teamControl: ProtoControl
):
    global fsimcontroler

    ball = fieldData.ball
    atacker = fieldData.robots[ATAKER_ID]
    ballPosition = ball.position.x, ball.position.y
    atackerPosition = atacker.position.x, atacker.position.y
    
    while True:
        vision.update()
        """ballPosition = ball.position.x, ball.position.y
        atackerPosition = atacker.position.x, atacker.position.y
        
        if between(ballPosition[0],0.4, 0.7) and between(ballPosition[1], -0.3, 0.3) and between(atackerPosition[1], -0.3, 0.3) :
            
            ballDirection = Geometry.directionalVector(atackerPosition, ballPosition)
            (leftSpeed, rightSpeed, error) = Motion.goOnDirection(ballDirection, atacker)
            teamControl.transmit_robot(ATAKER_ID, leftSpeed, rightSpeed)
            
        elif ball.position.x < 0:
            followBallY(ATAKER_ID,(0,1, ballPosition[1]))

            (leftSpeed, rightSpeed, error) = Motion.faceDirection(atacker, ballPosition, IS_LEFT_TEAM)
            teamControl.transmit_robot(ATAKER_ID, leftSpeed, rightSpeed)   
        else: 
            Motion.AtkOrbit(fsimcontroler, ATAKER_ID, IS_LEFT_TEAM)"""
 
def defensePlayerThread(
    fieldData: FieldData,
    vision: ProtoVision,
    teamControl: ProtoControl
):
    ball = fieldData.ball
    Defenser = fieldData.robots[DEFENSER_ID]
    global fsimcontroler

    vision.update()
    while True:
        vision.update()
        ballPosition = ball.position.x, ball.position.y
        defenserPosition = Defenser.position.x, Defenser.position.y

        if(ball.position.x > 0):
            
            followBallY(DEFENSER_ID,(-DEFENSE_LINE_DISTANCE_TO_GOAL, ballPosition[1]))

            (leftSpeed, rightSpeed, error) = Motion.faceDirection(Defenser, ballPosition, IS_LEFT_TEAM)
            teamControl.transmit_robot(DEFENSER_ID, leftSpeed, rightSpeed)      
        elif ball.position.x > Defenser.position.x:
            #arrancar a bola do campo de defesa
            ballDirection = Geometry.directionalVector(defenserPosition, ballPosition)
            (leftSpeed, rightSpeed, error) = Motion.goOnDirection(ballDirection, Defenser)
            teamControl.transmit_robot(DEFENSER_ID, leftSpeed, rightSpeed)
            
        else:
            
            spinIfCloseToBall(DEFENSER_ID)

def yGoalValue():
    global fsimcontroler
    fieldData = fsimcontroler[0]
    ball = fieldData.ball
    if(ball.position.y > 0.200):
        return 0.180
    elif(ball.position.y <-0.200):
        return -0.180
    return ball.position.y

def goalkeeperPlayerThread(
    fieldData: FieldData,   
    vision: ProtoVision,
    teamControl: ProtoControl
):
    ball = fieldData.ball
    goalkeeper = fieldData.robots[GOALKEEPER_ROBOT_ID]

    if IS_LEFT_TEAM:
        xCoordinateGoalkeeper = -GOAL_LINE_DISTANCE_TO_CENTER
    else:
        xCoordinateGoalkeeper = GOAL_LINE_DISTANCE_TO_CENTER
    
    targetPosition = (xCoordinateGoalkeeper, 0.0)
   
    #placeRobot( GOALKEEPER_ROBOT_ID, targetPosition)

    raio = 0.175
    
    while True:
        vision.update()
        """origem = [-GOAL_LINE_DISTANCE_TO_CENTER, yGoalValue()]
        goalkeeperPosition = (goalkeeper.position.x, goalkeeper.position.y)
        
        ballPosition = [ball.position.x, ball.position.y]
        ballDirection = Geometry.directionalVector(origem, ballPosition)
        

        if(Geometry.isClose(goalkeeperPosition, ballPosition, 0.15) and goalkeeperPosition[1] < yGoalValue()*1.25 ):
            targetPosition = Geometry.PointOnDirection(origem, ballDirection, 1.5 * raio)
            placeRobot(GOALKEEPER_ROBOT_ID, targetPosition)         
        else:
            targetPosition = Geometry.PointOnDirection(origem, ballDirection, raio)
            placeRobot(GOALKEEPER_ROBOT_ID, targetPosition)
            (leftSpeed, rightSpeed, error) = Motion.faceDirection(goalkeeper, ballPosition, IS_LEFT_TEAM)
            teamControl.transmit_robot(GOALKEEPER_ROBOT_ID, leftSpeed, rightSpeed)"""
    
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

    goalkeeperThread = threading.Thread(target=goalkeeperPlayerThread, args=fsimcontroler)
    defensorThread = threading.Thread(target=defensePlayerThread, args=fsimcontroler)
    atackerThread = threading.Thread(target=atackerPlayerThread, args=fsimcontroler)

    threads.append(goalkeeperThread)
    threads.append(defensorThread)
    threads.append(atackerThread)

    goalkeeperThread.start()
    defensorThread.start()
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
