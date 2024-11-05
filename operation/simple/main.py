import time
import threading

# from lib.comm.vision import ProtoVision
from communication.receiver.ssl_vision_receiver import SSLVisionReceiver as ProtoVision
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
    atacker = fieldData.robots[0]
    ballPosition = ball.position.x, ball.position.y
    atackerPosition = atacker.position.x, atacker.position.y
    
    while True:
        vision.update()

        ball = fieldData.ball
        atacker = fieldData.robots
        ballPosition = ball.position.x, ball.position.y
        atackerPosition = atacker.position.x, atacker.position.y
        ballDirection = Geometry.directionalVector(atackerPosition, ballPosition)

        (leftSpeed, rightSpeed, error) = Motion.GoOnDirection(ballDirection, atacker)
        teamControl.transmit_robot(0, leftSpeed, rightSpeed)   
  
 
        """ if between(ballPosition[0],0.4, 0.7) and between(ballPosition[1], -0.3, 0.3) and between(atackerPosition[1], -0.3, 0.3) :
            
            ballDirection = Geometry.directionalVector(atackerPosition, ballPosition)
            (leftSpeed, rightSpeed, error) = Motion.GoOnDirection(ballDirection, atacker)
            teamControl.transmit_robot(0, leftSpeed, rightSpeed)
            
        elif ball.position.x < 0:
            followBallY(0,(0,1, ballPosition[1]))

            (leftSpeed, rightSpeed, error) = Motion.FaceDirection(atacker, ballPosition, IS_LEFT_TEAM)
            teamControl.transmit_robot(0, leftSpeed, rightSpeed)   
        #else:
            #Motion.AtkOrbit(fsimcontroler, 0, IS_LEFT_TEAM)"""

def defensePlayerThread(
    fieldData: FieldData,
    vision: ProtoVision,
    teamControl: ProtoControl
):
    ball = fieldData.ball
    Defenser = fieldData.robots[1]
    global fsimcontroler

    '''vision.update()
    while True:
        vision.update()
        ball = fieldData.ball
        Defenser = fieldData.robots[1]
        ballPosition = ball.position.x, ball.position.y
        DefenserPosition = Defenser.position.x, Defenser.position.y
        ballDirection = Geometry.directionalVector(DefenserPosition, ballPosition)

        (leftSpeed, rightSpeed, error) = Motion.GoOnDirection(ballDirection, Defenser)
        teamControl.transmit_robot(1, leftSpeed, rightSpeed) 


        vision.update()
        
        ball = fieldData.ball
        Defenser = fieldData.robots[1]
        
        ballPosition = ball.position.x, ball.position.y
        defenserPosition = Defenser.position.x, Defenser.position.y
        
        if(ball.position.x > 0):
            
            #followBallY(1,(-DEFENSE_LINE_DISTANCE_TO_GOAL, ballPosition[1]))

            (leftSpeed, rightSpeed, error) = Motion.FaceDirection(Defenser, ballPosition, IS_LEFT_TEAM)
            teamControl.transmit_robot(1, leftSpeed, rightSpeed)      
        elif ball.position.x > Defenser.position.x:
            #arrancar a bola do campo de defesa
            ballDirection = Geometry.directionalVector(defenserPosition, ballPosition)
            (leftSpeed, rightSpeed, error) = Motion.GoOnDirection(ballDirection, Defenser)
            teamControl.transmit_robot(1, leftSpeed, rightSpeed)    '''
        # else:
            #spinIfCloseToBall(1)
            

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
    goalkeeper = fieldData.robots[2]

    if IS_LEFT_TEAM:
        xCoordinateGoalkeeper = -GOAL_LINE_DISTANCE_TO_CENTER
    else:
        xCoordinateGoalkeeper = GOAL_LINE_DISTANCE_TO_CENTER
    
    targetPosition = (xCoordinateGoalkeeper, 0.0)
   
    #placeRobot( 2, targetPosition)

    raio = 0.175
    
    while True:
        
        '''vision.update()
        ball = fieldData.ball
        goalkeeper = fieldData.robots[2]
        ballPosition = ball.position.x, ball.position.y
        goalkeeperPosition = goalkeeper.position.x, goalkeeper.position.y
        ballDirection = Geometry.directionalVector(goalkeeperPosition, ballPosition)

        (leftSpeed, rightSpeed, error) = Motion.GoOnDirection(ballDirection, goalkeeper)
        teamControl.transmit_robot(2, leftSpeed, rightSpeed)   '''

        """vision.update()
        ball = fieldData.ball
        goalkeeper = fieldData.robots[1]
        ballPosition = ball.position.x, ball.position.y
        goalkeeperPosition = goalkeeper.position.x, goalkeeper.position.y
        
        origem = [-GOAL_LINE_DISTANCE_TO_CENTER, yGoalValue()]
        goalkeeperPosition = (goalkeeper.position.x, goalkeeper.position.y)
        
        ballPosition = [ball.position.x, ball.position.y]
        ballDirection = Geometry.directionalVector(origem, ballPosition)
        

        if(Geometry.isClose(goalkeeperPosition, ballPosition, 0.15) and goalkeeperPosition[1] < yGoalValue()*1.25 ):
            targetPosition = Geometry.PointOnDirection(origem, ballDirection, 1.5 * raio)
            placeRobot(2, targetPosition)         
        else:
            targetPosition = Geometry.PointOnDirection(origem, ballDirection, raio)
            #placeRobot(2, targetPosition)
            (leftSpeed, rightSpeed, error) = Motion.FaceDirection(goalkeeper, ballPosition, IS_LEFT_TEAM)
            teamControl.transmit_robot(2, leftSpeed, rightSpeed)"""
    
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
