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

IS_LEFT_TEAM = FieldHelper.isLeftTeam(IS_YELLOW_TEAM, IS_YELLOW_LEFT_TEAM)

CONTROL_IP = CONFIGURATION["FIRASim"]["control"]["ip"]
CONTROL_PORT = CONFIGURATION["FIRASim"]["control"]["port"]

GOAL_WIDTH = CONFIGURATION["field"]["goal"]["width"]
FIELD_LENGTH = CONFIGURATION["field"]["length"]

GOAL_LINE_DISTANCE_TO_CENTER = 0.7
DEFENSE_LINE_DISTANCE_TO_GOAL = 0.3

ATACKER_ROBOT_ID = 0
DEFENSE_ROBOT_ID = 1
GOALKEEPER_ROBOT_ID = 2

def placeRobot(
    id: int,
    fieldData: FieldData,
    targetPosition:'tuple[float, float]',
    vision: ProtoVision,
    teamControl: ProtoControl
):
    robot = fieldData.robots[id]

    position = robot.position

    error = 0
    counter = 0
    maxCounter = 100

    vision.update()
    
    while not Geometry.isClose(
            (position.x, position.y),
            FIRASimHelper.normalizePosition(
                targetPosition[0],
                targetPosition[1],
                IS_LEFT_TEAM),
            0.05) \
        and counter < maxCounter:

        (leftSpeed, rightSpeed, error) = Motion.goToPoint(robot, targetPosition, IS_LEFT_TEAM, error)

        teamControl.transmit_robot(id, leftSpeed, rightSpeed)

        vision.update()

        counter += 1

    teamControl.transmit_robot(id, 0, 0)

def followBall(
    id: int,
    fieldData: FieldData,
    vision: ProtoVision,
    teamControl: ProtoControl
):
    robot = fieldData.robots[id]
    ball = fieldData.ball

    position = robot.position

    error = 0
    counter = 0
    maxCounter = 100

    vision.update()
    targetPosition = (ball.position.x, ball.position.y)
    
    while not Geometry.isClose(
            (position.x, position.y),
            FIRASimHelper.normalizePosition(
                targetPosition[0],
                targetPosition[1],
                IS_LEFT_TEAM),
            0.05) \
        and counter < maxCounter:

        vision.update()
        targetPosition = (ball.position.x, ball.position.y)

        (leftSpeed, rightSpeed, error) = Motion.goToPoint(robot, targetPosition, IS_LEFT_TEAM, error)

        teamControl.transmit_robot(id, leftSpeed, rightSpeed)

        counter += 1

def spinIfCloseToBall(
    id: int,
    robot: EntityData,
    vision: ProtoVision,
    teamControl: ProtoControl
):
    vision.update()

    ball = vision.field_data.ball
    position = (robot.position.x, robot.position.y)
    targetPosition = (ball.position.x, ball.position.y)

    if Geometry.isClose(
        (position[0], position[1]),
        FIRASimHelper.normalizePosition(targetPosition[0], targetPosition[1], IS_LEFT_TEAM),
        0.1):

        (leftSpeed, rightSpeed) = Motion.spin(True, 30)

        teamControl.transmit_robot(id, leftSpeed, rightSpeed)

        time.sleep(0.5)

    teamControl.transmit_robot(id, 0, 0)

def testToChangeAttackAndDefense(
    vision: ProtoVision,
    teamControl: ProtoControl,
    fieldData: FieldData
):
    vision.update()

    global DEFENSE_ROBOT_ID
    global ATACKER_ROBOT_ID

    defenseRobot = fieldData.robots[DEFENSE_ROBOT_ID]
    atackerRobot = fieldData.robots[ATACKER_ROBOT_ID]

    distance = atackerRobot.position.x - defenseRobot.position.x

    if distance < 0: #If the defender is in front of the atacker, then change functions.
        OLD_ATTACKER = ATACKER_ROBOT_ID
        ATACKER_ROBOT_ID = DEFENSE_ROBOT_ID
        DEFENSE_ROBOT_ID = OLD_ATTACKER



def defensePlayerThread(
    fieldData: FieldData,
    vision: ProtoVision,
    teamControl: ProtoControl
):
    ball = fieldData.ball
    defenseRobot = fieldData.robots[DEFENSE_ROBOT_ID]

    if IS_LEFT_TEAM:
        xCoordinateDefensor = -GOAL_LINE_DISTANCE_TO_CENTER + DEFENSE_LINE_DISTANCE_TO_GOAL
    else:
        xCoordinateDefensor = GOAL_LINE_DISTANCE_TO_CENTER - DEFENSE_LINE_DISTANCE_TO_GOAL
    
    targetPosition = (xCoordinateDefensor, 0)

    placeRobot(
        DEFENSE_ROBOT_ID,
        fieldData, 
        targetPosition,
        vision,
        teamControl
    )

    while True:
        vision.update()

        if IS_LEFT_TEAM:
            positionToDefendGoalY = ( (xCoordinateDefensor + 0.75) * (ball.position.y) / (ball.position.x + 0.75) )
        else:
            positionToDefendGoalY = ( (0.75 - xCoordinateDefensor) * ball.position.y / (0.75 - ball.position.x) )

        placeRobot(
            DEFENSE_ROBOT_ID, 
            fieldData, 
            (xCoordinateDefensor, positionToDefendGoalY),
            vision,
            teamControl
        )

        spinIfCloseToBall(
            DEFENSE_ROBOT_ID,
            defenseRobot,
            vision,
            teamControl
        )

def goalkeeperPlayerThread(
    fieldData: FieldData,
    vision: ProtoVision,
    teamControl: ProtoControl,
    visioTest: Receiver
):
    ball = fieldData.ball
    goalkeeperRobot = fieldData.robots[GOALKEEPER_ROBOT_ID]

    #dataTest = visioTest.dataFinal(visioTest)
    

    if IS_LEFT_TEAM:
        xCoordinateGoalkeeper = -GOAL_LINE_DISTANCE_TO_CENTER
    else:
        xCoordinateGoalkeeper = GOAL_LINE_DISTANCE_TO_CENTER
    
    targetPosition = (xCoordinateGoalkeeper, 0)
    
    placeRobot(
        GOALKEEPER_ROBOT_ID,
        fieldData,
        targetPosition,
        vision,
        teamControl)

    while True:
        vision.update()
        #print(f'Dados: {dataTest}')

        ballPosition = (ball.position.x, ball.position.y)

        intersection = (xCoordinateGoalkeeper, ballPosition[1])

        if intersection:
            if FieldHelper.isInsideField(
                intersection[0],
                intersection[1],
                FIELD_LENGTH,
                GOAL_WIDTH):

                placeRobot(
                    GOALKEEPER_ROBOT_ID,
                    fieldData,
                    intersection,
                    vision,
                    teamControl)
                spinIfCloseToBall(
                    GOALKEEPER_ROBOT_ID,
                    goalkeeperRobot,
                    vision,
                    teamControl)
                
def visionPlayerThread(
    fieldData: FieldData,
    vision: ProtoVision,
    teamControl: ProtoControl,
    visioTest: Receiver
):
    dataTest = visioTest.dataFinal(visioTest)
    cont = 0
    while(cont < 10):
        # print(f'Receiver: {dataTest}')
        # print(f'FieldData: {fieldData}')
        cont+=1

def atackerPlayerThread(
    fieldData: FieldData,
    vision: ProtoVision,
    teamControl: ProtoControl
):
    atackerRobot = fieldData.robots[ATACKER_ROBOT_ID]

    while True:
        
        followBall(ATACKER_ROBOT_ID, fieldData, vision, teamControl)

        spinIfCloseToBall(
            ATACKER_ROBOT_ID,
            atackerRobot,
            vision,
            teamControl)

        testToChangeAttackAndDefense(
            vision,
            teamControl,
            fieldData
        )        


def main():
    fieldData = FieldData()

    vision = ProtoVision(
        team_color_yellow=IS_YELLOW_TEAM, 
        field_data=fieldData)
    
    visionTest = Receiver
    
    teamControl = ProtoControl(
        team_color_yellow=IS_YELLOW_TEAM, 
        control_ip=CONTROL_IP, 
        control_port=CONTROL_PORT)

    threads = []

    args = (fieldData, vision, teamControl)
    argsTest = (fieldData, vision, teamControl, visionTest)

    visionThread = threading.Thread(target=visionPlayerThread, args=argsTest)
    goalkeeperThread = threading.Thread(target=goalkeeperPlayerThread, args=argsTest)
    defensorThread = threading.Thread(target=defensePlayerThread, args=args)
    atackerThread = threading.Thread(target=atackerPlayerThread, args=args)

    threads.append(goalkeeperThread)
    threads.append(defensorThread)
    threads.append(atackerThread)
    threads.append(visionThread)
    

    goalkeeperThread.start()
    defensorThread.start()
    atackerThread.start()
    visionThread.start()

    for thread in threads:
        thread.join()

if __name__ == '__main__':
    main()