from lib.comm.vision import ProtoVision
from lib.core.data import FieldData
from lib.comm.control import ProtoControl

import math

pidArgs = {'lastError': 0}

def pid(fieldData: FieldData, 
        targetPosition: tuple[float, float]):
    reversed = False

    robot = fieldData.robots[0]
    position = robot.position
    theta = robot.velocity.theta

    xTarget, yTarget = targetPosition

    print(f"Robot position: {position.x:.2f}, {position.y:.2f}")
    print(f"Robot velocity: {robot.velocity.x:.2f}, {robot.velocity.y:.2f}")
    print(f"Robot theta: {theta:.2f}")

    robotAngle = theta
    angleToTarget = math.atan2(yTarget - position.y, xTarget - position.x)

    print(f"Angle to target: {angleToTarget}")

    error = smallestAngleDiff(angleToTarget, robotAngle)

    if abs(error) > math.pi / 2.0 + math.pi / 20.0:
        reversed = True
        robotAngle = normalizeInPI(robotAngle + math.pi)
        error = smallestAngleDiff(angleToTarget, robotAngle)

    # Kp = 20, Kd = 0.1
    motorSpeed = (20.0 * error) + (2.5 * (error - pidArgs['lastError']))
    pidArgs['lastError'] = error

    baseSpeed = 30

    if motorSpeed > 30:
        motorSpeed = 30
    elif motorSpeed < -30:
        motorSpeed = -30

    if motorSpeed > 0:
        leftMotorSpeed = baseSpeed
        rightMotorSpeed = baseSpeed - motorSpeed
    else:
        leftMotorSpeed = baseSpeed + motorSpeed
        rightMotorSpeed = baseSpeed

    if reversed:
        if motorSpeed > 0:
            leftMotorSpeed = -baseSpeed + motorSpeed
            rightMotorSpeed = -baseSpeed
        else:
            leftMotorSpeed = -baseSpeed
            rightMotorSpeed = -baseSpeed - motorSpeed

    print(f"Left motor speed: {leftMotorSpeed:.2f}")
    print(f"Right motor speed: {rightMotorSpeed:.2f}")

    return leftMotorSpeed, rightMotorSpeed

def spin(clockwise):
    # TODO: colocar nas configurações
    spinPower = 30

    if clockwise:
        return spinPower, -spinPower
    
    return -spinPower, spinPower

def smallestAngleDiff(angle1, angle2):
    PI = math.pi
    angle = (angle2 - angle1) % (2 * PI)
    if angle >= PI:
        return angle - (2 * PI)
    elif angle < -PI:
        return angle + (2 * PI)
    return angle

def normalizeAngle(value, center, amplitude):
    value = value % (2 * amplitude)
    if value < -amplitude + center:
        value += 2 * amplitude
    elif value > amplitude + center:
        value -= 2 * amplitude
    return value

def normalizeInPI(radians):
    PI = math.pi
    return normalizeAngle(radians, 0, PI)

def main():
    fieldData = FieldData()

    vision = ProtoVision(
        team_color_yellow=True, 
        field_data=fieldData)
    
    yellowTeamControl = ProtoControl(
        team_color_yellow=True, 
        control_ip="127.0.0.1", 
        control_port=20011)
    
    vision.update()

    velocities = pid(fieldData, (0, 0.3))

    (leftSpeed, rightSpeed) = velocities

    yellowTeamControl.transmit_robot(0, leftSpeed, rightSpeed)

    vision.update()

if __name__ == '__main__':
    main()
