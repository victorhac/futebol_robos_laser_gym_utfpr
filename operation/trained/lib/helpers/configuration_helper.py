import json
import os

from ..helpers.field_helper import FieldHelper

class ConfigurationHelper:
    __configuration = None

    @staticmethod
    def getConfiguration():
        if ConfigurationHelper.__configuration is not None:
            return ConfigurationHelper.__configuration
        
        configurationFilePath = os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "configuration",
            "configuration.json")
        
        with open(configurationFilePath, 'r') as f:
            configuration = json.loads(f.read())

        ConfigurationHelper.__configuration = configuration

        return configuration
    
    @staticmethod
    def getRobotWheelRadius():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["robot"]["wheel"]["radius"]
    
    @staticmethod
    def getRobotSpeedBase():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["robot"]["speed"]["base"]
    
    @staticmethod
    def getRobotSpeedDeadZone():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["robot"]["speed"]["dead-zone"]
    
    @staticmethod
    def getRobotWidth():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["robot"]["width"]
    
    @staticmethod
    def getRobotLength():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["robot"]["length"]
    
    @staticmethod
    def getTeamIsYellowLeftTeam():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["team"]["is-yellow-left-team"]
    
    @staticmethod
    def getTeamIsYellowTeam():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["team"]["is-yellow-team"]
    
    @staticmethod
    def isLeftTeam():
        isYellowTeam = ConfigurationHelper.getTeamIsYellowTeam()
        isYellowLeftTeam = ConfigurationHelper.getTeamIsYellowLeftTeam()

        return FieldHelper.isLeftTeam(isYellowTeam, isYellowLeftTeam)
    
    @staticmethod
    def getTeamBlueNumberRobots():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["team"]["blue"]["number-robots"]
    
    @staticmethod
    def getTeamYellowNumberRobots():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["team"]["yellow"]["number-robots"]
    
    @staticmethod
    def getMotionPIDConstanstsKp():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["motion"]["pid"]["constants"]["Kp"]
    
    @staticmethod
    def getMotionPIDConstanstsKi():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["motion"]["pid"]["constants"]["Ki"]
    
    @staticmethod
    def getMotionPIDConstanstsKd():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["motion"]["pid"]["constants"]["Kd"]
    
    @staticmethod
    def getMotionCollisionAvoidanceMinDistance():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["motion"]["collision-avoidance"]["min-distance"]
    
    @staticmethod
    def getFIRASimControlIp():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["FIRASim"]["control"]["ip"]
    
    @staticmethod
    def getFIRASimControlPort():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["FIRASim"]["control"]["port"]
    
    @staticmethod
    def getFIRASimVisionIp():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["FIRASim"]["vision"]["ip"]
    
    @staticmethod
    def getFIRASimVisionPort():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["FIRASim"]["vision"]["port"]
    
    @staticmethod
    def getFieldLength():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["field"]["length"]
    
    @staticmethod
    def getFieldWidth():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["field"]["width"]
    
    @staticmethod
    def getFieldGoalWidth():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["field"]["goal"]["width"]
    
    @staticmethod
    def getFieldGoalAreaWidth():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["field"]["goal-area"]["width"]
    
    @staticmethod
    def getFieldGoalAreaLength():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["field"]["goal-area"]["length"]
    
    @staticmethod
    def getFieldBallRadius():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["field"]["ball"]["radius"]
    
    @staticmethod
    def getTrainingTimeStep():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["training"]["time-step"]
    
    @staticmethod
    def getTrainingEpisodeDuration():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["training"]["episode-duration"]
    
    @staticmethod
    def getTrainingVelocityClipValue():
        configuration = ConfigurationHelper.getConfiguration()
        return configuration["training"]["velocity-clip-value"]