class FieldHelper:
    @staticmethod
    def isLeftTeam(isYellowTeam: bool, isYellowLeftTeam: bool):
        return (isYellowLeftTeam and isYellowTeam) or (not isYellowLeftTeam and not isYellowTeam)
    
    @staticmethod
    def isInsideField(
        x: float, y: float,
        fieldWidth: float,
        fieldHeight: float
    ):
        
        return abs(x) < fieldWidth / 2 and abs(y) < fieldHeight / 2
    
    @staticmethod
    def getOpponentGoalPosition(
        fieldLength: float,
        isLeftTeam: bool
    ):
        if isLeftTeam:
            return (fieldLength / 2, 0)
        else:
            return (-fieldLength / 2, 0)
    
    @staticmethod
    def getOwnGoalPosition(
        fieldLength: float,
        isLeftTeam: bool
    ):
        if isLeftTeam:
            return (-fieldLength / 2, 0)
        else:
            return (fieldLength / 2, 0)