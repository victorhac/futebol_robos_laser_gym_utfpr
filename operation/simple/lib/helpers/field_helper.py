class FieldHelper:
    @staticmethod
    def isLeftTeam(isYellowTeam: bool, isYellowLeftTeam: bool):
        return (isYellowLeftTeam and isYellowTeam) or (not isYellowLeftTeam and not isYellowTeam)
    
    @staticmethod
    def isInsideField(
        x: float, y: float,
        fieldWidth: float,
        fieldHeight: float):
        
        return abs(x) < fieldWidth / 2 and abs(y) < fieldHeight / 2