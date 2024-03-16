class FieldHelper:
    @staticmethod
    def isLeftTeam(isYellowTeam: bool, isYellowLeftTeam: bool):
        return (isYellowLeftTeam and isYellowTeam) or (not isYellowLeftTeam and not isYellowTeam)