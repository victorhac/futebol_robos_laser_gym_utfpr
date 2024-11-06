from google.protobuf import json_format
from google.protobuf import message
from google.protobuf.timestamp_pb2 import Timestamp
from google.protobuf.duration_pb2 import Duration
import copy

class RefereeStage:
    NORMAL_FIRST_HALF_PRE = 0

class Command:
    HALT = 0

class GameState:
    HALT = 0

class Division:
    DIV_A = 0

class Team:
    UNKNOWN = -1
    YELLOW = 1
    BLUE = 2

    @staticmethod
    def value_of(color_str):
        if color_str == "YELLOW":
            return Team.YELLOW
        elif color_str == "BLUE":
            return Team.BLUE
        return Team.UNKNOWN

class MatchType:
    UNKNOWN_MATCH = 0

class GameEventType:
    TYPE_UNKNOWN = -1

class TeamInfo:
    def __init__(self):
        self.Name = ""
        self.OnPositiveHalf = False

class GameEvent:
    def __init__(self):
        self.Type = GameEventType.TYPE_UNKNOWN

    def ByTeam(self):
        return Team.UNKNOWN

class State:
    def __init__(self):
        self.Stage = RefereeStage.NORMAL_FIRST_HALF_PRE
        self.Command = Command.HALT
        self.GameState = GameState.HALT
        self.StageTimeElapsed = Duration()
        self.StageTimeLeft = Duration()
        self.MatchTimeStart = Timestamp()
        self.CurrentActionTimeRemaining = Duration()
        self.Division = Division.DIV_A
        self.FirstKickoffTeam = Team.YELLOW
        self.MatchType = MatchType.UNKNOWN_MATCH
        self.MaxBotsPerTeam = 0

        self.TeamState = {
            "YELLOW": self.new_team_info(),
            "BLUE": self.new_team_info(),
        }
        self.TeamState["YELLOW"].OnPositiveHalf = not self.TeamState["BLUE"].OnPositiveHalf
        self.GameEvents = []

    @staticmethod
    def new_team_info():
        return TeamInfo()

    def clone(self):
        return copy.deepcopy(self)

    def team_info(self, team: Team):
        return self.TeamState.get(team.name)

    def team_by_name(self, name: str) -> Team:
        for team_color, team_info in self.TeamState.items():
            if team_info.Name == name:
                return Team.value_of(team_color)
        return Team.UNKNOWN

    def to_json_string(self) -> str:
        try:
            return json_format.MessageToJson(self)
        except message.EncodeError as e:
            return str(e)

    def has_game_event_by_team(self, event_type: GameEventType, team: Team) -> bool:
        for game_event in self.GameEvents:
            if game_event.Type == event_type and game_event.ByTeam() == team:
                return True
        return False

    def find_game_events(self, event_type: GameEventType):
        return [event for event in self.GameEvents if event.Type == event_type]

    def find_game_events_by_team(self, event_type: GameEventType, team: Team):
        return [
            event for event in self.GameEvents
            if event.Type == event_type and (team == Team.UNKNOWN or event.ByTeam() == team)
        ]