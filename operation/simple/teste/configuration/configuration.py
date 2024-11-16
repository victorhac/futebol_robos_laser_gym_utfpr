import json

from domain.enums.manual_command_enum import ManualCommandEnum
from domain.enums.robot_behavior_enum import RobotBehaviorEnum

CONFIGURATION_FILE_PATH = "./configuration/configuration.json"
CONFIGURATION_PLAYING_FILE_PATH = "./configuration/configuration.Playing.json"
CONFIGURATION_SIMULATED_FILE_PATH = "./configuration/configuration.Simulated.json"
CONFIGURATION_REMOTE_SENDER_FILE_PATH = "./configuration/configuration.RemoteSender.json"
LAUNCH_FILE_PATH = "./configuration/launch.json"

class Configuration:
    _instance = None

    def __init__(self):
        self.mode = None
        self.environment_mode = None
        self.receive_data_from_remote = None

        self.keys_private_key_path = None
        self.keys_public_key_path = None

        self.remote_computer_bluetooth_address = None
        self.remote_computer_bluetooth_channel = None
        self.remote_computer_bluetooth_game_controller_channel = None
        self.remote_computer_game_controller_receiver_buffer_size = None

        self.sslvision_team_robot_id_mapping_0 = None
        self.sslvision_team_robot_id_mapping_1 = None
        self.sslvision_team_robot_id_mapping_2 = None
        self.sslvision_ip = None
        self.sslvision_port = None
        self.sslvision_receiver_buffer_size = None
        self.sslvision_foe_team_desired_goalkeeper_mapped_id = None

        self.grsim_control_ip = None
        self.grsim_control_port = None
        self.grsim_vision_ip = None
        self.grsim_vision_port = None

        self.firasim_control_ip = None
        self.firasim_control_port = None
        self.firasim_vision_ip = None
        self.firasim_vision_port = None

        self.game_controller_address = None
        self.game_controller_port = None
        self.game_controller_register_as_team = None
        self.game_controller_foe_team_goalkeeper_desired_id = None

        self.referee_address = None
        self.referee_port = None
        self.referee_receiver_buffer_size = None

        self.motion_pid_constants_kp = None
        self.motion_pid_constants_ki = None
        self.motion_pid_constants_kd = None

        self.rsoccer_training_time_step = None
        self.rsoccer_training_episode_duration = None

        self.field_length = None
        self.field_width = None
        self.field_goal_width = None
        self.field_goal_depth = None
        self.field_goalkeeper_area_length =None
        self.field_goalkeeper_area_width = None
        self.field_goalkeeper_area_radius = None
        self.field_ball_radius = None

        self.robot_wheel_radius = None
        self.robot_speed_max_radians_seconds = None
        self.robot_width = None
        self.robot_length = None

        self.team_name = None
        self.team_is_yellow_left_team = None
        self.team_is_yellow_team = None
        self.team_blue_number_robots = None
        self.team_yellow_number_robots = None
        self.team_roles_attacker_id = None
        self.team_roles_defensor_id = None
        self.team_roles_goalkeeper_id = None

        self.stop_distance_to_ball = None

        self.prepare_kickoff_team_position_attacker_x = None
        self.prepare_kickoff_team_position_attacker_y = None
        self.prepare_kickoff_team_position_defensor_x = None
        self.prepare_kickoff_team_position_defensor_y = None
        self.prepare_kickoff_team_position_goalkeeper_x = None
        self.prepare_kickoff_team_position_goalkeeper_y = None
        self.prepare_kickoff_foe_team_position_attacker_x = None
        self.prepare_kickoff_foe_team_position_attacker_y = None
        self.prepare_kickoff_foe_team_position_defensor_x = None
        self.prepare_kickoff_foe_team_position_defensor_y = None
        self.prepare_kickoff_foe_team_position_goalkeeper_x = None
        self.prepare_kickoff_foe_team_position_goalkeeper_y = None

        self.prepare_penalty_team_position_attacker_x = None
        self.prepare_penalty_team_position_attacker_y = None
        self.prepare_penalty_team_position_defensor_x = None
        self.prepare_penalty_team_position_defensor_y = None
        self.prepare_penalty_team_position_goalkeeper_x = None
        self.prepare_penalty_team_position_goalkeeper_y = None
        self.prepare_penalty_foe_team_position_attacker_x = None
        self.prepare_penalty_foe_team_position_attacker_y = None
        self.prepare_penalty_foe_team_position_defensor_x = None
        self.prepare_penalty_foe_team_position_defensor_y = None
        self.prepare_penalty_foe_team_position_goalkeeper_x = None
        self.prepare_penalty_foe_team_position_goalkeeper_y = None

        self.normal_start_after_penalty_target_attacker_position_y=None
        self.normal_start_after_penalty_target_attacker_position_x =None

        self.strategy_defensor_defense_line_x = None
        self.strategy_attacker_spin_radius_to_spin = None

        self.time_to_run = None

        self.runtime_manual_command = ManualCommandEnum.NONE
        self.runtime_behavior_robot_0 = RobotBehaviorEnum.NONE
        self.runtime_behavior_robot_1 = RobotBehaviorEnum.NONE
        self.runtime_behavior_robot_2 = RobotBehaviorEnum.NONE

    def _get_team_robot_id_to_internal_mapping(self):
        return {
            self.sslvision_team_robot_id_mapping_0: 0,
            self.sslvision_team_robot_id_mapping_1: 1,
            self.sslvision_team_robot_id_mapping_2: 2
        }
    
    def _get_team_robot_id_to_external_mapping(self):
        return {
            0: self.sslvision_team_robot_id_mapping_0,
            1: self.sslvision_team_robot_id_mapping_1,
            2: self.sslvision_team_robot_id_mapping_2
        }

    def get_robot_id_to_internal_mapped(self, robot_id):
        return self._get_team_robot_id_to_internal_mapping().get(robot_id)
    
    def get_robot_id_to_external_mapped(self, robot_id):
        return self._get_team_robot_id_to_external_mapping().get(robot_id)
    
    def get_is_left_team(self):
        return self.team_is_yellow_left_team == self.team_is_yellow_team

    def get_prepare_kickoff_defensor_position(self):
        return self.prepare_kickoff_team_position_defensor_x, self.prepare_kickoff_team_position_defensor_y
    
    def get_prepare_kickoff_attacker_position(self):
        return self.prepare_kickoff_team_position_attacker_x, self.prepare_kickoff_team_position_attacker_y
    
    def get_prepare_kickoff_goalkeeper_position(self):
        return self.prepare_kickoff_team_position_goalkeeper_x, self.prepare_kickoff_team_position_goalkeeper_y
    
    def get_foe_team_prepare_kickoff_defensor_position(self):
        return self.prepare_kickoff_foe_team_position_defensor_x, self.prepare_kickoff_foe_team_position_defensor_y
    
    def get_foe_team_prepare_kickoff_attacker_position(self):
        return self.prepare_kickoff_foe_team_position_attacker_x, self.prepare_kickoff_foe_team_position_attacker_y
    
    def get_foe_team_prepare_kickoff_goalkeeper_position(self):
        return self.prepare_kickoff_foe_team_position_goalkeeper_x, self.prepare_kickoff_foe_team_position_goalkeeper_y
    
    def get_positions(
        self,
        attacker_position: 'tuple[float, float]',
        defensor_position: 'tuple[float, float]',
        goalkeeper_position: 'tuple[float, float]'
    ):
        return {
            self.team_roles_attacker_id: attacker_position,
            self.team_roles_defensor_id: defensor_position,
            self.team_roles_goalkeeper_id: goalkeeper_position
        }
    
    def get_prepare_penalty_team_positions(self):
        return self.get_positions(
            (self.prepare_penalty_team_position_attacker_x, self.prepare_penalty_team_position_attacker_y),
            (self.prepare_penalty_team_position_defensor_x, self.prepare_penalty_team_position_defensor_y),
            (self.prepare_penalty_team_position_goalkeeper_x, self.prepare_penalty_team_position_goalkeeper_y)
        )
    
    def get_prepare_penalty_foe_team_positions(self):
        return self.get_positions(
            (self.prepare_penalty_foe_team_position_attacker_x, self.prepare_penalty_foe_team_position_attacker_y),
            (self.prepare_penalty_foe_team_position_defensor_x, self.prepare_penalty_foe_team_position_defensor_y),
            (self.prepare_penalty_foe_team_position_goalkeeper_x, self.prepare_penalty_foe_team_position_goalkeeper_y)
        )
    
    def get_prepare_kickoff_team_positions(self):
        return self.get_positions(
            self.get_prepare_kickoff_attacker_position(),
            self.get_prepare_kickoff_defensor_position(),
            self.get_prepare_kickoff_goalkeeper_position()
        )
    
    def get_prepare_kickoff_foe_team_positions(self):
        return self.get_positions(
            self.get_foe_team_prepare_kickoff_attacker_position(),
            self.get_foe_team_prepare_kickoff_defensor_position(),
            self.get_foe_team_prepare_kickoff_goalkeeper_position()
        )
    def get_normal_start_after_penalty_attacker_position(self):
        return (self.normal_start_after_penalty_target_attacker_position_x, self.normal_start_after_penalty_target_attacker_position_y)

    def get_object():
        if Configuration._instance is None:
            Configuration._instance = Configuration()

            data = Configuration.load_json()

            instance = Configuration._instance

            instance.mode = data["mode"]
            instance.environment_mode = data["environment-mode"]
            
            instance.receive_data_from_remote = data["receive-data-from-remote"]
            instance.keys_private_key_path = data["keys"]["private-key"]["path"]
            instance.keys_public_key_path = data["keys"]["public-key"]["path"]
            
            instance.remote_computer_bluetooth_address = data["remote-computer"]["bluetooth"]["address"]
            instance.remote_computer_bluetooth_channel = data["remote-computer"]["bluetooth"]["channel"]
            instance.remote_computer_bluetooth_game_controller_channel = data["remote-computer"]["bluetooth"]["game-controller"]["channel"]
            instance.remote_computer_game_controller_receiver_buffer_size = data["remote-computer"]["game-controller"]["receiver"]["buffer"]["size"]
            
            instance.sslvision_team_robot_id_mapping_0 = data["sslvision"]["team"]["robot-id-mapping"]["0"]
            instance.sslvision_team_robot_id_mapping_1 = data["sslvision"]["team"]["robot-id-mapping"]["1"]
            instance.sslvision_team_robot_id_mapping_2 = data["sslvision"]["team"]["robot-id-mapping"]["2"]
            instance.sslvision_ip = data["sslvision"]["ip"]
            instance.sslvision_port = data["sslvision"]["port"]
            instance.sslvision_receiver_buffer_size = data["sslvision"]["receiver"]["buffer"]["size"]
            instance.sslvision_foe_team_desired_goalkeeper_mapped_id = data["sslvision"]["foe-team"]["desired-goalkeeper-mapped-id"]
            
            instance.grsim_control_ip = data["grsim"]["control"]["ip"]
            instance.grsim_control_port = data["grsim"]["control"]["port"]
            instance.grsim_vision_ip = data["grsim"]["vision"]["ip"]
            instance.grsim_vision_port = data["grsim"]["vision"]["port"]
            
            instance.firasim_control_ip = data["firasim"]["control"]["ip"]
            instance.firasim_control_port = data["firasim"]["control"]["port"]
            instance.firasim_vision_ip = data["firasim"]["vision"]["ip"]
            instance.firasim_vision_port = data["firasim"]["vision"]["port"]
            
            instance.game_controller_address = data["game-controller"]["address"]
            instance.game_controller_port = data["game-controller"]["port"]
            instance.game_controller_register_as_team = data["game-controller"]["register-as-team"]
            instance.game_controller_foe_team_goalkeeper_desired_id = data["game-controller"]["foe-team"]["goalkeeper"]["desired-id"]
            
            instance.referee_address = data["referee"]["address"]
            instance.referee_port = data["referee"]["port"]
            instance.referee_receiver_buffer_size = data["referee"]["receiver"]["buffer"]["size"]
            
            instance.motion_pid_constants_kp = data["motion"]["pid"]["constants"]["kp"]
            instance.motion_pid_constants_ki = data["motion"]["pid"]["constants"]["ki"]
            instance.motion_pid_constants_kd = data["motion"]["pid"]["constants"]["kd"]
            
            instance.rsoccer_training_time_step = data["rsoccer"]["training"]["time-step"]
            instance.rsoccer_training_episode_duration = data["rsoccer"]["training"]["episode-duration"]
            
            instance.field_length = data["field"]["length"]
            instance.field_width = data["field"]["width"]
            instance.field_goal_width = data["field"]["goal"]["width"]
            instance.field_goal_depth = data["field"]["goal"]["depth"]
            instance.field_goalkeeper_area_length = data["field"]["goalkeeper-area"]["length"]
            instance.field_goalkeeper_area_width = data["field"]["goalkeeper-area"]["width"]
            instance.field_goalkeeper_area_radius = data["field"]["goalkeeper-area"]["radius"]
            instance.field_ball_radius = data["field"]["ball"]["radius"]
            
            instance.robot_wheel_radius = data["robot"]["wheel"]["radius"]
            instance.robot_speed_max_radians_seconds = data["robot"]["speed"]["max-radians-seconds"]
            instance.robot_width = data["robot"]["width"]
            instance.robot_length = data["robot"]["length"]
            
            instance.team_name = data["team"]["name"]
            instance.team_is_yellow_left_team = data["team"]["is-yellow-left-team"]
            instance.team_is_yellow_team = data["team"]["is-yellow-team"]
            instance.team_blue_number_robots = data["team"]["blue"]["number-robots"]
            instance.team_yellow_number_robots = data["team"]["yellow"]["number-robots"]
            instance.team_roles_attacker_id = data["team"]["roles"]["attacker"]["id"]
            instance.team_roles_defensor_id = data["team"]["roles"]["defensor"]["id"]
            instance.team_roles_goalkeeper_id = data["team"]["roles"]["goalkeeper"]["id"]
            
            instance.stop_distance_to_ball = data["stop"]["distance-to-ball"]
            
            instance.prepare_kickoff_team_position_attacker_y = data["prepare-kickoff"]["team"]["position"]["attacker"]["y"]
            instance.prepare_kickoff_team_position_attacker_x = data["prepare-kickoff"]["team"]["position"]["attacker"]["x"]
            instance.prepare_kickoff_team_position_defensor_y = data["prepare-kickoff"]["team"]["position"]["defensor"]["y"]
            instance.prepare_kickoff_team_position_defensor_x = data["prepare-kickoff"]["team"]["position"]["defensor"]["x"]
            instance.prepare_kickoff_team_position_goalkeeper_y = data["prepare-kickoff"]["team"]["position"]["goalkeeper"]["y"]
            instance.prepare_kickoff_team_position_goalkeeper_x = data["prepare-kickoff"]["team"]["position"]["goalkeeper"]["x"]
            instance.prepare_kickoff_foe_team_position_attacker_y = data["prepare-kickoff"]["foe-team"]["position"]["attacker"]["y"]
            instance.prepare_kickoff_foe_team_position_attacker_x = data["prepare-kickoff"]["foe-team"]["position"]["attacker"]["x"]
            instance.prepare_kickoff_foe_team_position_defensor_y = data["prepare-kickoff"]["foe-team"]["position"]["defensor"]["y"]
            instance.prepare_kickoff_foe_team_position_defensor_x = data["prepare-kickoff"]["foe-team"]["position"]["defensor"]["x"]
            instance.prepare_kickoff_foe_team_position_goalkeeper_y = data["prepare-kickoff"]["foe-team"]["position"]["goalkeeper"]["y"]
            instance.prepare_kickoff_foe_team_position_goalkeeper_x = data["prepare-kickoff"]["foe-team"]["position"]["goalkeeper"]["x"]
            
            instance.prepare_penalty_team_position_attacker_x = data["prepare-penalty"]["team"]["position"]["attacker"]["x"]
            instance.prepare_penalty_team_position_attacker_y = data["prepare-penalty"]["team"]["position"]["attacker"]["y"]
            instance.prepare_penalty_team_position_defensor_x = data["prepare-penalty"]["team"]["position"]["defensor"]["x"]
            instance.prepare_penalty_team_position_defensor_y = data["prepare-penalty"]["team"]["position"]["defensor"]["y"]
            instance.prepare_penalty_team_position_goalkeeper_x = data["prepare-penalty"]["team"]["position"]["goalkeeper"]["x"]
            instance.prepare_penalty_team_position_goalkeeper_y = data["prepare-penalty"]["team"]["position"]["goalkeeper"]["y"]
            instance.prepare_penalty_foe_team_position_attacker_x = data["prepare-penalty"]["foe-team"]["position"]["attacker"]["x"]
            instance.prepare_penalty_foe_team_position_attacker_y = data["prepare-penalty"]["foe-team"]["position"]["attacker"]["y"]
            instance.prepare_penalty_foe_team_position_defensor_x = data["prepare-penalty"]["foe-team"]["position"]["defensor"]["x"]
            instance.prepare_penalty_foe_team_position_defensor_y = data["prepare-penalty"]["foe-team"]["position"]["defensor"]["y"]
            instance.prepare_penalty_foe_team_position_goalkeeper_x = data["prepare-penalty"]["foe-team"]["position"]["goalkeeper"]["x"]
            instance.prepare_penalty_foe_team_position_goalkeeper_y = data["prepare-penalty"]["foe-team"]["position"]["goalkeeper"]["y"]
            
            instance.normal_start_after_penalty_target_attacker_position_x=data["normal-start"]["after-penalty"]["target-attacker-position"]["x"]
            instance.normal_start_after_penalty_target_attacker_position_y=data["normal-start"]["after-penalty"]["target-attacker-position"]["y"]

            instance.strategy_defensor_defense_line_x = data["strategy"]["defensor"]["defense-line"]["x"]
            instance.strategy_attacker_spin_radius_to_spin = data["strategy"]["attacker"]["spin"]["radius-to-spin"]

            instance.time_to_run = data["time"]["to-run"]

        return Configuration._instance
    
    @staticmethod
    def load_json():
        launch_data = Configuration.load_launch_json()

        if launch_data["mode"].lower() == "playing":
            with open(CONFIGURATION_PLAYING_FILE_PATH, 'r') as f:
                return json.load(f)
        elif launch_data["mode"].lower() == "simulated":
            with open(CONFIGURATION_SIMULATED_FILE_PATH, 'r') as f:
                return json.load(f)
        elif launch_data["mode"].lower() == "remotesender":
            with open(CONFIGURATION_REMOTE_SENDER_FILE_PATH, 'r') as f:
                return json.load(f)
        else:
            with open(CONFIGURATION_FILE_PATH, 'r') as f:
                return json.load(f)
        
    @staticmethod
    def load_launch_json():
        with open(LAUNCH_FILE_PATH, 'r') as f:
            return json.load(f)