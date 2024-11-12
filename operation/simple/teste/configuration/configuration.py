import json

CONFIGURATION_FILE_PATH = "./configuration/configuration.json"

class Configuration:
    _instance = None

    def __init__(self):
        self.mode = None
        self.environment_mode = None
        self.keys_private_key_path = None
        self.keys_public_key_path = None
        self.sslvision_team_robot_id_mapping_0 = None
        self.sslvision_team_robot_id_mapping_1 = None
        self.sslvision_team_robot_id_mapping_2 = None
        self.sslvision_foe_team_robot_id_mapping_0 = None
        self.sslvision_foe_team_robot_id_mapping_1 = None
        self.sslvision_foe_team_robot_id_mapping_2 = None
        self.sslvision_ip = None
        self.sslvision_port = None
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
        self.referee_address = None
        self.referee_port = None
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
        self.kickoff_position_left_team_atacker_position_y = None
        self.kickoff_position_left_team_atacker_position_x = None
        self.kickoff_position_left_team_defensor_position_y = None
        self.kickoff_position_left_team_defensor_position_x = None
        self.kickoff_position_left_team_goalkeeper_position_y = None
        self.kickoff_position_left_team_goalkeeper_position_x = None

    def get_object():
        if Configuration._instance is None:
            Configuration._instance = Configuration()

            data = Configuration.load_json()

            instance = Configuration._instance

            instance.mode = data["mode"]
            instance.environment_mode = data["environment-mode"]
            instance.keys_private_key_path = data["keys"]["private-key"]["path"]
            instance.keys_public_key_path = data["keys"]["public-key"]["path"]
            instance.sslvision_team_robot_id_mapping_0 = data["sslvision"]["team"]["robot-id-mapping"]["0"]
            instance.sslvision_team_robot_id_mapping_1 = data["sslvision"]["team"]["robot-id-mapping"]["1"]
            instance.sslvision_team_robot_id_mapping_2 = data["sslvision"]["team"]["robot-id-mapping"]["2"]
            instance.sslvision_foe_team_robot_id_mapping_0 = data["sslvision"]["foe-team"]["robot-id-mapping"]["0"]
            instance.sslvision_foe_team_robot_id_mapping_1 = data["sslvision"]["foe-team"]["robot-id-mapping"]["1"]
            instance.sslvision_foe_team_robot_id_mapping_2 = data["sslvision"]["foe-team"]["robot-id-mapping"]["2"]
            instance.sslvision_ip = data["sslvision"]["ip"]
            instance.sslvision_port = data["sslvision"]["port"]
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
            instance.referee_address = data["referee"]["address"]
            instance.referee_port = data["referee"]["port"]
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
            instance.kickoff_position_left_team_atacker_position_y = data["kickoff"]["position"]["left-team"]["atacker-position"]["y"]
            instance.kickoff_position_left_team_atacker_position_x = data["kickoff"]["position"]["left-team"]["atacker-position"]["x"]
            instance.kickoff_position_left_team_defensor_position_y = data["kickoff"]["position"]["left-team"]["defensor-position"]["y"]
            instance.kickoff_position_left_team_defensor_position_x = data["kickoff"]["position"]["left-team"]["defensor-position"]["x"]
            instance.kickoff_position_left_team_goalkeeper_position_y = data["kickoff"]["position"]["left-team"]["goalkeeper-position"]["y"]
            instance.kickoff_position_left_team_goalkeeper_position_x = data["kickoff"]["position"]["left-team"]["goalkeeper-position"]["x"]

        return Configuration._instance
    
    @staticmethod
    def load_json():
        with open(CONFIGURATION_FILE_PATH, 'r') as f:
            return json.load(f)