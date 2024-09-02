import json
import os

class ConfigurationUtils:
    __configuration = None

    @staticmethod
    def _get_configuration():
        if ConfigurationUtils.__configuration is not None:
            return ConfigurationUtils.__configuration
        
        configurationFilePath = os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "configuration",
            "configuration.json")
        
        with open(configurationFilePath, 'r') as f:
            configuration = json.loads(f.read())

        ConfigurationUtils.__configuration = configuration

        return configuration
    
    @staticmethod
    def _get_rsoccer_configuration():
        configuration = ConfigurationUtils._get_configuration()
        return configuration["rsoccer"]
    
    @staticmethod
    def _get_firasim_configuration():
        configuration = ConfigurationUtils._get_configuration()
        return configuration["firasim"]
    
    @staticmethod
    def _get_motion_configuration():
        configuration = ConfigurationUtils._get_configuration()
        return configuration["motion"]
    
    @staticmethod
    def get_motion_pid_constants():
        configuration = ConfigurationUtils._get_motion_configuration()
        return configuration["pid"]["constants"]
    
    @staticmethod
    def get_motion_pid_constants_kp():
        configuration = ConfigurationUtils.get_motion_pid_constants()
        return configuration["kp"]
    
    @staticmethod
    def get_motion_pid_constants_ki():
        configuration = ConfigurationUtils.get_motion_pid_constants()
        return configuration["ki"]
    
    @staticmethod
    def get_motion_pid_constants_kd():
        configuration = ConfigurationUtils.get_motion_pid_constants()
        return configuration["kd"]
    
    @staticmethod
    def get_motion_collision_avoidance_min_distance():
        configuration = ConfigurationUtils._get_motion_configuration()
        return configuration["collision-avoidance"]["min-distance"]
    
    @staticmethod
    def get_rsoccer_robot_wheel_radius():
        configuration = ConfigurationUtils._get_rsoccer_configuration()
        return configuration["robot"]["wheel"]["radius"]
    
    @staticmethod
    def get_rsoccer_robot_motor_max_rpm():
        configuration = ConfigurationUtils._get_rsoccer_configuration()
        return configuration["robot"]["motor"]["max-rpm"]
    
    @staticmethod
    def get_rsoccer_robot_speed_max_radians_seconds():
        configuration = ConfigurationUtils._get_rsoccer_configuration()
        return configuration["robot"]["speed"]["max-radians-seconds"]
    
    @staticmethod
    def get_rsoccer_robot_speed_dead_zone_meters_seconds():
        configuration = ConfigurationUtils._get_rsoccer_configuration()
        return configuration["robot"]["speed"]["dead-zone-meters-seconds"]
    
    @staticmethod
    def get_rsoccer_robot_width():
        configuration = ConfigurationUtils._get_rsoccer_configuration()
        return configuration["robot"]["width"]
    
    @staticmethod
    def get_rsoccer_robot_length():
        configuration = ConfigurationUtils._get_rsoccer_configuration()
        return configuration["robot"]["length"]
    
    @staticmethod
    def get_rsoccer_team_is_yellow_left_team():
        configuration = ConfigurationUtils._get_rsoccer_configuration()
        return configuration["team"]["is-yellow-left-team"]
    
    @staticmethod
    def get_rsoccer_team_is_yellow_team():
        configuration = ConfigurationUtils._get_rsoccer_configuration()
        return configuration["team"]["is-yellow-team"]
    
    @staticmethod
    def get_rsoccer_team_blue_number_robots():
        configuration = ConfigurationUtils._get_rsoccer_configuration()
        return configuration["team"]["blue"]["number-robots"]
    
    @staticmethod
    def get_rsoccer_team_yellow_number_robots():
        configuration = ConfigurationUtils._get_rsoccer_configuration()
        return configuration["team"]["yellow"]["number-robots"]
    
    @staticmethod
    def get_rsoccer_is_left_team():
        return ConfigurationUtils.get_rsoccer_team_is_yellow_left_team() \
            == ConfigurationUtils.get_rsoccer_team_is_yellow_team()
    
    @staticmethod
    def get_rsoccer_training_time_step_seconds():
        configuration = ConfigurationUtils._get_rsoccer_configuration()
        return configuration["training"]["time-step"]
    
    @staticmethod
    def get_rsoccer_training_episode_duration():
        configuration = ConfigurationUtils._get_rsoccer_configuration()
        return configuration["training"]["episode-duration"]
    
    @staticmethod
    def get_firasim_control_ip():
        configuration = ConfigurationUtils._get_firasim_configuration()
        return configuration["control"]["ip"]
    
    @staticmethod
    def get_firasim_control_port():
        configuration = ConfigurationUtils._get_firasim_configuration()
        return configuration["control"]["port"]
    
    @staticmethod
    def get_firasim_vision_ip():
        configuration = ConfigurationUtils._get_firasim_configuration()
        return configuration["vision"]["ip"]
    
    @staticmethod
    def get_firasim_vision_port():
        configuration = ConfigurationUtils._get_firasim_configuration()
        return configuration["vision"]["port"]
    
    @staticmethod
    def get_firasim_robot_wheel_radius():
        configuration = ConfigurationUtils._get_firasim_configuration()
        return configuration["robot"]["wheel"]["radius"]
    
    @staticmethod
    def get_firasim_robot_speed_max_radians_seconds():
        configuration = ConfigurationUtils._get_firasim_configuration()
        return configuration["robot"]["speed"]["max-radians-seconds"]
    
    @staticmethod
    def get_firasim_robot_width():
        configuration = ConfigurationUtils._get_firasim_configuration()
        return configuration["robot"]["width"]
    
    @staticmethod
    def get_firasim_robot_length():
        configuration = ConfigurationUtils._get_firasim_configuration()
        return configuration["robot"]["length"]
    
    @staticmethod
    def get_firasim_team_is_yellow_left_team():
        configuration = ConfigurationUtils._get_firasim_configuration()
        return configuration["team"]["is-yellow-left-team"]
    
    @staticmethod
    def get_firasim_team_is_yellow_team():
        configuration = ConfigurationUtils._get_firasim_configuration()
        return configuration["team"]["is-yellow-team"]
    
    @staticmethod
    def get_firasim_is_left_team():
        return ConfigurationUtils.get_firasim_team_is_yellow_left_team() \
            == ConfigurationUtils.get_firasim_team_is_yellow_team()
    
    @staticmethod
    def get_firasim_team_blue_number_robots():
        configuration = ConfigurationUtils._get_firasim_configuration()
        return configuration["team"]["blue"]["number-robots"]
    
    @staticmethod
    def get_firasim_team_yellow_number_robots():
        configuration = ConfigurationUtils._get_firasim_configuration()
        return configuration["team"]["yellow"]["number-robots"]
    
    @staticmethod
    def get_field_length():
        configuration = ConfigurationUtils._get_configuration()
        return configuration["field"]["length"]
    
    @staticmethod
    def get_field_width():
        configuration = ConfigurationUtils._get_configuration()
        return configuration["field"]["width"]
    
    @staticmethod
    def get_field_goal_width():
        configuration = ConfigurationUtils._get_configuration()
        return configuration["field"]["goal"]["width"]
    
    @staticmethod
    def get_field_goal_depth():
        configuration = ConfigurationUtils._get_configuration()
        return configuration["field"]["goal"]["depth"]
    
    @staticmethod
    def get_field_penalty_length():
        configuration = ConfigurationUtils._get_configuration()
        return configuration["field"]["penalty"]["length"]
    
    @staticmethod
    def get_field_penalty_width():
        configuration = ConfigurationUtils._get_configuration()
        return configuration["field"]["penalty"]["width"]
    
    @staticmethod
    def get_field_ball_radius():
        configuration = ConfigurationUtils._get_configuration()
        return configuration["field"]["ball"]["radius"]