import random
import numpy as np

from rsoccer_gym.Entities import Frame, Robot, Ball
from rsoccer_gym.Utils import KDTree
from stable_baselines3 import PPO

from lib.domain.ball_curriculum_behavior import BallCurriculumBehavior
from lib.domain.curriculum_task import CurriculumTask
from lib.domain.robot_curriculum_behavior import RobotCurriculumBehavior
from lib.enums.position_enum import PositionEnum
from lib.enums.robot_curriculum_behavior_enum import RobotCurriculumBehaviorEnum
from lib.environment.base_environment import BaseEnvironment
from lib.motion.motion_utils import MotionUtils

from lib.utils.configuration_utils import ConfigurationUtils
from lib.utils.rsoccer_utils import RSoccerUtils

TRAINING_EPISODE_DURATION = ConfigurationUtils.get_rsoccer_training_episode_duration()

NUMBER_ROBOTS_BLUE = ConfigurationUtils.get_rsoccer_team_blue_number_robots()
NUMBER_ROBOTS_YELLOW = ConfigurationUtils.get_rsoccer_team_yellow_number_robots()

V_WHEEL_DEADZONE = ConfigurationUtils.get_rsoccer_robot_speed_dead_zone_meters_seconds()

TIME_STEP = ConfigurationUtils.get_rsoccer_training_time_step_seconds()

# addapt this for your robot
MAX_MOTOR_SPEED = ConfigurationUtils.get_firasim_robot_speed_max_radians_seconds()

class BaseCurriculumEnvironment(BaseEnvironment):
    def __init__(
        self,
        task: CurriculumTask,
        render_mode="rgb_array",
        robot_id=0
    ):
        super().__init__(
            field_type=0,
            n_robots_blue=NUMBER_ROBOTS_BLUE,
            n_robots_yellow=NUMBER_ROBOTS_YELLOW,
            time_step=TIME_STEP,
            robot_id=robot_id,
            render_mode=render_mode)

        self.max_motor_speed = MAX_MOTOR_SPEED

        self.training_episode_duration = TRAINING_EPISODE_DURATION
        self.v_wheel_deadzone = V_WHEEL_DEADZONE

        self.task = task

        self.previous_ball_potential = None
        self.last_game_score = None
        self.error = 0

        self.opponent_model = None
        self.opponent_model_path = None
    
    def _has_episode_time_exceeded(self):
        elapsed_time = int(self.steps * self.time_step)

        if elapsed_time == 0:
            return False

        return elapsed_time % self.training_episode_duration == 0
    
    def _go_to_point_v_wheels(
        self,
        robot_id: int,
        is_yellow_team: bool,
        point: 'tuple[float, float]'
    ):
        if is_yellow_team:
            robot = RSoccerUtils.to_robot(self.frame.robots_yellow[robot_id])
        else:
            robot = RSoccerUtils.to_robot(self.frame.robots_blue[robot_id])

        velocities = MotionUtils.go_to_point(robot, point, self.error, self.max_motor_speed)

        #TODO: verificar se salvo o erro para cada robô
        (left_speed, right_speed, self.error) = velocities

        return left_speed, right_speed

    def _create_robot_by_behavior(self, behavior: RobotCurriculumBehavior):
        robot = self._get_robot_by_id(behavior.robot_id, behavior.is_yellow)

        robot_curriculum_behavior_enum = behavior.robot_curriculum_behavior_enum
        robot_id = robot.id
        is_yellow = behavior.is_yellow

        ball = self.get_ball()

        def create_robot(v_wheel_0, v_wheel_1):
            return self._create_robot(
                robot_id,
                is_yellow,
                v_wheel_0,
                v_wheel_1)

        if robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.BALL_FOLLOWING:
            left_speed, right_speed = self._go_to_point_v_wheels(
                robot_id,
                is_yellow,
                (ball.x, ball.y))
            
            velocity_alpha = behavior.get_velocity_alpha()

            return create_robot(left_speed * velocity_alpha, right_speed * velocity_alpha)
        elif robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.GOALKEEPER_BALL_FOLLOWING:
            if self._is_inside_own_goal_area((ball.x, ball.y), is_yellow):
                position = (ball.x, ball.y)
            else:
                max_y = self.get_penalty_width() / 2
                position = robot.x, np.clip(ball.y, -max_y, max_y)

            left_speed, right_speed = self._go_to_point_v_wheels(
                robot_id,
                is_yellow,
                position)
            
            velocity_alpha = behavior.get_velocity_alpha()
            
            return create_robot(left_speed * velocity_alpha, right_speed * velocity_alpha)
        elif robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.FROM_MODEL:
            actions = self._get_opponent_actions(behavior)

            left_speed, right_speed = self._actions_to_v_wheels(actions)
            velocity_alpha = behavior.get_velocity_alpha()

            return create_robot(left_speed * velocity_alpha, right_speed * velocity_alpha)
        
        return create_robot(0, 0)
    
    def _get_opponent_actions(self, behavior: RobotCurriculumBehavior):
        model = self._get_model(behavior.model_path)

        if model is None:
            return (0, 0)
        
        return model.predict(self._frame_to_opponent_observations(behavior.robot_id))[0]
    
    def _frame_to_opponent_observations(self, robot_id: int):
        raise NotImplementedError
    
    def _get_velocity_factor(self):
        rsoccer_max_motor_speed = (self.max_v / self.field.rbt_wheel_radius)
        return self.max_motor_speed / rsoccer_max_motor_speed

    def _actions_to_v_wheels(
        self,
        actions: np.ndarray
    ):
        left_wheel_speed = actions[0] * self.max_v
        right_wheel_speed = actions[1] * self.max_v

        left_wheel_speed, right_wheel_speed = np.clip(
            (left_wheel_speed, right_wheel_speed),
            -self.max_v,
            self.max_v)
        
        factor = self._get_velocity_factor()

        left_wheel_speed *= factor
        right_wheel_speed *= factor

        if abs(left_wheel_speed) < self.v_wheel_deadzone:
            left_wheel_speed = 0

        if abs(right_wheel_speed) < self.v_wheel_deadzone:
            right_wheel_speed = 0

        left_wheel_speed /= self.field.rbt_wheel_radius
        right_wheel_speed /= self.field.rbt_wheel_radius

        return left_wheel_speed, right_wheel_speed
        
    def _get_commands(self, actions):
        commands = []

        v_wheel0, v_wheel1 = self._actions_to_v_wheels(actions)

        robot = self._create_robot(
            self.robot_id,
            False,
            v_wheel0,
            v_wheel1)
        
        commands.append(robot)

        for i in range(self.n_robots_blue):
            if i == self.robot_id: continue

            behavior = self.task.get_blue_behaviors_by_robot_id(i)

            if behavior is None:
                commands.append(self._create_robot(i, False, 0, 0))
            else:
                commands.append(self._create_robot_by_behavior(behavior))

        for i in range(self.n_robots_yellow):
            behavior = self.task.get_yellow_behaviors_by_robot_id(i)

            if behavior is None:
                commands.append(self._create_robot(i, True, 0, 0))
            else:
                commands.append(self._create_robot_by_behavior(behavior))

        return commands
    
    def get_position_function_by_behavior(
        self,
        behavior: 'RobotCurriculumBehavior | BallCurriculumBehavior',
        relative_position: 'tuple[float, float] | None' = None
    ):
        position_enum = behavior.position_enum

        if isinstance(behavior, RobotCurriculumBehavior):
            is_yellow = behavior.is_yellow
        else:
            is_yellow = False

        if position_enum == PositionEnum.OWN_AREA:
            if is_yellow:
                return self._get_random_position_inside_opponent_area
            else:
                return self._get_random_position_inside_own_area
        elif position_enum == PositionEnum.GOAL_AREA:
            if is_yellow:
                return self._get_random_position_inside_opponent_penalty_area
            else:
                return self._get_random_position_inside_own_penalty_area
        elif position_enum == PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA:#mudar
            if is_yellow:
                return self._get_random_position_inside_opponent_area
            else:
                return self._get_random_position_inside_own_area
        elif position_enum == PositionEnum.OPPONENT_AREA:
            if not is_yellow:
                return self._get_random_position_inside_opponent_area
            else:
                return self._get_random_position_inside_own_area
        elif position_enum == PositionEnum.OPPONENT_GOAL_AREA:
            if not is_yellow:
                return self._get_random_position_inside_opponent_area
            else:
                return self._get_random_position_inside_own_area
        elif position_enum == PositionEnum.OPPONENT_AREA_EXCEPT_GOAL_AREA:#mudar
            if not is_yellow:
                return self._get_random_position_inside_opponent_area
            else:
                return self._get_random_position_inside_own_area
        elif position_enum == PositionEnum.RELATIVE_TO_BALL:
            return lambda: self._get_random_position_at_distance(behavior.distance, relative_position)
        elif position_enum == PositionEnum.RELATIVE_TO_OWN_GOAL:
            if not is_yellow:
                return lambda: self._get_random_position_at_distance(
                    behavior.distance,
                    self._get_own_goal_position())
            else:
                return lambda: self._get_random_position_at_distance(
                    behavior.distance,
                    self._get_opponent_goal_position())
        elif position_enum == PositionEnum.RELATIVE_TO_OPPONENT_GOAL:
            if is_yellow:
                return lambda: self._get_random_position_at_distance(
                    behavior.distance,
                    self._get_own_goal_position())
            else:
                return lambda: self._get_random_position_at_distance(
                    behavior.distance,
                    self._get_opponent_goal_position())
        elif position_enum == PositionEnum.FIELD:
            return self._get_random_position_inside_field
        
        return self._get_random_position_inside_own_area

    def _get_initial_positions_frame(self):
        frame: Frame = Frame()
        places = KDTree()
        min_distance = 0.15

        def theta(): return random.uniform(0, 360)

        def get_position(position_funcion):
            return BaseCurriculumEnvironment.get_position(places, min_distance, position_funcion)
        
        ball_position = self.get_position_function_by_behavior(self.task.ball_behavior)()

        places.insert(ball_position)

        frame.ball = Ball(x=ball_position[0], y=ball_position[1])

        for i in range(self.n_robots_blue):
            behavior = self.task.get_blue_behaviors_by_robot_id(i)

            if behavior is None:
                position = (10 + i, 10 + i)
            else:
                position_function = self.get_position_function_by_behavior(behavior, ball_position)
                position = get_position(position_function)
            
            frame.robots_blue[i] = Robot(
                x=position[0],
                y=position[1],
                theta=theta())

        for i in range(self.n_robots_yellow):
            behavior = self.task.get_yellow_behaviors_by_robot_id(i)

            if behavior is None:
                position = (20 + i, 20 + i)
            else:
                position_function = self.get_position_function_by_behavior(behavior, ball_position)
                position = get_position(position_function)

            frame.robots_yellow[i] = Robot(
                x=position[0],
                y=position[1],
                theta=theta())

        return frame
    
    def set_task(self, task: CurriculumTask):
        self.task = task

    def _get_model(self, model_path: str):
        if self.opponent_model_path != model_path:
            self.opponent_model = PPO.load(model_path)
            self.opponent_model_path = model_path

        return self.opponent_model