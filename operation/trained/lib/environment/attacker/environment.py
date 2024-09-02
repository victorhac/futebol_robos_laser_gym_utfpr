import math
import random
import time
import numpy as np

from gymnasium.spaces import Box

from rsoccer_gym.Entities import Frame, Robot, Ball
from rsoccer_gym.Utils import KDTree

from lib.domain.robot_curriculum_behavior import RobotCurriculumBehavior
from lib.enums.robot_curriculum_behavior_enum import RobotCurriculumBehaviorEnum
from lib.motion.motion_utils import MotionUtils

from ...utils.rsoccer_utils import RSoccerUtils
from ...utils.configuration_utils import ConfigurationUtils

from ...environment.base_environment import BaseEnvironment

TRAINING_EPISODE_DURATION = ConfigurationUtils.get_rsoccer_training_episode_duration()

NUMBER_ROBOTS_BLUE = ConfigurationUtils.get_rsoccer_team_blue_number_robots()
NUMBER_ROBOTS_YELLOW = ConfigurationUtils.get_rsoccer_team_yellow_number_robots()

V_WHEEL_DEADZONE = ConfigurationUtils.get_rsoccer_robot_speed_dead_zone_meters_seconds()

TIME_STEP = ConfigurationUtils.get_rsoccer_training_time_step_seconds()

# addapt this for your robot
MAX_MOTOR_SPEED = ConfigurationUtils.get_firasim_robot_speed_max_radians_seconds()

class Environment(BaseEnvironment):
    def __init__(self, render_mode="rgb_array", robot_id=1):
        super().__init__(
            field_type=0,
            n_robots_blue=NUMBER_ROBOTS_BLUE,
            n_robots_yellow=NUMBER_ROBOTS_YELLOW,
            time_step=TIME_STEP,
            robot_id=robot_id,
            render_mode=render_mode)

        self.max_motor_speed = MAX_MOTOR_SPEED

        self.action_space = Box(
            low=-1,
            high=1,
            shape=(2,),
            dtype=np.float32)

        self.observation_space = Box(
            low=-1,
            high=1,
            shape=(34,),
            dtype=np.float32)

        self.training_episode_duration = TRAINING_EPISODE_DURATION
        self.v_wheel_deadzone = V_WHEEL_DEADZONE

        self.behaviors: dict[str, list[RobotCurriculumBehavior]] = None

        self.previous_ball_potential = None
        self.episode_initial_time = 0
        self.last_game_score = 0
        self.error = 0
    
    def _is_done(self):
        if self._any_team_scored_goal():
            return True
        elif time.time() - self.episode_initial_time > TRAINING_EPISODE_DURATION:
            return True
        return False
    
    def _frame_to_observations(self):
        observation = []

        ball = self.get_ball()

        observation.extend([
            self.norm_x(ball.x),
            self.norm_y(-ball.y),
            self.norm_v(ball.v_x),
            self.norm_v(-ball.v_y)
        ])

        frame = self.frame

        for i in range(self.n_robots_blue):
            robot = frame.robots_blue[i]
            theta = -RSoccerUtils.get_corrected_angle(robot.theta) / np.pi

            observation.extend([
                self.norm_x(robot.x),
                self.norm_y(-robot.y),
                theta,
                self.norm_v(robot.v_x),
                self.norm_v(-robot.v_y)
            ])

        for i in range(self.n_robots_yellow):
            robot = frame.robots_yellow[i]
            theta = -RSoccerUtils.get_corrected_angle(robot.theta) / np.pi

            observation.extend([
                self.norm_x(robot.x),
                self.norm_y(-robot.y),
                theta,
                self.norm_v(robot.v_x),
                self.norm_v(-robot.v_y)
            ])

        return np.array(observation, dtype=np.float32)
    
    def _frame_to_opponent_observations(self):
        observation = []

        ball = self.get_ball()

        observation.extend([
            self.norm_x(-ball.x),
            self.norm_y(ball.y),
            self.norm_v(-ball.v_x),
            self.norm_v(ball.v_y)
        ])

        def get_norm_theta(robot: Robot):
            theta = -RSoccerUtils.get_corrected_angle(robot.theta)

            if theta < 0:
                theta += np.pi
            elif theta > 0:
                theta -= np.pi

            return theta / np.pi

        frame = self.frame

        for i in range(self.n_robots_yellow):
            robot = frame.robots_yellow[i]

            observation.extend([
                self.norm_x(-robot.x),
                self.norm_y(robot.y),
                get_norm_theta(robot),
                self.norm_v(-robot.v_x),
                self.norm_v(robot.v_y)
            ])

        for i in range(self.n_robots_blue):
            robot = frame.robots_blue[i]

            observation.extend([
                self.norm_x(-robot.x),
                self.norm_y(robot.y),
                get_norm_theta(robot),
                self.norm_v(-robot.v_x),
                self.norm_v(robot.v_y)
            ])

        return np.array(observation, dtype=np.float32)
    
    def _go_to_point_v_wheels(
        self,
        robot_id: int,
        is_yellow_team: bool,
        point: tuple[float, float]
    ):
        if is_yellow_team:
            robot = RSoccerUtils.to_robot(self.frame.robots_yellow[robot_id])
        else:
            robot = RSoccerUtils.to_robot(self.frame.robots_blue[robot_id])

        velocities = MotionUtils.go_to_point(robot, point, self.error, self.max_motor_speed)

        #verificar se salvo o erro para cada robô
        (left_speed, right_speed, self.error) = velocities

        return left_speed, right_speed
    

    def _create_robot_by_behavior(self, behavior: RobotCurriculumBehavior):
        robot = self._get_robot_by_id(behavior.id, behavior.is_yellow)

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

        if robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.STOPPED:
            return create_robot(0, 0)
        elif robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.BALL_FOLLOWING:
            # colocar moderação de velocidade
            left_speed, right_speed = self._go_to_point_v_wheels(
                robot_id,
                is_yellow,
                (ball.x, ball.y)
            )
            return create_robot(left_speed, right_speed)
        elif robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.FROM_MODEL:
            actions = behavior.model.predict(self._frame_to_opponent_observations())
            left_speed, right_speed = self._actions_to_v_wheels(actions, True)
            return create_robot(left_speed, right_speed)
        
        return create_robot(0, 0)
        
    def _get_commands(self, actions):
        commands = []

        v_wheel0, v_wheel1 = self._actions_to_v_wheels(actions, True)

        robot = self._create_robot(
            self.robot_id,
            False,
            v_wheel0,
            v_wheel1)
        
        commands.append(robot)

        blue_behaviors = self.behaviors["blue"]
        yellow_behaviors = self.behaviors["yellow"]

        for i in range(len(blue_behaviors)):
            if i == self.robot_id: continue
            behavior = blue_behaviors[i]
            commands.append(self._create_robot_by_behavior(behavior))

        for i in range(len(yellow_behaviors)):
            behavior = yellow_behaviors[i]
            commands.append(self._create_robot_by_behavior(behavior))

        return commands

    def _actions_to_v_wheels(
        self,
        actions: np.ndarray,
        is_own_team: bool
    ):
        left_wheel_speed = actions[0] * self.max_v
        right_wheel_speed = actions[1] * self.max_v

        left_wheel_speed, right_wheel_speed = np.clip(
            (left_wheel_speed, right_wheel_speed),
            -self.max_v,
            self.max_v)
        
        if is_own_team:
            rsoccer_max_motor_speed = (self.max_v / self.field.rbt_wheel_radius)
            factor = self.max_motor_speed / rsoccer_max_motor_speed
        else:
            factor = 1

        left_wheel_speed *= factor
        right_wheel_speed *= factor

        if abs(left_wheel_speed) < self.v_wheel_deadzone:
            left_wheel_speed = 0

        if abs(right_wheel_speed) < self.v_wheel_deadzone:
            right_wheel_speed = 0

        left_wheel_speed /= self.field.rbt_wheel_radius
        right_wheel_speed /= self.field.rbt_wheel_radius

        return left_wheel_speed, right_wheel_speed
    
    def _ball_gradient_reward(
        self,
        previous_ball_potential: float | None
    ):
        field_length = self.get_field_length()
        goal_depth = self.get_goal_depth()
        ball = self.get_ball()

        convert_m_to_cm = lambda x: x * 100

        length_cm = convert_m_to_cm(field_length)
        half_lenght = (field_length / 2) + goal_depth

        dx_d = convert_m_to_cm((half_lenght + ball.x))
        dx_a = convert_m_to_cm((half_lenght - ball.x))
        dy = convert_m_to_cm(ball.y)

        dist_1 = math.sqrt(dx_a ** 2 + 2 * dy ** 2)
        dist_2 = math.sqrt(dx_d ** 2 + 2 * dy ** 2)

        ball_potential = ((-dist_1 + dist_2) / length_cm - 1) / 2

        if previous_ball_potential is not None:
            diff = ball_potential - previous_ball_potential
            reward = np.clip(
                diff * 3 / self.time_step,
                -5.0,
                5.0)
        else:
            reward = 0
        
        return reward, ball_potential
        
    def _move_reward(self):
        ball = self.get_ball()
        robot = self._get_agent()

        ball_position = np.array([ball.x, ball.y])
        robot_position = np.array([robot.x, robot.y])
        
        robot_velocities = np.array([robot.v_x, robot.v_y])
        robot_ball_vector = ball_position - robot_position
        robot_ball_vector = robot_ball_vector / np.linalg.norm(robot_ball_vector)

        move_reward = np.dot(robot_ball_vector, robot_velocities)

        return np.clip(move_reward / 0.4, -5.0, 5.0)
    
    def _calculate_reward_and_done(self):
        reward = 0
        w_move = 0.2
        w_ball_grad = 0.8
        w_energy = 2e-4

        if self._any_team_scored_goal():
            if self._has_scored_goal():
                reward = 10
            else:
                reward = -10
        else:
            grad_ball_potential, ball_gradient = \
                self._ball_gradient_reward(self.previous_ball_potential)
            
            self.previous_ball_potential = ball_gradient

            move_reward = self._move_reward()
            energy_penalty = self._energy_penalty()

            reward = w_move * move_reward + \
                w_ball_grad * grad_ball_potential + \
                w_energy * energy_penalty
            
        is_done = self._is_done()

        if is_done:
            if self._any_team_scored_goal():
                self.last_game_score = 1 if self._has_scored_goal() else -1
            else:
                self.last_game_score = 0

        return reward, self._is_done()
    
    def _get_default_initial_positions_frame(self):
        def theta(): return random.uniform(0, 360)

        frame: Frame = Frame()

        ball_position = self._get_random_position_inside_opponent_area()

        frame.ball = Ball(x=ball_position[0], y=ball_position[1])

        min_distance = 0.15

        places = KDTree()

        places.insert(ball_position)

        frame.robots_blue[0] = Robot(x=ball_position[0] - 0.1, y=ball_position[1], theta=theta())
            
        def get_position(position_funcion):
            return Environment.get_position(places, min_distance, position_funcion)
        
        position_fns = [
            self._get_random_position_inside_own_penalty_area,
            self._get_random_position_inside_own_area
        ]

        for i in range(1, self.n_robots_blue):
            position = get_position(position_fns[i - 1])
            frame.robots_blue[i] = Robot(x=position[0], y=position[1], theta=theta())

        position_fns = [
            self._get_random_position_inside_opponent_penalty_area,
            self._get_random_position_inside_field,
            self._get_random_position_inside_opponent_area
        ]

        for i in range(self.n_robots_yellow):
            position = get_position(position_fns[i])
            frame.robots_yellow[i] = Robot(x=position[0], y=position[1], theta=theta())

        return frame

    def _get_initial_positions_frame(self):
        self.episode_initial_time = time.time()

        if self.behaviors is None:
            return self._get_default_initial_positions_frame()
        
        def theta(): return random.uniform(0, 360)

        frame: Frame = Frame()

        ball_position = self._get_random_position_inside_opponent_area()

        frame.ball = Ball(x=ball_position[0], y=ball_position[1])

        min_distance = 0.15

        places = KDTree()

        places.insert(ball_position)

        blue_behaviors = self.behaviors["blue"]
        yellow_behaviors = self.behaviors["yellow"]

        def get_position(position_funcion):
            return Environment.get_position(places, min_distance, position_funcion)

        for item in range(len(blue_behaviors)):
            behavior = blue_behaviors[item]

            position_function = self.get_position_function_by_position_enum(
                False,
                behavior.position_enum,
                ball_position)
            
            position = get_position(position_function)
            frame.robots_blue[item] = Robot(x=position[0], y=position[1], theta=theta())

        for item in range(len(yellow_behaviors)):
            behavior = yellow_behaviors[item]

            position_function = self.get_position_function_by_position_enum(
                True,
                behavior.position_enum,
                ball_position)
            
            position = get_position(position_function)
            frame.robots_yellow[item] = Robot(x=position[0], y=position[1], theta=theta())

        return frame
    
    def set_behaviors(self, behaviors: dict[str, list[RobotCurriculumBehavior]]):
        self.behaviors = behaviors