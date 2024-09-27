import math
import numpy as np

from gymnasium.spaces import Box

from rsoccer_gym.Entities import Robot

from lib.domain.curriculum_task import CurriculumTask
from lib.environment.base_curriculum_environment import BaseCurriculumEnvironment
from lib.geometry.geometry_utils import GeometryUtils

from ...utils.rsoccer_utils import RSoccerUtils

class Environment(BaseCurriculumEnvironment):
    def __init__(
        self,
        task: CurriculumTask,
        render_mode="rgb_array"
    ):
        super().__init__(
            task=task,
            render_mode=render_mode,
            robot_id=0)

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
        
        self.defensive_line_x = -.2
        self.last_robot_touched_ball = None

    def reset(
        self,
        *,
        seed=None,
        options=None
    ):
        self.last_robot_touched_ball = None
        return super().reset(seed=seed, options=options)
    
    def _is_done(self):
        if self._is_ball_inside_goal_area():
            return True
        elif not self._is_ball_inside_defensive_area() and\
                self.last_robot_touched_ball == self._get_agent():
            return True
        elif self._has_episode_time_exceeded():
            return True
        return False
    
    def _frame_to_observations(self):
        observation = []

        current_robot = self._get_robot_by_id(self.robot_id, False)
        ball = self.get_ball()

        def get_normalized_distance(distance: float):
            return distance / self._get_max_distance()
        
        def extend_observation_by_ball():
            current_robot_position = current_robot.x, -current_robot.y
            ball_position = ball.x, -ball.y

            distance = GeometryUtils.distance(current_robot_position, ball_position)
            angle = GeometryUtils.angle_between_points(current_robot_position, ball_position)

            observation.extend([
                get_normalized_distance(distance),
                angle / np.pi,
                self.norm_v(ball.v_x),
                self.norm_v(-ball.v_y)
            ])
        
        def extend_observation_by_current_robot():
            theta = -RSoccerUtils.get_corrected_angle(current_robot.theta) / np.pi

            observation.extend([
                self.norm_x(current_robot.x),
                self.norm_y(-current_robot.y),
                theta,
                self.norm_v(current_robot.v_x),
                self.norm_v(-current_robot.v_y)
            ])

        def extend_observation_by_robot(robot: Robot):
            if self._is_inside_field((robot.x, robot.y)):
                theta = -RSoccerUtils.get_corrected_angle(robot.theta) / np.pi

                current_robot_position = current_robot.x, -current_robot.y
                robot_position = robot.x, -robot.y

                distance = GeometryUtils.distance(current_robot_position, robot_position)
                angle = GeometryUtils.angle_between_points(current_robot_position, robot_position)

                observation.extend([
                    get_normalized_distance(distance),
                    angle / np.pi,
                    theta,
                    self.norm_v(robot.v_x),
                    self.norm_v(-robot.v_y)
                ])
            else:
                observation.extend([0, 0, 0, 0, 0])

        extend_observation_by_ball()
        extend_observation_by_current_robot()

        frame = self.frame

        for i in range(self.n_robots_blue):
            if i != self.robot_id:
                extend_observation_by_robot(frame.robots_blue[i])

        for i in range(self.n_robots_yellow):
            extend_observation_by_robot(frame.robots_yellow[i])

        return np.array(observation, dtype=np.float32)
    
    def _frame_to_opponent_observations(self, robot_id: int):
        observation = []

        current_robot = self._get_robot_by_id(robot_id, True)
        ball = self.get_ball()

        def get_norm_theta(robot: Robot):
            theta = -RSoccerUtils.get_corrected_angle(robot.theta)

            if theta < 0:
                theta += np.pi
            elif theta > 0:
                theta -= np.pi

            return theta / np.pi
        
        def get_normalized_distance(distance: float):
            return distance / self._get_max_distance()
        
        def extend_observation_by_ball():
            current_robot_position = -current_robot.x, current_robot.y
            ball_position = -ball.x, ball.y

            distance = GeometryUtils.distance(current_robot_position, ball_position)
            angle = GeometryUtils.angle_between_points(current_robot_position, ball_position)

            observation.extend([
                get_normalized_distance(distance),
                angle / np.pi,
                self.norm_v(-ball.v_x),
                self.norm_v(ball.v_y)
            ])

        def extend_observation_by_current_robot():
            theta = get_norm_theta(current_robot)

            observation.extend([
                self.norm_x(-current_robot.x),
                self.norm_y(current_robot.y),
                theta,
                self.norm_v(-current_robot.v_x),
                self.norm_v(current_robot.v_y)
            ])
        
        def extend_observation_by_robot(robot: Robot):
            if self._is_inside_field((robot.x, robot.y)):
                theta = get_norm_theta(robot)

                current_robot_position = -current_robot.x, current_robot.y
                robot_position = -robot.x, robot.y

                distance = GeometryUtils.distance(current_robot_position, robot_position)
                angle = GeometryUtils.angle_between_points(current_robot_position, robot_position)

                observation.extend([
                    get_normalized_distance(distance),
                    angle / np.pi,
                    theta,
                    self.norm_v(-robot.v_x),
                    self.norm_v(robot.v_y)
                ])
            else:
                observation.extend([0, 0, 0, 0, 0])

        extend_observation_by_ball()
        extend_observation_by_current_robot()

        frame = self.frame

        for i in range(self.n_robots_yellow):
            if robot_id != i:
                extend_observation_by_robot(frame.robots_yellow[i])

        for i in range(self.n_robots_blue):
            extend_observation_by_robot(frame.robots_blue[i])

        return np.array(observation, dtype=np.float32)

    def _ball_gradient_reward(
        self,
        previous_ball_potential: float
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

        if self._is_agent_inside_defensive_area() and\
                self._is_ball_inside_defensive_area():
            # _move_reward + _ball_gradient_reward com relação a (xdefesa, ybola)
            pass
        elif self._is_agent_inside_defensive_area() and\
                not self._is_ball_inside_defensive_area():
            # _move_reward com relação a ybola
            pass
        elif not self._is_agent_inside_defensive_area() and\
                self._is_ball_inside_defensive_area():
            # reward (_move_reward + _ball_gradient_reward com relação a (xdefesa, ybola)) - 3,001
            pass
        elif not self._is_agent_inside_defensive_area() and\
                not self._is_ball_inside_defensive_area():
            # _move_reward com relação a (xdefesa, ybola) - 1
            pass
            
        is_done = self._is_done()

        if is_done:
            if self._is_ball_inside_goal_area():
                # -10
                pass
            elif not self._is_ball_inside_defensive_area() and\
                    self.last_robot_touched_ball == self._get_agent():
                # +10
                pass

        return reward, is_done
    
    def _is_ball_inside_defensive_area(self):
        pass

    def _is_ball_inside_goal_area(self):
        pass

    def _is_agent_inside_defensive_area(self):
        pass

    def _get_security_position(self):
        pass

    def _get_ball_gradient_related_to_y_reward(self):
        pass

    def _get_ball_gradient_related_to_security_position_reward(self):
        pass

    def _try_set_last_robot_touched_ball(self):
        pass