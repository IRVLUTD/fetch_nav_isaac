import numpy as np
from configparser import ConfigParser
from src.fetch_nav_init.fetch_nav_init import FetchNavInit


class MoveFetch:
    def __init__(self, fetch_nav_init: FetchNavInit, config_path="./config/config.ini"):
        self._fetch_nav_init = fetch_nav_init
        self._wheel_radius = fetch_nav_init.wheel_radius
        self._wheel_base = fetch_nav_init.wheel_base
        self._base_prim_path = fetch_nav_init.base_prim_path
        self._base_name = fetch_nav_init.base_name
        self._fetchbot = fetch_nav_init.fetchbot

        self._config = ConfigParser()
        self._config.read([config_path])
        self._position_tolerance = float(self._config.get("MOVE_FETCH", "position_tolerance"))

        from omni.isaac.wheeled_robots.controllers import WheelBasePoseController
        from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
        from omni.isaac.core.articulations import Articulation

        self._wbp_controller = WheelBasePoseController(name="wbp_controller",
                                                       open_loop_wheel_controller=DifferentialController(
                                                           name="diff_controller",
                                                           wheel_radius=self._wheel_radius,
                                                           wheel_base=self._wheel_base),
                                                       is_holonomic=False)

        self._base_prim = Articulation(prim_path=self._base_prim_path, name=self._base_name)
        self._goal_position = np.array([0, 0])
        self._fetch_position = np.array([0, 0, 0])
        self._controller_tolerance = float(self._config.get("MOVE_FETCH", "position_tolerance"))
        self._x_max_correction = float(self._config.get("MOVE_FETCH", "x_max_correction"))
        self._y_max_correction = float(self._config.get("MOVE_FETCH", "y_max_correction"))

    @property
    def goal_position(self):
        return self._goal_position

    @goal_position.setter
    def goal_position(self, value):
        if isinstance(value, np.ndarray) and value.shape == (2,):
            self._goal_position = value
        else:
            raise ValueError("Goal position must be a numpy array of shape (2,)")

    @property
    def position_tolerance(self):
        return self._position_tolerance

    def execute_move_base(self, original_goal_position):
        fetch_current_position = self.fetch_position[:2]

        while not np.allclose(fetch_current_position, original_goal_position, atol=self._controller_tolerance):
            self._fetch_nav_init.simulation_context.step(render=True)
            fetch_current_position = self.fetch_position[:2]

        print(f"Reached ({fetch_current_position[0]}, {fetch_current_position[1]})...")

    def diff_move_fetch_callback(self, step_size=0.01):
        self.diff_move_fetch(goal_position=self._goal_position)

    def diff_move_fetch(self, goal_position: np.array):
        self._fetch_position, orientation = self._base_prim.get_world_pose()
        control_output = self._wbp_controller.forward(start_position=self._fetch_position,
                                                      start_orientation=orientation,
                                                      goal_position=goal_position)
        wheel_velocities = control_output.joint_velocities
        control_output.joint_velocities = np.array([0, 0, 0,  # shoulder, elbow, shoulder
                                                    0, 0, 0,  # elbow, shoulder, wrist
                                                    0, 0, 0,  # ,,head pan
                                                    0, wheel_velocities[0], wheel_velocities[1],
                                                    # end gripper, last two wheels
                                                    0, 0, 0])
        self._fetchbot.apply_action(control_output)

    def correct_coordinates(self, x, y, current_x, current_y):
        x_correct = x
        y_correct = y

        if abs(x - current_x) > self._controller_tolerance:
            slope = (y - current_y) / (x - current_x)
            x_correction = self._x_max_correction * np.cos(np.arctan(slope))
            x_correct = x - abs(x_correction) if (x - current_x) < 0 else x + abs(x_correction)
            if abs(y - current_y) > self._controller_tolerance:
                y_correction = self._y_max_correction * np.sin(np.arctan(slope))
                y_correct = y + abs(y_correction) if ((y - current_y) > 0) else y - abs(y_correction)
        elif abs(y - current_y) > self._controller_tolerance:
            y_correct = y - self._y_max_correction if (y - current_y) < 0 else y + self._y_max_correction

        return x_correct, y_correct

    @property
    def fetch_position(self):
        return self._fetch_position
