import numpy as np
from src.fetch_nav_init.fetch_nav_init import FetchNavInit


class MoveFetch:
    def __init__(self, fetch_nav_init: FetchNavInit):
        self._wheel_radius = fetch_nav_init.wheel_radius
        self._wheel_base = fetch_nav_init.wheel_base
        self._base_prim_path = fetch_nav_init.base_prim_path
        self._base_name = fetch_nav_init.base_name
        self._fetchbot = fetch_nav_init.fetchbot

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

    @property
    def goal_position(self):
        return self._goal_position

    @goal_position.setter
    def goal_position(self, value):
        if isinstance(value, np.ndarray) and value.shape == (2,):
            self._goal_position = value
        else:
            raise ValueError("Goal position must be a numpy array of shape (2,)")

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

    @property
    def fetch_position(self):
        return self._fetch_position
