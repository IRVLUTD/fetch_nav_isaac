import numpy as np


class MoveFetch:
    def __init__(self, fetchbot, wheel_radius, wheel_base, base_prim_path, base_name):
        from omni.isaac.wheeled_robots.controllers import WheelBasePoseController
        from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
        from omni.isaac.core.articulations import Articulation

        self._wbp_controller = WheelBasePoseController(name="wbp_controller",
                                                      open_loop_wheel_controller=DifferentialController(
                                                          name="diff_controller",
                                                          wheel_radius=wheel_radius, wheel_base=wheel_base),
                                                      is_holonomic=False)

        self._base_prim = Articulation(prim_path=base_prim_path, name=base_name)
        self._fetchbot = fetchbot

    def diff_move_fetch(self):
        position, orientation = self._base_prim.get_world_pose()
        control_output = self._wbp_controller.forward(start_position=position, start_orientation=orientation,
                                                     goal_position=np.array([-2, -1]))
        wheel_velocities = control_output.joint_velocities
        control_output.joint_velocities = np.array([0, 0, 0,  # shoulder, elbow, shoulder
                                                    0, 0, 0,  # elbow, shoulder, wrist
                                                    0, 0, 0,  # ,,head pan
                                                    0, wheel_velocities[0], wheel_velocities[1],
                                                    # end gripper, last two wheels
                                                    0, 0, 0])
        self._fetchbot.apply_action(control_output)