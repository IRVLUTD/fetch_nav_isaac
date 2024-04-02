import sys
import time

# import omni
sys.path.append("/home/sauravdosi/.local/share/ov/pkg/isaac_sim-2023.1.1/exts/omni.isaac.synthetic_utils")
sys.path.append("/home/sauravdosi/.local/share/ov/pkg/isaac_sim-2023.1.1/exts/omni.kit.editor")

from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers import WheelBasePoseController
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
import carb
import numpy as np
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils
import matplotlib.pyplot as plt
import cv2
from omni.isaac.core.articulations import Articulation
import omni
from omni.kit.viewport.utility import get_active_viewport
from omni.isaac.synthetic_utils import SyntheticDataHelper
import asyncio
import omni.isaac.core.utils.stage as stage_utils
import threading


class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        assets_root_path = get_assets_root_path()
        print(assets_root_path)
        # self.setup_scene()
        self._test_counter = -2.2
        self._output = 0
        self._i = 1
        self._time = time.time()
        self._current_frame = None
        self._wheel_radius = 0.06
        self._wheel_base = 0.37476
        self._forward_const_vel = 3
        self._timesteps_counter = 0
        self._number_of_timesteps = 0
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        # you configure a new server with /Isaac folder in it
        assets_root_path = get_assets_root_path()
        print(assets_root_path)
        if assets_root_path is None:
            # Use carb to log warnings, errors and infos in your application (shown on terminal)
            carb.log_error("Could not find nucleus server with /Isaac folder")
        # asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        # This will create a new XFormPrim and point it to the usd file as a reference
        # Similar to how pointers work in memory
        # add_reference_to_stage(usd_path=asset_path, prim_path="/World/Fancy_Robot")
        # Wrap the jetbot prim root under a Robot class and add it to the Scene
        # to use high level api to set/ get attributes as well as initializing
        # physics handles needed..etc.
        # Note: this call doesn't create the Jetbot in the stage window, it was already
        # created with the add_reference_to_stage
        # jetbot_robot = world.scene.add(Robot(prim_path="/World/Fancy_Robot", name="fancy_robot"))
        # Note: before a reset is called, we can't access information related to an Articulation
        # because physics handles are not initialized yet. setup_post_load is called after
        # the first reset so we can do so there
        fetch_path = "/home/sauravdosi/fetch_nav/fetch_com5.usd"
        jetbot_path = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Robots/Jetbot/jetbot.usd"
        # prim_path = "/World/fetch"

        # stage_utils.add_reference_to_stage(fetch_path, prim_path)
        # fetchbot_asset_path =
        self._fetchbot = world.scene.add(
            WheeledRobot(
                prim_path="/World/fetch",
                name="fetch",
                wheel_dof_names=["l_wheel_joint", "r_wheel_joint"],
                create_robot=True,
                usd_path=fetch_path
            )
        )

        # self._camera = Camera(
        # prim_path="/World/fetch/head_tilt_link/head_camera_link/camera",
        # frequency=30,
        # resolution=(1024, 1024)
        # )

        # self._camera.initialize()

        print("Init camera")
        # self.camera_feed()
        import omni
        from omni.kit.viewport.utility import get_viewport_from_window_name, get_active_viewport
        from omni.isaac.synthetic_utils import SyntheticDataHelper

        viewport = get_active_viewport()

        # Get the camera sensor from the viewport
        # camera_sensor = viewport.get_camera_sensor()

        # # Start receiving the camera feed
        # camera_sensor.start_receiving_frame()

        # # Get the latest frame from the camera feed
        # frame = camera_sensor.get_latest_frame()

        # # Display the frame
        # viewport.display_frame(frame)
        import cv2
        # import asyncio

        # window_names = ["Viewport", "Viewport 2"]

        # sd_helper = SyntheticDataHelper()
        # viewport_apis = [get_viewport_from_window_name(window_name) for window_name in window_names]
        # viewport_windows = [omni.ui.Workspace.get_window(window_name) for window_name in window_names]

        # for viewport_window, viewport_api in zip(viewport_windows, viewport_apis):
        #     if viewport_window is not None:
        #         viewport_window.visible = True
        #         viewport_api.set_active_camera("/OmniverseKit_Persp")

        stage = omni.usd.get_context().get_stage()
        # # editor = omni.kit.editor.get_editor_interface()
        # vpi = omni.kit.viewport.get_active_viewport()
        sdh = SyntheticDataHelper()
        print(viewport.get_active_camera())
        print(sdh.get_camera_params(viewport))

        # print(sdh.get_groundtruth(['rgb', 'depth', 'boundingBox2DTight'], viewport))

        # prim = stage.GetPrimAtPath("/World/fetch/head_tilt_link/head_camera_link/camera")

        # # -------------------------------------------------------------------
        # viewport.set_active_camera(str(prim.GetPath()))
        # print(viewport.get_active_camera())
        # print(sdh.get_groundtruth(['camera'], viewport))

        f_stop = threading.Event()
        # start calling f now and every 60 sec thereafter
        # self._get_camera_stream(f_stop)

        # while True:
        # cv2.imshow('WebCam', self._current_frame)
        # cv2.waitKey(1)

        # async def task():
        #     await sdh.initialize_async(["camera"], viewport)
        #     gt = sdh.get_groundtruth(["camera"], viewport, verify_sensor_init=False)
        #     rgb = gt["camera"]
        #     # depth = gt["depth"]

        #     cv2.imshow(f"camera", rgb)
        #     # cv2.imshow(f"depth", depth)
        #     cv2.waitKey(1)
        import matplotlib.pyplot as plt
        # self._camera.add_motion_vectors_to_frame()
        # asyncio.ensure_future(task())
        # while True:
        # rgba = np.uint8(self._camera.get_current_frame()["rgba"])
        # cv2.imshow("RGBA", rgba)
        # cv2.waitKey(-1)

        # editor.set_active_camera(prim.GetPath().pathString)
        # # -------------------------------------------------------------------

        # params = sdh.get_camera_params()

        # print("camera:", editor.get_active_camera())
        # for k in params:
        #     print("{}: {}".format(k, params[k]))

        # while True:
        # print(f"{i} th frame")
        # print(self._camera.get_current_frame)
        # i += 1

        # i = 0

        # camera.add_motion_vectors_to_frame()

        # while True:
        #     self._world.step(render=True)
        #     print(camera.get_current_frame())
        #     if i == 100:
        #         # points_2d = camera.get_image_coords_from_world_points(
        #         #     np.array([cube_3.get_world_pose()[0], cube_2.get_world_pose()[0]])
        #         # )
        #         # points_3d = camera.get_world_points_from_image_coords(points_2d, np.array([24.94, 24.9]))
        #         # print(points_2d)
        #         # print(points_3d)
        #         imgplot = plt.imshow(camera.get_rgba()[:3])
        #         plt.show()
        #         print(camera.get_current_frame()["motion_vectors"])
        #     if self._world.is_playing():
        #         if self._world.current_time_step_index == 0:
        #             self._world.reset()
        #     i += 1

        # robot = world.scene.add(Robot(prim_path="/World/fetch", name="fetch"))
        # print(robot)
        # print(self._fetchbot)

        print("Num of degrees of freedom before first reset: " + str(self._fetchbot.num_dof))  # prints None
        # print(f"NEW: {robot.num_dof}")
        position, orientation = self._fetchbot.get_world_pose()
        print(f"Initial Position: {position}")
        print(f"Initial Orientation:  + {orientation}")

        return

    def _get_camera_stream(self, f_stop):
        # do something here ...
        print("HELLO!")
        print(time.time() - self._time)
        self._time = time.time()
        frame = np.uint8(self._camera.get_current_frame()["rgba"])
        self._i += 1
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        self._current_frame = frame
        # import matplotlib.pyplot as plt
        # imgplot = plt.imshow(frame)
        # plt.show()
        # cv2.imwrite(f"/home/sauravdosi/test_img/{self._i}.jpg", frame)

        if not f_stop.is_set():
            # call f() again in 60 seconds
            threading.Timer(1 / 30, self._get_camera_stream, [f_stop]).start()

    def get_camera_feed(self, step_size=1 / 30):
        print(time.time() - self._time)
        self._time = time.time()
        print(f"{self._i} th frame")
        print(self._camera.get_current_frame())

    def camera_feed(self):
        sd_helper = SyntheticDataHelper()
        viewport_api = get_active_viewport()

        async def task():
            await sd_helper.initialize_async(["rgb", "depth"], viewport_api)
            gt = sd_helper.get_groundtruth(["rgb", "depth"], viewport_api, verify_sensor_init=False)
            rgb = gt["rgb"]
            depth = gt["depth"]

            print(rgb, depth)

            cv2.imshow("rgb", rgb)
            # cv2.imshow("depth", depth)
            cv2.waitKey(1)

        asyncio.ensure_future(task())

    # # wait for the key and come out of the loop
    # if cv2.waitKey(1) == ord('q'):
    #     break

    async def setup_post_load(self):
        print("HEY1")
        # self.setup_scene()
        self._world = self.get_world()
        print("HEY2")
        self._fetchbot = self._world.scene.get_object("fetch")
        print("HEY3")
        #     # Print info about the fetchbot after the first reset is called
        print("Num of degrees of freedom after first reset: " + str(self._fetchbot.num_dof))  # prints 2
        print("Joint Positions after first reset: " + str(self._fetchbot.get_joint_positions()))
        # This is an implicit PD controller of the jetbot/ articulation
        # setting PD gains, applying actions, switching control modes..etc.
        # can be done through this controller.
        # Note: should be only called after the first reset happens to the world
        self._fetchbot_articulation_controller = self._fetchbot.get_articulation_controller()
        # Adding a physics callback to send the actions to apply actions with every
        # physics step executed.
        # self._world.add_physics_callback("initialize_robot", callback_fn=self.move_initialize_robot)
        # self._fetchbot.apply_action(ArticulationAction(joint_positions=np.array(
        #     [1, 1, 1, # shoulder, elbow, shoulder
        #             1, 1, 1, # elbow, shoulder, wrist
        #             0, 0, 1, # ,,head pan
        #             0, 0, 0, # end gripper, last two wheels
        #             0, 0, 0])))
        print("HEY")
        self._my_controller = WheelBasePoseController(name="cool_controller",
                                                      open_loop_wheel_controller=DifferentialController(
                                                          name="simple_controller",
                                                          wheel_radius=self._wheel_radius, wheel_base=self._wheel_base),
                                                      is_holonomic=False)
        # self._world.add_physics_callback("sending_actions", callback_fn=self.move_robot)
        # self._world.add_physics_callback("getting camera feed", callback_fn=self.get_camera_feed)
        self._world.add_physics_callback("differential controller", callback_fn=self.diff_move_robot)
        # self.custom_move_robot(x_dist=2)
        return

    async def setup_post_reset(self):
        print("RESET")
        # works but fix errors
        self.setup_scene()
        self._world = self.get_world()
        self._fetchbot = self._world.scene.get_object("fetch")
        #     # Print info about the jetbot after the first reset is called
        print("Num of degrees of freedom after first reset: " + str(self._fetchbot.num_dof))  # prints 2
        # self.move_initialize_robot()
        print("Joint Positions after first reset: " + str(self._fetchbot.get_joint_positions()))

    def move_initialize_robot(self):
        # print("INIT")
        positions = np.array([1, 1, 1,  # shoulder, elbow, shoulder
                              1, 1, 1,  # elbow, shoulder, wrist
                              0, 0, 1,  # ,,head pan
                              0, 0, 0,  # end gripper, last two wheels
                              0, 0, 0])
        self._fetchbot_articulation_controller.apply_action(ArticulationAction(joint_positions=positions))
        return

    def find_prims_by_name(self, stage, prim_name: str):
        found_prims = [x for x in stage.Traverse() if x.GetName() == prim_name]
        return found_prims

    def move_robot(self, step_size=0.1):
        values = 5 * np.array([0, 0, 0,  # shoulder, elbow, shoulder
                               0, 0, 0,  # elbow, shoulder, wrist
                               0, 0, 0,  # ,,head pan
                               0, self._output, self._output,  # end gripper, last two wheels
                               0, 0, 0])
        # values = 5*np.array([1,1])
        # values = values
        self._fetchbot_articulation_controller.apply_action(ArticulationAction(joint_positions=None,
                                                                               joint_efforts=None,
                                                                               joint_velocities=values))
        if (self._output < 1):
            self._test_counter += 0.01
            self._output = 2.73 ** self._test_counter
            # print("Joint Positions after execution: " + str(self._fetchbot.get_joint_positions()))
        # print(f"output: {self._output}")
        return

    def diff_move_robot(self, step_size=0.1):
        stage = omni.usd.get_context().get_stage()
        prim_path = "/World/fetch/base_link"
        prim = Articulation(prim_path=prim_path, name="base_link")
        position, orientation = prim.get_world_pose()

        # position, orientation = self._fetchbot.get_world_pose()
        print(f"Position: {position}")
        print(f"Orientation:  {orientation}")
        # self._fetchbot.apply_action(self._my_controller.forward(start_position=position,
        #                                                     start_orientation=orientation,
        #                                                     goal_position=np.array([0.8, 0.8])))
        # print(orientation)
        control_output = self._my_controller.forward(start_position=position, start_orientation=orientation,
                                                     goal_position=np.array([-2, -1]))
        # print(control_output)

        wheel_velocities = control_output.joint_velocities
        control_output.joint_velocities = np.array([0, 0, 0,  # shoulder, elbow, shoulder
                                                    0, 0, 0,  # elbow, shoulder, wrist
                                                    0, 0, 0,  # ,,head pan
                                                    0, wheel_velocities[0], wheel_velocities[1],
                                                    # end gripper, last two wheels
                                                    0, 0, 0])
        self._fetchbot.apply_action(control_output)

    def custom_move_robot(self, x_dist):
        self._number_of_timesteps = 83.73456 * x_dist / (self._forward_const_vel * self._wheel_radius)
        print(self._number_of_timesteps)
        self._timesteps_counter = 0
        self._world.add_physics_callback("differential controller", callback_fn=self.move_robot_simple)
        # self._number_of_timesteps = 0

    def move_robot_simple(self, step_size=0.01):
        # print(time.time())
        print(self._timesteps_counter)
        # print(self._number_of_timesteps)
        values = np.array([0, 0, 0,  # shoulder, elbow, shoulder
                           0, 0, 0,  # elbow, shoulder, wrist
                           0, 0, 0,  # ,,head pan
                           0, self._forward_const_vel, self._forward_const_vel,  # end gripper, last two wheels
                           0, 0, 0])
        self._fetchbot_articulation_controller.apply_action(ArticulationAction(joint_positions=None,
                                                                               joint_efforts=None,
                                                                               joint_velocities=values))
        self._timesteps_counter += 1
        if self._timesteps_counter >= self._number_of_timesteps:
            print("STOP")
            values = np.array([0, 0, 0,  # shoulder, elbow, shoulder
                               0, 0, 0,  # elbow, shoulder, wrist
                               0, 0, 0,  # ,,head pan
                               0, 0, 0,  # end gripper, last two wheels
                               0, 0, 0])
            self._fetchbot_articulation_controller.apply_action(ArticulationAction(joint_positions=None,
                                                                                   joint_efforts=None,
                                                                                   joint_velocities=values))
