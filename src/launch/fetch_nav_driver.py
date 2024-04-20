import threading
import multiprocessing as mp
from omni.isaac.kit import SimulationApp
from src.fetch_nav_init.fetch_nav_init import FetchNavInit
from src.move_fetch.move_fetch import MoveFetch
from src.sensor_data.sensor_data import SensorData
import numpy as np


class FetchNavDriver:
    def __init__(self):
        self._simulation_app = SimulationApp(launch_config={"renderer": "RayTracedLighting", "headless": False})
        from omni.isaac.core import World
        self._world = World()
        self._fetch_nav_init = FetchNavInit(world=self._world, sim_app=self._simulation_app)
        self._fetch_nav_controller = MoveFetch(fetch_nav_init=self._fetch_nav_init)
        self._fetch_sensors = SensorData(self._fetch_nav_init)
        self._controller_tolerance = self._fetch_nav_controller.position_tolerance

        self._fetch_nav_init.simulation_context.initialize_physics()
        self._fetch_nav_init.simulation_context.add_physics_callback("differential controller",
                                                                     callback_fn=
                                                                     self._fetch_nav_controller.
                                                                     diff_move_fetch_callback)

        print("Updating the sim app after the initialization...")
        for _ in range(self._fetch_nav_init.init_frames):
            self._fetch_nav_init.simulation_context.render()

        self._fetch_nav_init.simulation_context.play()

        f_stop = threading.Event()
        self._fetch_sensors.get_rgb_camera_stream(f_stop)

        while True:
            print("Enter 'q' for both the parameters to quit the application")
            x = input("Enter x: ")
            y = input("Enter y: ")

            if str(x) == "q" or str(y) == "q":
                self._simulation_app.close()

            current_x = self._fetch_nav_controller.fetch_position[0]
            current_y = self._fetch_nav_controller.fetch_position[1]

            x_cord, y_cord = self._fetch_nav_controller.correct_coordinates(float(x), float(y), current_x, current_y)
            print(f"Corrected coordinates: {x_cord, y_cord}")

            original_goal_position = np.array([float(x), float(y)])
            corrected_goal_position = np.array([x_cord, y_cord])
            self._fetch_nav_controller.goal_position = corrected_goal_position

            print(f"Moving Fetch to ({x}, {y})...")
            self._fetch_nav_controller.execute_move_base(original_goal_position)


fetch_driver = FetchNavDriver()
