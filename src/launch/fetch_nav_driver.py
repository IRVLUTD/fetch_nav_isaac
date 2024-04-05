from omni.isaac.kit import SimulationApp
from src.fetch_nav_init.fetch_nav_init import FetchNavInit
from src.move_fetch.move_fetch import MoveFetch
import time
import numpy as np
class FetchNavDriver:
    def __init__(self):
        self._simulation_app = SimulationApp(launch_config={"renderer": "RayTracedLighting", "headless": False})
        from omni.isaac.core import World
        self._world = World()
        self._fetch_nav_init = FetchNavInit(self._world)
        self._fetch_nav_controller = MoveFetch(fetch_nav_init=self._fetch_nav_init)
        self._controller_tolerance = self._fetch_nav_controller.position_tolerance

        self._fetch_nav_init.simulation_context.initialize_physics()
        self._fetch_nav_init.simulation_context.add_physics_callback("differential controller",
                                         callback_fn=self._fetch_nav_controller.diff_move_fetch_callback)

        print("Updating the sim app after the initialization...")
        for _ in range(50):
            self._fetch_nav_init.simulation_context.render()

        self._fetch_nav_init.simulation_context.play()

        while True:
            print("Enter 'q' for both the parameters to quit the application")
            x = input("Enter x: ")
            y = input("Enter y: ")

            if str(x) == str(y) == "q":
                self._simulation_app.close()

            current_x = self._fetch_nav_controller.fetch_position[0]
            current_y = self._fetch_nav_controller.fetch_position[1]

            x_cord, y_cord = self.correct_coordinates(float(x), float(y), current_x, current_y)
            print(f"Corrected coordinates: {x_cord, y_cord}")

            original_goal_position = np.array([float(x), float(y)])
            corrected_goal_position = np.array([x_cord, y_cord])
            self._fetch_nav_controller.goal_position = corrected_goal_position

            print(f"Moving Fetch to ({x}, {y})...")
            self.execute_move_base(original_goal_position)

    def correct_coordinates(self, x, y, current_x, current_y):
        x_correct = x
        y_correct = y
        if abs(x - current_x) > self._controller_tolerance:
            x_correct = x - 0.1 if (x - current_x) < 0 else x + 0.1
            if abs(y - current_y) > self._controller_tolerance:
                y_correct = y + abs(0.1 * (y - current_y) / (x - current_x)) if (
                        (y - current_y) > 0) else y - abs(0.1 * (y - current_y) / (x - current_x))
        elif abs(y - current_y) > self._controller_tolerance:
            y_correct = y - 0.1 if (y - current_y) < 0 else y + 0.1

        return x_correct, y_correct

    def execute_move_base(self, original_goal_position):
        fetch_current_position = self._fetch_nav_controller.fetch_position[:2]

        while not np.allclose(fetch_current_position, original_goal_position, atol=self._controller_tolerance):
            self._fetch_nav_init.simulation_context.step(render=True)
            fetch_current_position = self._fetch_nav_controller.fetch_position[:2]

        print(f"Reached ({fetch_current_position[0]}, {fetch_current_position[1]})...")


fetch_driver = FetchNavDriver()
