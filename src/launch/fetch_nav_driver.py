from omni.isaac.kit import SimulationApp
import threading

from src.fetch_nav_init.fetch_nav_init import FetchNavInit
from src.move_fetch.move_fetch import MoveFetch
import time

class FetchNavDriver:
    def __init__(self):
        self._simulation_app = SimulationApp(launch_config={"renderer": "RayTracedLighting", "headless": False})
        from omni.isaac.core import World
        self._world = World()
        self._fetch_nav_init = FetchNavInit(self._world)
        self._fetch_nav_controller = MoveFetch(fetch_nav_init=self._fetch_nav_init)

        # while True:
        #     self._simulation_app.update()
        #     if cv2.waitKey(0) == ord("q"):
        #         print("Exiting...")
        #         break
        # except KeyboardInterrupt:
        #     pass
        # execution_thread = threading.Thread(target=self.execution_thread)

        # execution_thread.start()
        self._fetch_nav_controller.goal_position = np.array([1, -1])
        self._fetch_nav_init.simulation_context.initialize_physics()
        self._fetch_nav_init.simulation_context.add_physics_callback("differential controller",
                                         callback_fn=self._fetch_nav_controller.diff_move_fetch)
        for _ in range(50):
            self._fetch_nav_init.simulation_context.render()
        # self..stop()
        self._fetch_nav_init.simulation_context.play()

        while True:
            self._simulation_app.update()
        # input_thread = threading.Thread(target=self.input_thread)

        # input_thread.start()

        # input_thread.join()
        # execution_thread.join()

    def input_thread(self):
        print("Input thread started.")
        while True:
            user_input = str(input("Enter something: "))
            print("You entered:", user_input)
            if user_input == "q":
                self._simulation_app.close()

    def execution_thread(self):
        print("Execution thread started.")
        while True:
            self._simulation_app.update()

fetch_driver = FetchNavDriver()
