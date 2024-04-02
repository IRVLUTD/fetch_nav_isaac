from omni.isaac.kit import SimulationApp
import threading

from src.fetch_nav_init.fetch_nav_init import FetchNavInit
import time

class FetchNavDriver:
    def __init__(self):
        self._simulation_app = SimulationApp(launch_config={"renderer": "RayTracedLighting", "headless": False})
        from omni.isaac.core import World
        self._world = World()
        self._fetch_nave_init = FetchNavInit(self._world)
        # for _ in range(50):
        #     self._simulation_app.update()
        # while True:
        #     self._simulation_app.update()
        #     if cv2.waitKey(0) == ord("q"):
        #         print("Exiting...")
        #         break
        # except KeyboardInterrupt:
        #     pass
        input_thread = threading.Thread(target=self.input_thread)
        execution_thread = threading.Thread(target=self.execution_thread)

        execution_thread.start()
        # input_thread.start()

        # input_thread.join()
        execution_thread.join()

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
