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
        self._fetch_nave_init = FetchNavInit(self._world)
        self._fetch_nav_controller = MoveFetch(fetchbot=self._fetch_nave_init._fetchbot,
                                               wheel_radius=self._fetch_nave_init._wheel_radius,
                                               wheel_base=self._fetch_nave_init._wheel_base,
                                               base_prim_path=self._fetch_nave_init._base_prim_path,
                                               base_name=self._fetch_nave_init._base_name)
        # for _ in range(50):
        #     self._simulation_app.update()
        # while True:
        #     self._simulation_app.update()
        #     if cv2.waitKey(0) == ord("q"):
        #         print("Exiting...")
        #         break
        # except KeyboardInterrupt:
        #     pass
        self._fetch_nave_init._simulation_context.play()
        self._world.add_physics_callback("differential controller",
                                         callback_fn=self._fetch_nav_controller.diff_move_fetch)
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
