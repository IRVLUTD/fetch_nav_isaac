from omni.isaac.kit import SimulationApp
import multiprocessing
import threading
import sys
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
        self._controller_tolerance = 0.01

        # while True:
        #     self._simulation_app.update()
        #     if cv2.waitKey(0) == ord("q"):
        #         print("Exiting...")
        #         break
        # except KeyboardInterrupt:
        #     pass

        # self._fetch_nav_controller.goal_position = np.array([0, 0])
        self._fetch_nav_init.simulation_context.initialize_physics()
        self._fetch_nav_init.simulation_context.add_physics_callback("differential controller",
                                         callback_fn=self._fetch_nav_controller.diff_move_fetch_callback)

        for _ in range(50):
            self._fetch_nav_init.simulation_context.render()
        # self..stop()
        self._fetch_nav_init.simulation_context.play()
        # self.execution_thread()

        # input_thread.start()
        #
        # input_thread.join()
        # execution_thread.join()

        # Create a multiprocessing.Queue to communicate between processes
        queue = multiprocessing.Queue()

        # Start a process for taking user input
        # input_process = multiprocessing.Process(target=self.input_thread, args=(queue,))
        # input_process.start()
        count = 0

        # try:
            # Execute the program in the main process
        # while True:
        #     self._fetch_nav_init.simulation_context.step(render=True)
        #     # self.execution_thread()
        #     count += 1
        #     print(count)
        #     if count == 1000:
        #         break
                # time.sleep(1)  # Wait for a while before executing again
        # except KeyboardInterrupt:
        #     # If user interrupts with Ctrl+C, terminate the input process
        #     input_process.terminate()

            # Wait for the input process to finish
        # input_process.join()
        #
        # # Retrieve and print the user inputs from the queue
        # print("User inputs:")
        # while not queue.empty():
        #     print(queue.get())

        while True:
            x = input("Enter x: ")
            y = input("Enter y: ")
            if str(x) == str(y) == "q":
                self._simulation_app.close()

            current_x = self._fetch_nav_controller.fetch_position[0]
            current_y = self._fetch_nav_controller.fetch_position[1]
            x_cord, y_cord = self.correct_coordinates(float(x), float(y), current_x, current_y)
            print(f"Correct coordinates: {x_cord, y_cord}")
            # goal_position = np.array([x_cord - 0.1 if x_cord >= 0 else x_cord + 0.1,
            #                           y_cord - 0.1 if y_cord >= 0 else y_cord + 0.1])
            original_goal_position = np.array([float(x), float(y)])
            goal_position = np.array([x_cord, y_cord])
            self._fetch_nav_controller.goal_position = goal_position
            self.execution_thread(original_goal_position)

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
    def input_thread(self, queue):
        while True:
            # print("INPUT PROCESS")
            user_input = str(input("Enter something (or type 'quit' to exit): "))
            if user_input.lower() == 'quit':
                break
            queue.put(user_input)  # Put the user input into the queue

    def execution_thread(self, original_goal_position):
        fetch_current_position = self._fetch_nav_controller.fetch_position[:2]
        fetch_goal_position = self._fetch_nav_controller.goal_position
        # print(fetch_current_position)

        while not np.allclose(fetch_current_position, original_goal_position, atol=self._controller_tolerance):
            self._fetch_nav_init.simulation_context.step(render=True)
            fetch_current_position = self._fetch_nav_controller.fetch_position[:2]
            print(self._fetch_nav_controller.fetch_position[:2])
            # self.execution_thread()
            # count += 1
            # print(count)
            # if count >= 1000:
            #     break

fetch_driver = FetchNavDriver()

execution_thread = threading.Thread(target=fetch_driver.execution_thread)
# input_thread = threading.Thread(target=fetch_driver.input_thread)
# execution_thread.daemon = True
# execution_thread.start()
#
# while True:
#     if str(input("Type q to exit")) == 'q':
#         fetch_driver._simulation_app.close()
#         sys.exit()