from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from src.fetch_nav_init.fetch_nav_init import FetchNavInit
import time

class FetchNavDriver:
    def __init__(self):
        self._simulation_app = SimulationApp(launch_config={"renderer": "RayTracedLighting", "headless": False})
        self._world = World.instance()
        self._fetch_nave_init = FetchNavInit(self._world)
        time.sleep(10)
        self._simulation_app.close()

fetch_driver = FetchNavDriver()
