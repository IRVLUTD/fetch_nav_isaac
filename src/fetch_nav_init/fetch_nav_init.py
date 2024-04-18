from configparser import ConfigParser
import os
import ast


class FetchNavInit:
    def __init__(self, world, config_path="./config/config.ini"):
        self._config = ConfigParser()
        self._config.read([config_path])
        self._fetch_usd_dir = self._config.get("DIR", "fetch_usd_dir")
        self._fetch_usd_name = self._config.get("FETCH_INIT", "fetch_usd_name")
        self._fetch_usd_path = os.path.join(self._fetch_usd_dir, self._fetch_usd_name)
        self._fetch_prim_path = self._config.get("FETCH_INIT", "fetch_prim_path")
        self._fetch_name = self._config.get("FETCH_INIT", "fetch_name")
        self._base_prim_path = self._config.get("FETCH_INIT", "base_prim_path")
        self._base_name = self._config.get("FETCH_INIT", "base_name")
        self._wheel_radius = float(self._config.get("FETCH_INIT", "wheel_radius"))
        self._wheel_base = float(self._config.get("FETCH_INIT", "wheel_base"))
        self._wheel_names_dict = ast.literal_eval(self._config.get("FETCH_INIT", "wheel_names_dict"))
        self._camera_prim_path = self._config.get("SENSORS_INIT", "camera_prim_path")
        self._camera_fps = int(self._config.get("SENSORS_INIT", "camera_fps"))
        self._camera_resolution = ast.literal_eval(self._config.get("SENSORS_INIT", "camera_resolution"))

        from omni.isaac.core import SimulationContext

        self._simulation_context = SimulationContext(stage_units_in_meters=1.0)

        self._world = world
        self._fetchbot = None
        self._camera = None

        self.init_scene()
        self.init_sensors()

    @property
    def fetch_usd_dir(self):
        return self._config.get("DIR", "fetch_usd_dir")

    @property
    def fetch_usd_name(self):
        return self._config.get("FETCH_INIT", "fetch_usd_name")

    @property
    def fetch_usd_path(self):
        return os.path.join(self.fetch_usd_dir, self.fetch_usd_name)

    @property
    def fetch_prim_path(self):
        return self._config.get("FETCH_INIT", "fetch_prim_path")

    @property
    def fetch_name(self):
        return self._config.get("FETCH_INIT", "fetch_name")

    @property
    def base_prim_path(self):
        return self._config.get("FETCH_INIT", "base_prim_path")

    @property
    def base_name(self):
        return self._config.get("FETCH_INIT", "base_name")

    @property
    def wheel_radius(self):
        return float(self._config.get("FETCH_INIT", "wheel_radius"))

    @property
    def wheel_base(self):
        return float(self._config.get("FETCH_INIT", "wheel_base"))

    @property
    def wheel_names_dict(self):
        return ast.literal_eval(self._config.get("FETCH_INIT", "wheel_names_dict"))

    @property
    def camera_prim_path(self):
        return self._config.get("SENSORS_INIT", "camera_prim_path")

    @property
    def camera_fps(self):
        return int(self._config.get("SENSORS_INIT", "camera_fps"))

    @property
    def camera_resolution(self):
        return ast.literal_eval(self._config.get("SENSORS_INIT", "camera_resolution"))

    @property
    def simulation_context(self):
        return self._simulation_context

    @property
    def fetchbot(self):
        return self._fetchbot

    @property
    def world(self):
        return self._world

    @property
    def camera(self):
        return self._camera

    def init_scene(self):
        from omni.isaac.wheeled_robots.robots import WheeledRobot
        self._world.scene.add_default_ground_plane()
        self._fetchbot = self._world.scene.add(WheeledRobot(
                prim_path=self._fetch_prim_path,
                name=self._fetch_name,
                wheel_dof_names=[self._wheel_names_dict["left"], self._wheel_names_dict["right"]],
                create_robot=True,
                usd_path=self._fetch_usd_path
            )
        )
        # self._world.step(render=True)
        print("Fetch initialized")

    def init_sensors(self):
        from omni.isaac.sensor import Camera
        self._camera = Camera(
        prim_path=self._camera_prim_path,
        frequency=self._camera_fps,
        resolution=self._camera_resolution
        )
        self._camera.initialize()
        print("Camera initialized")

