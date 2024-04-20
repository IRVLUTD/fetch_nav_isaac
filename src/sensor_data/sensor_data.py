import threading
import time
import cv2
import numpy as np
import requests
from src.fetch_nav_init.fetch_nav_init import FetchNavInit
from src.synthetic_data.synthetic_data import SyntheticData

class SensorData:
    def __init__(self, fetch_nav_init: FetchNavInit, synthetic_data: SyntheticData):
        self._camera = fetch_nav_init.camera
        self._camera_fps = fetch_nav_init.camera_fps
        self._time = time.time()
        self._config = fetch_nav_init.config
        self._port = int(self._config.get("SENSOR_DATA", "sensor_data_handler_port"))
        self._synthetic_data = synthetic_data

    @staticmethod
    def calculate_time(func):
        def wrapper(*args, **kwargs):
            start_time = time.time()
            result = func(*args, **kwargs)
            end_time = time.time()
            print(f"Time taken by {func.__name__}: {end_time - start_time} seconds")
            return result

        return wrapper

    # @calculate_time
    def get_rgb_camera_stream(self, f_stop):
        frame = np.uint8(self._camera.get_current_frame()["rgba"])
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

        frame = self._synthetic_data.bbox_3d_data(frame)
        self.send_image(frame)

        if not f_stop.is_set():
            threading.Timer(1 / self._camera_fps, self.get_rgb_camera_stream, [f_stop]).start()

        # TODO: Add multiprocessing with shared memory for Camera object
        # while not f_stop.is_set():
        #     start = time.time()
        #     frame = np.uint8(camera.get_current_frame()["rgba"])
        #     frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        #     self.send_image(frame)
        #     end = time.time()
        #     time.sleep((1 / self._camera_fps) - (end - start) if (1/self._camera_fps) > (end - start) else 0)

    def send_image(self, frame):
        _, img_encoded = cv2.imencode('.jpg', frame)

        # Convert the encoded image to bytes
        image_bytes = img_encoded.tobytes()

        # Send the image to the receiver script
        response = requests.post(f'http://localhost:{self._port}/receive_image', files={'image': image_bytes},
                                 data={"sensor_name": "head_camera"})
