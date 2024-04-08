import threading
import time
import cv2
import numpy as np
import requests

from src.fetch_nav_init.fetch_nav_init import FetchNavInit


class SensorData:
    def __init__(self, fetch_nav_init: FetchNavInit):
        self._camera = fetch_nav_init.camera
        self._camera_fps = fetch_nav_init.camera_fps
        self._time = time.time()

    def get_rgb_camera_stream(self, f_stop):
        # print("HELLO!")
        # print(time.time() - self._time)
        # self._time = time.time()
        frame = np.uint8(self._camera.get_current_frame()["rgba"])
        # self._i += 1
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        self.send_image(frame)
        # self._current_frame = frame

        # cv2.imshow("RGB CAMERA", frame)
        # cv2.waitKey(1)
        # import matplotlib.pyplot as plt
        # imgplot = plt.imshow(frame)
        # plt.show()
        # cv2.imwrite(f"/home/sauravdosi/test_img/{self._i}.jpg", frame)

        if not f_stop.is_set():
            # call f() again in 1/30 seconds
            threading.Timer(1 / self._camera_fps, self.get_rgb_camera_stream, [f_stop]).start()

    def send_image(self, frame):
        _, img_encoded = cv2.imencode('.jpg', frame)

        # Convert the encoded image to bytes
        image_bytes = img_encoded.tobytes()

        # Send the image to the receiver script
        response = requests.post('http://localhost:5000/receive_image', files={'image': image_bytes,
                                                                               "sensor_name": "head_camera"})

        # Print the response from the receiver
        # print(response.text)