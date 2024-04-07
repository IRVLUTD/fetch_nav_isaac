import cv2
from src.fetch_nav_init.fetch_nav_init import FetchNavInit
class SensorData:
    def __init__(self, fetch_nav_init: FetchNavInit):
        self._camera = fetch_nav_init.camera
        self._camera_fps = fetch_nav_init.camera_fps

    def get_rgb_camera_stream(self, f_stop):
        print("HELLO!")
        print(time.time() - self._time)
        self._time = time.time()
        frame = np.uint8(self._camera.get_current_frame()["rgba"])
        # self._i += 1
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        self._current_frame = frame
        cv2.imshow("RGB CAMERA", frame)
        # import matplotlib.pyplot as plt
        # imgplot = plt.imshow(frame)
        # plt.show()
        # cv2.imwrite(f"/home/sauravdosi/test_img/{self._i}.jpg", frame)

        if not f_stop.is_set():
            # call f() again in 1/30 seconds
            threading.Timer(1 / self._camera_fps, self.get_rgb_camera_stream, [f_stop]).start()