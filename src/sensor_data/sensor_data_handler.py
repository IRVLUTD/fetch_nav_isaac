import time
from flask import Flask, request
import cv2
import numpy as np

app = Flask(__name__)

fps = 0
fps_start_time = time.time()


@app.route('/receive_image', methods=['POST'])
def receive_image():
    global fps
    global fps_start_time

    current_time = time.time()
    fps = 1 / (current_time - fps_start_time)
    fps_start_time = time.time()

    # Receive image bytes from the request
    image_bytes = request.files['image'].read()

    # Convert image bytes to a NumPy array
    nparr = np.frombuffer(image_bytes, np.uint8)

    # Decode the image array
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    sensor_name = str(request.form.get("sensor_name"))

    cv2.namedWindow('Received Image', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Received Image', 960, 540)

    cv2.putText(img, f'FPS: {int(fps)}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(img, f'Sensor: {sensor_name}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display the received image
    cv2.imshow('Received Image', img)
    cv2.waitKey(1)

    return 'Image received successfully!'


if __name__ == '__main__':
    app.run(debug=True)
