import time

from flask import Flask, request, render_template
import cv2
import numpy as np
from matplotlib import pyplot as plt
import threading

app = Flask(__name__)

image_queue = []

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

    # image_queue.append(img)

    sensor_name = str(request.files["sensor_name"].read())

    cv2.putText(img, f'FPS: {int(fps)}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(img, f'Sensor: {sensor_name}', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # Display the received image
    cv2.imshow('Received Image', img)
    cv2.waitKey(1)
    # cv2.destroyAllWindows()

    return 'Image received successfully!'

def run_app():
    app.run(debug=True)

def show_stream():
    ax1 = plt.subplot(111)

    # create image plot
    # im1 = ax1.imshow(image_queue[0])

    plt.ion()

    while True:
        if image_queue:
            ax1.imshow(image_queue[0])
        plt.pause(0.2)

if __name__ == '__main__':
    # thread2 = threading.Thread(target=show_stream)
    # thread2.start()
    app.run(debug=True)
    # thread2.join()



    # Create the second thread


    # Start both threads
    # thread1.start()
    #
    #
    # # Wait for both threads to finish
    # thread1.join()
    # thread2.join()

    print("Main Thread: Finished")
