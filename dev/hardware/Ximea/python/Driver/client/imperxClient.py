import zmq
import numpy as np
import time
import sys
import numpy as np
from threading import Thread

class recieve_camera():
    def __init__(self):
        self.context = zmq.Context()
        self.receiver = self.context.socket(zmq.SUB)
        self.receiver.connect("tcp://localhost:9000")
        self.receiver.setsockopt(zmq.SNDHWM, 1)
        self.receiver.setsockopt(zmq.RCVHWM, 1)
        self.receiver.setsockopt_string(zmq.SUBSCRIBE, "")

        image_bytes = self.receiver.recv()
        self.data = np.frombuffer(image_bytes, dtype=np.uint8).reshape((5120, 5120))

        self.stopped = False


    def start(self):
        Thread(target=self.get_data, args=()).start()
        return self

    def get_data(self):
        while True:
            if self.stopped:
                return

            start = time.time()
            image_bytes = self.receiver.recv()
            self.data = np.frombuffer(image_bytes, dtype=np.uint8).reshape((5120,5120))
            end = time.time()
            sys.stdout.write('\r' + 'ZMQ Thread it/s: ' + str(1 / (end - start)))

    def get_latest_image(self):
        return self.data

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True


if __name__ == "__main__":
    import cv2

    cam = recieve_camera()
    cam.start()

    while True:
        img = cam.get_latest_image()
        img = cv2.resize(img,(1000,1000))
        cv2.imshow("img",img)
        cv2.waitKey(1)