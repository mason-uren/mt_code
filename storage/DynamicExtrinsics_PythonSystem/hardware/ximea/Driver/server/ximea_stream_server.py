from hardware.ximea.Driver.server.Code_From_Manufacturer.ximea import xiapi
import zmq
from threading import Thread

gain = 12
exposure = 10000      # in microseconds
# exposure = 5000
# exposure = 8000
exp_priority = .99

def send_array( socket, A, flags = 0, copy = False, track = False ):
    """send a numpy array with metadata"""
    md = dict( dtype = str( A.dtype ),
               shape =      A.shape,
               )
    pass;
    socket.send_json( md, flags | zmq.SNDMORE )
    return socket.send(      A,  flags, copy = copy, track = track )

class ximea_camera:
    def __init__(self):
        print('Opening camera')
        self.img = xiapi.Image()
        self.cam = xiapi.Camera()
        self.cam.open_device()

        print('Changing Settings')
        print('Exposure = ' + str(exposure))
        print('Gain = ' + str(gain))

        self.cam.set_imgdataformat('XI_RAW8')
        self.cam.set_exp_priority(exp_priority)
        self.cam.set_exposure(exposure)
        self.cam.set_gain_direct(gain)
        #
        # self.cam.set_height(3000)
        # self.cam.set_width(3000)

        self.cam.set_acq_timing_mode('XI_ACQ_TIMING_MODE_FRAME_RATE_LIMIT')
        self.cam.set_framerate(30)

        self.cam.start_acquisition()
        self.cam.get_image(self.img)
        self.frame = self.img.get_image_data_numpy()

        self.stopped = False

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            self.cam.get_image(self.img)

    def read(self):
        # return the frame most recently read
        self.frame = self.img.get_image_data_numpy()
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

class send_data:
    def __init__(self,cam):
        self.cam = cam
        self.context = zmq.Context(10)
        self.sock = self.context.socket(zmq.PUB)
        self.sock.setsockopt(zmq.SNDHWM, 1)
        self.sock.setsockopt(zmq.RCVHWM, 1)
        self.sock.bind('ipc:///tmp/zmqtest')
        # self.sock.bind('tcp://192.27.172.158:8080')
        self.stopped = False

    def start(self):
        Thread(target=self.send, args=()).start()
        return self

    def send(self):
        import cv2

        while True:
            if self.stopped:
                return

            stuff = self.cam.read()
            # stuff = cv2.resize(stuff,(500,500))
            # send_array(self.sock, self.cam.read(), flags=0, copy=False, track=False)
            send_array(self.sock, stuff, flags=0, copy=False, track=False)

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

if __name__ == "__main__":
    import numpy as np

    cam = ximea_camera()
    cam.start()
    dat = send_data(cam)
    dat.start()

    while True:
        pass