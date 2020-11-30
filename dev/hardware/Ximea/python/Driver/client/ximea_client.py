import cv2
import zmq
import base64
import numpy
import time
import sys
from threading import Thread
import numpy as np
from helpers.Charuco_Specific.CharucoBoards import *

class ximea_recieve_camera():
    def __init__(self):
        self.context = zmq.Context(10)
        self.sock = self.context.socket(zmq.SUB)
        self.sock.connect('ipc:///tmp/zmqtest')
        # self.sock.connect('tcp://192.27.172.158:8080')
        self.sock.setsockopt_string(zmq.SUBSCRIBE, numpy.unicode(''))
        self.sock.setsockopt(zmq.SNDHWM, 1)
        self.sock.setsockopt(zmq.RCVHWM, 1)

        self.stopped = False
        self.data = recv_array(self.sock, flags=0, copy=False, track=False)

    def start(self):
        Thread(target=self.get_data, args=()).start()
        return self

    def get_data(self):
        while True:
            if self.stopped:
                return

            start = time.time()
            self.data =  recv_array(self.sock, flags=0, copy=False, track=False)
            end = time.time()
            # sys.stdout.write('\r' + 'ZMQ Thread it/s: ' + str(1 / (end - start)))

    def get_latest_image(self):
        return self.data

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True


def recv_array( socket, flags = 0, copy = False, track = False ):
    """recv a numpy array"""
    md = socket.recv_json( flags = flags )

    if md is None:
        return None

    msg = socket.recv(     flags = flags, copy = copy, track = track )
    buf = memoryview(msg)
    pass;  A = numpy.frombuffer( buf, dtype = md['dtype'] )
    return A.reshape(                         md['shape'] )

class ProcessFrames():
    def __init__(self, cam):
        self.cam = cam

        self.intrinsics = np.load('ximea_intrinsics.npy')
        self.dist = np.load('ximea_distCoeffs.npy')

        self.image = self.cam.get_latest_image()
        self.stopped = False
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

    def start(self):
        Thread(target=self.process_frames, args=()).start()
        return self

    def process_frames(self):
        squareLength = boards['Fiducial']['squareLength']
        markerLength = boards['Fiducial']['markerLength']
        charucoX = boards['Fiducial']['charucoX']
        charucoY = boards['Fiducial']['charucoY']
        board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, self.aruco_dict)

        while True:
            if self.stopped:
                return

            start = time.time()
            image = self.cam.get_latest_image()
            image = cv2.resize(image, (1000, 1000))
            corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(image, self.aruco_dict)
            chcorners,ids2,refus,recover = cv2.aruco.refineDetectedMarkers(image, board, corners, ids, rejectedImgPts)
            cv2.aruco.drawDetectedMarkers(image, chcorners, ids2, (0, 255, 0))

            retval,rvec,tvec = cv2.aruco.estimatePoseBoard(corners,ids,board,self.intrinsics,self.dist)

            if retval > 0:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
                image = cv2.aruco.drawAxis(image, self.intrinsics, self.dist, rvec, tvec, .026)

                print(np.transpose(tvec))

            self.image = image

            end = time.time()
            # sys.stdout.write('\r' + 'Image Processing Thread it/s: ' + str(1 / (end - start)))

    def get_frame(self):
        return self.image


class XimeaGuiThread():
    def __init__(self, ximea):
        self.ximea = ximea
        self.ximea.start()
        self.image = self.ximea.get_frame()
        self.stopped = False

    def start(self):
        Thread(target=self.process_frames, args=()).start()
        return self

    def process_frames(self):
        while True:
            if self.stopped:
                return

            start = time.time()
            self.image = self.ximea.get_frame()
            self.image = cv2.resize(self.image,(1000,1000))
            cv2.imshow('ximea',self.image)
            cv2.waitKey(1)
            end = time.time()
            # sys.stdout.write('\r' + 'GUI Thread it/s: ' + str(1 / (end - start)))

if __name__ == "__main__":
    cam = ximea_recieve_camera()
    cam.start()
    image = cam.get_latest_image()
    # cv2.imwrite("ximea.jpg",image)

    while(True):
        image = cam.get_latest_image()
        image = cv2.resize(image,(1000,1000))

        cv2.imshow('ximea', image)
        cv2.waitKey(1)