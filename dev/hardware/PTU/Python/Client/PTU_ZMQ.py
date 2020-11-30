import zmq
import numpy as np
from threading import Thread
from enum import Enum
import time
import math

class PTU(Enum):
    Velocity = 0
    Position = 1

class PTU_SETPOINT_STATUS(Enum):
    Moving = 0
    Converged = 1

class PTU_Angle_Reader():
    def __init__(self, recieve_port = 8000):
        self.stopped = False

        self.pan = None
        self.tilt = None

        # Define ZMQ for Reciving PTU Angles
        recieve_port_string = "tcp://localhost:9001"

        self.context_angles = zmq.Context(1)
        self.receiver_angles = self.context_angles.socket(zmq.SUB)
        self.receiver_angles.setsockopt_string(zmq.SUBSCRIBE, "")
        self.receiver_angles.setsockopt(zmq.SNDHWM, 1)
        self.receiver_angles.setsockopt(zmq.RCVHWM, 1)
        self.receiver_angles.connect(recieve_port_string)

    def start(self):
        self.stopped = False
        Thread(target=self.update, args=()).start()

    def stop(self):
        self.start = True

    def get_angles(self):
        return self.pan, self.tilt

    def update(self):

        while(True):
            time.sleep(1/30)
            angles = self.receiver_angles.recv()
            self.pan, self.tilt = np.frombuffer(angles, dtype=np.float)

            if self.stopped == True:
                return

class PTU_Driver(PTU_Angle_Reader):
    def __init__(self, recieve_port = 8000,send_port = 5556):
        super(PTU_Driver,self).__init__(recieve_port)
        self.stopped = True

        self.mode = PTU.Velocity

        self.pan_vel = 0
        self.tilt_vel = 0

        self.pan_angle = None
        self.tilt_angle = None

        self.prev_tilt = None
        self.prev_pan = None

        self.time = time.time()
        self.interval = 1

        self.status_time = time.time()
        self.status_interval = .4

        # create a pair to control PTU exclusivly from this class
        send_port_string = "ipc:///tmp/ptu_sub"
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.setsockopt(zmq.RCVHWM, 1)
        self.socket.bind(send_port_string)

    def start(self):
        super(PTU_Driver,self).start()
        self.stopped = False
        Thread(target=self.send_command, args=()).start()

    def stop(self):
        super(PTU_Driver,self).stop()
        self.stopped = True

    def set_control_mode(self,PTU):
        self.mode = PTU

    def set_velocity(self,pan,tilt):
        self.pan_vel = pan
        self.tilt_vel = tilt

    def set_angles(self,pan_angle,tilt_angle):
        self.pan_angle = pan_angle
        self.tilt_angle = tilt_angle

    def get_status(self,constraint = .1):
            if(self.mode == PTU.Position):
                self.prev_tilt = None
                self.prev_pan = None

                pan_high_thresh = abs(self.pan_angle) + constraint
                pan_low_thresh = abs(self.pan_angle) - constraint

                tilt_high_thresh = self.tilt_angle + constraint
                tilt_low_thresh = self.tilt_angle - constraint

                if((self.pan <= pan_high_thresh) and (self.pan >= pan_low_thresh)) \
                        and ((self.tilt <= tilt_high_thresh) and (self.tilt >= tilt_low_thresh)):
                    return PTU_SETPOINT_STATUS.Converged
                else:
                    return PTU_SETPOINT_STATUS.Moving

            # if(self.mode == PTU.Velocity):
            #     if(self.prev_tilt is None):
            #         self.prev_tilt = self.tilt
            #         self.prev_pan = self.pan
            #
            #     inter = self.status_time - time.time()
            #     if (abs(inter) >= self.status_interval):
            #         self.status_time = time.time()
            #         self.prev_pan = self.pan
            #         self.prev_tilt = self.tilt
            #
            #     pan_high_thresh = abs(self.prev_pan) + constraint
            #     pan_low_thresh = abs(self.prev_pan) - constraint
            #
            #     tilt_high_thresh = self.prev_tilt + constraint
            #     tilt_low_thresh = self.prev_tilt - constraint
            #
            #     if((self.pan <= pan_high_thresh) and (self.pan >= pan_low_thresh)) \
            #             and ((self.tilt <= tilt_high_thresh) and (self.tilt >= tilt_low_thresh)):
            #         print("Not MOving")
            #         return PTU_SETPOINT_STATUS.Converged
            #
            #     print("Moving")
            #     return PTU_SETPOINT_STATUS.Moving

    def send_command(self):
        while(True):
            inter = self.time - time.time()
            if(abs(inter) >= self.interval):
                self.time = time.time()

                if self.mode == PTU.Velocity:
                    self.socket.send_string("%f %f %f" % (self.mode.value, self.pan_vel, self.tilt_vel))
                else:
                    if(self.pan_angle == None or self.tilt_angle == None):
                        continue
                    else:
                        self.socket.send_string("%f %f %f" % (self.mode.value, self.pan_angle, self.tilt_angle))

            if self.stopped == True:
                return

if __name__ == "__main__":
    # Required Setup to start PTU Driver
    ptu_angles = PTU_Driver()
    ptu_angles.start()

    # Example Driver Program for Controlling PTU in Position Mode
    ptu_angles.set_control_mode(PTU.Position)
    ptu_angles.set_angles(90,  0)

    # ptu_angles = PTU_Angle_Reader()
    # ptu_angles.start()

    # Example Driver Program for Controlling PTU in Position Mode
    # ptu_angles.set_control_mode(PTU.Velocity)
    # ptu_angles.set_velocity(1000,0)

    while(True):
        x,y = ptu_angles.get_angles()
        print(x,y)