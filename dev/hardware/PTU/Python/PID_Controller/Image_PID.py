from hardware.PTU.Python.Client.PTU_ZMQ import  *
from hardware.Ximea.python.Driver.client.ximea_client import  *
from helpers.Charuco_Specific.CharucoBoards import *
from helpers.Charuco_Specific.ChArUcoHelpers import *
from hardware.PTU.Python.Client.PTU_ZMQ import *

squareLength = boards['Fiducial']['squareLength']
markerLength = boards['Fiducial']['markerLength']
charucoX = boards['Fiducial']['charucoX']
charucoY = boards['Fiducial']['charucoY']

MAX_VELOCITY = 800

SETPOINT = 38                    # Charuco ID

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

id_to_pixel_fine = None

def find_fine(cam):
    while (True):
        image = cam.get_latest_image()
        image = cv2.blur(image, (3, 3))
        image, pixel_coordinates_in_orignal_image = search_by_crop_for_charucoBoard(image, aruco_dict, [3, 3],
                                                                                    (800, 800))

        markers, ids, rejectedImgPts = cv2.aruco.detectMarkers(image, aruco_dict)
        if (ids is not None and len(ids) > 5):
            ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markers, ids, image, board)

            # Offset Detected Image to correspond with upsampled imperx
            offset_imper_cornerx = np.tile(pixel_coordinates_in_orignal_image, (chorners.shape[0], 1, 1))
            chorners = np.add(chorners, offset_imper_cornerx)

            id_to_pixel_fine = charuco_coordinates_as_list(chids, chorners)

        else:
            return None

class PTU_PID_SETPOINT_STATUS(Enum):
    Found = 0
    NotFound = 1
    NoIds = 2
    Converged = 3

class PTU_PID():
    def __init__(self,
                 cam,
                 KP=.23,
                 KD=.8,
                 KI =.8):

        self.stopped = False

        self.KP = KP
        self.KD = KD
        self.KI = KI

        self.cam = cam

        self.pan_vel = 0
        self.tilt_vel = 0

        self.pan_error = 0
        self.tilt_error = 0

        self.x = 0
        self.y = 0

        self.status = PTU_PID_SETPOINT_STATUS.NoIds

    def start(self):
        self.cam.start()
        Thread(target=self.run, args=()).start()
        return self

    def stop(self):
        self.stop = True

    def set_KP(self,KP):
        self.KP = KP

    def set_KI(self,KI):
        self.KI = KI

    def set_KD(self,KD):
        self.KD = KD

    # def coarse_find(self):
    #     return None

    def run(self):
        id_to_pixel = None

        while (True):
            if ((abs(self.pan_error) < 5) and (abs(self.tilt_error) < 5)):
                self.status = PTU_PID_SETPOINT_STATUS.Converged
                self.pan_vel = 0
                self.tilt_vel = 0

            if(self.stopped == True):
                return

            image = self.cam.get_latest_image()
            image = cv2.blur(image, (3, 3))

            RESOLUTION_Y = image.shape[0]  # Pixels
            RESOLUTION_X = image.shape[1]  # Pixels

            CENTER_X = int(RESOLUTION_X / 2)  # Pixels
            CENTER_Y = int(RESOLUTION_Y / 2)  # Pixels

            image_downsampled = cv2.resize(image,(1500,1500))
            corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(image_downsampled,aruco_dict)
            if (ids is not None and len(ids) > 5):
                ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(corners, ids, image_downsampled, board)
                id_to_pixel = charuco_coordinates_as_list(chids, chorners)


            if id_to_pixel is not None and SETPOINT in id_to_pixel:
                self.status = PTU_PID_SETPOINT_STATUS.Found

                self.x, self.y = id_to_pixel[SETPOINT]


                self.x = self.x * (RESOLUTION_X/1500)
                self.y = self.y * (RESOLUTION_Y/1500)

                self.pan_error = (CENTER_X - self.x)
                self.tilt_error = (CENTER_Y - self.y)

                self.pan_vel = (self.KP * (CENTER_X - self.x))
                self.tilt_vel = -1*(self.KP * (CENTER_Y - self.y))

                if (abs(self.tilt_vel) > MAX_VELOCITY and self.tilt_vel < 0):
                    self.tilt_vel = -MAX_VELOCITY
                elif (abs(self.tilt_vel) > MAX_VELOCITY and self.tilt_vel > 0):
                    self.tilt_vel = MAX_VELOCITY

                if (abs(self.pan_vel) > MAX_VELOCITY and self.pan_vel < 0):
                    self.pan_vel = -MAX_VELOCITY
                elif (abs(self.pan_vel) > MAX_VELOCITY and self.pan_vel > 0):
                    self.pan_vel = MAX_VELOCITY
            else:
                self.pan_vel = 0
                self.tilt_vel = 0
                self.status = PTU_PID_SETPOINT_STATUS.NotFound
        else:
            self.pan_vel = 0
            self.tilt_vel = 0
            self.status = PTU_PID_SETPOINT_STATUS.NoIds

    def get_SetPoint_Location(self):
        return self.x,self.y

    def get_Error(self):
        return self.pan_error,self.tilt_error

    def get_SETPOINT_status(self):
        return self.status

    def get_control(self):
        return self.pan_vel,self.tilt_vel

if __name__ == "__main__":
    cam = ximea_recieve_camera()
    pid = PTU_PID(cam)
    pid.start()

    ptu_angles = PTU_Driver()
    ptu_angles.start()

    while(True):
        pan_vel , tilt_vel = pid.get_control()
        ptu_angles.set_velocity(pan_vel, tilt_vel)