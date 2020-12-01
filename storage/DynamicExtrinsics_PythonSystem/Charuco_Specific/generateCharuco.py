import cv2
from Charuco_Specific.CharucoBoards import *


squareLength = boards['Ximea']['squareLength']     # in pixels
markerLength = boards['Ximea']['markerLength']
charucoX = 40
charucoY = 20

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

img = board.draw((3840,2160))
cv2.imwrite("monitor_calibration_photo.jpg",img)