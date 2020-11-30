import os
import cv2
from cv2 import aruco
import numpy as np

if __name__ == "__main__":


    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
    squareLength = 40
    markerLength = 30
    board = aruco.CharucoBoard_create(5, 7, squareLength, markerLength, aruco_dict)
    board = board.draw((1000, 1000))

    cv2.imshow("stuff", board)

    while True:
        if (cv2.waitKey(30) >= 0):
            break