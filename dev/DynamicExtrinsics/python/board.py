import cv2

# TODO: load from Scott's file
# TODO: import these from somewhere rather than hard coding it everywhere
board_dict = {'charucoX': 40, 'charucoY': 20, 'squareLength': .02352, 'markerLength': .016}
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board = cv2.aruco.CharucoBoard_create(*board_dict.values(), dictionary)


# This is an [B, 3] array of the 3d positions of each chessboard corner as
# measured in meters relative to the top-left corner of the board.
board.chessboardCorners
