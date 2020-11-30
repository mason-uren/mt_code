#pragma once

#include <iostream>
#include <string>

// Relevant OpenCV imports
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

// Flag to determine whether to print rvec, tvec, etc. when processing image
#define DEBUG_VERBOSE true

using namespace std;

tuple<cv::Ptr<cv::aruco::CharucoBoard>,cv::Mat> createBoard(int charucoX, int charucoY, float squareLength, float markerLength, cv::Ptr<cv::aruco::Dictionary> dictionary);

tuple<vector<cv::Point2f>,vector<double>> charuco_reprojection_error(cv::Ptr<cv::aruco::CharucoBoard> board, vector<cv::Point2f> chorners,
    vector<int> chids, cv::Vec3d rvec, cv::Vec3d tvec, cv::Mat intrinsics, cv::Mat dist);

tuple<cv::Mat,double> get_pose(vector<vector<cv::Point2f>> markerCorners, vector<int> markerIds, vector<cv::Point2f> chorners,
    vector<int> chids, cv::Ptr<cv::aruco::CharucoBoard> board, cv::Mat intrinsics, cv::Mat dist, bool debug_verbose=DEBUG_VERBOSE);


tuple<cv::Mat,double,vector<int>,vector<cv::Point2f>> estimate_Pose_Charucoboard(cv::Mat image, cv::Ptr<cv::aruco::CharucoBoard> board,
    cv::Mat intrinsics, cv::Mat dist, cv::Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000));

vector<string> read_folder(char foldername[]);
