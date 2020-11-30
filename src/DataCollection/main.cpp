
#include <iostream>
#include <string>
using namespace std;
#include <dirent.h>

// Relevant OpenCV imports
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "EstimatePoseCharucoBoard.h"
#include "PoseEstimator.h"  //in Algorithms/PoseEstimation/

// Note that flag to determine whether to print is defined in EstimatePoseCharucoBoard

// Flag which dictates whether to save the charuco board
#define SAVE_BOARD true
// Location where to save/access the board image
#define SAVE_LOC "/home/aofeldman/Desktop/BoardImage.tif"

// Information about the Charuco Board used
// Number of squares across
#define CHARUCO_X 40
// Number of squares down
#define CHARUCO_Y 20
// Marker size including padding
#define SQUARE_LENGTH 0.0235 // m
//Marker size excluding padding
#define MARKER_LENGTH 0.01665 // m

//The folder where images to process are stored
#define FOLDER_NAME "/home/aofeldman/ximeaData/"

// Intrinsics and Distortion Information
#define FX 43899.698855472634
#define FY 43890.1307964898
#define U0 3999.154838190154
#define V0 3072.158186370326
#define K1 1.5039527114541007
#define K2 25.57793307780351
#define P1 -0.00011390420440196816
#define P2 0.002741305919934763
#define K3 17.6388975146445

int main()
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
    cv::Ptr<cv::aruco::CharucoBoard> board;
    cv::Mat boardImage;
    auto tup = createBoard(CHARUCO_X, CHARUCO_Y, SQUARE_LENGTH, MARKER_LENGTH, dictionary);
    board = get<0>(tup);
    boardImage = get<1>(tup);

    if (SAVE_BOARD)
    {
        cv::imwrite(SAVE_LOC, boardImage);
    }

    // Initialize the intrinsics array
    //double fx = 4.38996989*pow(10,4), fy = 4.38901398*pow(10,4), u0 = 3.99915489*pow(10,3), v0 = 3.07215819*pow(10,3);

    //cv::Mat intrinsics = (cv::Mat_<double>(3,3) << fx, 0, u0, 0, fy, v0, 0, 0, 1);
    cv::Mat intrinsics = (cv::Mat_<double>(3,3) << FX, 0, U0, 0, FY, V0, 0, 0, 1);
    cout << "Intrinsics= " << endl << " " << intrinsics << endl << endl;

    //double k1 = 1.50395272, k2 = 2.55779323*pow(10,1), p1 = -1.13904182*pow(10,-4), p2 = 2.74130725*pow(10,-3), k3 = 1.76388975*pow(10,1);

    //cv::Mat dist = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);
    cv::Mat dist = (cv::Mat_<double>(1,5) << K1, K2, P1, P2, K3);
    cout << "Dist= " << endl << " " << dist << endl << endl;

    // Alternative way for pose estimation is to use PoseEstimator:
    PoseEstimator pose_estimator(intrinsics, dist, board, 0.25, true);
    
    // Read in the dataset
    //char foldername[] = FOLDER_NAME;
    // Compute mean error over the entire dataset
    double overallErr = 0;
    int numImages = 0;
    //vector<string> fileList = read_folder(foldername);
    vector<string> fileList = read_folder(FOLDER_NAME);
    for (int i = 0; i < fileList.size(); i++)
    {
        //string filename = foldername + fileList[i];
        string filename = FOLDER_NAME + fileList[i];
        cout << "filename: " << filename << endl;
        cv::Mat image = cv::imread(filename, 0);
        if (!image.data)
        {
            cout << "Failed to find image" << endl;
        }
        double imgErr;
        if(0) //use EstimatePoseCharucoBoard.{cpp,h}
        {
            auto tup2 = estimate_Pose_Charucoboard(image, board, intrinsics, dist);
            cv::Mat extrinsics_4x4 = get<0>(tup2);
            imgErr = get<1>(tup2);
        }
        else //use PoseEstimator
        {
            pose_estimator.estimate_pose_charucoboard(image, &imgErr, nullptr);
            //pose_estimator.get_local_pose_charucoboard(rvec, tvec);
            //pose_estimator.get_detected_chessboard_corners(corners, ids);
        }
        overallErr += imgErr;
        numImages++;
    }
    cout << "Number of images in dataset: " << numImages << endl;
    cout << "Mean error over dataset: " << overallErr / numImages << endl;
}

// Returns list of filenames in a folder
vector<string> read_folder(char foldername[])
{

	vector<string> fileList;

	//auto dir = opendir(p);
	auto dir = opendir(foldername);
	// Fails if dir is a null pointer
	if (dir)
	{
		while (auto f = readdir(dir))
		{
			if (!f->d_name || f->d_name[0] == '.') continue;

			fileList.push_back(f->d_name);
		}
		closedir(dir);
	}
	return fileList;
}
