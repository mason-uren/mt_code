#include "StereoReconstruct3d.h"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

void BoeingMetrology::StereoReconstruct3d::StereoReconstruct3d::CreateDisparityMap(const std::pair<BoeingMetrology::Calibration::IntrinsicData, cv::Mat> & leftimage, 
    const std::pair<BoeingMetrology::Calibration::IntrinsicData, cv::Mat> & rightimage, const bool & rectifyImages, cv::Mat & disp)
{
    int ndisparities = 16 * 5;   /**< Range of disparity */
    int SADWindowSize = 21; /**< Size of the block window. Must be odd */

    cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(ndisparities, SADWindowSize);
    //cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,    //int minDisparity
    //    96,     //int numDisparities
    //    5,      //int SADWindowSize
    //    600,    //int P1 = 0
    //    2400,   //int P2 = 0
    //    10,     //int disp12MaxDiff = 0
    //    16,     //int preFilterCap = 0
    //    2,      //int uniquenessRatio = 0
    //    20,    //int speckleWindowSize = 0
    //    30,     //int speckleRange = 0
    //    true);  //bool fullDP = false

    // Rectify the images
    cv::Mat leftimagerect, rightimagerect;
    if (rectifyImages)
    {
        cv::undistort(leftimage.second, leftimagerect, leftimage.first.cameraMatrix, leftimage.first.distortionCoeffs);
        cv::undistort(rightimage.second, rightimagerect, rightimage.first.cameraMatrix, rightimage.first.distortionCoeffs);
    }
    else
    {
        leftimagerect = leftimage.second.clone();
        rightimagerect = rightimage.second.clone();
    }

    // Convert the images to greyscale
    cv::Mat leftimagegrey, rightimagegrey;
    cv::cvtColor(leftimagerect, leftimagegrey, CV_BGR2GRAY);
    cv::cvtColor(rightimagerect, rightimagegrey, CV_BGR2GRAY);

    // Compute the disparity map
    sbm->compute(leftimagegrey, rightimagegrey, disp);

    // Display as 8-bit
    cv::Mat dispnorm_bm;
    double minVal; double maxVal;
    cv::minMaxLoc(disp, &minVal, &maxVal);
    disp.convertTo(dispnorm_bm, CV_8UC1, 255 / (maxVal - minVal));
   // cv::imshow("BM", dispnorm_bm);

    //wait for 40 milliseconds
    //int c = cv::waitKey(40);

}


void BoeingMetrology::StereoReconstruct3d::StereoReconstruct3d::CreatePointCloudFromStereoPair(const std::pair<BoeingMetrology::Calibration::IntrinsicData, cv::Mat> & leftimage,
    const std::pair<BoeingMetrology::Calibration::IntrinsicData, cv::Mat> & rightimage, const cv::Mat & R, const cv::Mat & T, const bool & rectifyImages, cv::Mat & xyz)
{
    // Create a disparity map as 16-bit signed ints
    cv::Mat disp;
    CreateDisparityMap(leftimage, rightimage, rectifyImages, disp);

    // Verify image size
    cv::Size imgSize(leftimage.second.size());
    if (leftimage.second.size() != rightimage.second.size())
        throw std::runtime_error("StereoReconstruct3d::CreatePointCloudFromStereoPair: Image sizes do not match");

    // Convert disparity map to floats
    if (disp.type() != CV_16SC1)
        throw std::runtime_error("StereoReconstruct3d::CreatePointCloudFromStereoPair: Matrix type not as expected");
    cv::Mat dispf;
    disp.convertTo(dispf, CV_32F, 1 / 16.0);

    // Get the Q (disparity-to-depth mapping) matrix
    // 1      0            0         -cx
    // 0      1            0         -cy
    // 0      0            0          f
    // 0      0            -1/Tx      (cx - cx')/Tx
    // where cx and cx' are the principal points of the two cameras
    // Tx is the baseline length
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(leftimage.first.cameraMatrix, leftimage.first.distortionCoeffs, rightimage.first.cameraMatrix, rightimage.first.distortionCoeffs, imgSize,
        R, T, R1, R2, P1, P2, Q);

    // Compute point cloud as 32-bit float
    cv::reprojectImageTo3D(dispf, xyz, Q, true);
    //xyz = cv::Mat(dispf.size(), CV_32FC3);
    //cv::Mat_<double> vec_tmp(4, 1);
    //for (int y = 0; y < dispf.rows; y++)
    //{
    //    for (int x = 0; x < dispf.cols; x++)
    //    {
    //        vec_tmp(0) = x; vec_tmp(1) = y; vec_tmp(2) = dispf.at<float>(y, x); vec_tmp(3) = 1;
    //        vec_tmp = Q*vec_tmp;
    //        vec_tmp /= vec_tmp(3);
    //        cv::Vec3f &point = xyz.at<cv::Vec3f>(y, x);
    //        point[0] = vec_tmp(0);
    //        point[1] = vec_tmp(1);
    //        point[2] = vec_tmp(2);
    //    }
    //}
}

// Export a point cloud to file
void BoeingMetrology::StereoReconstruct3d::StereoReconstruct3d::SaveXYZ(const char* filename, const cv::Mat& xyz)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for (int y = 0; y < xyz.rows; y++)
    {
        for (int x = 0; x < xyz.cols; x++)
        {
            cv::Vec3f point = xyz.at<cv::Vec3f>(y, x);
            if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}