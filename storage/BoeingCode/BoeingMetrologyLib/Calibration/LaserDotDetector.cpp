#include "LaserDotDetector.h"

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>
#include "TypeDefs.h"
#include "Calibration/Observation/LaserDotObservationPoints.h"
#include <opencv2/imgproc.hpp>
#include <cv.h>
#include <opencv2/opencv.hpp>

using namespace BoeingMetrology::Calibration;
    
#define VIZCIRCLES

void BoeingMetrology::Calibration::LaserDotDetector::detectRedLaserDotFromGrayscaleImageUserInput(const cv::Mat& inputImageDistorted, const cv::Ptr<cv::aruco::CharucoBoard>& board, 
    const std::string & resultsFname, const Calibration::IntrinsicData & intrinsicData, Observation::LaserDotObservationPoints& laserDotObservationPoints, const float & intensityThreshold /*= 220.0f*/)
{
    // Convert to greyscale if necessary
    cv::Mat inputImageSingleChannel = inputImageDistorted.clone();
    if (inputImageDistorted.channels() > 1)
    {
        std::cout << "Converting to single-channel." << std::endl;
        cv::cvtColor(inputImageDistorted, inputImageSingleChannel, cv::COLOR_BGR2GRAY);
    }

    // Initialize the output with the greyscale image
    laserDotObservationPoints.image = inputImageSingleChannel.clone();

    // Add the user-selected true detection to the output object
    try
    {
        std::vector<cv::Point2f> temp;
        this->selectDotCircle(temp, laserDotObservationPoints);

        // Additional processing to refine user selection
        // TBD

        // Undistort the point
        cv::undistortPoints(laserDotObservationPoints.positions, laserDotObservationPoints.positions, intrinsicData.cameraMatrix, intrinsicData.distortionCoeffs, cv::noArray(), intrinsicData.cameraMatrix);

    }
    catch (...)
    {
        throw std::runtime_error("User selection of laser dot was cancelled");
    }
}

void  BoeingMetrology::Calibration::LaserDotDetector::detectRedLaserDotFromGrayscaleImage(const cv::Mat& inputImageDistorted, const cv::Ptr<cv::aruco::CharucoBoard>& board,
    const std::string & resultsFname, const Calibration::IntrinsicData & intrinsicData, Observation::LaserDotObservationPoints& laserDotObservationPoints, const float & intensityThreshold)
{
    this->detectRedLaserDotFromGrayscaleImage(inputImageDistorted, board, resultsFname, laserDotObservationPoints, intensityThreshold);

    // Undistort the point
    cv::undistortPoints(laserDotObservationPoints.positions, laserDotObservationPoints.positions, intrinsicData.cameraMatrix, intrinsicData.distortionCoeffs, cv::noArray(), intrinsicData.cameraMatrix);
}

void  BoeingMetrology::Calibration::LaserDotDetector::detectRedLaserDotFromGrayscaleImage(const cv::Mat& inputImage, const cv::Ptr<cv::aruco::CharucoBoard>& board, 
    const std::string & resultsFname, Observation::LaserDotObservationPoints& laserDotObservationPoints, const float & intensityThreshold)
{
    std::vector<cv::Vec3f> circlesBoard;

    //std::cout << "Channels in image = " << inputImage.channels() << std::endl;
    cv::Mat inputImageSingleChannel = inputImage.clone();
    if (inputImage.channels() > 1)
    {
        std::cout << "Converting to single-channel." << std::endl;
        cv::cvtColor(inputImage, inputImageSingleChannel, cv::COLOR_BGR2GRAY);
    }

    // Initialize the output with the greyscale image
    laserDotObservationPoints.image = inputImageSingleChannel.clone();

    cv::Mat grayImage, grayImageProcessed, grayImageCopy;
    std::vector< cv::Vec3f> circlesFound;

#if defined(DEBUG) && defined(VIZCIRCLES)
    inputImageSingleChannel.copyTo(grayImageCopy);
#endif
    preProcessImageForRedDot(inputImageSingleChannel, grayImageProcessed);

    getHoughCircles(grayImageProcessed, circlesFound);

    int numOfCircles = (int)circlesFound.size();

    std::cout << "Number of circles found = " << numOfCircles << std::endl;

    float numRows = (float)grayImageProcessed.rows;
    float numCols = (float)grayImageProcessed.cols;

    float minMeanIntensity = 99999999999.0f, maxMeanIntensity = 0.0;
    std::vector<float> meanIntensities;
    for (int i = 0; i < numOfCircles; i++)
    {
        cv::Vec3f circle = circlesFound[i];
        float radius = circle[2];
        cv::Mat roi = grayImageProcessed(cv::Range((int)cv::max(0.0f, circle[1] - radius / 2), (int)cv::min(numRows, circle[1] + radius / 2 + 1)),
        cv::Range((int)cv::max(0.0f, circle[0] - radius / 2), (int)cv::min(numCols, circle[0] + radius / 2 + 1)));
        cv::Mat1b mask(roi.rows, roi.cols);
        float mean = (float)cv::mean(roi, mask)[0];

        if (mean > intensityThreshold) meanIntensities.push_back(mean);
        if (mean > maxMeanIntensity) maxMeanIntensity = mean;
        if (mean < minMeanIntensity) minMeanIntensity = mean;
        if (mean > intensityThreshold)
        {
            cv::Vec3f cir(circle[0], circle[1], circle[2]);
            //cir[0] = circle[0]; cir[1] = circle[1]; cir[2] = circle[2]; 
            circlesBoard.push_back(cir);
//#if defined(DEBUG) && defined(VIZCIRCLES)
//            cv::circle(grayImageCopy, cv::Point(cir[0], cir[1]), cir[2], cv::Scalar(0, 0, 255), 6, cv::LINE_AA);
//            cv::circle(grayImageCopy, cv::Point(cir[0], cir[1]), 25, cv::Scalar(0, 255, 0), 6, cv::LINE_AA);
//#endif
        }

    }

    sort(meanIntensities.begin(), meanIntensities.end());
    int numberOfCirclesRetained = (int)circlesBoard.size();
    std::cout << "Number of circles retained = " << numberOfCirclesRetained << std::endl;
    if (numberOfCirclesRetained == 0)
        throw std::runtime_error("Failed to detect any candidate laser dots");

    //for (int i = 0; i < cv::min(25, numberOfCirclesRetained); i++)
    //{
    //    std::cout << i << ": element from top = " << meanIntensities[numberOfCirclesRetained - i - 1] << std::endl;
    //}

    std::cout << "Max and min MeanIntensities = " << maxMeanIntensity << ", " << minMeanIntensity << std::endl;

#pragma region Filter out circles that are not on the CharucoBoard
    std::vector<float> extremeCoords;
	CalibrationObject::ArucoBoard::getCharucoBoardExtremePoints(inputImageSingleChannel, board, extremeCoords);
    if (extremeCoords.size() == 0)
        throw std::runtime_error("LaserDotDetector: Failed to detect aruco board");

    float minX = extremeCoords[0]; float maxX = extremeCoords[1];
    float minY = extremeCoords[2]; float maxY = extremeCoords[3];
    std::cout << "== MinX, MaxX, MinY, and MaxY = " << minX << ", " << maxX << ", " << minY << ", " << maxY << std::endl;
    int counter = 0;
    while (counter < circlesBoard.size() - 1)
    {
        float cirX = circlesBoard[counter][0]; float cirY = circlesBoard[counter][1];
        if ((cirX < minX || cirX > maxX) || (cirY < minY || cirY > maxY))
        {
            //std::cout << "== Popping out a circle (x,y): (" << cirX << ", " << cirY << ")..." << std::endl;
            circlesBoard.erase(circlesBoard.begin() + counter);
        }
        else
        {
            counter++;
        }
    }
    std::cout << "Presenting user with " << circlesBoard.size() << " candidate laser dot detections" << std::endl;
#pragma endregion

    std::vector<cv::Point2f> initialDetections;

    for (int i = 0; i < circlesBoard.size(); i++)
    {
        cv::Vec3f circle = circlesBoard[i];
        initialDetections.push_back(cv::Point2f(circle[0], circle[1]));
    }

    if (resultsFname != "")
        cv::imwrite(resultsFname, grayImageProcessed);

    if (initialDetections.size() == 0)
        throw std::runtime_error("Failed to detect laser dot");

#ifdef VIZCIRCLES
    try
    {
        // Add the user-selected true detection to the output object
        this->selectDotCircle(initialDetections, laserDotObservationPoints);
    }
    catch (...)
    {
        throw std::runtime_error("User selection of laser dot was cancelled");
    }
#endif
}

void  BoeingMetrology::Calibration::LaserDotDetector::selectDotCircle(const std::vector<cv::Point2f> & initialDetections, Observation::LaserDotObservationPoints& laserDotObservationPoints) 
{
    cv::Mat imgBGR;
    MouseCallbackParams mcb;
    mcb.circleRadius = 100;
    mcb.scaleX = 1.0; mcb.scaleY = 1.0;
    mcb.circlesBoard = initialDetections;
    mcb.selIndx = -1;
    mcb.pixelCoord = cv::Point2i(0, 0);

    cv::Mat image = laserDotObservationPoints.image.clone();

    if (image.channels() > 1) {
        imgBGR = image.clone();
    }
    else { cv::cvtColor(image, imgBGR, cv::COLOR_GRAY2BGR); } // Plot colorful circles for greater user visibility

    for (int i = 0; i < mcb.circlesBoard.size(); i++) {
        circle(imgBGR, mcb.circlesBoard[i], (int)(mcb.circleRadius + 0.5), cv::Scalar(0, 255, 0), 12, cv::LINE_AA);
    }
    if (mcb.scaleX < 1.0 || mcb.scaleY < 1.0) {
        resize(imgBGR, imgBGR, cv::Size(0, 0), mcb.scaleX, mcb.scaleY, cv::INTER_LINEAR);
    }

    const std::string windowName = "**** Click the Correct Circle ****";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::setMouseCallback(windowName, this->selectDotCircleCallback, (void*)&mcb);
    imshow(windowName, imgBGR);

    cv::waitKey(0);
    cv::destroyAllWindows();

    laserDotObservationPoints.positions.clear();
    if (mcb.selIndx != -1)
        laserDotObservationPoints.positions.push_back(mcb.circlesBoard[mcb.selIndx]); // User selected an existing circle
    else if (mcb.pixelCoord != cv::Point2i(0, 0))
        laserDotObservationPoints.positions.push_back(cv::Point2f(mcb.pixelCoord)); // User selected a new coordinate
    else
        throw std::runtime_error("Failed to detect laser dot");
}

void  BoeingMetrology::Calibration::LaserDotDetector::selectDotCircleCallback(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONUP) {
        MouseCallbackParams* mcb = (MouseCallbackParams*)userdata;
        mcb->pixelCoord = cv::Point2i(x, y);
        float cx, cy, r2 = mcb->circleRadius*mcb->circleRadius, d2;  // circle center X, circle center Y, test radius squared, distance of click to circle center squared
        for (int i = 0; i < mcb->circlesBoard.size(); i++)
        {
            cv::Point2f circle = mcb->circlesBoard[i];
            cx = mcb->scaleX*circle.x; cy = mcb->scaleY*circle.y;
            d2 = (x - cx)*(x - cx) + (y - cy)*(y - cy);
            if (d2 <= r2) {
                // The user click inside one of the circles
                std::cout << "Selected circle at (" << x << " ," << y << "); index = " << i << std::endl;
                mcb->selIndx = i;
            }
        }
    }
}

void BoeingMetrology::Calibration::LaserDotDetector::BackprojectLaserDotsTo3D(const Observation::LaserDotObservationPoints & laserDotObservationPoints, const IntrinsicData & intrinsicData, 
    const std::vector<cv::Point3f> & cameraObjectPoints, const std::vector<cv::Point2f> & cameraImagePoints, std::map<POSE_NAME, std::pair<cv::Point2f, cv::Point3f>> & laserDot3dCoords)
{
    // Find the transformation between camera and object space
    cv::Mat rotationVector, boardToCameraTrans(3, 1, CV_64F);
    solvePnP(cameraObjectPoints, cameraImagePoints, intrinsicData.cameraMatrix, intrinsicData.distortionCoeffs, rotationVector, boardToCameraTrans);

    // Unwrap the compact rotation matrix format
    cv::Mat boardToCameraRot(3, 3, CV_64F);
    Rodrigues(rotationVector, boardToCameraRot);

    for (const auto & detection2d : laserDotObservationPoints.positions)
    {
        cv::Point3d backprojected = intrinsicData.BackProjectOntoCalibrationBoard(boardToCameraRot, boardToCameraTrans, detection2d, false);

        // Update output
        laserDot3dCoords[laserDotObservationPoints.poseName] = { detection2d, backprojected };
    } // End loop through 2d detections
}

void BoeingMetrology::Calibration::LaserDotDetector::BestFit3dLine(const std::vector<cv::Point3d> & pts, std::pair<cv::Point3d, cv::Point3d> & meanDir)
{
    // Pack the points into an Nx3 matrix and compute the mean of each column.
    cv::Mat lhs = cv::Mat::zeros((int)pts.size(), 3, CV_64FC1);
    int row = 0;
    cv::Point3d mean(0.0, 0.0, 0.0);
    for (const auto & pt : pts)
    {
        lhs.at<double>(row, 0) = pt.x;
        lhs.at<double>(row, 1) = pt.y;
        lhs.at<double>(row, 2) = pt.z;
        row++;

        mean += pt;
    }
    mean /= (double)pts.size();

    // Shift all data to be centered about the mean
    for (int r = 0; r < lhs.rows; r++)
    {
        lhs.at<double>(r, 0) -= mean.x;
        lhs.at<double>(r, 1) -= mean.y;
        lhs.at<double>(r, 2) -= mean.z;
    }

    // w: calculated singular values
    // u: calculated left singular values
    // vt: transposed matrix of right singular values (we use the top row)
    cv::SVD s(lhs);
    cv::Mat w, u, vt;
    s.compute(lhs, w, u, vt);

    // Store the normalized result
    cv::Point3d unnormResult(vt.at<double>(0, 0), vt.at<double>(0, 1), vt.at<double>(0, 2));
    meanDir = { mean, unnormResult /= cv::norm(unnormResult) };
}

double BoeingMetrology::Calibration::LaserDotDetector::BestFit3dLineAndEstimateOrigin(const std::pair < std::vector<cv::Point3d>, std::vector<double>> &pts, std::pair<cv::Point3d, cv::Point3d> & posDir)
{
    // Best fit a line.  Get the mean position through which the line passes and the directional unit vector.
    std::pair<cv::Point3d, cv::Point3d> meanDir;
    LaserDotDetector::BestFit3dLine(pts.first, meanDir);

    // Enforce a positive z-component.  Here we assume the Acuity and camera generally point along +z.
    if (meanDir.second.z < 0.0)
        meanDir.second *= -1.0;

    // Initialize error
    double error = 0.0;

    // Initialize the shifted origin estimate
    cv::Point3d originEstimateShifted(0.0, 0.0, 0.0);

    // Loop through points
    for (size_t idx = 0; idx < pts.first.size(); idx++)
    {
        // Shift the point by the mean, through which our line will pass
        cv::Point3d ptshifted = pts.first[idx] - meanDir.first;

        // Project this point onto the directional unit vector
        double projDist = meanDir.second.ddot(ptshifted);
        cv::Point3d projectedPosition = meanDir.second * projDist;

        // Compute the residual
        double residual = cv::norm(projectedPosition - ptshifted);
        error += residual;

        // Apply the depth value provided along the best-fit direction from the projected point back to 
        // estimate the depth value origin
        originEstimateShifted += projectedPosition - meanDir.second * pts.second[idx];
    }

    // Finish shifted origin estimate
    originEstimateShifted /= (double)pts.first.size();

    // The origin is shifted to be mean-centered.  We need to undo this.
    cv::Point3d originEstimate = originEstimateShifted + meanDir.first;

    // Finalize output
    posDir = { originEstimate, meanDir.second };

    // Return average residual of best-fit direction process
    error /= (double)pts.first.size();
    return error;
}


void BoeingMetrology::Calibration::LaserDotDetector::JsonSerialize(Json::Value &jsonNode) const
{

}

void BoeingMetrology::Calibration::LaserDotDetector::JsonDeserialize(const Json::Value &jsonNode)
{

}

void BoeingMetrology::Calibration::LaserDotDetector::preProcessImageForRedDot(const cv::Mat& inputImage, cv::Mat& outputImage)
{
	dilate(inputImage, outputImage, cv::Mat());
	threshold(outputImage, outputImage, 200, 255, CV_THRESH_BINARY);
}

void BoeingMetrology::Calibration::LaserDotDetector::getHoughCircles(const cv::Mat& inputImage, std::vector<cv::Vec3f>& circles)
{
	double dpParam = 1;
	double minDist = 200;
	double upperThreshCanny = 200;
	double threshForCenter = 5;
	int minRadius = 10;
	int maxRadius = 300;
	HoughCircles(inputImage, circles, cv::HOUGH_GRADIENT, dpParam, minDist, upperThreshCanny, threshForCenter, minRadius, maxRadius);
	//cout << "Number of centers detected = " << circles.size()<<endl;
}



