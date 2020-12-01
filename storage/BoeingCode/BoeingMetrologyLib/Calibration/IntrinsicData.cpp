#include "IntrinsicData.h"
#include "json/reader.h"
#include <fstream>
#include <iostream>
#include "Utilities/Utilities.h"
#include <opencv2/imgproc.hpp>

namespace BoeingMetrology
{
namespace Calibration
{

	//const int IntrinsicData::serializedSize = 13;

	/*IntrinsicData::IntrinsicData()
	{
		cameraMatrix = cv::Matx33d::zeros();
		cameraMatrix(2, 2) = 1;
		distortionCoeffs = cv::Mat(5, 1, CV_64FC1);
		posesBeforeAndAfterFiltering = { 0, 0 };
		poseDataPath = "";
		pixelSize = std::make_pair(0, 0);
		imageSize = std::make_pair(0, 0);
	}*/

	IntrinsicData::IntrinsicData(Json::Value aCalibrationTarget)
	:calibrationTargetJson(aCalibrationTarget)
{
		cameraMatrix = cv::Matx33d::zeros();
		cameraMatrix(2, 2) = 1;
		distortionCoeffs = cv::Mat::zeros(5, 1, CV_64FC1);
}

	IntrinsicData::IntrinsicData(const std::string & name, const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeffs)
{
    this->name = name;
    this->cameraMatrix = cameraMatrix;
    this->distortionCoeffs = distortionCoeffs;
	this->posesBeforeAndAfterFiltering = { 0, 0 };
	this->poseDataPath = "";
}

	IntrinsicData IntrinsicData::Clone() const
{
	IntrinsicData dst = IntrinsicData(this->calibrationTargetJson);
	dst.name = this->name;
	dst.rmsError = this->rmsError;
	dst.posesBeforeAndAfterFiltering = this->posesBeforeAndAfterFiltering;
	dst.distortionCoeffs = this->distortionCoeffs.clone();
	for (int r = 0; r < this->cameraMatrix.rows; r++)
		for (int c = 0; c < this->cameraMatrix.cols; c++)
			dst.cameraMatrix(r, c) = this->cameraMatrix(r, c);
	dst.pixelSize = this->pixelSize;
	dst.imageSize = this->imageSize;
    dst.poseDataPath = this->poseDataPath;
	return dst;
}

	cv::Point2f IntrinsicData::GetPrincipalPoint() const
{
    return cv::Point2f((float)this->cameraMatrix(0, 2), (float)this->cameraMatrix(1, 2));
}

	IntrinsicData IntrinsicData::AverageIntrinsicDatas(const IntrinsicData& rhs, const double & weight /*= 0.5*/) const
{
    // Initialize output to this class.  A weight of 1.0 gives you back this class.  
    IntrinsicData result = this->Clone();
    result.cameraMatrix = weight * result.cameraMatrix + (1.0 - weight) * rhs.cameraMatrix;
    result.distortionCoeffs = weight * result.distortionCoeffs + (1.0 - weight) * rhs.distortionCoeffs;
    result.rmsError = 0.0;
    return result;
}

	double IntrinsicData::GetFocalLengthInPhysicalUnits() const
{
    if (this->pixelSize.first == 0 || this->pixelSize.second == 0)
        throw std::runtime_error(std::string("IntrinsicData::GetFocalLengthInPhysicalUnits() failed.  Pixel size is zero! - ") + name);

    return (0.5 * (this->cameraMatrix(0, 0) * this->pixelSize.first + this->cameraMatrix(1, 1) * this->pixelSize.second));
}

	void IntrinsicData::AppendPoseDataPath(const std::string & newPath)
{
    if (this->poseDataPath == "")
        this->poseDataPath = newPath;
    else
        this->poseDataPath = this->poseDataPath + ", " + newPath;
}

	cv::Point3d IntrinsicData::BackProjectOntoCalibrationBoard(const cv::Mat & boardToCameraRot, const cv::Mat & boardToCameraTrans,
    const cv::Point2f & pixelCoord, const bool & undistort /*= false*/) const
{
    // Optionally undistort the pixel
    cv::Point2f undistortedPixelCoord = pixelCoord;
    if (undistort)
    {
        std::vector<cv::Point2f> pixelCoordVec{ pixelCoord }, undistortedPixelCoordVec;
        cv::undistortPoints(pixelCoordVec, undistortedPixelCoordVec, this->cameraMatrix, this->distortionCoeffs, cv::noArray(), this->cameraMatrix);
        undistortedPixelCoord = undistortedPixelCoordVec.front();
    }

    // The ray origin is the pinhole
    cv::Point3d origin(0.0, 0.0, 0.0);

    // The ray intersecting the virtual image plane.  Units are pixels.
    cv::Point3d uvCoordInPixels(undistortedPixelCoord.x - this->cameraMatrix(0, 2),
        undistortedPixelCoord.y - this->cameraMatrix(1, 2),
        0.5 * (this->cameraMatrix(0, 0) + this->cameraMatrix(1, 1)));

    // Normalized ray direction
    cv::Point3d ray = uvCoordInPixels - origin;
    ray /= cv::norm(ray);

    // We will cast the ray onto the calibration board plane defined with a point and a normal
    cv::Point3d boardZspecifiedInCameraFrame(boardToCameraRot.at<double>(0, 2), boardToCameraRot.at<double>(1, 2), boardToCameraRot.at<double>(2, 2));
    cv::Point3d knownPointOnBoardSpecifiedInCameraFrame(boardToCameraTrans.at<double>(0, 0), boardToCameraTrans.at<double>(1, 0), boardToCameraTrans.at<double>(2, 0));

    // Cast the ray onto the calibration board, units are in meters
    double denom = boardZspecifiedInCameraFrame.ddot(ray);
    cv::Point3d backprojected(0.0, 0.0, 0.0);
    if (std::abs(denom) > 0.0001)
    {
        double t = (knownPointOnBoardSpecifiedInCameraFrame - origin).ddot(boardZspecifiedInCameraFrame) / denom;
        backprojected = origin + ray * t;
    }
    return backprojected;
}

	void IntrinsicData::JsonDeserialize(const Json::Value &jsonNode)
{

	try
	{
		// Name
		this->name = jsonNode["name"].asString();

        // Timestamp
        this->timestamp = jsonNode["timestamp"].asString();

		// RMS error
		this->rmsError = jsonNode["rmsError"].asDouble();

		// Focal length and principal point
		unsigned int idx = 0;
		for (int r = 0; r < this->cameraMatrix.rows; r++)
		{
			for (int c = 0; c < this->cameraMatrix.cols; c++)
			{
				this->cameraMatrix(r, c) = jsonNode["cameraMatrix"][idx].asDouble();
				idx++;
			}
		}

		// 5x1 distortion coefficients
		this->distortionCoeffs = cv::Mat(5, 1, CV_64F);
		idx = 0;
		for (int r = 0; r < this->distortionCoeffs.rows; r++)
		{
			for (int c = 0; c < this->distortionCoeffs.cols; c++)
			{
				this->distortionCoeffs.at<double>(r, c) = jsonNode["distortionCoeffs"][idx].asDouble();
				idx++;
			}
		}

		// Pixel size
		this->pixelSize = { jsonNode["pixelSize"][0U].asDouble(), jsonNode["pixelSize"][1].asDouble() };

		// Image size
        this->imageSize = { jsonNode["imageSize"][0U].asInt(), jsonNode["imageSize"][1].asInt() };

		//Poses before and after filtering posedata
			this->posesBeforeAndAfterFiltering = jsonNode["poseCounts"].isNull() ? std::make_pair(0,0) : std::make_pair(jsonNode["poseCounts"][0U].asInt(), jsonNode["poseCounts"][1].asInt());

        this->poseDataPath = jsonNode["poseDataPath"].asString();
	}
	catch (...)
	{
        std::cout << "ERROR: Failed to parse JSON: " << jsonNode.toStyledString();
		throw;
	}

}

	void IntrinsicData::JsonSerialize(Json::Value &jsonNode) const
{
	// Name
	jsonNode["name"] = this->name;

    // Timestamp 
    jsonNode["timestamp"] = this->timestamp;

	// RMS Error
	jsonNode["rmsError"] = this->rmsError;

	// Camera matrix (focal length and principal point)
	for (int r = 0; r < this->cameraMatrix.rows; r++)
	{
		for (int c = 0; c < this->cameraMatrix.cols; c++)
		{
			jsonNode["cameraMatrix"].append((double)this->cameraMatrix(r, c));
		}
	}

	// Distortion coefficients
	for (int r = 0; r < this->distortionCoeffs.rows; r++)
	{
		for (int c = 0; c < this->distortionCoeffs.cols; c++)
		{
			jsonNode["distortionCoeffs"].append((double)this->distortionCoeffs.at<double>(r, c));
		}
	}

	// Pixel size
	jsonNode["pixelSize"].append(this->pixelSize.first);
	jsonNode["pixelSize"].append(this->pixelSize.second);

	// Image size
	jsonNode["imageSize"].append(this->imageSize.first);
    jsonNode["imageSize"].append(this->imageSize.second);

	jsonNode["poseCounts"].append(this->posesBeforeAndAfterFiltering.first);
	jsonNode["poseCounts"].append(this->posesBeforeAndAfterFiltering.second);
    
    jsonNode["poseDataPath"] = this->poseDataPath;

    if (this->calibrationTargetJson != Json::ValueType::nullValue)
    	jsonNode["CalibrationTarget"] = this->calibrationTargetJson;
}

	cv::Point2f IntrinsicData::UndistortPixel(cv::Point2f srcPixel)
{
	// Undistort the pixel 
	// Row based
	float distortedX = srcPixel.x;
	float distortedY = srcPixel.y;

	
	double *k = distortionCoeffs.ptr<double>(0);
	

	double fx, fy, ifx, ify, cx, cy;
	int iters = 1;
	/*cv::Mat distMat = intrinsics.distortionCoeffs;*/


	iters = 5;
	//cv::Matx33d camMat = intrinsics.cameraMatrix;
	fx = cameraMatrix(0, 0);
	fy = cameraMatrix(1, 1);
	ifx = 1.0 / fx;
	ify = 1.0 / fy;
	cx = cameraMatrix(0, 2);
	cy = cameraMatrix(1, 2);


	double x, y, x0, y0;

	x = distortedX;
	y = distortedY;
	x0 = (x - cx)*ifx;
	x = x0;
	y0 = (y - cy)*ify;
	y = y0;

	for (int jj = 0; jj < iters; jj++)
	{
		double r2 = x*x + y*y;
		double icdist = 1. / (1 + ((k[4] * r2 + k[1])*r2 + k[0])*r2);
		double deltaX = 2 * k[2] * x*y + k[3] * (r2 + 2 * x*x);
		double deltaY = k[2] * (r2 + 2 * y*y) + 2 * k[3] * x*y;
		x = (x0 - deltaX)*icdist;
		y = (y0 - deltaY)*icdist;
	}

	return cv::Point2f((float)(x*fx + cx), (float)(y*fy + cy));
}

	cv::Vec3d IntrinsicData::ProjectPixel(cv::Point2f pixel)
	{
		cv::Point2f undistorted = UndistortPixel(pixel);

		cv::Vec3d pixelRay = cv::Vec3d();
		pixelRay(0) = (cx() - undistorted.x) / fx();
		pixelRay(1) = (undistorted.y - cy()) / fy();
		pixelRay(2) = 1;

		cv::normalize(pixelRay);
		
		return pixelRay;
	}

	std::string IntrinsicData::GetCameraName() const
{
    return this->name;
}

	std::string IntrinsicData::GetTimestamp() const
{
    return this->timestamp;
}

}//namespace Calibration
}//namespace BoeingMetrology
