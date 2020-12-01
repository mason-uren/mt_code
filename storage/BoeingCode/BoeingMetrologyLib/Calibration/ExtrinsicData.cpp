#include "ExtrinsicData.h"
#include "json/reader.h"
#include <fstream>
#include "Utilities/Utilities.h"
#include <opencv2/opencv.hpp>


namespace BoeingMetrology
{
namespace Calibration
{

	//const int ExtrinsicData::serializedSize = 12;

	ExtrinsicData::ExtrinsicData(const std::string & name, const std::string & refCameraName, const std::string & poseDataPath)
	{
    this->name = name;
    this->refCameraName = refCameraName;
    this->poseDataPath = poseDataPath;
}

	void ExtrinsicData::JsonDeserialize(const Json::Value &jsonNode)
{
    try
    {
        this->timestamp = jsonNode["timestamp"].asString();
        if (jsonNode["name"].asString() != "")
        {
            // Standard format
            this->name = jsonNode["name"].asString();
            this->refCameraName = jsonNode["refCameraName"].asString();
            this->transform = cv::Matx34d::zeros();
            unsigned int idx = 0;
            if (jsonNode["transform"][0].isString())
            {
                std::stringstream ss;
                ss << "ERROR: ExtrinsicData::JsonDeserialize passed JSON that has the transform saved as strings not numbers." << std::endl << jsonNode.toStyledString();
                throw std::runtime_error(ss.str());
            }
			for (int r = 0; r < transform.rows; r++)
			{
				for (int c = 0; c < transform.cols; c++)
				{
					transform(r, c) = jsonNode["transform"][idx].asDouble();
					idx++;
				}
			}
		}
		else if (jsonNode["Name"].asString() != "")
		{
			// Legacy format
			this->name = jsonNode["Name"].asString();
			this->refCameraName = jsonNode["ReferenceCameraName"].asString();
			this->transform = cv::Matx34d::zeros();
			unsigned int idx = 0;
			for (int r = 0; r < transform.rows; r++)
			{
				for (int c = 0; c < transform.cols; c++)
				{
					transform(r, c) = jsonNode["CameraMatrix"][idx].asDouble();
					idx++;
				}
			}
		}
		else
			throw std::runtime_error("ExtrinsicData::JsonDeserialize() : Unrecognized json format");
        if (jsonNode["poseDataPath"].asString() != "")
        {
            this->poseDataPath = jsonNode["poseDataPath"].asString();
        }
        else
        {
            this->poseDataPath = "";
        }

        this->rmsError = jsonNode["rmsError"].isNull() ? 0.0 : jsonNode["rmsError"].asDouble();

        this->allignmentError = jsonNode["allignmentError"].isNull() ? 0.0 : jsonNode["alignmentError"].asDouble();

        this->intrinsicsTimestamp = jsonNode["intrinsicsTimestamp"].isNull() ? "" : jsonNode["intrinsicsTimestamp"].asString();

    }
    catch (...)
    {
        std::cout << "ERROR: Failed to parse JSON: " << jsonNode.toStyledString();
        throw;
    }

}


	void ExtrinsicData::SetRotationMatrix(const cv::Mat & rotMat)
{
    if (rotMat.rows < 3 || rotMat.cols < 3)
        throw "ExtrinsicData::SetRotationMatrix: input number of rows and columns must be at least 3 ";

    cv::Mat newRotMat = rotMat.clone();
    if (newRotMat.depth() == CV_32F)
        newRotMat.convertTo(newRotMat, CV_64F);
    else if (newRotMat.depth() != CV_64F)
        throw "ExtrinsicData::SetRotationMatrix: input matrix must be of type CV_32FC1 or CV_64FC1";
    if (cv::norm(cv::Point3d(newRotMat.at<double>(0, 0), newRotMat.at<double>(1, 0), newRotMat.at<double>(2, 0)).cross(cv::Point3d(newRotMat.at<double>(0, 1), newRotMat.at<double>(1, 1), newRotMat.at<double>(2, 1)))
        - cv::Point3d(newRotMat.at<double>(0, 2), newRotMat.at<double>(1, 2), newRotMat.at<double>(2, 2))) > 1e-5)
        throw "ExtrinsicData::SetRotationMatrix: invalid rotation matrix -- columns are not orthonormal";

    try
    {
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                transform(r, c) = newRotMat.at<double>(r, c);
    }
    catch (...)
    {
        throw;
    }
}

	void ExtrinsicData::SetTranslationVector(const cv::Mat & transMat)
{
    if (!(transMat.cols == 3 && transMat.rows == 1) && !(transMat.cols == 1 && transMat.rows == 3))
        throw "ExtrinsicData::SetTranslationVector: input must be 3x1 or 1x3";

    cv::Mat newtransMat = transMat.clone();
    if (newtransMat.depth() == CV_32F)
        newtransMat.convertTo(newtransMat, CV_64F);
    else if (newtransMat.depth() != CV_64F)
        throw "ExtrinsicData::SetTranslationVector: input matrix must be of type CV_32FC1 or CV_64FC1";

    try
    {
        int idx = 0;
        for (int r = 0; r < newtransMat.rows; r++)
            for (int c = 0; c < newtransMat.cols; c++)
            {
                transform(idx, 3) = newtransMat.at<double>(r, c);
                idx++;
            }
    }
    catch (...)
    {
        throw;
    }
}

	void ExtrinsicData::SetTranslationVector(const cv::Point3f & transVec)
{
    try
    {
        transform(0, 3) = transVec.x;
        transform(1, 3) = transVec.y;
        transform(2, 3) = transVec.z;

        if (std::isnan(transform(0, 3)))
            throw std::runtime_error("ExtrinsicData::SetTranslationVector() called with nan inputs");
    }
    catch (...)
    {
        throw;
    }
}

	void ExtrinsicData::SetTranslationVector(const cv::Vec3f & transVec)
{
    try
    {
        transform(0, 3) = transVec[0];
        transform(1, 3) = transVec[1];
        transform(2, 3) = transVec[2];

        if (std::isnan(transform(0, 3)))
            throw std::runtime_error("ExtrinsicData::SetTranslationVector() called with nan inputs");
    }
    catch (...)
    {
        throw;
    }
}

	cv::Mat ExtrinsicData::GetRotationMatrix() const
{
    cv::Mat output(3, 3, CV_64FC1);
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            output.at<double>(r, c) = transform(r, c);

    return output;
}

	cv::Mat ExtrinsicData::GetTranslationVector() const
{
    if (std::isnan(transform(0, 3)))
        throw std::runtime_error("ExtrinsicData::GetTranslationVector(): " + this->name + " transform is uninitialized");

    cv::Mat output(3, 1, CV_64FC1);
    output.at<double>(0, 0) = transform(0, 3);
    output.at<double>(1, 0) = transform(1, 3);
    output.at<double>(2, 0) = transform(2, 3);

    return output;
}

	cv::Point3d ExtrinsicData::GetOpticalAxisVector() const
{
    // Return the z-component of the camera specified in the reference system
    cv::Point3d opticalAxis;
    opticalAxis.x = this->transform(0, 2);
    opticalAxis.y = this->transform(1, 2);
    opticalAxis.z = this->transform(2, 2);
    return opticalAxis;
}

	std::vector<double> ExtrinsicData::GetRodriguesVector() const
{
	cv::Mat rotationMatrix = GetRotationMatrix();

	//before we pass the matrix over to ceres we need to left rotate on y to fix world/camera frames
	//rotationMatrix = yRotation * rotationMatrix;
	cv::Mat rodriguesMatrix = cv::Mat(3, 1, CV_64FC1);
	cv::Rodrigues(rotationMatrix, rodriguesMatrix);
	double *R = rodriguesMatrix.ptr<double>();

	cv::Mat translationMatrix = GetTranslationVector();
	translationMatrix.convertTo(translationMatrix, CV_64FC1);
	double *T = translationMatrix.ptr<double>();

    std::vector<double> result = std::vector<double>(6, 0);
    result[0] = R[0];
    result[1] = R[1];
    result[2] = R[2];

    result[3] = T[0];
    result[4] = T[1];
    result[5] = T[2];
    return result;
}

	cv::Point3d ExtrinsicData::TransformFromThisCameraToWorldFrame(const cv::Point3d & ptInCameraFrame) const
{
    cv::Mat src(3, 1, CV_64FC1);
    src.at<double>(0, 0) = ptInCameraFrame.x;
    src.at<double>(1, 0) = ptInCameraFrame.y;
    src.at<double>(2, 0) = ptInCameraFrame.z;

    // Transform the point from this camera frame to the world frame
    cv::Mat result = this->GetRotationMatrix() * src + this->GetTranslationVector();
    return cv::Point3d(result.at<double>(0, 0), result.at<double>(1, 0), result.at<double>(2, 0));
}

	cv::Point3d ExtrinsicData::TransformFromThisCameraToWorldFrameInverse(const cv::Point3d & ptInCameraFrame) const
{
    cv::Mat src(3, 1, CV_64FC1);
    src.at<double>(0, 0) = ptInCameraFrame.x;
    src.at<double>(1, 0) = ptInCameraFrame.y;
    src.at<double>(2, 0) = ptInCameraFrame.z;

    // Transform the point from this camera frame to the world frame
    cv::Mat result = this->GetRotationMatrix().t() * (src - this->GetTranslationVector());
    return cv::Point3d(result.at<double>(0, 0), result.at<double>(1, 0), result.at<double>(2, 0));
}


	std::string ExtrinsicData::GetCameraName() const
{
    return this->name;
}

	std::string ExtrinsicData::GetTimestamp() const
{
    return this->timestamp;
}

	void ExtrinsicData::JsonSerialize(Json::Value &jsonNode) const
{

    jsonNode["name"] = this->name;
    jsonNode["timestamp"] = this->timestamp;
    jsonNode["refCameraName"] = this->refCameraName;
    Json::Value transformJson;
    unsigned int idx = 0;
    for (int r = 0; r < transform.rows; r++)
    {
        for (int c = 0; c < transform.cols; c++)
        {
            transformJson[idx] = transform(r, c);
            idx++;
        }
    }
    jsonNode["transform"] = transformJson;

    jsonNode["poseDataPath"] = this->poseDataPath;

    jsonNode["rmsError"] = this->rmsError;

    jsonNode["allignmentError"] = this->allignmentError;

    jsonNode["intrinsicsTimestamp"] = this->intrinsicsTimestamp;
}


}//namespace Calibration
}//namespace BoeingMetrology
