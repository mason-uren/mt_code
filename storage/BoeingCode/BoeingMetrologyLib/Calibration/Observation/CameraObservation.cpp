#include "CameraObservation.h"

namespace BoeingMetrology
{
namespace Calibration
{
namespace Observation
{

	void CameraObservation::JsonSerialize(Json::Value &jsonNode) const
	{
		jsonNode["rms"] = rms;
		jsonNode["fileName"] = fileNameOfObservation;

		Json::Value transformArray(Json::arrayValue);
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				int pos = j * 3 + i;
				transformArray[pos] = transform(i, j);
			}
		}
		jsonNode["transform"] = transformArray;

		ObservationPoints<float>::JsonSerialize(jsonNode);
	}

	void CameraObservation::JsonDeserialize(const Json::Value &jsonValue)
	{
		try
		{
			rms = jsonValue["rms"].asDouble();
			fileNameOfObservation = jsonValue["fileName"].asString();
			Json::Value transformArray = jsonValue["transform"];
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 4; ++j)
				{
					int pos = j * 3 + i;
					transform(i, j) = transformArray[pos].asDouble();
				}
			}

			ObservationPoints<float>::JsonDeserialize(jsonValue);
		}
		catch (...)
		{
			throw;
		}
	}

	void CameraObservation::SetRotationMatrix(const cv::Mat & rotMat)
	{
		if (rotMat.rows < 3 || rotMat.cols < 3)
			throw "CameraObservation::SetRotationMatrix: input number of rows and columns must be at least 3 ";

		cv::Mat newRotMat = rotMat.clone();
		if (newRotMat.depth() == CV_32F)
			newRotMat.convertTo(newRotMat, CV_64F);
		else if (newRotMat.depth() != CV_64F)
			throw "ExtrinsicData::SetRotationMatrix: input matrix must be of type CV_32FC1 or CV_64FC1";
		if (cv::norm(cv::Point3d(newRotMat.at<double>(0, 0), newRotMat.at<double>(1, 0), newRotMat.at<double>(2, 0)).cross(cv::Point3d(newRotMat.at<double>(0, 1), newRotMat.at<double>(1, 1), newRotMat.at<double>(2, 1)))
			- cv::Point3d(newRotMat.at<double>(0, 2), newRotMat.at<double>(1, 2), newRotMat.at<double>(2, 2))) > 1e-5)
			throw "CameraObservation::SetRotationMatrix: invalid rotation matrix -- columns are not orthonormal";

		try
		{
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
					this->transform(r, c) = newRotMat.at<double>(r, c);
		}
		catch (...)
		{
			throw;
		}
	}

	void CameraObservation::SetTranslationVector(const cv::Mat & transMat)
	{
		if (!(transMat.cols == 3 && transMat.rows == 1) && !(transMat.cols == 1 && transMat.rows == 3))
			throw "CameraObservation::SetTranslationVector: input must be 3x1 or 1x3";

		cv::Mat newtransMat = transMat.clone();
		if (newtransMat.depth() == CV_32F)
			newtransMat.convertTo(newtransMat, CV_64F);
		else if (newtransMat.depth() != CV_64F)
			throw "CameraObservation::SetTranslationVector: input matrix must be of type CV_32FC1 or CV_64FC1";

		try
		{
			int idx = 0;
			for (int r = 0; r < newtransMat.rows; r++)
				for (int c = 0; c < newtransMat.cols; c++)
				{
					this->transform(idx, 3) = newtransMat.at<double>(r, c);
					idx++;
				}
		}
		catch (...)
		{
			throw;
		}
	}

	cv::Mat CameraObservation::GetRotationMatrix() const
	{
		cv::Mat output(3, 3, CV_64FC1);
		for (int r = 0; r < 3; r++)
			for (int c = 0; c < 3; c++)
				output.at<double>(r, c) = this->transform(r, c);

		return output;
	}

	cv::Mat CameraObservation::GetTranslationVector() const
	{
		if (std::isnan(this->transform(0, 3)))
			throw std::runtime_error("CameraObservation::GetTranslationVector(): transform is uninitialized");

		cv::Mat output(3, 1, CV_64FC1);
		output.at<double>(0, 0) = this->transform(0, 3);
		output.at<double>(1, 0) = this->transform(1, 3);
		output.at<double>(2, 0) = this->transform(2, 3);

		return output;
	}

	double CameraObservation::GetRangeToTarget() const
	{
		return cv::norm(this->GetTranslationVector());
	}

	double CameraObservation::Estimate3dPoseEuler(const IntrinsicData & intrinsicData,
		double & rx, double & ry, double & rz, cv::Mat & t, std::map<MARKER_IDENTIFIER, cv::Point2f> & outliers, const bool & computeRms /* = false */)
	{
		// r is rotation matrix from board to camera
		cv::Mat r;
		double rms_ = this->Estimate3dPose(intrinsicData, r, t, outliers, computeRms);

		// Store the result
		this->SetRotationMatrix(r);
		this->SetTranslationVector(t);

		// Compute Euler angles rx, ry, rz in degrees
        Boeing::Utilities::Math::Transformations::euler_from_matrix<double>(r, "rxyz", rx, ry, rz);

		return rms_;
	}

	CameraObservation CameraObservation::undistortObservations(IntrinsicData intrinsics)
	{
		CameraObservation undistorted = CameraObservation(*this);

		for (int i = 0; i < undistorted.observedPoints.size(); ++i)
		{
			cv::Point2f undistortedPixel = intrinsics.UndistortPixel(undistorted.observedPoints[i]);
			undistorted.observedPoints[i] = undistortedPixel;
		}
		return undistorted;
	}



	void CameraObservation::SetRotationTranslation(cv::Matx34d transform_)
	{
		transform = transform_;
	}

	void CameraObservation::SerializeStream(std::ostream &strm) const
	{
		// Camera info
		Json::Value root;
		JsonSerialize(root);
		Json::StyledStreamWriter json_writer;
		json_writer.write(strm, root);
	}

	void CameraObservation::DeserializeStream(std::istream &strm)
	{
		Json::Reader jsonReader;
		Json::Value root;
		//Read through the JSON file and collect the points
		if (jsonReader.parse(strm, root))
		{
			JsonDeserialize(root);
		}
		else
		{
			throw std::runtime_error("CameraObservation::DeserializeStream(): Failed to parse json.");
		}
	}

	void CameraObservation::SerializeFile(std::string fileName) const
	{
		//Write the serialization to the file
		std::ofstream strm(fileName);
		SerializeStream(strm);
		strm.close();
	}

	void CameraObservation::DeserializeFile(std::string fileName)
	{
		//Read the content from the file and de-serialize into the class
		std::ifstream strm(fileName);
		DeserializeStream(strm);
		strm.close();
	}
}//namespace Observation
}//namespace Calibration
}//namespace BoeingMetrology


