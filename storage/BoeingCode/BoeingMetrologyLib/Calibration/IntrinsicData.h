#ifndef BOEINGMETROLOGYLIB_INTRINSICDATA_H
#define BOEINGMETROLOGYLIB_INTRINSICDATA_H

#include <string>
#include <cv.h>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Calibration
	{
        class BOEINGMETROLOGYLIB_API IntrinsicData : public Boeing::Interface::Serializer
		{
		public:

            // Sensor name
			std::string name = "";

            // Timestamp YYYYMMDD
            std::string timestamp = "";

			// As specified in https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
			// [fx,  0, cx
			//   0, fy, cy
			//   0,  0,  1]
			// All units are pixels.
			cv::Matx33d cameraMatrix;

			double fx() { return cameraMatrix(0, 0); }
			double cx() { return cameraMatrix(0, 2); }
			double fy() { return cameraMatrix(1, 1); }
			double cy() { return cameraMatrix(1, 2); }


			// 5x1 matrix (CV_64FC1)
			// As specified in https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
			// k1, k2, p1, p2, k3
			// p1 and p2 are tangential distortion coefficients
			// kn values are radial distortion coefficients
			cv::Mat distortionCoeffs;

			double k1() { return distortionCoeffs.at<double>(0); }
			double k2() { return distortionCoeffs.at<double>(1); }
			double k3() { return distortionCoeffs.at<double>(4); }
			double p1() { return distortionCoeffs.at<double>(2); }
			double p2() { return distortionCoeffs.at<double>(3); }

			// Pixel size along width, height direction in meters
			std::pair<double, double> pixelSize;

			// Image width, height in pixels
			std::pair<int, int> imageSize;
            
			// The output of cv::calibrateCamera()
			double rmsError = 0.0;
			//the number of poses before and after filtering out poses by RMS in the intrinsics computation
			std::pair<int, int> posesBeforeAndAfterFiltering;

            // The path to the pose data from which the intrinsics were generated
            std::string poseDataPath = "";

			//the json for the calibration board against which these intrinsics were collected
			Json::Value calibrationTargetJson;

            IntrinsicData(const std::string & name, const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeffs);

			IntrinsicData(Json::Value aCalibrationTargetJson = Json::Value(Json::ValueType::nullValue));

			// Return a deep copy of this object
			IntrinsicData Clone() const;

			// Comparison operator (ignores rmsError deltas)
			friend bool operator==(const IntrinsicData& lhs, const IntrinsicData& rhs)
			{
				if (lhs.name != rhs.name)
					return false;
				//if (lhs.rmsError != rhs.rmsError)
				//	return false;
				if (lhs.distortionCoeffs.size() != rhs.distortionCoeffs.size())
					return false;
				for (int r = 0; r < rhs.distortionCoeffs.rows; r++)
					for (int c = 0; c < rhs.distortionCoeffs.cols; c++)
						if (lhs.distortionCoeffs.at<double>(r, c) != rhs.distortionCoeffs.at<double>(r, c))
							return false;
				for (int r = 0; r < rhs.cameraMatrix.rows; r++)
					for (int c = 0; c < rhs.cameraMatrix.cols; c++)
						if (lhs.cameraMatrix(r, c) != rhs.cameraMatrix(r, c))
							return false;

				return true;
			}

            // Get the principal point in image coordinates
            cv::Point2f GetPrincipalPoint() const;

            // Average this intrinsic data with a second intrinsic data.  An optional weight can be provided.
            // A weight of 1.0 gives you back this class (the left-hand side).  
            IntrinsicData AverageIntrinsicDatas(const IntrinsicData& rhs, const double & weight = 0.5) const;

            // Get the focal length in physical units by averaging fx and fy and scaling by pixelSize
            // Throws exception if a parameter is zero (undefined).
            double GetFocalLengthInPhysicalUnits() const;

            // Append to the pose data path with comma separator
            void AppendPoseDataPath(const std::string & newPath);

            // Backproject a pixel coordinate onto the calibration board plane to get it's 
            // 3D coordinate in the camera frame, in the plane of the calibration board.  Optionally,
            // undistort the point first (if you haven't already done this then you need to).
            cv::Point3d BackProjectOntoCalibrationBoard(const cv::Mat & boardToCameraRot, const cv::Mat & boardToCameraTrans, 
                const cv::Point2f & pixelCoord, const bool & undistort = false) const;

			// Populate this object from the input stream.
			// Throws exception on failure.
			void JsonDeserialize(const Json::Value &jsonNode) override;

			// Populate the output Json from this object's data members.
			// Throws exception on failure.
			void JsonSerialize(Json::Value &jsonNode) const override;

			cv::Point2f UndistortPixel(cv::Point2f src);

			cv::Vec3d ProjectPixel(cv::Point2f pixel);

            // Get camera name
            std::string GetCameraName() const;

            // Get timestamp
            std::string GetTimestamp() const;
		};
	}
}

#endif
