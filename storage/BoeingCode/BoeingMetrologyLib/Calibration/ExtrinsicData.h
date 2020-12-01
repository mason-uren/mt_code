#ifndef BOEINGMETROLOGYLIB_EXTRINSICDATA_H
#define BOEINGMETROLOGYLIB_EXTRINSICDATA_H

#include <string>
#include <cv.h>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Calibration
	{
        // Container for sensor extrinsics in a reference camera's frame
        class BOEINGMETROLOGYLIB_API ExtrinsicData : public Boeing::Interface::Serializer
		{
		public:

            // Sensor name
            std::string name = "";

            // Timestamp YYYYMMDD
            std::string timestamp = "";

			// The reference frame relating this camera via "transform"
			std::string refCameraName;

			// The affine transformation between the camera and the reference frame
			// Camera frame as defined https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
			// Translation (position) components are in meters. 
			// This transform will take a point specified in this camera's frame as p_c and 
			// convert it into the world frame via p_world = R * p_c + T, where R corresponds to 
			// the top-left 3x3 components and p_c and T are column vectors.  
			// In other words this is a transform from this camera's frame to the world frame.  
			cv::Matx34d transform;

			//path to the pose data from which the extrinsics were created
			std::string poseDataPath;

			//timestamp of the associated intrinsics for this camera
			std::string intrinsicsTimestamp;

			//the in-set reprojection error of board poses for the extrinsics after computation
			double rmsError=0.0;

			//the in-set 3d-allignment error of board poses for the extrinsics after computation
			double allignmentError = 0.0;

            ExtrinsicData() {};

            // Initialize everything but the transform
            ExtrinsicData(const std::string & name, const std::string & refCameraName, const std::string & poseDataPath);
			
			// Populate this object from the input Json.
			// Throws exception on failure.
			void JsonDeserialize(const Json::Value &jsonNode) override;

			// Populate the output Json from this object's data members.
			// Throws exception on failure.
			void JsonSerialize(Json::Value &jsonNode) const override;

			// Set the 3x3 rotation matrix.
			// Throws exception on failure.
			// Camera frame as defined https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
			void SetRotationMatrix(const cv::Mat & rotMat);

			// Set the translation vector.
			// Throws exception on failure.
			// Camera frame as defined https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
			// Translation (position) components are in meters.  
			void SetTranslationVector(const cv::Mat & transMat);
			void SetTranslationVector(const cv::Vec3f & transVec);
            void SetTranslationVector(const cv::Point3f & transVec);

			// Get the 3x3 rotation matrix (CV_64FC1)
			// Camera frame as defined https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
			cv::Mat GetRotationMatrix() const;

			// Get the 3x1 translation vector (CV_64FC1)
			// Camera frame as defined https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
			// Translation (position) components are in meters.  
			cv::Mat GetTranslationVector() const;

            // Get the optical axis unit vector specified in the reference frame
            cv::Point3d GetOpticalAxisVector() const;

			// Get the 6 element (6x1) Rotation/Translationn Rodruigues Vector
			// Defined as 3 axis rodrigues rotation elements R followed by standard 3 axis Translation vector
			// Primarily for use with Ceres bundleAdjustment
			std::vector<double> GetRodriguesVector() const;

            // Transform a point from this camera's frame to the world (reference) frame
            // point in world frame = R * point in camera frame + T
            cv::Point3d TransformFromThisCameraToWorldFrame(const cv::Point3d & ptInCameraFrame) const;

			// Transform a point from this camera's frame to the world (reference) frame
			// point in world frame = R.t() * (point in camera frame - T)
			cv::Point3d TransformFromThisCameraToWorldFrameInverse(const cv::Point3d & ptInCameraFrame) const;

            // Get camera name
            std::string GetCameraName() const;

            // Get timestamp
            std::string GetTimestamp() const;
		};
	}
}

#endif
