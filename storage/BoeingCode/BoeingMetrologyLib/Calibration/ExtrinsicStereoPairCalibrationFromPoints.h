#ifndef BOEINGMETROLOGYLIB_EXTRINSICSTEREOPAIRCALIBRATIONFROMPOINTS_H
#define BOEINGMETROLOGYLIB_EXTRINSICSTEREOPAIRCALIBRATIONFROMPOINTS_H

#include "cv.h"
#include <vector>
#include <string>
#include <map>
#include "MultiCameraIntrinsicData.h"
#include "MultiCameraExtrinsicData.h"
#include "Calibration/Observation/CameraObservation.h"
#include "Calibration/Observation/MultiPoseObservations.h"

#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Calibration
	{

        class BOEINGMETROLOGYLIB_API ExtrinsicStereoPairCalibrationFromPoints
		{
		public:
			ExtrinsicStereoPairCalibrationFromPoints() = delete;

			// Pull out the observations for the camera pair we want to calibrate
			ExtrinsicStereoPairCalibrationFromPoints(const std::vector<std::string> & camNames,
				MultiPoseObservations & observations, MultiCameraIntrinsicData & multiCameraIntrinsicData);

			// Perform stereo calibration based on mutually observed observation points.  Output extrinsics for the second camera (camNames[1])
			// are specified in the first camera's coordinate frame.  
			void CalibrateStereoPair(MultiCameraExtrinsicData & multiCameraExtrinsicData);

            // For each shared pose, compute the extrinsics between the camera pair and the associated reprojection
            // error of shared calibration board points
            void ComputePerPoseStereoReprojectionError(std::map<double, POSE_NAME> & stereoReprojectionErrorPerPose);

		protected:

			// Image size -- must be common to both cameras
			cv::Size _imgSize;

			// The two camera names 
			std::vector<std::string> _camNames;
            // The path to the imagery
            std::string _poseDataPath;

			// The vector of control points for each marker of each pose that was 
			// observed by both cameras
			std::vector<std::vector<ObservationPoints<float>::CONTROL_POINT>> _cameraObjectPoints;

			// A map indexed by camera.  Each camera contains a vector of observation 
			// points for each marker of each pose that was observed by both cameras
			std::map<std::string, std::vector<std::vector<ObservationPoints<float>::OBSERVATION_POINT>>> _cameraImagePoints;

            // The pose names corresponding to the observations
            std::vector<std::string> _poseNames;

			// A pointer to the intrinsics data
			MultiCameraIntrinsicData * _multiCameraIntrinsicData;
		};
	}
}

#endif
