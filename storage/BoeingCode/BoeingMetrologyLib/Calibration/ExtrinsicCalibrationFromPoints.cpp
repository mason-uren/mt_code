#include "ExtrinsicCalibrationFromPoints.h"
#include <queue>
#include "json/value.h"
#include "opencv2/core.hpp"
#include "Calibration/CameraCalibrationUtilities.h"
#include <iostream>

using namespace BoeingMetrology::Calibration;

BoeingMetrology::ExtrinsicCalibrationFromPoints::ExtrinsicCalibrationFromPoints(BoeingMetrology::Calibration::Pose::MultiPoseGraph & poseGraph)
{
    _poseGraph = &poseGraph;
}

double BoeingMetrology::ExtrinsicCalibrationFromPoints::Calibrate(MultiCameraExtrinsicData & multiCameraExtrinsicData, const std::vector<std::string> & lockedCameraNames)
{
    // Verification of the pose graph
    if (_poseGraph == nullptr || !_poseGraph->IsInitialized())
        throw std::runtime_error("ExtrinsicCalibrationFromPoints::Calibrate: poseGraph is uninitialized");

    if (!_poseGraph->IsConnected())
        throw std::runtime_error("ExtrinsicCalibrationFromPoints::Calibrate: poseGraph is not connected");

    MultiCameraExtrinsicData initialExtrinsics;
	std::map<CAMERA_NAME, double> perCamera2dErrors = std::map<CAMERA_NAME, double>();
	std::map<CAMERA_NAME, double > perCamera3dErrors = std::map<CAMERA_NAME, double>();
    _poseGraph->_verbose = false;
    _poseGraph->packageMultiCameraExtrinsicData(initialExtrinsics);
	
    auto reprojectionError = _poseGraph->optimizeExtrinsics(lockedCameraNames, perCamera2dErrors, perCamera3dErrors, true);

    _poseGraph->packageMultiCameraExtrinsicData(multiCameraExtrinsicData);

	//populate the per-camera 2d and 3d errors for the extrinsics objects
	for (auto cameraErrorPair : perCamera2dErrors)
	{
		CAMERA_NAME camera = cameraErrorPair.first;
		multiCameraExtrinsicData.cameraData[camera].allignmentError = perCamera3dErrors[camera];
		multiCameraExtrinsicData.cameraData[camera].rmsError = perCamera2dErrors[camera];
	}

    // Report results
    std::cout << "ExtrinsicCalibrationFromPoints: Before, After" << std::endl;
    for (const auto & cam : initialExtrinsics.cameraData)
    {
        std::cout << cam.first << std::endl;
        for (int r = 0; r < cam.second.transform.rows; r++)
        {
            for (int c = 0; c < cam.second.transform.cols; c++)
            {
                std::cout << cam.second.transform(r, c) << ", " << multiCameraExtrinsicData.cameraData[cam.first].transform(r, c) << std::endl;
            }
        }
        std::cout << std::endl;
    }

    return reprojectionError;
}
