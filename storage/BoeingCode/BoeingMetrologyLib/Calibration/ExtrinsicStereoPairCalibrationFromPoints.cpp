#include "ExtrinsicStereoPairCalibrationFromPoints.h"
#include <queue>
#include "json/value.h"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "Calibration/CameraCalibrationUtilities.h"
#include <iostream>
#include <thread>
#include "Calibration/Observation/CameraObservation.h"
#include "MultiCameraIntrinsicData.h"

using namespace BoeingMetrology::Calibration::Observation;
BoeingMetrology::Calibration::ExtrinsicStereoPairCalibrationFromPoints::ExtrinsicStereoPairCalibrationFromPoints(const std::vector<std::string> & camNames,
	MultiPoseObservations & observations, MultiCameraIntrinsicData & multiCameraIntrinsicData)
{
	// Copy the camera names
	if (camNames.size() != 2)
	{
		throw std::runtime_error("CalibrateStereoPair currently only supports 2 cameras");
	}
    this->_camNames = camNames;

	// Copy the image size
	_imgSize = observations.cameraImageSize[camNames[0]];
	if (_imgSize != observations.cameraImageSize[camNames[1]])
	{
		throw std::runtime_error("CalibrateStereoPair image sizes differ");
	}

	// Get common observations across the two cameras
    observations.RetrieveMutualObservationPoints(camNames, this->_cameraObjectPoints, this->_cameraImagePoints, this->_poseNames);
    _poseDataPath = observations.folderNameOfObservations;

	// Intrinsics information
	_multiCameraIntrinsicData = &multiCameraIntrinsicData;
}

void BoeingMetrology::Calibration::ExtrinsicStereoPairCalibrationFromPoints::CalibrateStereoPair(MultiCameraExtrinsicData & multiCameraExtrinsicData)
{
	cv::Mat R, T, E, F;
	double rms = stereoCalibrate(_cameraObjectPoints, _cameraImagePoints[_camNames[0]], _cameraImagePoints[_camNames[1]], 
		_multiCameraIntrinsicData->cameraData[_camNames[0]].cameraMatrix, _multiCameraIntrinsicData->cameraData[_camNames[0]].distortionCoeffs, 
		_multiCameraIntrinsicData->cameraData[_camNames[1]].cameraMatrix, _multiCameraIntrinsicData->cameraData[_camNames[1]].distortionCoeffs, 
		_imgSize, R, T, E, F, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));

	std::cout << "stereoCalibrate RMS = " << rms << std::endl;

	// Set the reference camera
	ExtrinsicData extrinsicData;
	extrinsicData.name = _camNames[0];
	extrinsicData.refCameraName = _camNames[0];
	extrinsicData.SetTranslationVector(cv::Mat::zeros(3, 1, CV_64FC1));
	extrinsicData.SetRotationMatrix(cv::Mat::eye(3, 3, CV_64FC1));
    extrinsicData.poseDataPath = _poseDataPath;
	multiCameraExtrinsicData.cameraData[_camNames[0]] = extrinsicData;

	// Set the second camera
	ExtrinsicData extrinsicData2;
	extrinsicData2.name = _camNames[1];
	extrinsicData2.refCameraName = _camNames[0];
	extrinsicData2.SetTranslationVector(T);
    extrinsicData2.poseDataPath = _poseDataPath;

	// Rodrigues angles to 3x3 matrix
	cv::Mat rotMat(3, 3, CV_64FC1);
	cv::Rodrigues(R, rotMat);

	// Set rotation components
	extrinsicData2.SetRotationMatrix(rotMat);

	// Append to the list
	multiCameraExtrinsicData.cameraData[_camNames[1]] = extrinsicData2;
}