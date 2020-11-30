#ifndef METROLOGY2020_PTMODELINTERFACE_H
#define METROLOGY2020_PTMODELINTERFACE_H

#include <iostream>
#include <string>

// JSON
#include <nlohmann/json.hpp>

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

// Utilities
#include "Matrix/Matrix3d.h"

//static const std::string MODEL_KEY{ Presets::Model::Key };

class PTModelInterface {
public:
	PTModelInterface() = default;
	virtual ~PTModelInterface() = default;

	virtual void loadModel(bool emptyModel) = 0;
	virtual void loadModel(const std::string & file, const std::string & modelKey = MODEL_KEY) = 0;
	virtual void loadModel(nlohmann::json & jsonConfig, const std::string & modelKey = MODEL_KEY) = 0;

	virtual void saveModel() = 0;
	virtual void updateModel() = 0;
	virtual void displayModel() = 0;

	// Apply
	virtual typename Matrix3d<double, std::string>::E_Matrix apply(
		typename Matrix3d<double, std::string>::E_Matrix & extrinsic,
		const typename Matrix3d<double, std::string>::E_Matrix & panAngles,
		const typename Matrix3d<double, std::string>::E_Matrix & tiltAngles) = 0;
	virtual void apply(
		cv::Mat & extrinsic,
		const typename Matrix3d<double, std::string>::E_Matrix & panAngles,
		const typename Matrix3d<double, std::string>::E_Matrix & tiltAngles) = 0;
	virtual void apply(
		cv::Mat & extrinsic, 
		const double & panAngle, 
		const double & tiltAngle) = 0;
	virtual void apply(
		typename Matrix3d<double, std::string>::E_Matrix & extrinsic,
		const double & panAngle, 
		const double & tiltAngle) = 0;

	// Apply Inverse
	virtual typename Matrix3d<double, std::string>::E_Matrix applyInverse(
		typename Matrix3d<double, std::string>::E_Matrix & localExtrinsics,
		const typename Matrix3d<double, std::string>::E_Matrix & panAngles,
		const typename Matrix3d<double, std::string>::E_Matrix & tiltAngles) = 0;
	virtual void applyInverse(
		cv::Mat & extrinsic, 
		const double & panAngle, 
		const double & tiltAngle) = 0;

};


#endif // METROLOGY2020_PTMODELINTERFACE_H
