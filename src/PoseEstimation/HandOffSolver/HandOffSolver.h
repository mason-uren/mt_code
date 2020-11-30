#ifndef METROLOGY2020_HANDOFFSOLVER_H
#define METROLOGY2020_HANDOFFSOLVER_H

#include <string>
#include <iostream>
#include <memory>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// HRL
#include <Shared/Presets.h>
#include <Shared/SharedStructs.h>

// FIXME - should be Models
#include <Model/PanTiltModel.h>

// Utilities
#include <ErrorHandler/ErrorHandler.h>
#include <OpenCVHelpers/Transformations/PoseTransformations.h>

#include "SixDOF.h"

class HandOffSolver: public cv::LMSolver::Callback {
public:
	HandOffSolver() = default;
    HandOffSolver(const cv::Mat & p3d,
                  const Model::PanTilt<> & cadModel,
				  const cv::Mat & cameraMatrix, 
				  const cv::Size & imageSize,
				  const std::string & loggerID = Presets::Logger::DEFAULT_ID, 
				  ErrorHandler * errorHandler = ErrorHandler::getInstance()) :
        ptModel(cadModel),
        p3d(p3d),
		cameraMatrix(cameraMatrix),
		imageSize(imageSize),
		loggerID(loggerID),
        eHandler(errorHandler)
	{}
    HandOffSolver(const Model::PanTilt<> & cadModel,
                  const cv::Mat & cameraMatrix,
                  const cv::Size & imageSize,
                  const std::string & loggerID = Presets::Logger::DEFAULT_ID,
                  ErrorHandler * errorHandler = ErrorHandler::getInstance()) :
        ptModel(cadModel),
        p3d(),
        cameraMatrix(cameraMatrix),
        imageSize(imageSize),
        loggerID(loggerID),
        eHandler(errorHandler)
    {}
	~HandOffSolver() override {}

	// Copy construction & assignment
	HandOffSolver(const HandOffSolver & obj) = delete;
	HandOffSolver & operator=(const HandOffSolver & obj) = delete;

	// Move construction & assignment
	HandOffSolver(HandOffSolver && obj) :
		ptModel(obj.ptModel),
		sharedPose(obj.sharedPose),
		p3d(obj.p3d),
		cameraMatrix(obj.cameraMatrix),
		imageSize(obj.imageSize),
		solver(obj.solver),
		loggerID(obj.loggerID),
		eHandler(ErrorHandler::getInstance())
	{}
	HandOffSolver & operator=(HandOffSolver && obj) {
		ptModel = std::move(obj.ptModel);
		sharedPose = std::move(obj.sharedPose);
		p3d = std::move(obj.p3d);
		cameraMatrix = std::move(obj.cameraMatrix);
		imageSize = std::move(obj.imageSize);
		solver = std::move(obj.solver);
		loggerID = std::move(obj.loggerID);
		eHandler = ErrorHandler::getInstance();
		return *this;
	}


	//cv::Point2d estimate_pose_charucoboard(const SixDOF & sharedPose, const cv::Point3f & boardCenter, const cv::Point2d & ptInitGuess = cv::Point2d{ 0, 0 });
	void setImageSize(const cv::Size & imageSize);

private:
	bool compute(cv::InputArray param, cv::OutputArray, cv::OutputArray Jacobian) const override;

	Model::PanTilt<> ptModel{};
	SixDOF sharedPose{};
	cv::Mat p3d{};
	cv::Mat cameraMatrix{};
	cv::Size imageSize{};
	std::shared_ptr<cv::LMSolver> solver;

	std::string loggerID{};

	// Static pointers!
	ErrorHandler * eHandler;
};

#endif // METROLOGY2020_HANDOFFSOLVER_H

