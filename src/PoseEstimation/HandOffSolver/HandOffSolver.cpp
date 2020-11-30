#include "HandOffSolver.h"

//cv::Point2d HandOffSolver::estimate_pose_charucoboard(const SixDOF & sharedPose, const cv::Point3f & boardCenter, const cv::Point2d & ptInitGuess) {
//	// Shared Pose & P3d
//	this->sharedPose = sharedPose;
//    this->p3d = cv::Mat{ this->sharedPose.extrinsics * (cv::Mat_<double>(4, 1) << boardCenter.x, boardCenter.y, boardCenter.z, 1.0) };
//
//	auto sstream{ std::stringstream{} };
//	auto ptRads{ cv::Mat{cv::Point2d{deg2rad(ptInitGuess.x), deg2rad(ptInitGuess.y)}} };
//
//	sstream << "PT ( " << ptInitGuess.x << ", " << ptInitGuess.y << ") Rad ( " << ptRads.at<double>(0) << ", " << ptRads.at<double>(1) << ")" << std::endl;
//	this->eHandler->report(sstream.str(), Shared::Error::Severity::DEBUG, this->loggerID);
//	sstream.str("");
//
//	// LMSolver
//	this->solver = cv::LMSolver::create(this, 100);
//	auto iterations{ this->solver->run(ptRads) };
//
//	auto estPanTilt{ cv::Point2d{rad2deg(ptRads.at<double>(0)), rad2deg(ptRads.at<double>(1))} };
//
//	sstream << "Estimated PT: ( " << estPanTilt << ")" << "\n"
//		    << "Iterations: " << iterations << std::endl;
//	this->eHandler->report(sstream.str(), Shared::Error::Severity::DEBUG, this->loggerID);
//	sstream.str("");
//
//	return estPanTilt;
//}

void HandOffSolver::setImageSize(const cv::Size & imageSize) {
	this->imageSize = imageSize;
}

/**
 * OpenCV LMSovler callback
 */
bool HandOffSolver::compute(cv::InputArray param, cv::OutputArray err, cv::OutputArray Jacobian) const {
	static const double EPSILON{ 1e-8 };

	auto sstream{ std::stringstream{} };
	

	// Lambda functions
	auto Objective = [&](const cv::Mat & pantilt) {
		//sstream << "(Objective) Passed pantilt:\n" << pantilt << std::endl;
		//this->eHandler->report(sstream.str(), Shared::Error::Severity::DEBUG, this->loggerID);
		//sstream.str("");

		//		assert(pantilt.rows == 1 && pantilt.cols == 2);
		assert(pantilt.rows == 2 && pantilt.cols == 1);

		// Translate 3D world frame, coordinate into assocaited ximea frame use Dynamic Extrinsics model
		cv::Mat extrinsics{ cv::Mat::eye(4, 4, CV_64F) };
		// Note: necessary since base class ::compute() is <const>
		auto cadModel{ this->ptModel };
		cadModel.apply(extrinsics, pantilt.at<double>(0), pantilt.at<double>(1));

		cv::Mat Pc{ extrinsics * this->p3d };

		// Goal: X & Y components of Pc (camera coordinate) = 0
		// e_x = fx * Pc[0] / Pc[2] - imWidth / 2 + cx
		// e_y = fy * Pc[1] / Pc[2] - imHeight / 2 + cy
		assert(this->imageSize.height == Presets::Imaging::Camera::Ximea::IMAGE_HEIGHT);
		assert(this->imageSize.width == Presets::Imaging::Camera::Ximea::IMAGE_WIDTH);

		cv::Mat result{};
		result.push_back(this->cameraMatrix.at<double>(0, 0) * Pc.at<double>(0) / Pc.at<double>(2) - this->imageSize.width / 2.0 + this->cameraMatrix.at<double>(0, 2));
		result.push_back(this->cameraMatrix.at<double>(1, 1) * Pc.at<double>(1) / Pc.at<double>(2) - this->imageSize.height / 2.0 + this->cameraMatrix.at<double>(1, 2));

		//sstream << "(Objective) Extrinsics:\n" << extrinsics << "\n"
		//	<< "(Objective) Pc:\n" << Pc << "\n"
		//	<< "(Objective) ePT( " << result.at<double>(0) << ", " << result.at<double>(1) << ")\n" << std::endl;
		//this->eHandler->report(sstream.str(), Shared::Error::Severity::DEBUG, this->loggerID);
		//sstream.str("");

		return result;
	};
	// END lambda

	//sstream << "==============================================" << std::endl;
	//this->eHandler->report(sstream.str(), Shared::Error::Severity::DEBUG, this->loggerID);
	//sstream.str("");

	// Input
	auto input{ param.getMat() };
	int elements{ input.checkVector(1) };
	CV_Assert(elements == 2);

	//sstream << "(compute) Input:\n" << input << "\n"
	//		<< "(compute) P3d: " << this->p3d << std::endl;
	//this->eHandler->report(sstream.str(), Shared::Error::Severity::DEBUG, this->loggerID);
	//sstream.str("");

	// Error
	err.create(elements, 1, CV_64F);
	CV_Assert(err.getMat().rows == elements && err.getMat().cols == 1);

	// Checking Jacobian...
	if (Jacobian.needed()) {
		Jacobian.create(input.rows, elements, CV_64F);
		CV_Assert(Jacobian.getMat().rows == 2 && Jacobian.getMat().cols == 2);

		//sstream << "(compute) J: ( " << Jacobian.getMat().rows << ", " << Jacobian.getMat().cols << ")" << std::endl;
		//this->eHandler->report(sstream.str(), Shared::Error::Severity::DEBUG, this->loggerID);
		//sstream.str("");
	}

	// Resutls...
	auto result{ Objective(param.getMat()) };
	//sstream << "(compute) Result:\n" << result << "\n" << std::endl;
	//this->eHandler->report(sstream.str(), Shared::Error::Severity::DEBUG, this->loggerID);
	//sstream.str("");

	// Update errors...
	// Note: err = E_actual - E_ideal; in our case E_ideal = 0
	err.getMat().at<double>(0) = result.at<double>(0);
	err.getMat().at<double>(1) = result.at<double>(1);
	//sstream << "(intermediate) Err:\n" << err.getMat() << std::endl;
	//this->eHandler->report(sstream.str(), Shared::Error::Severity::DEBUG, this->loggerID);
	//sstream.str("");

	// Update jacobian...
	if (Jacobian.needed()) {
		// Ex:
		// d_pan/d_tilt => EPSILON
		// d_ex/y = e_x/y1 - e_x/y0

		// | d_ex / d_p  |  d_ex / d_t  |
		// | d_ey / d_p  |  d_ey / d_t  |
		for (auto i = 0; i < Jacobian.getMat().rows; i++) {
			for (auto j = 0; j < Jacobian.getMat().cols; j++) {
				auto _input{ input.clone() };
				_input.at<double>(j) += EPSILON;
				Jacobian.getMat().at<double>(i, j) = (Objective(_input).at<double>(i) - result.at<double>(i)) / EPSILON;
			}
		}
	}

	//sstream << "(compute) Input:\n" << param.getMat() << "\n"
	//		<< "(compute) Error:\n" << err.getMat() << "\n"
	//		<< "Jacobian... " << (Jacobian.needed() ? "Needed" : "Not-needed") << "\n"
	//		<< "(compute) J:\n" << Jacobian.getMat() << "\n"
	//		<< "==============================================" << std::endl;
	//this->eHandler->report(sstream.str(), Shared::Error::Severity::DEBUG, this->loggerID);
	//sstream.str("");
	

	return true;
}
