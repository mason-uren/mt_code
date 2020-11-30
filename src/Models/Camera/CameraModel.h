//
// Created by U'Ren, Mason R (VEN) on 4/23/20.
//

#ifndef METROLOGY2020_ALGORITHMS_CAMERAMODEL_H
#define METROLOGY2020_ALGORITHMS_CAMERAMODEL_H

#include <string>
#include <sstream>

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

// Npy
#include <CppNpy/numpy.h>

// HRL
#include <Shared/ModelsConfig.h>
#include <Shared/Presets.h>
#include <Shared/SharedStructs.h>

#include "Matrix/Matrix3d.h"

template<typename Primitive = double, typename Key = std::string>
class CameraModel: public ConfigInterface {
private:
    struct EigenMatrix;
    struct CVMatrix;

	inline void checkCameraMatrix(const int & rows, const int & cols) {
		if (rows != cols || rows != 3) {
			throw "Invalid intrinsics matrix dimensions, expecting 3 x 3.";
		}
	}

	inline void checkDistortionCoeffs(const int & rows, const int & cols) {
		if (rows != 1 || cols != 5) {
			throw "Invalid distCoeffs matrix dimensions, expecting 1 x 5.";
		}
	}

	inline void scaleCameraMatrix(cv::Mat & matrix, const double & scaleFactor) {
		matrix.at<double>(0, 0) /= scaleFactor; // focal length in X
		matrix.at<double>(1, 1) /= scaleFactor; // focal length in Y
		matrix.at<double>(0, 2) /= scaleFactor; // principle point in X
		matrix.at<double>(1, 2) /= scaleFactor; // principle point in Y
	}

	inline void scaleCameraMatrix(typename Matrix3d<double>::E_Matrix & matrix, const double & scaleFactor) {
		matrix(0, 0) /= scaleFactor;
		matrix(1, 1) /= scaleFactor;
		matrix(0, 2) /= scaleFactor;
		matrix(1, 2) /= scaleFactor;
	}

public:
	explicit CameraModel(const std::string & caller) : ConfigInterface(caller) {}
	CameraModel(const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeffs, const double & scale = 1, const std::string & caller = Presets::Logger::DEFAULT_ID) :
		ConfigInterface(caller),
		cvMatrix(CVMatrix{cameraMatrix, distortionCoeffs})
	{
		auto sstream{ std::stringstream{} };

		try {
			checkCameraMatrix(cvMatrix.cameraMatrix.rows, cvMatrix.cameraMatrix.cols);
			checkDistortionCoeffs(cvMatrix.distortionCoeffs.rows, cvMatrix.distortionCoeffs.cols);

			cv::cv2eigen(cvMatrix.cameraMatrix, eigenMatrix.cameraMatrix);
			cv::cv2eigen(cvMatrix.distortionCoeffs, eigenMatrix.distortionCoeffs);

			if (scale != 1) {
				scaleCameraMatrix(cvMatrix.cameraMatrix, scale);
				scaleCameraMatrix(eigenMatrix.cameraMatrix, scale);
			}

			// Log camera matrix and distortion coeffs
			sstream.str("");
			sstream << caller << "\n";
			for (const auto & matrix : std::unordered_map<std::string, cv::Mat>{ {"Camera Matrix", cvMatrix.cameraMatrix}, { "Distortion Coeffs", cvMatrix.distortionCoeffs } }) {
				sstream << matrix.first << " " << matrix.second;
				eHandler->report(sstream.str(), Shared::Error::DEBUG, caller);
				sstream.str("");
			}
		}
		catch (const cv::Exception & except) {
			sstream.str("");
			sstream << "(CameraModel) " << except.what();
			eHandler->report(sstream.str(), Shared::Error::Severity::KILL_AND_EXIT);
		}
	}
	CameraModel(const std::string & matrixPath, const std::string & distCoeffsPath, const double & scale = 1, const std::string & caller = Presets::Logger::DEFAULT_ID) :
		ConfigInterface(caller),
		eigenMatrix(
			EigenMatrix{
				CameraModel<Primitive, Key>::readModel(matrixPath, caller),
				CameraModel<Primitive, Key>::readModel(distCoeffsPath, caller)
			}
		)
	{
		auto sstream{ std::stringstream{} };

		try {
			checkCameraMatrix(eigenMatrix.cameraMatrix.rows(), eigenMatrix.cameraMatrix.cols());
			checkDistortionCoeffs(eigenMatrix.distortionCoeffs.rows(), eigenMatrix.distortionCoeffs.cols());

			cv::eigen2cv(eigenMatrix.cameraMatrix, cvMatrix.cameraMatrix);
			cv::eigen2cv(eigenMatrix.distortionCoeffs, cvMatrix.distortionCoeffs);

			if (scale != 1) {
				scaleCameraMatrix(cvMatrix.cameraMatrix, scale);
				scaleCameraMatrix(eigenMatrix.cameraMatrix, scale);
			}

			// Log camera matrix and distortion coeffs
			sstream.str("");
			sstream << caller << "\n";
			sstream << "Camera Matrix Path: " << matrixPath << std::endl;
			sstream << "Disortion Coeffs Path: " << distCoeffsPath;
			eHandler->report(sstream.str(), Shared::Error::Severity::INFO, caller);
			sstream.str("");


			for (const auto & matrix : std::unordered_map<std::string, cv::Mat>{ {"Camera Matrix", cvMatrix.cameraMatrix}, { "Distortion Coeffs", cvMatrix.distortionCoeffs } }) {
				sstream << matrix.first << " " << matrix.second;
				eHandler->report(sstream.str(), Shared::Error::DEBUG, caller);
				sstream.str("");
			}
		}
		catch (const cv::Exception & except) {
			sstream.str("");
			sstream << "(CameraModel) " << except.what();
			eHandler->report(sstream.str(), Shared::Error::Severity::KILL_AND_EXIT);
		}
	}
	explicit CameraModel(const Model::Camera::Intrinsics & cameraConfig, const double & scale = 1) :
		CameraModel(cameraConfig.cameraMatrix.path, cameraConfig.distortionCoeff.path, scale)
	{}

	// Functions
	void changeCaller(const std::string & name) {
	    ConfigInterface::name = name;
	}
	bool setup(const std::string & matrixPath, const std::string distCoeffsPath, const double & scale = 1, const std::string & caller = Presets::Logger::DEFAULT_ID) {
		eigenMatrix = EigenMatrix{
					CameraModel<Primitive, Key>::readModel(matrixPath, caller),
					CameraModel<Primitive, Key>::readModel(distCoeffsPath, caller)
		};

		auto sstream{ std::stringstream{} };

		try {
			checkCameraMatrix(eigenMatrix.cameraMatrix.rows(), eigenMatrix.cameraMatrix.cols());
			checkDistortionCoeffs(eigenMatrix.distortionCoeffs.rows(), eigenMatrix.distortionCoeffs.cols());

			cv::eigen2cv(eigenMatrix.cameraMatrix, cvMatrix.cameraMatrix);
			cv::eigen2cv(eigenMatrix.distortionCoeffs, cvMatrix.distortionCoeffs);

			if (scale != 1) {
				scaleCameraMatrix(cvMatrix.cameraMatrix, scale);
				scaleCameraMatrix(eigenMatrix.cameraMatrix, scale);
			}

			// Log camera matrix and distortion coeffs
			sstream.str("");
			sstream << caller << "\n";
			sstream << "Camera Matrix Path: " << matrixPath << std::endl;
			sstream << "Disortion Coeffs Path: " << distCoeffsPath;
			eHandler->report(sstream.str(), Shared::Error::Severity::INFO, caller);
			sstream.str("");

			for (const auto & matrix : std::unordered_map<std::string, cv::Mat>{ {"Camera Matrix", cvMatrix.cameraMatrix}, { "Distortion Coeffs", cvMatrix.distortionCoeffs } }) {
				sstream << matrix.first << " " << matrix.second;
				eHandler->report(sstream.str(), Shared::Error::DEBUG, caller);
				sstream.str("");
			}

			return true;
		}
		catch (const std::exception & except) {
			sstream.str("");
			sstream << "(CameraModel) " << except.what();
			eHandler->report(sstream.str(), Shared::Error::Severity::KILL_AND_EXIT);
			return false;
		}
	}

	bool setup(const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeffs, const double & scale = 1, const std::string & caller = Presets::Logger::DEFAULT_ID) {
		auto sstream{ std::stringstream{} };

		try {
			checkCameraMatrix(cameraMatrix.rows, cameraMatrix.cols);
			checkDistortionCoeffs(distortionCoeffs.rows, distortionCoeffs.cols);

			cvMatrix.cameraMatrix = cameraMatrix;
			cvMatrix.distortionCoeffs = distortionCoeffs;

			if (scale != 1) {
				scaleCameraMatrix(cvMatrix.cameraMatrix, scale);
				scaleCameraMatrix(eigenMatrix.cameraMatrix, scale);
			}

			// Log camera matrix and distortion coeffs
			sstream.str("");
			sstream << caller << "\n";
			for (const auto & matrix : std::unordered_map<std::string, cv::Mat>{ {"Camera Matrix", cvMatrix.cameraMatrix}, { "Distortion Coeffs", cvMatrix.distortionCoeffs } }) {
				sstream << matrix.first << " " << matrix.second;
				eHandler->report(sstream.str(), Shared::Error::DEBUG, caller);
				sstream.str("");
			}

			return true;
		}
		catch (const std::exception & except) {
			sstream.str("");
			sstream << "(CameraModel) " << except.what();
			eHandler->report(sstream.str(), Shared::Error::Severity::KILL_AND_EXIT);
			return false;
		}
	}

	bool setup(const Model::Camera::Intrinsics & cameraConfig, const double scale = 1, const std::string & caller = Presets::Logger::DEFAULT_ID) {
		return setup(cameraConfig.cameraMatrix.path, cameraConfig.distortionCoeff.path, scale, caller);
	}

    void setKeys(const std::vector<std::string> & keys) {
        assert(keys.size() == 2);

        uvCornersKey = keys[0];
        cornerIDKey = keys[1];
    }
    void setKeys(const std::string & UVcornersKey, const std::string & IDsKey) {
        uvCornersKey = UVcornersKey;
        cornerIDKey = IDsKey;
    }
    ~CameraModel() override = default;

	// Variables
    std::string uvCornersKey{};
    std::string cornerIDKey{};

    EigenMatrix eigenMatrix;
    CVMatrix cvMatrix;

private:
    struct EigenMatrix {
		typename Matrix3d<Primitive, Key>::E_Matrix cameraMatrix;
		typename Matrix3d<Primitive, Key>::E_Matrix distortionCoeffs;
    };
    struct CVMatrix {
		cv::Mat cameraMatrix;
		cv::Mat distortionCoeffs;
    };

    /**
    *
    * @param type
    * @param filePath
    * @return
    */
    static typename Matrix3d<Primitive, Key>::E_Matrix readModel(const std::string & filePath, const std::string & caller = typeid(CameraModel).name()) {
        // Read *.npy file
        std::vector<int> shape{};
        std::vector<Primitive> data{};

		try {
			aoba::LoadArrayFromNumpy(filePath, shape, data);
		}
		catch (const std::exception & except) {
			auto sstream{ std::stringstream{} };
			sstream << "Error while trying to load camera intrinsics: " << except.what();
			ErrorHandler::getInstance()->report(sstream.str(), Shared::Error::WARN, caller);
		}

        // Allocate mem on stack
        Matrix3d<Primitive, Key> metrics{};
        auto _shape{std::vector<unsigned long long>(shape.begin(), shape.end())};
        metrics.dataToMatrix(data.data(), "CameraModel::readModel(internal-call)", shape, metrics.isRowMajor());

        return metrics();
    }
};

#endif //METROLOGY2020_ALGORITHMS_CAMERAMODEL_H