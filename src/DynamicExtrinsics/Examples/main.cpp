//
// Created by U'Ren, Mason R (VEN) on 3/30/20.
//

#include <string>
#include <array>
#include <fstream>
#include <unordered_map>

// HRL
#include <Shared/Presets.h>
#include <Shared/OSDefines.h>
#include <Shared/SharedStructs.h>

// Models
#include <ConfigParser/ModelsJSONParser.h>
#include <Board/FiducialModel.h>

// Utilities
#include <ErrorHandler/ErrorHandler.h>

#include "Camera/CameraModel.h"
#include "ConfigParser/DynamicExtrinsicsJSONParser.h"
#include "Extrinsics/ModelFit.h"
#include "Extrinsics/ModelError.h"
#include "H5/H5Parser.h"
#include "Model/Preprocess.h"
#include "Model/PanTiltModel.h"
#include "SystemConfigParser/SystemJSONParser.h"


//#define ENABLE_TEST_ENV
//#define SAVE_AND_EXIT
//#define DEBUG_EXAMPLE
//#define BLANK_MAIN

// Typedefs
typedef double datatype;
//typedef float datatype;

void testEnv();

/*
 * FIXME - implementation needs to be included in system framework
 */
int main() {
#ifndef BLANK_MAIN
	ErrorHandler::getInstance()->report("Dynamic Extrinsics Tests");

#ifdef ENABLE_TEST_ENV
    testEnv();
    return 0;
#endif

    Presets::DataSet::Key dsKey{};
    std::vector<Matrix3d<datatype, Presets::DataSet::Key>> dataset{};

	// System JSON Config
	auto systemConfig{SystemJSONParser{} };
	if (!systemConfig.loadSystemConfig()) {
		ErrorHandler::getInstance()->report(
			"Failed to load system config JSON.",
			Shared::Error::KILL_AND_EXIT
		);
		return EXIT_FAILURE;
	}

    // Parse Models JSON Config
    auto modelsConfig{ModelsJSONParser() };
    if (!modelsConfig.loadModelsConfig()) {
    	return EXIT_FAILURE;
    }

    // Dynamic Extrinsics JSON Config
    auto dynamicExtrinsicsConfig{DynamicExtrinsicsJSONParser() };
    if (!dynamicExtrinsicsConfig.loadDynamicExtrinsicConfig()) {
		ErrorHandler::getInstance()->report(
			"Failed to load dynamic extrinsics config JSON.",
			Shared::Error::KILL_AND_EXIT
		);
        return EXIT_FAILURE;
    }

    // Create camera objects (ximea & imperx)
    auto ximeaConfig{modelsConfig.ximea.at(systemConfig.pairs.at(systemConfig.mode.components.front()).camera.intrinsics)};
    auto imperxConfig{modelsConfig.imperx.at(systemConfig.wFrame.intrinsics)};
    auto cameraModel{std::unordered_map<std::string, CameraModel<datatype>>{
            {Presets::Imaging::Camera::XIMEA, CameraModel<datatype>{ximeaConfig}},
            {Presets::Imaging::Camera::IMPERX, CameraModel<datatype>{imperxConfig}}
    }};
    cameraModel.at(Presets::Imaging::Camera::XIMEA).setKeys({Presets::DataSet::Types::UV_CORNERS_XIMEA, Presets::DataSet::Types::XIMEA_CHIDS});
    cameraModel.at(Presets::Imaging::Camera::IMPERX).setKeys({Presets::DataSet::Types::UV_CORNERS_IMPERX, Presets::DataSet::Types::IMPERX_CHIDS});

    // Create charuco board
    auto fiducialModel{FiducialModel(&modelsConfig)};

    /**
     * Load files...
     */
	auto files{std::array<std::string, 2>{
            dynamicExtrinsicsConfig.fiducialPositions.map[Presets::FiducialPosition::BOT_LEFT][0].path,
            dynamicExtrinsicsConfig.fiducialPositions.map[Presets::FiducialPosition::BOT_RIGHT][0].path}};
    if (!H5Parser<datatype>().loadH5Files(dataset, files)) {
		ErrorHandler::getInstance()->report("Failed to load files", Shared::Error::Severity::KILL_AND_EXIT);
        return EXIT_FAILURE;
    }

	ErrorHandler::getInstance()->report("Successfully loaded files...", Shared::Error::Severity::DEBUG);
    for (auto & path : files) {
        ErrorHandler::getInstance()->report("File(s): " + path, Shared::Error::Severity::DEBUG);
    }

    /*
     * Pre-processing
     */
	ErrorHandler::getInstance()->report("Begin Preprocessing...", Shared::Error::Severity::DEBUG);
	auto preprocessor{Preprocessor<datatype>(fiducialModel)};
    for (auto &extrinsic : dataset) {
        // Create PanTiltType::RAD container
        if (!extrinsic.createBuffer(Presets::DataSet::PanTiltAngles::RAD, extrinsic.getDimensions(Presets::DataSet::PanTiltAngles::DEG), extrinsic.isRowMajor())) {
			ErrorHandler::getInstance()->report("Failed to create additional buffer : ID ( " + Presets::DataSet::PanTiltAngles::RAD + ")", Shared::Error::Severity::KILL_AND_EXIT);
            return EXIT_FAILURE;
        }
        preprocessor.transformAnglesInPlace(extrinsic, Presets::DataSet::PanTiltAngles::RAD);
        // Create Pan and Tilt containers
        preprocessor.splitPanTilt(extrinsic);

        // TODO - need to make stripping matrices optional
        if (!dynamicExtrinsicsConfig.datasetConfig.panRanges.empty()) {

        }
        if (!dynamicExtrinsicsConfig.datasetConfig.tiltRanges.empty()) {
            preprocessor.stripMatricesAtIdx(extrinsic, Presets::DataSet::PanTiltAngles::DEG, 0.05);
        }

        preprocessor.undistortImageCorners(extrinsic, cameraModel);
    }

#ifdef DEBUG_EXAMPLE
    int fileIdx{};
    for (auto i = 0; i < files.size(); i++) {
        std::cout << "Dataset: " << files[i] << std::endl;
        dataset[i].info();
        std::string file{"../Data/Extrinsic-r" + std::to_string(i) + "-"};
        dataset[i].writeToFiles(file);
    }

#ifdef SAVE_AND_EXIT
    std::cout << "Skipping model fitting. Exiting" << std::endl;
    return EXIT_SUCCESS;
#endif
#endif

    /**
     * Fit Dynamic Params
     */

    // Load PTU Model
    auto PTUModel{Model::PanTilt<datatype, Presets::DataSet::Key>()};
//    PTUModel.loadModel(parser.model.input.path);
    PTUModel.loadModel();

#ifdef DEBUG_EXAMPLE
    PTUModel.displayModel();
#endif

    std::vector<Presets::DataSet::Key> paramsToFit{};

    // Timer
    auto init{std::chrono::high_resolution_clock::now()};
    auto dynamicExtrinsics{DynamicExtrinsics<datatype>(fiducialModel, dynamicExtrinsicsConfig.algOptimization)};


    if (dynamicExtrinsicsConfig.examples.runDynamicModelFitting) {
        paramsToFit = std::vector<Presets::DataSet::Key> {
                Presets::Model::Types::X_CAMERA, Presets::Model::Types::Y_CAMERA, Presets::Model::Types::Z_CAMERA,
                Presets::Model::Types::RX_CAMERA, Presets::Model::Types::RY_CAMERA, Presets::Model::Types::RZ_CAMERA
        };
        dynamicExtrinsics.fitDynamicParams(PTUModel,
                                           dataset,
                                           paramsToFit,
                                           &DynamicExtrinsics<datatype>::dynamicExplodedLoss);
    }

    if (dynamicExtrinsicsConfig.examples.runStaticModelFitting) {
        paramsToFit = std::vector<Presets::DataSet::Key> {
                Presets::Model::Types::X_PTU, Presets::Model::Types::Y_PTU, Presets::Model::Types::Z_PTU,
                Presets::Model::Types::RX_PTU, Presets::Model::Types::RY_PTU, Presets::Model::Types::RZ_PTU
        };
        dynamicExtrinsics.fitStaticParams(PTUModel,
                                          dataset,
                                          paramsToFit,
                                          &DynamicExtrinsics<datatype>::staticExplodedLoss);
    }

    if (!(dynamicExtrinsicsConfig.examples.runStaticModelFitting || dynamicExtrinsicsConfig.examples.runDynamicModelFitting)) {
        ErrorHandler::getInstance()->report("No model fitting was selected. (See DynamicExtrinsicsConfig.json)");
    }

    std::chrono::duration<double, std::milli> delta = (std::chrono::high_resolution_clock::now() - init);
	ErrorHandler::getInstance()->report("Elapsed Time: ( " + std::to_string(delta.count()) + ")");

    PTUModel.displayModel();
    PTUModel.saveModel(dynamicExtrinsicsConfig.model.output.path);

    ErrorHandler::getInstance()->report("Dynamic Extrinsics Complete. Good Bye.");

#endif // BLANK_MAIN
    return 0;
}

void testEnv() {
    /**
     * Testing camera model efficiency
     */
//     for (auto i = 0; i < 10; i++) {
//
////         auto chessboardCorners{typename Metrics<double, std::string>::E_Matrix(Model::Charuco::CHESSBOARD_CORNERS.rows,
////                                                                                Model::Charuco::CHESSBOARD_CORNERS.cols)};
//
//         auto init{std::chrono::high_resolution_clock::now()};
//         Model::Charuco::CHESSBOARD_CORNERS;
//         std::chrono::duration<double, std::milli> delta = (std::chrono::high_resolution_clock::now() - init);
//         std::cout << "Elapsed Time: " << delta.count() << std::endl;
//     }


    /**
     * Eigen matrix block initialization testing
     * - arbitrary number of 10
     */
//    std::vector<std::vector<double>> mat(10, {0.1, 0.2, 0.3});
//    std::cout << "Expected:" << std::endl;
//    for (auto & row : mat) {
//        std::cout << row[0] << " " << row[1] << " " << row[2] << std::endl;
//    }
//
//    std::cout << "Actual:" << std::endl;
//
//    auto init{std::chrono::high_resolution_clock::now()};
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _mat{10, 3};
//    for (auto i = 0; i < 3; i++) {
//         _mat.col(i) = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Constant(10, 1, mat[0][i]);
//    }
//    std::chrono::duration<double, std::milli> delta = (std::chrono::high_resolution_clock::now() - init);
//    std::cout << "Elapsed Time: " << delta.count() << std::endl;
//
//    std::cout << _mat << std::endl;


    /**
     * Euler Angle testing
     */
//#include <unsupported/Eigen/EulerAngles>
//
//    typedef Eigen::EulerAngles<datatype, Eigen::EulerSystemZYX> EulerAngles;
//    EulerAngles angles(0.0150479, 0.20123, -0.0375593);
//    std::cout << angles.toRotationMatrix() << std::endl;

    /**
     * Chessboard corners testing
     */
//#include "Models/Board/CharucoBoard.h"
//
//    std::cout << Model::Charuco::board()->chessboardCorners << std::endl;

	static const std::string PTU_MODEL_JSON{"/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/dev/DynamicExtrinsics/python/cad_models/2019-09-04-cad-model.json"};

    /**
     * PTU Model Testing
     */
    auto PTUModel{Model::PanTilt<datatype, Presets::DataSet::Key>()};
//    PTUModel.loadModel(PTU_MODEL_JSON);
    PTUModel.displayModel();
//
//    std::vector<DataSet::Key> paramsToFit{
//            DataSet::CadModelTypes::X_PTU, DataSet::CadModelTypes::Y_PTU, DataSet::CadModelTypes::Z_PTU,
//            DataSet::CadModelTypes::RX_PTU, DataSet::CadModelTypes::RY_PTU, DataSet::CadModelTypes::RZ_PTU
//    };
//    Eigen::Matrix<datatype, Eigen::Dynamic, 1> d{Eigen::Matrix<datatype, Eigen::Dynamic, 1>::Zero(6)};
//    PTUModel.updateModel(paramsToFit, d);
//    PTUModel.displayModel();
//
//    /*
//     * Note: model.apply() has many overloaded functions
//     * Ex:
//     * apply(eigen::mat, eigen::mat, eigen::mat)
//     * apply(eigen::mat, primitive, primitive)
//     * apply(cv::mat, eigen::mat, eigen::mat)
//     * apply(cv::mat, primitive, primitive)
//     */
//
//
//    /**
//     * Transformation Testing
//     */
    typename Matrix3d<datatype, Presets::DataSet::Key>::E_Matrix _extrinsic{4, 4};
    _extrinsic << 0.125498059021483, 0.7845304625616427, 0.6072579274860513, 0.5536529283812169,
            0.9019420918281371, -0.3451699737786084, 0.2595344913308416, 0.2398878129563952,
            0.4132199174416981, 0.5151404104848398, -0.7509192082474564, 1.167593042966149,
            0, 0, 0, 1;

//    auto _inverse{typename Metrics<double, std::string>::E_Matrix{_extrinsic.transpose()}};

//    std::cout << "Inverse:\n" << _inverse << std::endl;

    cv::Mat tempExtrinisc{};
//    cv::Mat tempInv{};
    cv::eigen2cv(_extrinsic, tempExtrinisc);
//    cv::eigen2cv(_inverse, tempInv);

    std::cout << tempExtrinisc << std::endl;


    PTUModel.apply(tempExtrinisc, 1.4077826, 0.158825); //26
//    PTUModel.apply(tempInv, 1.40778, 0.158825);

    std::cout << "---\nResult:" << std::endl;
    std::cout << tempExtrinisc << std::endl;
//    std::cout << "InverseRes:\n" << tempInv << std::endl;

////    std::cout << "OpenCV apply()" << std::endl;
////    auto identity{Metrics<datatype, DataSet::Key>::createIdentityMatrices({1, 4, 4})};
////    typename Metrics<datatype, DataSet::Key>::E_Matrix reshaped{
////        typename Metrics<datatype, DataSet::Key>::E_Map(identity().data(), 4, 4)
////    };
////
////
////    std::cout << reshaped << std::endl;
////    auto mat{cv::Mat()};
////    cv::eigen2cv(reshaped, mat);
////    PTUModel.apply(mat, 4, 9);
////    std::cout << mat << std::endl;
//
////    std::cout << "---" << std::endl;
////    std::cout << "Rigid" << std::endl;
////    auto extrinsic{Metrics<datatype, int>{}};
////    extrinsic.createBuffer(0, {1, 16}, true);
////    extrinsic() = Metrics<datatype, DataSet::Key>::createIdentityMatrices({1, 4, 4})(); // Eigen::MatrixXd::Identity(4, 4);
//
////    PTUModel.applyRigid(extrinsic,
////                        PTUModel[{DataSet::CadModelTypes::RX_PTU, DataSet::CadModelTypes::RZ_PTU}],
////                        PTUModel[{DataSet::CadModelTypes::X_PTU, DataSet::CadModelTypes::Z_PTU}]);
////
////    std::cout << extrinsic << std::endl;
//////
////    std::cout << "---" << std::endl;
////    std::cout << "Pans" << std::endl;
////    PTUModel.applyPans(extrinsic,
////            Eigen::Matrix<double, 1, 1>(1.4077826) * PTUModel[DataSet::CadModelTypes::PAN_SCALE] * -1);
////
////    std::cout << extrinsic << std::endl;
////
//////    typename Metrics<datatype, DataSet::Key>::E_Matrix extrZ{1, 16};
//////    extrZ << -0.31168191, -0.2129099, 0.92602579, -1.01406177, 0.91974446, -0.31231345, 0.23776129, 0.35963153, 0.23858858, 0.92581299, 0.29316515, 0.51633436, 0, 0, 0, 1;
//////    std::cout << "before " << extrZ << std::endl;
////
////    std::cout << "---" << std::endl;
////    std::cout << "Z-Rigid" << std::endl;
////    PTUModel.applyZRigid(extrinsic, PTUModel[DataSet::CadModelTypes::RZ_OFFSET], PTUModel[DataSet::CadModelTypes::Z_OFFSET]);
////
////    std::cout << extrinsic << std::endl;
////
////    std::cout << "---" << std::endl;
////    std::cout << "Tilts" << std::endl;
////    PTUModel.applyTilts(extrinsic,
////            Eigen::Matrix<double, 1, 1>(0.158825) * PTUModel[DataSet::CadModelTypes::TILT_SCALE] * -1);
////    std::cout << extrinsic << std::endl;
////
////    std::cout << "---" << std::endl;
////    std::cout << "Rigid" << std::endl;
////    PTUModel.applyRigid(extrinsic,
////                        PTUModel[{DataSet::CadModelTypes::RX_CAMERA, DataSet::CadModelTypes::RZ_CAMERA}],
////                        PTUModel[{DataSet::CadModelTypes::X_CAMERA, DataSet::CadModelTypes::Z_CAMERA}]);
////
////    std::cout << extrinsic << std::endl;
//
//    /**
//     * Sample cv -> eigen -> conversion
//     */
//     // Given
//    auto cvMat{cv::Mat(2, 2, CV_64F)};
//
//    // Give eigen dims
//    typename Metrics<double, DataSet::Key>::E_Matrix eMat(cvMat.rows, cvMat.cols);
//    cv::cv2eigen(cvMat, eMat);
//
//    // Given
//    auto _eMat{typename Metrics<double, Shared::Key>::E_Matrix(2, 2)};
//
//    // Create cvMat
//    auto _cvMat{cv::Mat{}}; // Don't need to specify size
//    cv::eigen2cv(_eMat, _cvMat);
}


