#ifndef METROLOGY2020_DATACOLLECTOR_H
#define METROLOGY2020_DATACOLLECTOR_H

#include <memory>
#include <string>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>
#include <sstream>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

// HRL
#include <Shared/DataCollectionConfig.h>

// Utilities
#include "ThreadInterface/ThreadInterface.h"
#include "H5/H5Parser.h"
#include "OpenCVHelpers/Video/VideoObject.h"
#include "OpenCVHelpers/CSV/MatRecorder.h"

// Models
#include "Board/FiducialModel.h"

// Pose Estimation
#include "PoseEstimator.h"
#include "CollectionGenerator/CollectionGenerator.h"
#include "ConfigParser/DataCollectionJSONParser.h"

// LiMeInterface
#include "Threads/DeviceInterface/DeviceInterface.h"
#include "Controllers/PanTilt/PanTiltController.h"

// Models
#include "Board/FiducialModel.h"

// Debugging Defines
//#define LOG_POSE_ESTIMATION
//#define VISIT_POSE_ONLY
//#define SAVE_POSE_TO_CSV

constexpr int MIN_IMAGES_TO_PROCESS = 15;

class DataCollector: public ThreadInterface {
private:
	struct Image;
public:
	DataCollector(
		DataCollectionJSONParser * parser,
		const FiducialModel & fiducialModel,
		const bool visualizeOutput = false,
		const bool debug = false,
		const std::string & name = "DataCollector") :
			ThreadInterface(name),
			debug(debug),
			fiducialModel(fiducialModel),
			collectionGenerator(nullptr),
			panTiltController(nullptr),
			matrices(std::unordered_map<std::string, Mat3d>{}),
			imageHandles(std::unique_ptr<std::unordered_map<std::string, Image>>(new std::unordered_map<std::string, Image>())),
			poseEstimators(std::unique_ptr<std::unordered_map<std::string, PoseEstimator>>(new std::unordered_map<std::string, PoseEstimator>())),
			poseHandles(std::unique_ptr<std::unordered_map<std::string, std::vector<double>>>(new std::unordered_map<std::string, std::vector<double>>())),
			atCollectionSite(false),
			readyToSend(true),
			showVisuals(visualizeOutput),
			CHESSBOARD_CRNS_SIZE((int) fiducialModel.CHESSBOARD_CORNERS().size()),
			ID_PADDING((int) CHESSBOARD_CRNS_SIZE, -1),
			CORNER_PADDING((int) CHESSBOARD_CRNS_SIZE),
			pointsCollected(0),
			csvRecorders(std::unique_ptr<std::unordered_map<std::string, MatRecorder *>>(new std::unordered_map<std::string, MatRecorder *>()))
	{
		ThreadInterface::loadConfig(parser);

		// Create separate logging instance
		logger->addInstance(getName());
		eHandler->report("Welcome to Data Collection instance ID: " + getName(), Shared::Error::INFO, getName());
	}
	~DataCollector() override {
        delete panTiltController;
	}


	// Copy constructor/assignment
	DataCollector(const DataCollector & obj) = delete;
    DataCollector & operator=(const DataCollector & obj) = delete;

    // Move constructor/assignment
    DataCollector(DataCollector && obj)  noexcept :
        fiducialModel(std::move(obj.fiducialModel)),
        collectionGenerator(std::move(obj.collectionGenerator)),
        panTiltController(nullptr),
        matrices(std::move(obj.matrices)),
        imageHandles(std::move(obj.imageHandles)),
        poseEstimators(std::move(obj.poseEstimators)),
        poseHandles(std::move(obj.poseHandles)),
        atCollectionSite(obj.atCollectionSite),
        readyToSend(obj.readyToSend),
        showVisuals(obj.showVisuals),
        CHESSBOARD_CRNS_SIZE(obj.CHESSBOARD_CRNS_SIZE),
        ID_PADDING(obj.ID_PADDING),
        CORNER_PADDING(obj.CORNER_PADDING),
        pointsCollected(obj.pointsCollected),
        csvRecorders(std::move(obj.csvRecorders))
    {}
    DataCollector & operator=(DataCollector && obj) noexcept {
        fiducialModel = std::move(obj.fiducialModel);
        collectionGenerator = std::move(obj.collectionGenerator);
        panTiltController = nullptr;
        matrices = std::move(obj.matrices);
        imageHandles = std::move(obj.imageHandles);
        poseEstimators = std::move(obj.poseEstimators);
        poseHandles = std::move(obj.poseHandles);
        atCollectionSite = obj.atCollectionSite;
        readyToSend = obj.readyToSend;
        showVisuals = obj.showVisuals;
//        CHESSBOARD_CRNS_SIZE = obj.CHESSBOARD_CRNS_SIZE;
//        ID_PADDING = obj.ID_PADDING;
//        CORNER_PADDING = obj.CORNER_PADDING;
        pointsCollected = obj.pointsCollected;
        csvRecorders = std::move(obj.csvRecorders);
        return *this;
    }

	bool setup(void * params = nullptr) override;
	void run() override;

	// For VideoObject
	bool isVisualizing() {
		return showVisuals;
	}

protected:
	bool cleanAndExit() override;


private:
	// Functions 
	bool visitStagingArea();
	void logPoseInfo(const std::string & id, const std::vector<double> & target);
	void logPoseIrregularities(const std::string & ptu, const std::vector<double> & pose, const Shared::Error::Severity & minSeverity = Shared::Error::Severity::WARN);
	bool estimatePose(const std::string & camera,
					  double & reprojectionError,
					  cv::Mat & extrinsics,
					  std::vector<cv::Point2f> & uvCorners,
					  std::vector<int> & cornerIDs, 
					  const std::vector<int> gridPosition = std::vector<int>());
	void saveEstimatedPose(const std::string & camera, const cv::Mat & extrinsics, const cv::Mat & uvCorners, const cv::Mat & cornerIDs);
	void savePTPose(const std::string & ptu, const std::vector<double> & pose);
	void formatCollectedData();
	void saveToH5();	
	void saveImages();
    auto mappedKey(const std::string & key, std::unordered_map<std::string, Mat3d> & map) -> decltype(map.at(key));

	// Variables
	std::unordered_map<std::string, Mat3d> matrices{};

    FiducialModel fiducialModel{};
	//std::unordered_map<std::string, CameraModel<double>> cameraModels{};

	std::unique_ptr<CollectionGenerator> collectionGenerator;
	std::unique_ptr<std::unordered_map<std::string, Image>> imageHandles{};
	std::unique_ptr<std::unordered_map<std::string, PoseEstimator>> poseEstimators{};
	std::unique_ptr<std::unordered_map<std::string, std::vector<double>>> poseHandles{};
	std::unique_ptr<std::unordered_map<std::string, MatRecorder *>> csvRecorders{};

	PanTiltController * panTiltController;

	bool atCollectionSite{};
	bool readyToSend{};
	bool showVisuals{};	
	int pointsCollected{};

	static bool shouldCollectPose;
	
	// Const variable declarations
	const int CHESSBOARD_CRNS_SIZE; // = (int) Model::Charuco::board()->chessboardCorners.size();
	const std::vector<int> ID_PADDING; //{ { (int) Model::Charuco::board()->chessboardCorners.size(), -1} };
	const std::vector<cv::Point2f> CORNER_PADDING; //{ Model::Charuco::board()->chessboardCorners.size() };

	bool cvKeyListener(const int & key);
	cv::Mat getDisplayImage(const cv::Mat originalImg, const int & minImgWidthInPixels = 1024);

	struct Image {
		int badImagesProcessed{};
		std::vector<cv::Mat> saved{};
		cv::Mat image{};
		

		bool isBlurry() {
			// Should we collect pose?
			if (MIN_IMAGES_TO_PROCESS < 2) {
				shouldCollectPose = true;
			}
			else if (MIN_IMAGES_TO_PROCESS == 2) {
				shouldCollectPose = badImagesProcessed == (MIN_IMAGES_TO_PROCESS - 1);
			}
			else {
				auto processedImgs{ badImagesProcessed > 0 ? badImagesProcessed : 1 };
				shouldCollectPose = !(processedImgs % (MIN_IMAGES_TO_PROCESS - 1));
			}
			// Is blurry?
			return  ++badImagesProcessed <= MIN_IMAGES_TO_PROCESS;
		}

		void resetProcessedImages() {
			badImagesProcessed = 0;
		}
	};

	void logClassState();

	static std::string UUID() {
		static int uuidGenerator{};
		return std::to_string(uuidGenerator++);
	}

	bool debug;
};

#endif // METROLOGY2020_DATACOLLECTOR_H