#include <iostream>
#include <thread>
#include <vector>
#include <array>
#include <unordered_map>
#include <algorithm>
#include <iomanip>
#include <future>
#include <chrono>
#include <stdio.h>
#include <queue>
#include <sstream>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

// HRL
#include <Shared/OSDefines.h>
#include <Shared/SharedStructs.h>
#include <Shared/InterfaceConfig.h>

/**
 * FIXME - should only include <app-name>.h file
 * Application Includes
 */

// Interface
#include "LiMeInterface/Threads/Ximea/XimeaThread.h"
#include "LiMeInterface/Threads/Ximea/XimeaThread.h"
#include "LiMeInterface/Threads/ImperX/ImperxThread.h"
#include "LiMeInterface/Threads/PanTilt/PanTiltThread.h"
#include "LiMeInterface/ConfigParser/InterfaceJSONParser.h"

// Error/Logger Module
#include "Utilities/Logger/Logger.h"
#include "Utilities/ErrorHandler/ErrorHandler.h"

// System/Run-Time Configuration
#include "Utilities/SystemConfigParser/SystemJSONParser.h"

// Models Configuration
#include "Models/ConfigParser/ModelsJSONParser.h"
#include "Models/Board/FiducialModel.h"

// Data Collection
#include "DataCollection/ConfigParser/DataCollectionJSONParser.h"
#include "DataCollection/DataCollector/DataCollector.h"

// Dynamic Extrinsics
#include "DynamicExtrinsics/ConfigParser/DynamicExtrinsicsJSONParser.h"

// Fiducial Tracking
#include "TrackingFiducial/ConfigParser/TrackingJSONParser.h"
#include "TrackingFiducial/FiducialTracker.h"

// TODO - consider moving to outside class
constexpr bool isBlurry(int x) { return x < 10; }

// Static variables
static std::unordered_map<std::string, DeviceInterface *> components{};
static std::unordered_map<std::string, bool> saveImageFlags{};
static std::unordered_map<std::string, bool> poseEstimationFlags{};
static std::unordered_map<std::string, cv::Mat> rtImages{}; // Only holds most recent image
static std::unordered_map<std::string, std::vector<double>> rtPoses{}; // Only holds most recent pose

// JSON Configs
std::shared_ptr<SystemJSONParser> systemConfig{};
std::shared_ptr<InterfaceJSONParser> interfaceConfig{};
std::shared_ptr<ModelsJSONParser> modelsConfig{};

// Functions
void welcomeMessage();
bool WINAPI winExitHandler(DWORD signal);
std::unordered_map<std::string, cv::Mat> initializeRTCameraBuffer();
std::unordered_map<std::string, std::vector<double>> initializeRTPTUPoses();

// FIXME - OpenCV Helpers (offload helpers to auxilary file - Utilities (MOD))
bool cvKeyListener(const int & key);
cv::Mat getDisplayImage(const cv::Mat originalImg, const int & minImgWidthInPixels = 1024);
void writeCSV(const std::string & filename, const cv::Mat & mat);

// Modes
void clientInterface();
void dataCollection();
void dynamicExtrinsics();
void tracking();
void test();
void experiment();

/**
 * Main
 */
int main(int argc, char **argv)
{
	// Initialize Logger & ErrorHandler (should always be first step)
	ErrorHandler::getInstance()->setListeningLevel(Shared::Error::Severity::DEBUG);


	welcomeMessage();

	// Register Exit Handler
	if (!SetConsoleCtrlHandler((PHANDLER_ROUTINE)winExitHandler, true)) {
		return EXIT_FAILURE;
	}

	// Parse System JSON Config
	auto systemConfig = std::make_shared<SystemJSONParser>();
	if (!systemConfig->loadSystemConfig()) {
		return EXIT_FAILURE;
	}

	// Parse Interface JSON Config
	interfaceConfig = std::make_shared<InterfaceJSONParser>();
	if (!interfaceConfig->loadInterfaceConfig()) {
		return EXIT_FAILURE;
	}

	// Parse Models JSON Config
	modelsConfig = std::make_shared<ModelsJSONParser>();
	if (!modelsConfig->loadModelsConfig()) {
		return EXIT_FAILURE;
	}

	// Source environment and camera intrinsics config...
	DeviceInterface::sourceEnvironment(systemConfig->pairs, systemConfig->wFrame);
	DeviceInterface::sourceCameraIntrinsics(modelsConfig->ximea, modelsConfig->imperx);

	// PTUs
	for (auto & ptu : interfaceConfig->ptus) {
		DeviceInterface::addComponent(ptu.id, new PanTiltThread(ptu));
	}

	// Cameras
	for (auto & camera : interfaceConfig->cameras) {
		switch (camera.type) {
			case Interface::Imaging::Type::XIMEA:
				DeviceInterface::addComponent(camera.id, new XimeaThread(camera));
				break;
			case Interface::Imaging::Type::IMPERX:
				DeviceInterface::addComponent(camera.id, new ImperxThread(camera));
				break;
			default:
				ErrorHandler::getInstance()->report("Unknown camera type <" + std::to_string(static_cast<int>(camera.type)) + ">.", Shared::Error::Severity::KILL_AND_EXIT);
				break;
		}
	}

	// Real-time camera image storage (only holds most recent image)
	rtImages = initializeRTCameraBuffer();
	rtPoses = initializeRTPTUPoses();

	if (!DeviceInterface::initialize()) {
		DeviceInterface::safeExit();
		return EXIT_FAILURE;
	}

	// Sanity check!
	if (!ErrorHandler::getInstance()->shouldContinue()) {
		ErrorHandler::getInstance()->report("Error: Sanity check failed. Expect exit.");
		DeviceInterface::safeExit();
		return EXIT_FAILURE;
	}

	// FIXME - offload to function/class
	// Reset camera image save flags
	for (auto & camera : DeviceInterface::getActiveCameras()) {
		saveImageFlags[camera->getName()] = false;
		poseEstimationFlags[camera->getName()] = false;
	}

	// Log mode
	Logger::getInstance()->logMsg(Shared::Log::Msg("Mode: (" + systemConfig->mode.run + ") - Begin main thread..."));

	std::cout << "Mode: (" + systemConfig->mode.run + ") - Begin main thread..." << std::endl;

	switch (systemConfig->runningMode(systemConfig->mode.run)) {
		case Presets::Mode::Enum::INTERFACE: clientInterface(); break;
		case Presets::Mode::Enum::TEST: test(); break;
		case Presets::Mode::Enum::DATA_COLLECTION: dataCollection(); break;
		case Presets::Mode::Enum::TRACKING: tracking(); break;
		case Presets::Mode::Enum::EXPERIMENTAL: experiment(); break;
	}


	// Clean up any/remaining threads (will clean up Logger/ErrorHandler as well)
	DeviceInterface::safeExit();

	ErrorHandler::getInstance()->report("Exiting... Good Bye.");

	return EXIT_SUCCESS;
}

/**
 * Interface Environment
 */
void clientInterface() {
	static std::vector<int> count((int) interfaceConfig->cameras.size(), 0);
	static int index{};

	bool bKeepRunning = true;
	while (bKeepRunning) {
		// Check for errors...
		if (!ErrorHandler::getInstance()->shouldContinue()) {
			break;
		}

		// OpenCV - Keystroke listener
		int key{ cv::waitKey(1) };
		if (!(bKeepRunning = cvKeyListener(key))) {
			continue;
		}

		// Camera Loop
		for (auto & camera : DeviceInterface::getActiveCameras()) {
			// Get image from camera
			if (!camera->listen(&rtImages[camera->getName()])) {
				std::cerr << "Error: Failed to get image for < " << camera->getName() << ">" << std::endl;
				continue;
			}

			// Display Images
			cv::imshow(string("Image from ") + camera->getName(), getDisplayImage(rtImages[camera->getName()]));

			// Write to disk
			if (saveImageFlags[camera->getName()]) {
				if (index >= interfaceConfig->cameras.size()) {
					index = 0;
				}

				auto sstream{ std::stringstream{} };
				sstream << std::setfill('0') << std::setw(2) << count[index++]++;
				cv::imwrite(camera->getName() + "_" + sstream.str() + ".tiff", rtImages[camera->getName()]);
				cout << "Captured image saved in " << camera->getName() + ".tiff" << endl;
				saveImageFlags[camera->getName()] = false;
				// Clean
				sstream.str("");
			}
		}

		// PTU Loop
		for (auto & ptu : DeviceInterface::getActivePTUs()) {
			// Listen for keyboard input (a, s, d, w)
			ptu->cliListener(key);

			// Grab current position
			if (!ptu->listen(&rtPoses[ptu->getName()])) {
				ErrorHandler::getInstance()->report(
					"Failed to acquire PTU poses for unit ( " + ptu->getName() + ")",
					Shared::Error::Severity::WARN
				);
				continue;
			}

			// Display current position
			std::cout << ptu->getName() << " Pan: " << rtPoses[ptu->getName()][(int) Interface::PanTilt::Axis::PAN]
										<< " Tilt: " << rtPoses[ptu->getName()][(int) Interface::PanTilt::Axis::TILT] << std::endl;
		}
	}
}

/**
 * Data Collection Environment
 */
void dataCollection() {
	auto dataCollector{ 
		new DataCollector(new DataCollectionJSONParser(), FiducialModel(modelsConfig, "B0", cv::aruco::DICT_4X4_1000), true, true)
	};
	if (dataCollector->start()) {
		ErrorHandler::getInstance()->report("Successfully started DataCollector");
	}
	else {
		ErrorHandler::getInstance()->report(
			"Something bad occurred starting DataCollector",
			Shared::Error::Severity::KILL_AND_EXIT
		);
	}

	if (dataCollector->isVisualizing()) {
		while (!VideoObject::isDone()) {
			VideoObject::updateAllDisplayWindows();
			Sleep(10);
		}
	}

	// Wait for thread to complete...
	dataCollector->wait();
}

/**
 * Tracking Environment
 */
void tracking() {
	ErrorHandler::getInstance()->report("Entered Tracking Environment");
	auto hasWFOV{interfaceConfig->hasWFOV(DeviceInterface::wFrame.id)};
	if (!FiducialTracker::sourceInstances(new TrackingJSONParser(), 
										  FiducialModel(modelsConfig, "B1", cv::aruco::DICT_4X4_1000), 
										  hasWFOV)) {
		ErrorHandler::getInstance()->report(
			"(MAIN) Failed to source Tracking JSON.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return;
	}

	if (FiducialTracker::startInstances()) {
		ErrorHandler::getInstance()->report("(MAIN) Successfully started Fiducial Tracking. Instances: ( " + std::to_string(FiducialTracker::numberOfInstances()) + ")");
	}
	else {
		ErrorHandler::getInstance()->report(
			"Something bad occured while starting fiducial tracking instances.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
	}

	while (!VideoObject::isDone()) {
		VideoObject::updateAllDisplayWindows();
		Sleep(10);
	}

	// Wait for threads to complete
	FiducialTracker::allWait();
}

/**
 * Experimental
 */
void experiment() {
	ErrorHandler::getInstance()->report("Entered Experimental Environment");

	// Parse Data Collection JSON Config
	//auto dataCollectionParser{ DataCollectionJSONParser() };
	//if (!dataCollectionParser.loadDataCollectionConfig()) {
	//	return;
	//}
	//// Positions Map
	//auto collectionGen{ CollectionGenerator(dataCollectionParser.collection, dataCollectionParser.scanPattern) };
	//collectionGen.generatePositionMap();

	//// Record position map
	//ErrorHandler::getInstance()->report(collectionGen.toString());
	//ErrorHandler::getInstance()->report("# of Poses: " + collectionGen.numberOfPoses());

	//// Pose Estimator (Assumes only 1 Ximea camera and  1 Imperx camera)
	//std::unordered_map<std::string, PoseEstimator> estimatorMap{};

	//for (auto & camera : components[Shared::Device::Type::CAMERA]) {
	//	if (camera->getName().find("ximea") != std::string::npos) {
	//		estimatorMap.insert(
	//			{
	//				camera->getName(),
	//				PoseEstimator(dataCollectionParser.ximea.intrinsics.path,
	//					dataCollectionParser.ximea.distortionCoeff.path,
	//					Model::Charuco::board())
	//			}
	//		);
	//	}
	//	else {
	//		estimatorMap.insert(
	//			{
	//				camera->getName(),
	//				PoseEstimator(dataCollectionParser.imperX.intrinsics.path,
	//					dataCollectionParser.imperX.distortionCoeff.path,
	//					Model::Charuco::board())
	//			}
	//		);
	//	}
	//}

	//
	//// Logic Variables
	//bool atPose{};
	//bool collect{};
	//bool readyToSend{ true }; // Base-case is true (@ home)
	//std::unordered_map<std::string, int> badImgsProcessed{};
	//std::vector<cv::Point2d> posePair(2, cv::Point2d{0, 0});
	//int iterations{};
	//int index{};
	//auto data{ cv::Mat{} };
	//auto _data{ std::vector<double>{} };

	//static const int MAX_ITER = 50;
	//static const int SAMPLE_SIZE = 1;

	//bool bKeepRunning = true;
	//while (bKeepRunning) {
	//	// Check for errors...
	//	if (!ErrorHandler::getInstance()->shouldContinue()) {
	//		break;
	//	}

	//	// OpenCV - Keystroke listener
	//	int key{ cv::waitKey(1) };
	//	if (!(bKeepRunning = cvKeyListener(key))) {
	//		continue;
	//	}

	//	// Camera Loop
	//	for (auto & camera : components[Shared::Device::Type::CAMERA]) {
	//		// Get image from camera
	//		if (!camera->listen(&rtImages[camera->getName()])) {
	//			std::cerr << "Error: Failed to get image for < " << camera->getName() << ">" << std::endl;
	//			continue;
	//		}

	//		// Display Images
	//		cv::imshow(string("Image from ") + camera->getName(), getDisplayImage(rtImages[camera->getName()]));

	//		if (atPose) {
	//			// Ensure images evaluated are not blurry
	//			if (isBlurry(badImgsProcessed[camera->getName()]++)) {
	//				if (!camera->listen(&rtImages[camera->getName()])) {
	//					std::cerr << "Error: Failed to get image for < " << camera->getName() << ">" << std::endl;
	//				}

	//				std::cout << "Removed blurry img(s): " << badImgsProcessed[camera->getName()] << std::endl;

	//				continue;
	//			}
	//			else {
	//				// Reset 
	//				badImgsProcessed[camera->getName()] = 0;
	//			}
	//			collect = true;

	//			// Pose Estimation
	//			double reprojectionErr{};
	//			if (!estimatorMap.at(camera->getName()).estimate_pose_charucoboard(rtImages[camera->getName()], &reprojectionErr)) {
	//				ErrorHandler::getInstance()->report(
	//					"Failed to estimate pose!",
	//					Shared::Error::Severity::KILL_AND_EXIT
	//				);
	//				break;
	//			}
	//			ErrorHandler::getInstance()->report("Succesfully Estimated Pose.");

	//			// Rvec and Tvec
	//			auto rVec{ cv::Mat{} };
	//			auto tVec{ cv::Mat{} };
	//			if (!estimatorMap.at(camera->getName()).get_rvec_tvec_charucoboard(rVec, tVec)) {
	//				ErrorHandler::getInstance()->report(
	//					"Failed to calculate rVec and tVec!",
	//					Shared::Error::Severity::WARN
	//				);
	//				break;
	//			}

	//			// Add to data
	//			_data.push_back(reprojectionErr);
	//			auto tempRVec{ std::vector<double>{rVec.begin<double>(), rVec.end<double>()} };
	//			auto tempTVec{ std::vector<double>{tVec.begin<double>(), tVec.end<double>()} };
	//			_data.insert(_data.end(), tempRVec.begin(), tempRVec.end());
	//			_data.insert(_data.end(), tempTVec.begin(), tempTVec.end());
	//		}

	//	}

	//	// PTU Loop
	//	for (auto & ptu : components[Shared::Device::Type::PTU]) {
	//		// Listen for keyboard input (a, s, d, w)
	//		ptu->cliListener(key);

	//		// Grab current position
	//		if (!ptu->listen(&rtPoses[ptu->getName()])) {
	//			ErrorHandler::getInstance()->report(
	//				"Failed to acquire PTU poses for unit ( " + ptu->getName() + ")",
	//				Shared::Error::Severity::WARN
	//			);
	//			continue;
	//		}

	//		// Display current position
	//		std::cout << ptu->getName() << " Pan: " << rtPoses[ptu->getName()][(int)Interface::PanTilt::Axis::PAN]
	//			<< " Tilt: " << rtPoses[ptu->getName()][(int)Interface::PanTilt::Axis::TILT] << std::endl;

	//		if (readyToSend) {
	//			// Save next position into pose pair
	//			if (!iterations) {
	//				if (collectionGen.hasNextPTPose()) {
	//					auto target{ collectionGen.getNextPTPose() };
	//					posePair[1] = cv::Point2d{ target[0], target[1] };
	//				}
	//				else {
	//					bKeepRunning = false;
	//					ErrorHandler::getInstance()->report("No more collection points to process. Expecting exit.");
	//				}
	//			}

	//			if (!index) {
	//				iterations++;
	//				ErrorHandler::getInstance()->report("Iteration(s): ( " + std::to_string(iterations) + ")");
	//			}

	//			// Send ptu to next collection point...
	//			index = ++index % 2;
	//			auto target{ std::vector<double>{posePair[index].x, posePair[index].y} };
	//			ptu->step(&target);
	//			ErrorHandler::getInstance()->report("Sent PTU. Target: PT (" + std::to_string(posePair[index].x) + "," + std::to_string(posePair[index].y) + ")");
	//			readyToSend = false;				
	//		}
	//		// At target?
	//		else if (static_cast<PanTiltController *>(ptu->getDevice())->hasReachedTarget()) {
	//			ErrorHandler::getInstance()->report(
	//				"Reached Pose: P/T ( " + std::to_string(rtPoses[ptu->getName()][(int)Interface::PanTilt::Axis::PAN]) +
	//				", " + std::to_string(rtPoses[ptu->getName()][(int) Interface::PanTilt::Axis::TILT]) + ")"
	//			);

	//			static int counter{};

	//			if (!atPose) {
	//				// Record reprojection error, r-vec, and t-vec first
	//				atPose = true;
	//				continue;
	//			}

	//			if (!collect) {
	//				continue;
	//			}
	//	

	//			// Record position N times... (looking for momentum swing)
	//			if (counter++ < SAMPLE_SIZE) {
	//				// Collect position data
	//				if (!_data.empty()) {
	//					auto temp{ std::vector<double>{rtPoses[ptu->getName()][0], rtPoses[ptu->getName()][1],
	//						posePair[index].x, posePair[index].y} };
	//					_data.insert(_data.end(), temp.begin(), temp.end());
	//				}
	//				else {
	//					_data = std::vector<double>{rtPoses[ptu->getName()][0], rtPoses[ptu->getName()][1],
	//						posePair[index].x, posePair[index].y};
	//				}


	//				//auto _data{ std::vector<double>{
	//				//	// Actual
	//				//	rtPoses[ptu->getName()][0], rtPoses[ptu->getName()][1],
	//				//	// Expected
	//				//	posePair[index].x, posePair[index].y
	//				//	} };
	//				std::cout << "Write to Mat" << std::endl;
	//				data.push_back(cv::Mat(1, 11, CV_64F, _data.data()));
	//				_data.clear();
	//			}
	//			else {
	//				if (!index) {
	//					iterations %= MAX_ITER;
	//				}
	//				readyToSend = true;
	//				atPose = false;
	//				collect = false;
	//				counter = 0;
	//			}
	//		}
	//	}
	//}

	//// Save data to file
	//writeCSV("Logs/ptu_swing_expr_data.csv", data);
}

/*
 * Testing Enviroment
 * - Setup to debug blurry images
 */
void test() {
	//auto numCameras{DeviceInterface::getActiveCameras().size()};
	//static std::vector<int> count((int) numCameras, 0);
	//static int index{};

	////// Parse Dynamic Extrinsics JSON Config
	////auto dynamicExtrinicsParser{ DynamicExtrinsicsJSONParser() };
	////if (!dynamicExtrinicsParser.loadDynamicExtrinsicConfig()) {
	////	return;
	////}
	//// Pose Estimator (Assumes only 1 Ximea camera and  1 Imperx camera)
	//std::unordered_map<std::string, PoseEstimator> estimatorMap{};
	//std::cout << DeviceInterface::ximea.at("xi_I0").cameraMatrix.path << std::endl;
	////for (auto & camera : DeviceInterface::getActiveCameras()) {
	////	if (camera->getName().find("ximea") != std::string::npos) {
	////		estimatorMap.insert(
	////			{
	////				camera->getName(), 
	////				PoseEstimator(
	////					DeviceInterface::ximea.at("xi_I0").cameraMatrix.path,
	////					DeviceInterface::ximea.at("xi_I0").distortionCoeff.path,
	////					Model::Charuco::board()
	////				)
	////			}
	////		);
	////	}
	////	else {
	////		/*estimatorMap.insert(
	////			{
	////				camera->getName(),
	////				PoseEstimator(
	////					DeviceInterface::imperx.at(camera->getSensorID()).cameraMatrix.path,
	////					DeviceInterface::imperx.at(camera->getSensorID()).distortionCoeff.path,
	////					Model::Charuco::board()
	////				)
	////			}
	////		);*/
	////	}
	////	estimatorMap.at(camera->getName()).set_board_detection_threshold(100);
	////}

	//bool bKeepRunning = true;
	//while (bKeepRunning) {
	//	// Check for errors...
	//	if (!ErrorHandler::getInstance()->shouldContinue()) {
	//		break;
	//	}

	//	// OpenCV - Keystroke listener
	//	int key{ cv::waitKey(1) };
	//	if (!(bKeepRunning = cvKeyListener(key))) {
	//		continue;
	//	}

	//	// Camera Loop
	//	for (auto & camera : DeviceInterface::getActiveCameras()) {
	//		// Get image from camera
	//		if (!camera->listen(&rtImages[camera->getName()])) {
	//			std::cerr << "Error: Failed to get image for < " << camera->getName() << ">" << std::endl;
	//			continue;
	//		}

	//		// Display Images
	//		cv::imshow(string("Image from ") + camera->getName(), getDisplayImage(rtImages[camera->getName()]));

	//		// Write to disk
	//		if (saveImageFlags[camera->getName()]) {
	//			if (index >= config.cameras.size()) {
	//				index = 0;
	//			}

	//			cv::imwrite(camera->getName() + "_" + std::to_string(count[index++]++) + ".tiff", rtImages[camera->getName()]);
	//			cout << "Captured image saved in " << camera->getName() + ".tiff" << endl;
	//			saveImageFlags[camera->getName()] = false;
	//		}

	//		if (poseEstimationFlags[camera->getName()]) {
	//			// Attempt Pose Estimation...
	//			if (!estimatorMap.at(camera->getName()).estimate_pose_charucoboard(rtImages[camera->getName()])) {
	//				ErrorHandler::getInstance()->report(
	//					"Failed to estimate pose!",
	//					Shared::Error::Severity::KILL_AND_EXIT
	//				);
	//				break;
	//			}
	//			ErrorHandler::getInstance()->report("Succesfully Estimated Pose.");

	//			// Get camera extrinsics
	//			auto extrinsics{ cv::Mat{} };
	//			if (!estimatorMap.at(camera->getName()).get_extrinsics4x4_charucoboard(extrinsics)) {
	//				ErrorHandler::getInstance()->report(
	//					"Failed to grab camera extrinsics!",
	//					Shared::Error::Severity::KILL_AND_EXIT
	//				);
	//				break;
	//			}

	//			// Get UV corners and associated IDs
	//			auto uvCorners{ std::vector<cv::Point2f>{} };
	//			auto cornerIDs{ std::vector<int>{} };
	//			if (!estimatorMap.at(camera->getName()).get_corner_uv_id_charucoboard(uvCorners, cornerIDs)) {
	//				ErrorHandler::getInstance()->report(
	//					"Failed to acquire UV corners and associated IDs!",
	//					Shared::Error::Severity::KILL_AND_EXIT
	//				);
	//				break;
	//			
	//			}

	//			std::cout << "\rMarker(s): ( " << uvCorners.size() << ")" << std::endl;
	//			poseEstimationFlags[camera->getName()] = false;
	//		}
	//	}

	//	// PTU Loop
	//	for (auto & ptu : DeviceInterface::getActivePTUs()) {
	//		// Listen for keyboard input (a, s, d, w)
	//		ptu->cliListener(key);

	//		// Grab current position
	//		if (!ptu->listen(&rtPoses[ptu->getName()])) {
	//			ErrorHandler::getInstance()->report(
	//				"Failed to acquire PTU poses for unit ( " + ptu->getName() + ")",
	//				Shared::Error::Severity::WARN
	//			);
	//			continue;
	//		}

	//		// Display current position
	//		std::cout << ptu->getName() << " Pan: " << rtPoses[ptu->getName()][(int) Interface::PanTilt::Axis::PAN]
	//			<< " Tilt: " << rtPoses[ptu->getName()][(int) Interface::PanTilt::Axis::TILT] << std::endl;
	//	}
	//}
}

bool WINAPI winExitHandler(DWORD signal) {
	switch (signal) {
	case CTRL_C_EVENT:
	case CTRL_CLOSE_EVENT:
	case CTRL_SHUTDOWN_EVENT: {
		//safeExit();
		DeviceInterface::safeExit();
		break;
	}
	}

	return true; // TODO - not confident in this impl.
}

void welcomeMessage() {
	std::string welcome{ "Closed Loop Metrology\n=====================" };
	std::cout << welcome << std::endl;

	ErrorHandler::getInstance()->report(welcome);


	//Logger::getInstance()->logMsg(Shared::Log::Msg(welcome));

	//if (!Logger::getInstance()->isListening()) {
	//	Logger::getInstance()->flush();
	//}
}

bool cvKeyListener(const int & key) {
	if (key == 'q' || key == 'Q') //Quit
	{
		//safeExit();
		DeviceInterface::safeExit();
		return false;
	}
	else if (key == 'C' || key == 'c') //Save 1 image from each cmaera
	{
		for (auto &flag : saveImageFlags)
		{
			flag.second = true;
		}
	}
	else if (key == 'P' || key == 'p') {
		for (auto & flag : poseEstimationFlags) {
			flag.second = true;
		}
	}

	// Default - true (keep running)
	return true;
}

std::unordered_map<std::string, cv::Mat> initializeRTCameraBuffer() {
	static std::unordered_map<std::string, cv::Mat> rtCameraBuffer{};
	for (auto & camera : DeviceInterface::getActiveCameras()) {
		rtCameraBuffer.insert({ camera->getName(), cv::Mat{} });
	}
	return rtCameraBuffer;
}


std::unordered_map<std::string, std::vector<double>> initializeRTPTUPoses() {
	static std::unordered_map<std::string, std::vector<double>> rtPTUBuffer{};
	for (auto & ptu : DeviceInterface::getActivePTUs()) {
		rtPTUBuffer.insert({ ptu->getName(), std::vector<double>(2) });
	}
	return rtPTUBuffer;
}

/**
 * OpenCV Helpers
 */
cv::Mat getDisplayImage(const cv::Mat originalImg, const int & minImgWidthInPixels) {
	// Prep image for display
	cv::Mat displayImg(originalImg);
	float scale = 1.0;
	do {
		scale *= 0.5;
		cv::resize(displayImg, displayImg, cv::Size(), scale, scale);
	} while (displayImg.cols > minImgWidthInPixels);

	return displayImg;
}

void writeCSV(const std::string & filename, const cv::Mat & mat) {
	ofstream myfile;
	myfile.open(filename.c_str());
	myfile << cv::format(mat, cv::Formatter::FMT_CSV) << std::endl;
	myfile.close();
}
