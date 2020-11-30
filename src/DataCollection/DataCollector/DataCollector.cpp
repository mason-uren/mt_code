#include "DataCollector.h"

// Static variable initialization
bool DataCollector::shouldCollectPose = false;

bool DataCollector::setup(void * params) {
	bool result{};

	// Load JSON Config
	auto config{this->Config<DataCollectionJSONParser>()};
	if (!(result = config->loadDataCollectionConfig())) {
		this->eHandler->report(
			"Failed to load DataCollection config!",
			Shared::Error::Severity::KILL_AND_EXIT,
			this->name
		);
		return result;
	}

	// Generate collection map...
	this->collectionGenerator = std::unique_ptr<CollectionGenerator>(new CollectionGenerator(config->collection, config->scanPattern, 11, this->eHandler)); // Note: 11 is random seed
	this->collectionGenerator->generatePositionMap();
	// Record positions
	this->eHandler->report(this->collectionGenerator->toString(), Shared::Error::Severity::INFO, this->name);
	this->eHandler->report("# of Poses: " + std::to_string(this->collectionGenerator->numberOfPoses()), Shared::Error::Severity::INFO, this->name);
	
	for (auto & camera : DeviceInterface::getActiveCameras()) {
		// Setup pose estimators based on connected cameras...
		auto isXimea{ camera->getName().find("ximea") != std::string::npos };

		// NOTE: Data collector will only be expecting one ximea camera
		auto intrinsics{ isXimea ? 
			DeviceInterface::ximea.at(DeviceInterface::pairs.begin()->second.camera.intrinsics) : 
			DeviceInterface::imperx.at(DeviceInterface::wFrame.intrinsics)
		};
		
		auto markerScale{isXimea ? 0.2 : 0.7};
		// Setup pose estimators...
		this->poseEstimators->emplace<std::string, PoseEstimator>(camera->getName(), PoseEstimator{ intrinsics, this->fiducialModel, this->debug, markerScale, 1, this->getName() } );

		// Setup image handlers...
		std::pair<std::string, Image> imagePair(camera->getName(), Image{});
		this->imageHandles->insert(imagePair);

		// Apply fixed charuco board detection threshold different than the default:
		this->poseEstimators->at(camera->getName()).set_board_detection_threshold(100); //this should be in config json
	}

	for (auto & ptu : DeviceInterface::getActivePTUs()) {
		// Setup pose handles
		this->poseHandles->insert({ ptu->getName(), std::vector<double>{ 0, 0 } });

#ifdef SAVE_POSE_TO_CSV
		this->csvRecorders->insert({ ptu->getName(), new MatRecorder(this->eHandler) });
#endif
	}

	return result;
}

void DataCollector::run() {
	this->running = this->initialized;

	// Temporaries
	double reprojectionError{};
	auto extrinsics{ cv::Mat{} };
	auto uvCorners{ std::vector<cv::Point2f>{} };
	auto cornerIDs{ std::vector<int>{} };
	auto nearPoseCounter{ int {} };
	auto gridPosition{ std::vector<int>{} };
	// NOTE: not unset (overwritten)
	auto currentPose{ std::vector<double>(2, 0) };
	auto targetPose{ currentPose };
	auto prevPose{ currentPose };
	auto deltaPose{ currentPose };


	// Lambda functions 
	auto padUVCorners = [&]() {
		auto padding{ this->CORNER_PADDING };
		std::copy(uvCorners.begin(), uvCorners.end(), padding.begin());
		return cv::Mat(this->CHESSBOARD_CRNS_SIZE, 2, CV_32F, padding.data()).clone();
	};

	auto padCornerIDs = [&]() {
		auto padding{ this->ID_PADDING };
		std::copy(cornerIDs.begin(), cornerIDs.end(), padding.begin());
		return cv::Mat(padding).clone().t();
	};

	auto clearTemporaries = [&]() {
		reprojectionError = 0;
		extrinsics = cv::Mat{};
		uvCorners = std::vector<cv::Point2f>{};
		cornerIDs = std::vector<int>{};
		nearPoseCounter = 0;
		gridPosition = std::vector<int>{};
	};

	// Visit staging area (will always visit, Default: is current position)
	this->visitStagingArea();

	// Main loop...
	try {
		while (running && this->eHandler->shouldContinue()) { 
			// Listen for user key
			if (!this->cvKeyListener(VideoObject::getUserKey())) {
				this->eHandler->report("Exit triggered by cv::waitKey", Shared::Error::INFO, this->name);
				break;
			}

			// PTU Loop
			for (auto & ptu : DeviceInterface::getActivePTUs()) {
				this->panTiltController = static_cast<PanTiltController *>(ptu->getDevice());


				// Grab current position
				// Deep copy via assignment
				prevPose = this->poseHandles->at(ptu->getName());
				if (!ptu->listen(&this->poseHandles->at(ptu->getName()))) {
					this->eHandler->report(
						"Failed to acquire PTU poses for unit ( " + ptu->getName() + ")",
						Shared::Error::Severity::KILL_AND_EXIT, this->name
					);
					break;
				}

				if (readyToSend) {
					// Report if pose has slipped!!
					this->logPoseIrregularities(ptu->getName(), prevPose);

					if (this->collectionGenerator->hasNextPTPose()) {
						targetPose = this->collectionGenerator->getNextPTPose(gridPosition);
						ptu->step(&targetPose);
						this->readyToSend = false;
						this->atCollectionSite = false;
						this->eHandler->report(
							"Sent PTU. Target: PT (" + std::to_string(targetPose[0]) + "," + std::to_string(targetPose[1]) + ")", 
							Shared::Error::Severity::DEBUG, this->name
						);
					}
					else {
						this->eHandler->report("No more collection points to process. Expecting exit.", Shared::Error::Severity::DEBUG, this->name);
					}
				}
				// At target?
				else if (panTiltController->hasReachedTarget()) {
					this->atCollectionSite = true;


					if (this->shouldCollectPose) {
						this->shouldCollectPose = false;
#ifndef VISIT_POSE_ONLY
						// Collect data...
						this->savePTPose(ptu->getName(), this->poseHandles->at(ptu->getName()));
						this->eHandler->report("Recorded Pose", Shared::Error::Severity::DEBUG, this->name);
						this->eHandler->report("Acquisition PT: " + std::to_string(this->pointsCollected++), Shared::Error::Severity::DEBUG, this->name);
#endif
#ifdef SAVE_POSE_TO_CSV
						std::vector<double> aquiredVsTarget{ this->poseHandles->at(ptu->getName()) };
						aquiredVsTarget.insert(aquiredVsTarget.end(), targetPose.begin(), targetPose.end());
						auto data{ cv::Mat(aquiredVsTarget) };
						this->csvRecorders->at(ptu->getName())->write(data.t());
#endif
					}


					this->eHandler->report(
						"[" + std::to_string(gridPosition[0]) + ", " + std::to_string(gridPosition[1]) + "] " +
						"Reached Pose: P/T ( " + std::to_string(this->poseHandles->at(ptu->getName()).at((int)Interface::PanTilt::Axis::PAN)) +
						", " + std::to_string(this->poseHandles->at(ptu->getName()).at((int)Interface::PanTilt::Axis::TILT)) + ")",
						Shared::Error::Severity::DEBUG, this->name
					);
				}
				else {
					// Display current position
					this->logPoseInfo(ptu->getName(), targetPose);
				}
			}

			// Camera loop
			for (auto & camera : DeviceInterface::getActiveCameras()) {
				// Always be listening for camera images (For developer/client viewing)
				if (!camera->listen(&this->imageHandles->at(camera->getName()).image)) {
					this->eHandler->report(
						"Error: Failed to get image for < " + camera->getName() + ">",
						Shared::Error::Severity::WARN, this->name
					);
					continue;
				}

				// Display Images
				if (this->showVisuals) {
					VideoObject::imshow(camera->getName(), this->getDisplayImage(this->imageHandles->at(camera->getName()).image));
				}

				if (this->atCollectionSite) {
					// Remove blurry images before processing
					if (this->imageHandles->at(camera->getName()).isBlurry()) {
						this->eHandler->report(
							"Removing blurry img(s): " + std::to_string(this->imageHandles->at(camera->getName()).badImagesProcessed),
							Shared::Error::Severity::DEBUG, this->name	
						);
						continue;
					}
					else {
						this->imageHandles->at(camera->getName()).resetProcessedImages();
					}

					// Save image (for offline analysis)
					this->imageHandles->at(camera->getName()).saved.push_back(this->imageHandles->at(camera->getName()).image);

#ifndef VISIT_POSE_ONLY
					if (!this->estimatePose(camera->getName(), reprojectionError, extrinsics, uvCorners, cornerIDs, gridPosition)) {
						break;
					}

#ifdef LOG_POSE_ESTIMATION
					// TODO - remove me (DEBUG) [VERIFIED!]
					auto sstream{ std::stringstream{} };
					sstream << extrinsics;
					this->eHandler->report("Extrinsics\n" + sstream.str(), Shared::Error::Severity::DEBUG, this->name);
					sstream.str("");

					sstream << uvCorners;
					this->eHandler->report("UV-Corners\n" + sstream.str(), Shared::Error::Severity::DEBUG, this->name);
					sstream.str("");

					for (auto & val : cornerIDs) {
						sstream << val << " ";
					}
					this->eHandler->report("IDs\n" + sstream.str(), Shared::Error::Severity::DEBUG, this->name);
					sstream.str("");
					// END
#endif

					// Collect data...
					this->saveEstimatedPose(camera->getName(), extrinsics, padUVCorners(), padCornerIDs());

					// Clear temporary objects...
					clearTemporaries();
#endif

					// Inform system of completion.
					this->readyToSend = true;
					this->eHandler->report("Succesfully Estimated Pose.", Shared::Error::Severity::INFO, this->name);
				}
			}

			// Exit condition
			this->running = !(!this->collectionGenerator->hasNextPTPose() && this->atCollectionSite && this->readyToSend);

			Sleep(10);
		}
	}
	catch (const std::out_of_range & error) {
		this->logClassState();
		this->eHandler->report(
			this->getName() + ":" + error.what(),
			Shared::Error::Severity::KILL_AND_EXIT, this->name
		);
	}
	catch (...) {
		this->logClassState();
		this->eHandler->report(
			this->getName() + ": Unknown error has occured. Trigger Exit.",
			Shared::Error::Severity::KILL_AND_EXIT, this->name
		);
	}

	if (this->showVisuals) {
		VideoObject::setDone();
	}

	if (true) { //this->Config<DataCollectionJSONParser>()->saveImages) {
		this->saveImages();
	}

#ifndef VISIT_POSE_ONLY
	// Format collected data...
	this->formatCollectedData();

	// Write to h5 file...
	this->saveToH5();
#endif

#ifdef SAVE_POSE_TO_CSV
	// Write PTU collection positions to separate csv files
	for (auto & ptu : DeviceInterface::getActivePTUs()) {
		this->csvRecorders->at(ptu->getName())->save(ptu->getName() + "_CollectionPositions.csv");
	}
#endif
}

bool DataCollector::cleanAndExit() {
	this->running = false;

	if (this->initialized) {
		// Close all OpenCV windows...
		if (!VideoObject::isDone()) {
			VideoObject::setDone();
		}
	}

	return !this->running;
}

bool DataCollector::visitStagingArea() {
	bool hasReachedStaging{};
	auto config{ this->Config<DataCollectionJSONParser>() };

	auto sstream{ std::stringstream{} };
	std::vector<double> target{ config->stagingPt };

	// Lambda Functions
	// TODO - should probably be moved to higher class (used by DataCollector/PanTiltController)
	auto displayStatus = [&](const std::vector<double> position, const bool atHome) {
		sstream.str("");
		try {
			sstream << "\rSend to Staging [ID: " + this->panTiltController->getName() + "] => P/T ( " <<
				position.at((int)Interface::PanTilt::Axis::PAN) << ", " <<
				position.at((int)Interface::PanTilt::Axis::TILT) << ")"; // << std::endl;
			std::cout << sstream.str() << std::flush;

		}
		catch (const std::out_of_range & error) {
			// FIXME - add ERRORHANDLER
			std::cout << error.what() << std::endl;
		}
	};
	// END Lambda Functions

	while (!hasReachedStaging) {
		// Assuming 1 camera mounted to PTU
		for (auto & ptu : DeviceInterface::getActivePTUs()) {
			this->panTiltController = static_cast<PanTiltController *>(ptu->getDevice());
			if (!ptu->listen(&this->poseHandles->at(ptu->getName()))) {
				this->eHandler->report(
					"Failed to acquire PTU poses for unit ( " + ptu->getName() + ")",
					Shared::Error::Severity::KILL_AND_EXIT, this->name
				);
				break;
			}

			// If current and staging points are not equivalent (within EPSILON)
			hasReachedStaging = panTiltController->angleEquivalence(this->poseHandles->at(ptu->getName()), target);

			// Display current position...
			displayStatus(this->poseHandles->at(ptu->getName()), hasReachedStaging);

			// If current and staging points are not equivalent (within EPSILON)
			if (readyToSend && !hasReachedStaging) { //this->panTiltController->angleEquivalence(this->poseHandles->at(ptu->getName()), config->stagingPt)) {
				auto target{ config->stagingPt };
				ptu->step(static_cast<void *>(&target));
				this->eHandler->report("Sent PTU. Staging: PT (" +
					std::to_string(config->stagingPt[0]) + "," +
					std::to_string(config->stagingPt[1]) + ")",
					Shared::Error::Severity::DEBUG, this->name);
				readyToSend = false;
			}
			// Shoud be <hasReachedTarget()>
			else if (hasReachedStaging) {
				this->eHandler->report("Reached staging area!", Shared::Error::Severity::DEBUG, this->name);
				readyToSend = true;
				hasReachedStaging = true;
			}
			else {
				this->logPoseInfo(ptu->getName(), config->stagingPt);
			}
		}

		Sleep(50);
	}

	// TODO - add timeout
	return true;
}

void DataCollector::logPoseInfo(const std::string & id, const std::vector<double> & target) {
	auto currentPose{ this->poseHandles->at(id) };
	auto deltaPose{ std::vector<double>{
		std::abs(target[(int)Interface::PanTilt::Axis::PAN] - currentPose[(int)Interface::PanTilt::Axis::PAN]),
		std::abs(target[(int)Interface::PanTilt::Axis::TILT] - currentPose[(int)Interface::PanTilt::Axis::TILT])
	} };
	auto sstream{ std::stringstream{} };
	sstream << id << " Pan/Tilt: (" << currentPose.at((int)Interface::PanTilt::Axis::PAN) << ", " << currentPose.at((int)Interface::PanTilt::Axis::TILT) <<
		") Delta: ( " << deltaPose.at((int)Interface::PanTilt::Axis::PAN) << ", " << deltaPose.at((int)Interface::PanTilt::Axis::TILT) << ")";

	// Log Current position (DEBUG)
	this->eHandler->report(sstream.str(), Shared::Error::Severity::DEBUG, this->name);
}

void DataCollector::logPoseIrregularities(const std::string & ptu, const std::vector<double> & prevPose, const Shared::Error::Severity & minSeverity) {
	try {
		// Check if position has changed
		auto equivalent{ panTiltController->angleEquivalence(prevPose, this->poseHandles->at(ptu), true) };

		auto currentPose{ this->poseHandles->at(ptu) };
		auto delta{ std::vector<double>{std::abs(currentPose[0] - prevPose[0]), std::abs(currentPose[1] - prevPose[1])} };

		if (!equivalent) {
			this->eHandler->report(
				"PT Mis-match Error: Expected ( " + std::to_string(prevPose[(int)Interface::PanTilt::Axis::PAN]) + "," + std::to_string(prevPose[(int)Interface::PanTilt::Axis::TILT]) + "), " +
				"Observed: ( " + std::to_string(currentPose.at((int)Interface::PanTilt::Axis::PAN)) + "," + std::to_string(currentPose.at((int)Interface::PanTilt::Axis::TILT)) + "), " + 
				"Delta: ( " + std::to_string(delta[0]) + ", " + std::to_string(delta[1]) + ")",
				minSeverity, this->name
			);
		}
	}
	catch (const std::logic_error & err) {
		this->eHandler->report(
			err.what(),
			Shared::Error::Severity::KILL_AND_EXIT, this->name
		);
	}
}

bool DataCollector::estimatePose(const std::string & camera, 
								 double & reprojectionError, 
								 cv::Mat & extrinsics, 
								 std::vector<cv::Point2f> & uvCorners, 
								 std::vector<int> & cornerIDs, 
								 const std::vector<int> gridPosition) {
#ifndef VISIT_POSE_ONLY
	// Pose Estimation
	if (!this->poseEstimators->at(camera).estimate_pose_charucoboard(this->imageHandles->at(camera).image, &reprojectionError)) {
		ErrorHandler::getInstance()->report(
			camera + ": Failed to estimate pose!",
			Shared::Error::Severity::KILL_AND_EXIT
		);

		return false;
	}

	this->eHandler->report(
		gridPosition.empty() ? 
			"ReprojectionError: ( " + std::to_string(reprojectionError) + ")" :
			"[" + std::to_string(gridPosition[0]) + ", " + std::to_string(gridPosition[1]) + "] " + "ReprojectionError: ( " + std::to_string(reprojectionError) + ")",
		Shared::Error::Severity::DEBUG, this->name
	);

	// Get camera extrinsics...
	if (!this->poseEstimators->at(camera).get_extrinsics4x4_charucoboard(extrinsics)) {
		ErrorHandler::getInstance()->report(
			camera + ": Failed to grab camera extrinsics!",
			Shared::Error::Severity::KILL_AND_EXIT, this->name
		);

		return false;
	}

	// UV-corners
	if (!this->poseEstimators->at(camera).get_corner_uv_id_charucoboard(uvCorners, cornerIDs)) {
		ErrorHandler::getInstance()->report(
			camera + ": Failed to acquire UV corners and associated IDs!",
			Shared::Error::Severity::KILL_AND_EXIT, this->name
		);

		return false;
	}

#endif 

	return true;
}

void DataCollector::saveEstimatedPose(const std::string & camera, const cv::Mat & extrinsics, const cv::Mat & uvCorners, const cv::Mat & cornerIDs) {
	static const std::string CALLER{ typeid(&DataCollector::saveEstimatedPose).name() };

	try {
		// Check if camera contains <ximea>...
		std::string key{};
		auto isXimea{ camera.find("ximea") != std::string::npos };

		// Extrinsics
		key = (isXimea ? Presets::DataSet::Types::XIMEA_EXT : Presets::DataSet::Types::IMPERX_EXT);
		this->mappedKey(key, this->matrices).mat.push_back(extrinsics);

		// UV-Corners
		key = (isXimea ? Presets::DataSet::Types::UV_CORNERS_XIMEA : Presets::DataSet::Types::UV_CORNERS_IMPERX);
        this->mappedKey(key, this->matrices).mat.push_back(uvCorners);

		// Corner IDs
		key = (isXimea ? Presets::DataSet::Types::XIMEA_CHIDS : Presets::DataSet::Types::IMPERX_CHIDS);
        this->mappedKey(key, this->matrices).mat.push_back(cornerIDs);
	}
	catch (const std::out_of_range & error) {
		this->eHandler->report(
			this->name + ":" + CALLER + ":" + error.what(),
			Shared::Error::Severity::KILL_AND_EXIT, this->name
		);
	}
}

void DataCollector::savePTPose(const std::string & ptu, const std::vector<double> & pose) {
	static const std::string CALLER{ typeid(&DataCollector::savePTPose).name() };

	try {
        this->mappedKey(Presets::DataSet::PanTiltAngles::DEG, this->matrices).mat.push_back(cv::Mat(pose).clone().t());
	} 
	catch (const std::out_of_range & error) {
		this->eHandler->report(
			this->name + ":" + CALLER + ":" + error.what(),
			Shared::Error::Severity::KILL_AND_EXIT, this->name
		);
	}
}

void DataCollector::formatCollectedData() {
	for (auto & data : this->matrices) {
		// Extrinsics
		if (data.first == Presets::DataSet::Types::XIMEA_EXT || data.first == Presets::DataSet::Types::IMPERX_EXT) {
			data.second.dimensions = std::vector<int>{data.second.mat.rows / data.second.mat.cols, data.second.mat.cols, data.second.mat.cols};
		}
		// UV Corners
		else if (data.first == Presets::DataSet::Types::UV_CORNERS_XIMEA || data.first == Presets::DataSet::Types::UV_CORNERS_IMPERX) {
			data.second.dimensions = std::vector<int>{data.second.mat.rows / this->CHESSBOARD_CRNS_SIZE, this->CHESSBOARD_CRNS_SIZE, data.second.mat.cols};
		}
		// Corner IDs
		else if (data.first == Presets::DataSet::Types::XIMEA_CHIDS || data.first == Presets::DataSet::Types::IMPERX_CHIDS) {
			data.second.dimensions = std::vector<int>{data.second.mat.rows, data.second.mat.cols};
		}
		// PT
		else if (data.first == Presets::DataSet::PanTiltAngles::DEG) {
			data.second.dimensions = std::vector<int>{ data.second.mat.rows, data.second.mat.cols };
		}
		else {
			this->eHandler->report(
				"Error: Unknown dataset <" + data.first + ">",
				Shared::Error::Severity::WARN, this->name
			);
		}
	}
}

void DataCollector::saveToH5() {
	// Write to file
	if (!H5Parser<double, std::string>().writeH5File(this->matrices, this->Config<DataCollectionJSONParser>()->h5Handle.pathHandle.path)) {
		ErrorHandler::getInstance()->report(
			"Failed to write data to H5 file to " + this->Config<DataCollectionJSONParser>()->h5Handle.pathHandle.path,
			Shared::Error::WARN, this->name
		);
	}
	else {
		ErrorHandler::getInstance()->report("Successfully wrote H5 file.", Shared::Error::Severity::DEBUG, this->name);
	}
}

void DataCollector::saveImages() {
	//int index{};
	//std::vector<int> count((int)DeviceInterface::getActiveCameras().size(), 0);
	auto sstream{ std::stringstream{} };

	for (const auto & camera : DeviceInterface::getActiveCameras()) {
		int index{};
		for (const auto & image : this->imageHandles->at(camera->getName()).saved) {
			sstream << std::setfill('0') << std::setw(3) << index++;
			cv::imwrite(camera->getName() + "_" + sstream.str() + ".tiff", image);
			sstream.str("");
		}
	}
}

// Should be treated like private function
auto DataCollector::mappedKey(const std::string &key, std::unordered_map<std::string, Mat3d> & map) -> decltype(map.at(key)) {
    if (map.find(key) != map.end()) {
        return map.at(key);
    }
    else {
        map[key] = Mat3d{};
        return map.at(key);
    }
};

bool DataCollector::cvKeyListener(const int & key) {
	if (key == 'q' || key == 'Q') //Quit
	{
		this->eHandler->report(
			this->name + ": Trigger stop via passed key <" + std::to_string(key) + ">",
			Shared::Error::Severity::KILL_AND_EXIT, this->name
		);
		return false;
	}

	return true;
}

cv::Mat DataCollector::getDisplayImage(const cv::Mat originalImg, const int & minImgWidthInPixels) {
	// Prep image for display
	cv::Mat displayImg(originalImg);
	float scale = 1.0;
	do {
		scale *= 0.5;
		cv::resize(displayImg, displayImg, cv::Size(), scale, scale);
	} while (displayImg.cols > minImgWidthInPixels);

	return displayImg;
}

void DataCollector::logClassState() {
	const auto imgHandlesStr = [](const std::unordered_map<std::string, Image> & imgHandles) {
		auto sstream{ std::stringstream{} };
		for (auto & handle : imgHandles) {
			sstream << "\t\t" << handle.first << "\n";
		}
		return sstream.str();
	};

	const auto poseEstimatorsStr = [](const std::unordered_map<std::string, PoseEstimator> & poseEstimators) {
		auto sstream{ std::stringstream{} };
		for (auto & handle : poseEstimators) {
			sstream << "\t\t" << handle.first << "\n";
		}
		return sstream.str();
	};

	const auto poseHandlesStr = [](const std::unordered_map<std::string, std::vector<double>> & poseHandles) {
		auto sstream{ std::stringstream{} };
		for (auto & handle : poseHandles) {
			sstream << "\t\t" << handle.first << "\n";
		}
		return sstream.str();
	};

	this->eHandler->report(this->name + ": Crash State\n"
		"\tImage Handles\n" + "\t\tSize: " + std::to_string(this->imageHandles->size()) + "\n" + imgHandlesStr(*this->imageHandles) + "\n" +
		"\tPose Estimator\n" + "\t\tSize: " + std::to_string(this->poseEstimators->size()) + "\n" + poseEstimatorsStr(*this->poseEstimators) + "\n" +
		"\tPose Handles size:" + std::to_string(this->poseHandles->size()) + "\n" + poseHandlesStr(*this->poseHandles),
		Shared::Error::Severity::DEBUG, this->name
	);
}