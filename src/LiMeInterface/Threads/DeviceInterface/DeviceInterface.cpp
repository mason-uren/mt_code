#include "DeviceInterface.h"

// Static variable initialization
std::unordered_map<std::string, DeviceInterface *> DeviceInterface::components{};
std::unordered_map<std::string, System::Pair> DeviceInterface::pairs{};
System::Camera DeviceInterface::wFrame{};
std::unordered_map<std::string, CameraModel<double>> DeviceInterface::ximea{};
std::unordered_map<std::string, CameraModel<double>> DeviceInterface::imperx{};
std::vector<DeviceInterface *> DeviceInterface::cameras{};
std::vector<DeviceInterface *> DeviceInterface::ptus{};

/**
 * Member Functions
 */
void DeviceInterface::deviceInfo(const bool consoleDisplay) {
    Shared::Device::Info devInfo{};
	this->info(&devInfo);

	// Log Device Info
	this->eHandler->report("Thread ( " + this->name + "): Attached device..." + devInfo.toString());

	// If logger thread is not running, manually flush
	if (!this->logger->isListening()) {
		this->logger->flush();
	}

	// Print to console
    if (consoleDisplay) {
        devInfo.display();
    }
}

std::string DeviceInterface::getSensorID() const {
	return this->sensorID;
}

/**
 * Static Functions
 */
void DeviceInterface::sourceEnvironment(const std::unordered_map<std::string, System::Pair> & targetPairs, const System::Camera & wFrame) {
	DeviceInterface::pairs = targetPairs;
	DeviceInterface::wFrame = wFrame;
}

void DeviceInterface::sourceCameraIntrinsics(const std::unordered_map<std::string, Model::Camera::Intrinsics> & ximea, 
											 const std::unordered_map<std::string, Model::Camera::Intrinsics> & imperx) {
    // Lambda functions
    auto generateCameraModels = [&](const std::unordered_map<std::string, Model::Camera::Intrinsics> & models) {
        auto cameraModels{std::unordered_map<std::string, CameraModel<double>>{}};
        for (const auto & model : models) {
            cameraModels.insert({
                model.first,
                CameraModel<double>{model.second.cameraMatrix.path, model.second.distortionCoeff.path}
            });
        }

        return cameraModels;
    };
    // END lambda

    DeviceInterface::ximea = generateCameraModels(ximea);
    DeviceInterface::imperx = generateCameraModels(imperx);
}

bool DeviceInterface::addComponent(const std::string & id, DeviceInterface * comp) {
	try {
		DeviceInterface::components.insert({ id, comp });
	} 
	catch (const std::bad_alloc & error) {
		ErrorHandler::getInstance()->report(
			error.what(),
			Shared::Error::Severity::KILL_AND_EXIT
		);
	}

	// Verify component was added
	return DeviceInterface::components.find(id) != DeviceInterface::components.end();
}

std::vector<DeviceInterface *> & DeviceInterface::getActiveCameras(const bool reset) {
	static const std::string CALLER{ typeid(&DeviceInterface::getActiveCameras).name() };
	static bool once{};

	// Only compose cameras once
	if (once) {
		if (!reset) {
			return DeviceInterface::cameras;
		}
	}
	else {
		once = true;
	}

	try {
		//
		for (auto & pair : DeviceInterface::pairs) {
			DeviceInterface::cameras.push_back(DeviceInterface::components.at(pair.second.camera.id));
		}
		// Check if world frame is available
		if (DeviceInterface::components.find(DeviceInterface::wFrame.id) != DeviceInterface::components.end()) {
			DeviceInterface::cameras.push_back(DeviceInterface::components.at(DeviceInterface::wFrame.id));
		}
		else {
			ErrorHandler::getInstance()->report(
				CALLER + ": No <world_frame> initialized. Component is not visible. See InterfaceConfig.json", 
				Shared::Error::Severity::WARN
			);
		}
	}
	catch (const std::out_of_range & error) {
		ErrorHandler::getInstance()->report(
			error.what(),
			Shared::Error::Severity::KILL_AND_EXIT
		);
	}
	return DeviceInterface::cameras;
}

std::vector<DeviceInterface *> & DeviceInterface::getActivePTUs(const bool reset) {
	static bool once{};
	// Only compose ptus once
	if (once) {
		if (!reset) {
			return DeviceInterface::ptus;
		}
	}
	else {
		once = true;
	}

	try {
		for (auto & pair : DeviceInterface::pairs) {
			DeviceInterface::ptus.push_back(DeviceInterface::components.at(pair.second.ptu));
		}
	}
	catch (const std::out_of_range & error) {
		ErrorHandler::getInstance()->report(
			error.what(),
			Shared::Error::Severity::KILL_AND_EXIT
		);
	}
	return DeviceInterface::ptus;
}

bool DeviceInterface::initialize() {
	if (DeviceInterface::components.empty()) {
		ErrorHandler::getInstance()->report(
			"No components attached to system.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}
	for (auto & comp : DeviceInterface::components) {
		if (!comp.second->start()) {
			ErrorHandler::getInstance()->report(
				"Triggerd exit -> ( " + comp.first + ")",
				Shared::Error::Severity::KILL_AND_EXIT
			);
			return false;
		}
	}
	return true;
}

bool DeviceInterface::safeExit() {
	for (auto & comp : DeviceInterface::components) {
		comp.second->stop();
	}

	ErrorHandler::getInstance()->report("All attached ThreadInterface instances closed.");

	return true;
}