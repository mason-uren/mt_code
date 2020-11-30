#include "InterfaceJSONParser.h"

bool InterfaceJSONParser::loadInterfaceConfig() {
	return this->loadInterfaceConfig(this->path + FILE_SEP + this->configFile);
}

bool InterfaceJSONParser::loadInterfaceConfig(const std::string & filePath) {
	bool result{};
	std::string key{};

	using json = nlohmann::json;

	json jsonConfig{};
	if (!(result = this->openJSON(jsonConfig, filePath))) {
		return result;
	}

	/**
	 * Check for config ID
	 */
	key = "id";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			"<" + static_cast<std::string>(typeid(this).name()) + "> : Invalid JSON. <" + key + "> must be specified.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}
	this->id = jsonConfig[key].get<std::string>();

	/**
	 * Parse valid cameras
	 */
	if (!jsonConfig.contains("camera")) {
		this->eHandle->report(
			this->id + ": No cameras to initialize.",
			Shared::Error::Severity::WARN
		);
	}
	else {
		if (!(result = jsonConfig["camera"].is_array())) {
			this->eHandle->report(
				this->id + ": Invalid JSON camera initialization.",
				Shared::Error::KILL_AND_EXIT
			);
			return result;
		}

		for (auto & camera : jsonConfig["camera"]) {
			// Ensure client intends to create object
			if (!camera["valid"].is_null() && camera["valid"].get<bool>()) {
				auto _camera{ Interface::Imaging::Camera{} };

				// REQUIRED
				// Type
				if (!camera["type"].is_null()) {
					auto name{ camera["type"].get<std::string>() };
					if (name == Presets::Imaging::Camera::XIMEA) {
						_camera.type = Interface::Imaging::Type::XIMEA;
					}
					else if (name == Presets::Imaging::Camera::IMPERX) {
						_camera.type = Interface::Imaging::Type::IMPERX;
					}
					else {
						// Skip processing invalid camera
						continue;
					}

					// Name
					_camera.name = camera["type"].get<std::string>();
				}
				else {
					this->eHandle->report(
						this->id + ": Required value <type> is null.",
						Shared::Error::Severity::WARN
					);

					// Skip processing invalid camera
					continue;
				}

				// ID
				if (!camera["id"].is_null()) {
					auto _id{ camera["id"].get<std::string>() };

					// Verify ID has not been spoken for
					if (std::find(this->cameraIDs.begin(), this->cameraIDs.end(), _id) != this->cameraIDs.end()) {
						// Found duplicate ID; reject camera
						continue;
					}

					_camera.id = _id;
					this->cameraIDs.push_back(_id);
				}
				else {
					this->eHandle->report(
						this->id + ": Required value <img_buffer_size> is null.",
						Shared::Error::Severity::WARN
					);

					// Skip processing invalid camera
					continue;
				}

				// Image Buffer
				if (!camera["img_buffer_size"].is_null()) {
					_camera.imageBufferSize = camera["img_buffer_size"].get<int>();
				}
				else {
					this->eHandle->report(
						this->id + ": Required value <img_buffer_size> is null.",
						Shared::Error::Severity::WARN
					);

					// Skip processing invalid camera
					continue;
				}

				// OPTIONAL
				// Serial No
				if (!camera["serial_no"].is_null()) {
					_camera.serialNo = camera["serial_no"].get<std::string>();
				}

				// FIXME - should be a required variable
				//		 - required for ximea; why not imperx
				if (!camera["image"].is_null()) {
					_camera.image = Interface::Imaging::Image{};

					// Exposure
					if (!camera["image"]["exposure"].is_null()) {
						_camera.image.exposure = camera["image"]["exposure"].get<int>();
					}
					// Image Format
					if (!camera["image"]["img_format"].is_null()) {
						this->loadImageFormat(_camera.image, camera["image"]["img_format"].get<std::string>());
					}
					// Pixel Depth
					if (!(_camera.image.pixelDepth || camera["image"]["pixel_depth"].is_null())) {
						_camera.image.pixelDepth = camera["image"]["pixel_depth"].get<int>();
					}
					// Auto White-balance
					if (!camera["image"]["auto_white_balance"].is_null()) {
						_camera.image.autoWhiteBalance = camera["image"]["auto_white_balance"].get<bool>();
					}
					// Downsampling Type
					if (!camera["image"]["downsampling_type"].is_null()) {
						this->loadDownsamplingType(_camera.image, camera["image"]["downsampling_type"].get<std::string>());
					}
					// Aperture
					if (!camera["image"]["aperture"].is_null()) {
						_camera.image.aperture = camera["image"]["aperture"].get<float>();
					}
				}

				// If Ximea, load connected slot
				if (camera["type"].get<std::string>() == Presets::Imaging::Camera::XIMEA) {
					key = "pcie_slot_position";
					if (!this->isKeySet(camera, key)) {
						this->eHandle->report(
							this->id + ": Invalid JSON. Ximea <" + _camera.id + "> did not designate a PCIe slot position. Camera will not be initialized.",
							Shared::Error::Severity::WARN
						);
					}
					else {
						_camera.pcieSlot = camera[key].get<int>();
					}
				}

				// Add camera
				this->cameras.push_back(_camera);
			}
		}
	}

	/**
	 * Parse valid PTUs
	 */
	if (!jsonConfig.contains("ptu")) {
		this->eHandle->report(
			this->id + ": No PTUs to initialize.",
			Shared::Error::Severity::WARN
		);
	}
	else {
		if (!(result = jsonConfig["ptu"].is_array())) {
			this->eHandle->report(
				this->id + ": Invalid JSON PTU initialization.",
				Shared::Error::KILL_AND_EXIT
			);
			return result;
		}
		for (auto & ptu : jsonConfig["ptu"]) {
			if (!ptu["valid"].is_null() && ptu["valid"].get<bool>()) {
				auto _ptu{ Interface::PanTilt::PTU{} };

				// REQUIRED
				// ID
				if (!ptu["id"].is_null()) {
					_ptu.id = ptu["id"].get<std::string>();
				}
				else {
					this->eHandle->report(
						this->id + ": Required value <id> is null.",
						Shared::Error::Severity::WARN
					);
					// Skip processing invalid PTU
					continue;
				}
				// Network
				if (!ptu["network"].is_null()) {					
					// Publisher/Subscriber
					if (!this->loadNetworkProtocol(_ptu.network, ptu["network"], "client") || 
						!this->loadNetworkProtocol(_ptu.network, ptu["network"], "server")) {
						// Skip processing invalid PTU
						continue;
					}
				}
				else {
					this->eHandle->report(
						this->id + ": Required value <network> is null.",
						Shared::Error::Severity::WARN
					);

					// Skip processing invalid PTU
					continue;
				}

				// OPTIONAL
				// Axes
				if (!ptu["axes"].is_null()) {
					this->loadAxesContext(_ptu.axes, ptu["axes"]);
				}
				else {
					// DEFAULT
					auto jsonObj{nlohmann::json{}};
					this->loadAxesContext(_ptu.axes, jsonObj);
					this->eHandle->report(
						this->id + ": Optional value <axes> is null.",
						Shared::Error::Severity::WARN
					);
				}

				// PID Controller
				if (!ptu["pid"].is_null()) {
					this->loadPIDController(_ptu.pid, ptu["pid"]);
				}
				else {
					// DEFAULT
					this->eHandle->report(
						this->id + ": Optional value <pid> is null. Defaulting to INTERNAL",
						Shared::Error::Severity::WARN
					);
				}

				// Add PTU
				this->ptus.push_back(_ptu);
			}
		}
	}

	return this->validateLoadedJSON();
}

bool InterfaceJSONParser::hasWFOV(const std::string & ID) {
	return std::find(this->cameraIDs.begin(), this->cameraIDs.end(), ID) != this->cameraIDs.end();
}

void InterfaceJSONParser::loadImageFormat(Interface::Imaging::Image & image, const std::string & format) {
	if (format == "XI_MONO8") {
		image.imgFormat = XI_MONO8;
		image.pixelDepth = 1;
	}
	else if (format == "XI_MONO16") {
		image.imgFormat = XI_MONO16;
		image.pixelDepth = 1;
	}
	else if (format == "XI_RAW8") {
		image.imgFormat = XI_RAW8;
		image.pixelDepth = 1;
	}
	else if (format == "XI_RAW16") {
		image.imgFormat = XI_RAW16;
		image.pixelDepth = 1;
	}
	else if (format == "XI_RGB24") {
		image.imgFormat = XI_RGB24;
		image.pixelDepth = 3;
	}
	else if (format == "XI_RGB32") {
		image.imgFormat = XI_RGB32;
		image.pixelDepth = 3;
	}
	else {
		// DEFAULT
		image.imgFormat = XI_MONO8;
		image.pixelDepth = 1;

		this->eHandle->report(
			this->id + ": Unsupported image format <" + format + ">. Defaulting to XI_MONO8",
			Shared::Error::Severity::WARN
		);
	}
}

void InterfaceJSONParser::loadDownsamplingType(Interface::Imaging::Image & image, const std::string & dwnSmplType) {
	if (dwnSmplType == "XI_BINNING") {
		image.downsamplingType = XI_BINNING;
	}
	else if (dwnSmplType == "XI_SKIPPING") {
		image.downsamplingType = XI_SKIPPING;
	}
	else {
		// DEFAULT
		image.downsamplingType = XI_BINNING;

		this->eHandle->report(
			this->id + ": Unsupported image format <" + dwnSmplType + ">. Defaulting to XI_MONO8",
			Shared::Error::Severity::WARN
		);
	}
}

bool InterfaceJSONParser::loadNetworkProtocol(Interface::Network::Pattern & pattern, nlohmann::json & json, const std::string & key) {
	auto protocol{ key == "client" ? &pattern.client : &pattern.server };
	// Ensure Pub/Sub is populated
	if (json[key].is_null()) {
		return false;
	}

	// IP
	if (!json[key]["ip"].is_null()) {
		protocol->ip = json[key]["ip"].get<std::string>();
	}
	else {
		this->eHandle->report(
			this->id + ": Invalid <" + key + "> IP",
			Shared::Error::Severity::WARN
		);
		return false;
	}

	// PORT
	if (!json[key]["port"].is_null()) {
		protocol->port = json[key]["port"].get<std::string>();
	}
	else {
		this->eHandle->report(
			this->id + ": Invalid <" + key + "> port",
			Shared::Error::Severity::WARN
		);
		return false;
	}

	return true;
}

void InterfaceJSONParser::loadAxesContext(Interface::PanTilt::Axes & axes, nlohmann::json & json) {
	std::string key{};
	std::string subKey{};

	// Lambda functions
	auto defaultAxisDomain = [&](const int & axis = -1) {
		switch (axis) {
			case (int) Interface::PanTilt::Axis::PAN:
				axes.domain.pan = std::vector<double>{ -Presets::PanTilt::Axis::Domain::DEFAULT_PAN, Presets::PanTilt::Axis::Domain::DEFAULT_PAN };
				break;
			case (int)Interface::PanTilt::Axis::TILT:
				axes.domain.tilt = std::vector<double>{ -Presets::PanTilt::Axis::Domain::DEFAULT_TILT, Presets::PanTilt::Axis::Domain::DEFAULT_TILT };
				break;
			default:
				axes.domain.pan = std::vector<double>{ -Presets::PanTilt::Axis::Domain::DEFAULT_PAN, Presets::PanTilt::Axis::Domain::DEFAULT_PAN };
				axes.domain.tilt = std::vector<double>{ -Presets::PanTilt::Axis::Domain::DEFAULT_TILT, Presets::PanTilt::Axis::Domain::DEFAULT_TILT };
				break;
		}
		
		this->eHandle->report(
			this->id + ": Axes <" + key + ":" + subKey + "> is not specified. Defaulting according to Presets.h",
			Shared::Error::Severity::WARN
		);
	};

	auto checkDomainFormat = [&]() {
		if (!(this->isKeySet(json[key], subKey) && json[key][subKey].is_array())) {
			defaultAxisDomain();
			return false;
		}
		auto axisDomain{ json[key][subKey].get<std::vector<double>>() };
		// FIXME - dangerously rigid implementation
		auto axis{subKey == "pan" ? (int) Interface::PanTilt::Axis::PAN : (int)Interface::PanTilt::Axis::TILT };
		switch (axisDomain.size()) {
			case 0:
				this->eHandle->report("Defaulting axis domain. See Presets.h");
				defaultAxisDomain(axis);
				return false;
			case 2:
				return true;
			default:
				this->eHandle->report("Invalid JSON. Bad axis domain <" + subKey + ">.");
				defaultAxisDomain();
				return false;
		}
		//if (axisDomain.size() != 2) {
		//	this->eHandle->report("Invalid JSON. Bad axis domain <" + subKey + ">.");
		//	defaultAxisDomain(this->eHandle);
		//	return false;
		//}
		return true;
	};
	// End lambda functions

	// Domain
	key = "domain";
	if (this->isKeySet(json, key)) {
		subKey = "pan";
		if (checkDomainFormat()) {
			axes.domain.pan = json[key][subKey].get<std::vector<double>>();
		}
		
		subKey = "tilt";
		if (checkDomainFormat()) {
			axes.domain.tilt = json[key][subKey].get<std::vector<double>>();
		}
	}
	else {
		// DEFAULT - Domain
		defaultAxisDomain();
	}
	// Sort domain low -> high
	std::sort(axes.domain.pan.begin(), axes.domain.pan.end());
	std::sort(axes.domain.tilt.begin(), axes.domain.tilt.end());

	// Voltage
	if (!json["voltages"].is_null()) {
		axes.volt.max = json["voltages"]["max"].is_null() ? Presets::PanTilt::Axis::Voltage::DEFAULT_MAX : json["voltages"]["max"].get<int>();
		axes.volt.min = json["voltages"]["min"].is_null() ? Presets::PanTilt::Axis::Voltage::DEFAULT_MIN : json["voltages"]["min"].get<int>();
		axes.volt.cli = json["voltages"]["cli_motion"].is_null() ? Presets::PanTilt::Axis::Voltage::DEFAULT_CLI : json["voltages"]["cli_motion"].get<int>();
	}
	else {
		// DEFAULT - Volt
		axes.volt.max = Presets::PanTilt::Axis::Voltage::DEFAULT_MAX;
		axes.volt.min = Presets::PanTilt::Axis::Voltage::DEFAULT_MIN;
		axes.volt.cli = Presets::PanTilt::Axis::Voltage::DEFAULT_CLI;

		this->eHandle->report(
			this->id + ": Axes <Voltage> is not specified. Defaulting according to SharedStructs.h",
			Shared::Error::Severity::WARN
		);

	}
	// Precision
	if (!json["precision"].is_null()) {
		axes.precision.epsilon = json["precision"]["epsilon"].get<float>();
	}
	else {
		axes.precision.epsilon = Presets::PanTilt::Axis::Precision::EPSILON;

		this->eHandle->report(
			this->id + ": Axes <Precision> is not specified. Defaulting according to SharedStructs.h",
			Shared::Error::Severity::WARN
		);
	}
}

void InterfaceJSONParser::loadPIDController(Interface::PanTilt::PID & pid, nlohmann::json & json) {
	std::string key{};
	std::string subKey{};

	// Lambda Functions
	auto checkAccelerationFormat = [&](double & accel) {
		if (!this->isKeySet(json[key], subKey)) {
			this->eHandle->report(
				this->id + ": Tuning variable <" + key + ":" + subKey + "not found. Default: " + std::to_string(Presets::PanTilt::PID::DEFAULT_ACCELERATION_FACTOR),
				Shared::Error::Severity::WARN
			);
		}
		auto axisAccel{json[key][subKey].get<double>()};
		if (0 > axisAccel && axisAccel > 10) {
			accel = json[key][subKey].get<double>();
		}
		else {

		}
		accel = (0 > axisAccel && axisAccel > 10) ? json[key][subKey].get<double>() : Presets::PanTilt::PID::DEFAULT_ACCELERATION_FACTOR;
	};
	// END Lambda functions


	// Type
	key = "type";
	if (!this->isKeySet(json, key)) {
		this->eHandle->report(
			this->id + ": Unknown PID <type>. Default: INTERNAL",
			Shared::Error::Severity::WARN
		);
	}
	else {
		auto clientSelection{ json[key].get<std::string>() };
		if (clientSelection == Presets::PanTilt::PID::INTERNAL) {
			pid.type = Interface::PanTilt::PID_TYPE::INTERNAL;
		}
		else if (clientSelection == Presets::PanTilt::PID::EXTERNAL) {
			pid.type = Interface::PanTilt::PID_TYPE::EXTERNAL;

			// Exponential Factor
			key = "exponential_factor";
			if (!this->isKeySet(json, key)) {
				this->eHandle->report(
					this->id + ": Tuning variable <" + key + "> not found. Default: " + std::to_string(Presets::PanTilt::PID::EXPONENTIAL_FACTOR),
					Shared::Error::Severity::WARN
				);
				pid.eFactor = Presets::PanTilt::PID::EXPONENTIAL_FACTOR;
			}
			else {
				auto factor{ json[key].get<double>() };
				pid.eFactor = factor;
				this->eHandle->report(this->id + ": <exponential_factor> set to ( " + std::to_string(factor) + ")");
			}

			// Acceleration
			key = "acceleration_factor";
			if (!this->isKeySet(json, key)) {
				this->eHandle->report(
					this->id + ": Tuning variable <" + key + "not found. Default: " + std::to_string(Presets::PanTilt::PID::DEFAULT_ACCELERATION_FACTOR)
				);
				pid.accelerationFactor.pan = Presets::PanTilt::PID::DEFAULT_ACCELERATION_FACTOR;
				pid.accelerationFactor.tilt = Presets::PanTilt::PID::DEFAULT_ACCELERATION_FACTOR;
			}
			else {
				subKey = "pan";
				checkAccelerationFormat(pid.accelerationFactor.pan);

				subKey = "tilt";
				checkAccelerationFormat(pid.accelerationFactor.tilt);
			}
		}
		else {
			pid.type = Interface::PanTilt::PID_TYPE::INTERNAL;
			this->eHandle->report(
				this->id + ": Unknown pid controller type <" + clientSelection + ">. Default: INTERNAL",
				Shared::Error::Severity::WARN
			);
		}
	}
}

bool InterfaceJSONParser::validateLoadedJSON() {
	if (!this->ptus.size()) {
		this->eHandle->report(
			this->id + ": No valid PTUs.",
			Shared::Error::Severity::WARN
		);
	}
	else {
		this->eHandle->report(this->id + ": # of Valid PTUs ( " + std::to_string(this->ptus.size()) + ")");
	}

	if (!this->cameras.size()) {
		this->eHandle->report(
			this->id + ": No valid cameras",
			Shared::Error::WARN
		);
	}
	else {
		this->eHandle->report(this->id + ": # of Valid Cameras ( " + std::to_string(this->cameras.size()) + ")");
	}

	return true;
}