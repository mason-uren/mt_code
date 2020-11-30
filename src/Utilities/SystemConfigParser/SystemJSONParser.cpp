#include "SystemJSONParser.h"

bool SystemJSONParser::loadSystemConfig() {
	return this->loadSystemConfig(this->path + FILE_SEP + this->configFile);
}

bool SystemJSONParser::loadSystemConfig(const std::string & filePath) {
	using json = nlohmann::json;
	auto key{ std::string{} };

	// Open config file...
	json jsonConfig{};
	if (!this->openJSON(jsonConfig, filePath)) {
		return false;
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
	 * Parse running mode
	 */
	key = "mode";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": Invalid JSON config. Must specify <" + key + ">",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}

	auto modeConfig{ System::Mode{} };
	if (!this->loadRunningState(modeConfig, jsonConfig[key])) {
		return false;
	}
	this->mode = modeConfig;

	/**
	 * Parse environment
	 */
	if (!this->loadEnvironment(this->mode.components, jsonConfig)) {
		return false;
	}

	return true;
}

bool SystemJSONParser::validateLoadedJSON() {
	// No file paths to verify
	return true;
}

bool SystemJSONParser::loadRunningState(System::Mode & modeConfig, nlohmann::json & json) {
	auto key{ std::string{"run"} };
	if (!this->isKeySet(json, key)) {
		this->eHandle->report(
			this->id + ": Unset <run> state. Default: <interface>",
			Shared::Error::Severity::WARN
		);
		modeConfig.run = Presets::Mode::INTERFACE;
	}
	else {
		modeConfig.run = json[key].get<std::string>();
	}

	key = "components";
	if (!this->isKeySet(json, key) && modeConfig.run != Presets::Mode::DYNAMIC_EXTRINSICS) {
		this->eHandle->report(
			this->id + ": Invalid JSON. Must specify <" + key + "> for the system.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}

	if (!json[key].is_array()) {
		this->eHandle->report(
			this->id + ": Invalid JSON. <" + key + "> must be of array type",
			Shared::Error::Severity::KILL_AND_EXIT
		);
	}
	for (const auto & component : json[key]) {
		modeConfig.components.push_back(component);
	}

	return true;
}

bool SystemJSONParser::loadEnvironment(const std::vector<std::string> & targetComps, nlohmann::json & json) {
	auto pairsConfig{ std::unordered_map<std::string, System::Pair>{} };
	auto key{std::string{}};
	auto subKey{std::string{}};

	key = "pairs";
	if (!this->isKeySet(json, key)) {
		this->eHandle->report(
			this->id + ": Invalid JSON config. Must specify <" + key + ">",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}

	// Verify/load desired components
	for (const auto & comp : targetComps) {
	    if (!this->isKeySet(json[key], comp)) {
            this->eHandle->report(
                    this->id + ": Unrecognized component name <" + comp + ">. Component will not be added/initialized.",
                    Shared::Error::Severity::WARN
            );

            continue;
	    }

        auto _pair{System::Pair{}};
	    if (!this->loadComponentPair(_pair, json[key][comp])) {
	        continue;
	    }
	    pairsConfig.insert({comp, _pair});
	}

	// Assign to <pairs>
	this->pairs = std::move(pairsConfig);

	// Load world frame
	key = Presets::Tracking::Context::WORLD_FRAME; // "world_frame";
	if (!this->isKeySet(json, key)) {
	    this->eHandle->report(
	        this->id + ": Invalid JSON. No <" + key + "> specified.",
	        Shared::Error::Severity::KILL_AND_EXIT
        );
	    return false;
	}

	// Load Camera
	auto cameraParams{ System::Camera{} };
	if (!this->loadCameraParams(cameraParams, json[key])) {
		return false;
	}

	// Assign directly to world frame
	this->wFrame = cameraParams;

	return true;
}

bool SystemJSONParser::loadCameraParams(System::Camera & camera, nlohmann::json & json) {
	auto key{ std::string{} };
	auto subKey{ std::string{"id"} };

	key = "camera";
	if (!this->isKeySet(json, key)) {
		this->eHandle->report(
			this->id + ": Invalid JSON. Component pairs must specify a <" + key + ">.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}

//	auto cameraJSON{ json[key] };
	if (!this->isKeySet(json[key], subKey)) {
		this->eHandle->report(
			this->id + ": Invalid JSON. Component pair: <" + key + ":" + subKey + "> must had valid ID.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}
	camera.id = json[key][subKey].get<std::string>();

	std::string parent{key};
    key = "intrinsics";
	if (!this->isKeySet(json[parent], key)) {
		this->eHandle->report(
			this->id + ": Invalid JSON. Unknown camera intriniscs.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}
	if (!this->isKeySet(json[parent][key], subKey)) {
		this->eHandle->report(
			this->id + ": Invalid JSON. Intrinsics must assign an <" + subKey + "> value.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}
	camera.intrinsics = json[parent][key][subKey].get<std::string>();

	return true;
}

bool SystemJSONParser::loadComponentPair(System::Pair &pair, nlohmann::json &json) {
    auto key{std::string{}};
    std::string subKey{"id"};

	// Load Camera
	auto cameraParams{ System::Camera{} };
	if (!this->loadCameraParams(cameraParams, json)) {
		return false;
	}
	pair.camera = cameraParams;

	// Load PTU
	key = "ptu";
    if (!this->isKeySet(json, key)) {
        this->eHandle->report(
                this->id + ": Invalid JSON. Component pairs must specify a <" + key + ">.",
                Shared::Error::Severity::KILL_AND_EXIT
        );
        return false;
    }

    if (!this->isKeySet(json[key], subKey)) {
        this->eHandle->report(
                this->id + ": Invalid JSON. Component pair: <" + key + ":" + subKey + "> must had valid ID.",
                Shared::Error::Severity::KILL_AND_EXIT
        );
        return false;
    }

    pair.ptu = json[key][subKey].get<std::string>();

	// Load Extrinsics
	key = "extrinsics";
	if (!this->isKeySet(json, key)) {
		this->eHandle->report(
			this->id + ": Invalid JSON. Component pairs must specify a <" + key + "> model.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}

	if (!this->isKeySet(json[key], subKey)) {
		this->eHandle->report(
			this->id + "Invalid JSON. Component pair: <" + key + ":" + subKey + "> must have valid ID.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
	}
	pair.extrinsics = json[key][subKey].get<std::string>();

    return true;
}

Presets::Mode::Enum SystemJSONParser::runningMode(const std::string & mode) {
	if (mode == Presets::Mode::INTERFACE) {
		return Presets::Mode::Enum::INTERFACE;
	}
	if (mode == Presets::Mode::TEST) {
		return Presets::Mode::Enum::TEST;
	}
	if (mode == Presets::Mode::DATA_COLLECTION) {
		return Presets::Mode::Enum::DATA_COLLECTION;
	}
	if (mode == Presets::Mode::TRACKING) {
		return Presets::Mode::Enum::TRACKING;
	}
	if (mode == Presets::Mode::EXPERIMENTAL) {
		return Presets::Mode::Enum::EXPERIMENTAL;
	}
	if (mode == Presets::Mode::DYNAMIC_EXTRINSICS) {
		return Presets::Mode::Enum::DYNAMIC_EXTRINSICS;
	}

	this->eHandle->report("Unrecognized mode: <" + mode + ">. Defaulting mode: INTERFACE. See SystemConifg.json");
	return Presets::Mode::Enum::INTERFACE;
}