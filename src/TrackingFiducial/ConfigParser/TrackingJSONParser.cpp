#include "TrackingJSONParser.h"

bool TrackingJSONParser::loadTrackingConfig() {
	return this->loadTrackingConfig(this->path + FILE_SEP + this->configFile);
}

bool TrackingJSONParser::loadTrackingConfig(const std::string & filePath) {
	auto key{ std::string{} };

	using json = nlohmann::json;

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
			this->id + ": Invalid JSON. No running <" + key + "> specified. Will attempt to default at runtime",
			Shared::Error::Severity::WARN
		);
	}
	else {
		// FIXME - adhoc
		this->mode = jsonConfig[key]["run"].get<std::string>();
	}

	/**
	 * Parse model fit
	 */
	key = "models";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": Invalid JSON. CAD models must be specified.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}

	auto modelConfig{ std::unordered_map<std::string, Shared::File::PathHandle>{} };
	if (!this->loadModelsConfig(modelConfig, jsonConfig[key])) {
		return false;
	}
	this->models = std::move(modelConfig);

	/**
	 * Parse staging point config
	 */
	key = "staging_point";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": No <" + key + "> specified. Default: [0, 0] (Home Position).",
			Shared::Error::Severity::WARN
		);
		this->stagingPositions.insert({ Presets::Tracking::StagingArea::SHARED, std::vector<double>{0, 0} });
	}
	else {
		auto stagingConfig{ std::unordered_map<std::string, std::vector<double>>{} };
		if (!this->loadStagingPositions(stagingConfig, jsonConfig[key])) {
			return false;
		}
		this->stagingPositions = std::move(stagingConfig);
	}


	/**
	 * Parse marker detection scales
	 */
	key = "marker_detection_scale";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": No <" + key + "> specified. Defaulting... Ximea: 0.2, Imperx: 0.7",
			Shared::Error::Severity::WARN	
		);

		this->markerDetectionScales.insert({Presets::Tracking::Context::XIMEA_FRAME, Presets::Tracking::Marker::XI_SCALE});
		this->markerDetectionScales.insert({Presets::Tracking::Context::IMPERX_FRAME, Presets::Tracking::Marker::IPX_SCALE});
	}
	else {
		for (const auto & pair : jsonConfig[key].items()) {
			this->markerDetectionScales.insert({pair.key(), pair.value()});
		}
	}
	

	/**
	 * Parse visual config
	 */
	key = "visual";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": No <" + key + "> option specified. Defaulting... Visual: ON, Scale: 0.1",
			Shared::Error::Severity::WARN
		);
		
		this->display.visible = true;
		this->display.scale = 0.1;
	}
	else {
		auto displayConf{ Tracking::Visualization::Display{} };
		if (!this->loadVisualConfig(displayConf, jsonConfig[key])) {
			return false;
		}
		this->display = std::move(displayConf);
	}

	/**
	 * Parse video recording options
	 */
	key = "record";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": No <" + key + "> option specified. Defaulting... Input: OFF, Output: OFF",
			Shared::Error::Severity::WARN
		);

		this->input.record = false;
		this->output.record = false;
	}
	else {
		auto inputConfig{ Tracking::Visualization::Video{} };
		auto outputConfig{ Tracking::Visualization::Video{} };
		if (!this->loadRecordingConfig(inputConfig, outputConfig, jsonConfig[key])) {
			return false;
		}

		this->input = std::move(inputConfig);
		this->output = std::move(outputConfig);
	}

	return this->validateLoadedJSON();
}

bool TrackingJSONParser::validateLoadedJSON() {
	// Don't validate video files
	// TODO - add postfix checking

	//for (const auto & model : this->models) {
	//	if (!this->verifyPath(model.second)) {
	//		return false;
	//	}
	//}
	return true;
}

bool TrackingJSONParser::loadModelsConfig(std::unordered_map<std::string, Shared::File::PathHandle> & modelsConfig, nlohmann::json & jsonConfig) {
	static const std::string CALLER{ typeid(&TrackingJSONParser::loadModelsConfig).name() };

	for (const auto & modelPair : jsonConfig.items()) {
		auto pathHandle{ Shared::File::PathHandle{} };
		if (!this->loadPathHandleConfig(pathHandle, modelPair.value(), CALLER)) {
			return false;
		}
		
		try {
			// Insert path...
			modelsConfig.insert({
				modelPair.key(),
				pathHandle
				});
		} 
		catch (const std::bad_alloc & error) {
			this->eHandle->report(
				this->id + ": " + error.what(),
				Shared::Error::Severity::KILL_AND_EXIT
			);
			return false;
		}
	}

	return true;
}

bool TrackingJSONParser::loadStagingPositions(std::unordered_map<std::string, std::vector<double>> & stagingConfig, nlohmann::json & jsonConfig) {
	auto key{ std::string{} };

	for (const auto & stagingPair : jsonConfig.items()) {
		if (!stagingPair.value().is_array()) {
			this->eHandle->report(
				this->id + ": Invalid JSON. <" + stagingPair.key() + "> must be an array."
			);
			return false;
		}

		// Ensure the point is valid
		if (stagingPair.value().size() != 2) {
			this->eHandle->report(
				this->id + ": <" + stagingPair.key() + "> Incorrect staging point format. Must be an array of size -> (2)",
				Shared::Error::Severity::KILL_AND_EXIT
			);
			return false;
		}

		try {
			// Initialize pair...
			stagingConfig.insert({
				stagingPair.key(),
				std::vector<double>{stagingPair.value().at(0), stagingPair.value().at(1)}
				});
		}
		catch (const std::out_of_range & error) {
			this->eHandle->report(
				this->id + ": " + error.what(),
				Shared::Error::Severity::KILL_AND_EXIT
			);
			return false;
		}
	}

	return true;
}

bool TrackingJSONParser::loadVisualConfig(Tracking::Visualization::Display & display, nlohmann::json & jsonConfig) {
	static const std::string CALLER{ typeid(&TrackingJSONParser::loadVisualConfig).name() };
	auto key{ std::string{"display"} };

	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": Optional parameter <" + key + "> not specified. Default: false",
			Shared::Error::Severity::WARN
		);

		display.visible = false;
		display.scale = 1;
	}
	else {
		display.visible = jsonConfig[key].get<bool>();

		key = "scale";
		bool result{};
		if (!(result = this->isKeySet(jsonConfig, key))) {
			this->eHandle->report(
				this->id + ": Optional parameter <" + key + "> not specified. Default: 0.5",
				Shared::Error::Severity::WARN
			);
		}

		display.scale = result ? jsonConfig[key].get<double>() : 0.25;
	}

	return true;
}

bool TrackingJSONParser::loadRecordingConfig(Tracking::Visualization::Video & input, Tracking::Visualization::Video & output, nlohmann::json & jsonConfig) {
	static const std::string CALLER{typeid(&TrackingJSONParser::loadVisualConfig).name()};
	auto key{ std::string{} };

	// Lambda function(s)
	auto loadVideo = [&](Tracking::Visualization::Video & video) {
		if (!this->isKeySet(jsonConfig, key)) {
			this->eHandle->report(
				this->id + ": Optional parameter <" + key + "> not specified. Default: <" + key + "> recording OFF",
				Shared::Error::Severity::WARN
			);

			video.record = false;
		}
		else {
			if (!this->loadPathHandleConfig(video.video, jsonConfig[key], CALLER)) {
				return false;
			}
			video.record = true;
		}

		return true;
	};
	// END lambda

	key = "input";
	if (!loadVideo(input)) {
		return false;
	}

	key = "output";
	if (!loadVideo(output)) {
		return false;
	}

	return true;
}