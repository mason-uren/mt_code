#include "ModelsJSONParser.h"

bool ModelsJSONParser::loadModelsConfig() {
	return this->loadModelsConfig(this->path + FILE_SEP + this->configFile);
}

bool ModelsJSONParser::loadModelsConfig(const std::string & filePath) {
	std::string key{};

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
			Shared::Error::KILL_AND_EXIT
		);
		return false;
	}
	this->id = jsonConfig[key].get<std::string>();

	/**
	 * Parse board definitions
	 */
	key = "charuco_boards";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": Invalid JSON. No <" + key + "> to initialize.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
	}
	auto chArUcoBoardDefs{std::unordered_map<std::string, Model::Board>()};
	if (!this->loadChArUcoBoardDefinitions(chArUcoBoardDefs, jsonConfig[key])) {
		return false;
	}
	this->chArUcoBoards = std::move(chArUcoBoardDefs);

	/**
	 * Parse camera intrinsics
	 */
	key = "intrinsics";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": Invalid JSON. No <" + key + "> to initialize.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}

	// Ximea 
	auto subKey{ Presets::Imaging::Camera::XIMEA };
	auto ximeaIntrinsics{ std::unordered_map<std::string, Model::Camera::Intrinsics>{} };
	if (!this->isKeySet(jsonConfig[key], subKey)) {
		this->eHandle->report(
			this->id + ": Invalid JSON. No <" + key + ":" + subKey + "> to initialize.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}
	if (!this->loadCameraIntrinsics(ximeaIntrinsics, jsonConfig[key][subKey])) {
		return false;
	}
	this->ximea = std::move(ximeaIntrinsics);

	// Imperx 
	subKey = Presets::Imaging::Camera::IMPERX;
	auto imperxIntrinsics{ std::unordered_map<std::string, Model::Camera::Intrinsics>{} };
	if (!this->isKeySet(jsonConfig[key], subKey)) {
		this->eHandle->report(
			this->id + ": Invalid JSON. No <" + key + ":" + subKey + "> to initialize.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}
	if (!this->loadCameraIntrinsics(imperxIntrinsics, jsonConfig[key][subKey])) {
		return false;
	}
	this->imperx = std::move(imperxIntrinsics);

	return this->validateLoadedJSON();
}

bool ModelsJSONParser::validateLoadedJSON() {
	for (const auto & intrinsicMap : {this->ximea, this->imperx}) {
		for (const auto & cameraIntrinsics : intrinsicMap) {
			if (!this->verifyPaths({ cameraIntrinsics.second.cameraMatrix, cameraIntrinsics.second.distortionCoeff })) {
				return false;
			}
		}
	}

	return true;
}

bool ModelsJSONParser::loadChArUcoBoardDefinitions(std::unordered_map<std::string, Model::Board> & boardDefs, nlohmann::json & json) {
	static const std::string CALLER{ typeid(&ModelsJSONParser::loadChArUcoBoardDefinitions).name() };

	auto parseBoardDefinition = [&](Model::Board & boardConfig, nlohmann::json & jsonConfig) {
		for (const std::string & key : { "square_length", "marker_length", "squares_X", "squares_Y" }) {
			if (!this->isKeySet(jsonConfig, key)) {
				this->eHandle->report(
					this->id + ": Invalid JSON. <" + key + "> was not specified.",
					Shared::Error::Severity::KILL_AND_EXIT
				);
				return false;
			}

			if (key == "square_length") {
				boardConfig.squareLength = jsonConfig[key].get<double>();
			}
			else if (key == "marker_length") {
				boardConfig.markerLength = jsonConfig[key].get<double>();
			}
			else if (key == "squares_X") {
				boardConfig.squaresX = jsonConfig[key].get<double>();
			}
			else if (key == "squares_Y") {
				boardConfig.squaresY = jsonConfig[key].get<double>();
			}
			else {
				this->eHandle->report(
					this->id + "Invalid JSON. Unrecognized board key <" + key + ">",
					Shared::Error::Severity::KILL_AND_EXIT
				);
				return false;
			}
		}
		return true;
	};

	std::string key{};
	for (auto & boardJSON : json.items()) {
		auto board{ Model::Board{} };
		if (!parseBoardDefinition(board, boardJSON.value())) {
			return false;
		}
		boardDefs.insert({boardJSON.key(), board});
	}

	return true;

}

bool ModelsJSONParser::loadCameraIntrinsics(std::unordered_map<std::string, Model::Camera::Intrinsics> & intrinsicsMap, nlohmann::json &jsonConfig) {
	static const std::string CALLER{ typeid(&ModelsJSONParser::loadCameraIntrinsics).name() };
	std::string key{};

	for (auto & json : jsonConfig.items()) {
		auto intrinsics{ Model::Camera::Intrinsics{} };
		// Intrisics
		key = "camera_matrix";
		if (!this->isKeySet(json.value(), key)) {
			this->eHandle->report(
				this->id + ": Invalid JSON. Intrinsics <" + key + "> not found.",
				Shared::Error::Severity::KILL_AND_EXIT
			);
			return false;
		}
		if (!this->loadPathHandleConfig(intrinsics.cameraMatrix, json.value()[key], CALLER)) {
			return false;
		}

		// Distortion coeff
		key = "distortion_coeff";
		if (!this->isKeySet(json.value(), key)) {
			this->eHandle->report(
				this->id + ": Invalid JSON. Intrinsics <" + key + "> not found.",
				Shared::Error::Severity::KILL_AND_EXIT
			);
			return false;
		}
		if (!this->loadPathHandleConfig(intrinsics.distortionCoeff, json.value()[key], CALLER)) {
			return false;
		}

		intrinsicsMap.insert({json.key(), std::move(intrinsics)});
	}

	return true;
}