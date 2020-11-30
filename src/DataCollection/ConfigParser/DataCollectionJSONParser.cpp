#include "DataCollectionJSONParser.h"

bool DataCollectionJSONParser::loadDataCollectionConfig() {
	return this->loadDataCollectionConfig(this->path + FILE_SEP + this->configFile);
}

bool DataCollectionJSONParser::loadDataCollectionConfig(const std::string & filePath) {
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
	 * Parse scanning pattern
	 */
	key = "scan_pattern";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": Invalid JSON. <" + key + "> must be specified",
			Shared::Error::KILL_AND_EXIT
		);
		return false;
	}

	auto patternConfig{ DataCollection::ScanPattern{} };
	if (!this->parseScanPattern(patternConfig, jsonConfig[key])) {
		return false;
	}
	this->scanPattern = std::move(patternConfig);

	/**
	 * Parse grid
	 */
	key = "grid";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": Invalid JSON config. Must specify <" + key + ">",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}
	
	// REQUIRED
	auto collectionConfig{ DataCollection::CollectionConfig{} };
	if (!this->isKeySet(jsonConfig[key], "pan")) {
		this->eHandle->report(
			this->id + ": Invalid JSON. <pan> domain has not been specified.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}
	if (!(this->parseAxisDomain(collectionConfig.pan, jsonConfig[key]["pan"], "pan") &&
			(this->parseAxisDomain(collectionConfig.tilt, jsonConfig[key]["tilt"], "tilt")))) {
		return false;
	}
	// Save result
	this->collection = std::move(collectionConfig);
		

	//  OPTIONAL
	key = "save_images_for_offline_proc";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": Data collection will not store images",
			Shared::Error::Severity::WARN
		);
	}
	else {
		this->saveImages = jsonConfig[key].get<bool>();
	}

	/**
	 * Parse staging point config
	 */
	key = "staging_point";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": No <" + key + "> specified. Default: [0, 0] (Home position)",
			Shared::Error::Severity::WARN
		);
	}
	else {
		if (!jsonConfig[key].is_array()) {
			this->eHandle->report(
				this->id + ": Invalid JSON. <" + key + "> should be of type=array., Default: [0, 0]",
				Shared::Error::Severity::WARN
			);
		}
		else {
			this->stagingPt[0] = jsonConfig[key][0].get<double>();
			this->stagingPt[1] = jsonConfig[key][1].get<double>();
		}
	}

	
	

	/**
	 * Parse h5 file location
	 */
	key = "h5";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": Invalid JSON. Must specify H5-file save location.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}

	auto h5HandleConfig{ DataCollection::H5Handle{} };
	if (!this->loadH5HandleConfig(h5HandleConfig, jsonConfig[key])) {
		return false;
	}
	this->h5Handle = std::move(h5HandleConfig);

	return this->validateLoadedJSON();
}

bool DataCollectionJSONParser::validateLoadedJSON() {
	return true;
}

bool DataCollectionJSONParser::parseAxisDomain(DataCollection::Domain & domain, nlohmann::json & jsonConfig, const std::string & axis) {
	static constexpr size_t DOMAIN_SIZE = 2;
	std::string key{};

	// Range
	key = "range";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": Invalid JSON. No <range>. specified axis: " + axis,
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}

	domain.range = jsonConfig[key].get<std::vector<double>>();

	if (domain.range.size() != DOMAIN_SIZE) {
		this->eHandle->report(
			this->id + ": Invalid JSON. Range parameters. axis: " + axis,
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}

	// Slots
	key = "slots";
	if (!this->isKeySet(jsonConfig, key)) {
		this->eHandle->report(
			this->id + ": Invalid JSON. No <slots> specified axis: " + axis,
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}

	int slots{ jsonConfig[key].get<int>() };
	if (slots > 0 ) {
		domain.slots = jsonConfig[key].get<int>();
	}
	else {
		this->eHandle->report(
			this->id + ": Invalid JSON. Bad <" + key + "> value: " + std::to_string(slots),
			Shared::Error::Severity::KILL_AND_EXIT
		);
		domain.slots = 1;
		this->eHandle->report("Default: Slots ( " + std::to_string(domain.slots) + ")");
	}

	return true;
}

bool DataCollectionJSONParser::loadH5HandleConfig(DataCollection::H5Handle & h5Handle, nlohmann::json & jsonConfig) {
	static const std::string CALLER{ typeid(&DataCollectionJSONParser::loadH5HandleConfig).name() };
	
	return this->loadPathHandleConfig(h5Handle.pathHandle, jsonConfig, CALLER);
}

bool DataCollectionJSONParser::parseScanPattern(DataCollection::ScanPattern & scanPattern, nlohmann::json & json) {
	auto key{ std::string{"pattern"} };
	if (!this->isKeySet(json, key)) {
		this->eHandle->report(
			this->id + ": Invalid JSON. No designated <" + key + ">. Default: <" + Presets::DataCollection::ScanningPattern::VERTICAL_RASTER + ">",
			Shared::Error::Severity::WARN
		);
		// Default
		scanPattern.pattern = Presets::DataCollection::ScanningPattern::VERTICAL_RASTER;
	}
	else {
		scanPattern.pattern = json[key].get<std::string>();
	}

	// Optional - if <pattern> is <random>
	key = "origin";
	if (scanPattern.pattern != Presets::DataCollection::ScanningPattern::RANDOM) {
		if (!this->isKeySet(json, key)) {
			this->eHandle->report(
				this->id + ": Invalid JSON. <" + key + "> is only optional if using <random> scan pattern",
				Shared::Error::Severity::KILL_AND_EXIT
			);
			return false;
		}
		else {
			scanPattern.origin = json[key].get<std::string>();
		}
	}

	return true;
}