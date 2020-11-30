//
// Created by U'Ren, Mason R (VEN) on 6/25/20.
//

#include "DynamicExtrinsicsJSONParser.h"

bool DynamicExtrinsicsJSONParser::loadDynamicExtrinsicConfig() {
    return this->loadDynamicExtrinsicConfig(this->path + FILE_SEP + this->configFile);
}

/**
 * @fn bool ConfigParser::loadDynamicExtrinsicConfig(const std::string &filePath)
 * @param filePath full (or relative) file path of target dynamic extrinsic configuration JSON
 * @return was the JSON loaded successfully
 */
bool DynamicExtrinsicsJSONParser::loadDynamicExtrinsicConfig(const std::string &filePath) {
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
	if (!(result = this->isKeySet(jsonConfig, key))) {
		this->eHandle->report(
			"<" + static_cast<std::string>(typeid(this).name()) + "> : Invalid JSON. <" + key + "> must be specified.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return result;
	}
	this->id = jsonConfig[key].get<std::string>();

	/**
	 * Parse fiducial positions config
	 */
	key = "fiducial-positions";
	if (!(result = this->isKeySet(jsonConfig, key))) {
		this->eHandle->report(
			this->id + ": Invalid JSON. No <" + key + "> to initialize.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return result;
	}
	
	auto fiducialPositionsConfig{ DE::FiducialPositions{} };
	if (!(result = this->loadFiducialPositionsConfig(fiducialPositionsConfig, jsonConfig[key]))) {
		return result;
	}
	this->fiducialPositions = std::move(fiducialPositionsConfig);

    /**
     * Parse datasets
     */
	key = "datasets";
    if (!(result = this->isKeySet(jsonConfig, key))) {
		this->eHandle->report(
			this->id + ": Invalid JSON. No <" + key + "> to initialize.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
        return result;
    }

    auto datasetConf{DE::Dataset{}};
    if (!(result = this->loadDataSetsConfig(datasetConf, jsonConfig[key]))) {
        return result;
    }
    this->datasetConfig = std::move(datasetConf);

    /**
     * Parse model config
     */
	key = "model";
     if (!(result = this->isKeySet(jsonConfig, key))) {
		 this->eHandle->report(
			 this->id + ": Invalid JSON. No <" + key + "> to initialize.",
			 Shared::Error::Severity::KILL_AND_EXIT
		 );
         return result;
     }

     auto modelConf{DE::Model{}};
     if (!this->loadModelConfig(modelConf, jsonConfig[key])) {
        return result;
     }
     this->model = std::move(modelConf);

    /**
     * Parse algorithm paramets
     * OPTIONAL
     */
	key = "alg-optimization";
     if (!(result = this->isKeySet(jsonConfig, key))) {
		 this->eHandle->report(this->id + ": No algorithm parameters specified. Using Default Presets.");
     }
     else {
        auto optimizationConfig{DE::Optimization::LFBGS{}};
        this->loadAlgOptConfig(optimizationConfig, jsonConfig[key]);
        this->algOptimization = optimizationConfig;
     }

     /**
      * Parse examples
      */
	 key = "examples";
      if (!(result = this->isKeySet(jsonConfig, key))) {
          this->eHandle->report(this->id + "No examples specified. Default: static ( ON) and dynamic ( ON).");
      }
      else {
          auto examplesConfig{DE::Examples{}};
          this->loadExamples(examplesConfig, jsonConfig[key]);
          this->examples = examplesConfig;
      }

     return this->validateLoadedJSON();
}

bool DynamicExtrinsicsJSONParser::loadFiducialPositionsConfig(DE::FiducialPositions & fiducialPositions, nlohmann::json & jsonConfig) {
	static const std::string CALLER{ typeid(&DynamicExtrinsicsJSONParser::loadFiducialPositionsConfig).name() };
	static const int MIN_FIDUCIAL_POSITIONS{2};

	bool result{};
	int validPoses{};

	for (auto & position : {
			Presets::FiducialPosition::TOP_LEFT,
			Presets::FiducialPosition::TOP_RIGHT,
			Presets::FiducialPosition::BOT_LEFT,
			Presets::FiducialPosition::BOT_RIGHT
		}) {
		// Check position
		if (!this->isKeySet(jsonConfig, position)) {
			this->eHandle->report(this->id + ": Fiducial pose <" + position + "> is not set.");
			continue;
		}

		// Check round
		if (!this->isKeySet(jsonConfig[position], "round")) {
			this->eHandle->report(this->id + ": No rounds for <" + position + ">.");
		}
		else {
			if (!jsonConfig[position]["round"].is_array() || jsonConfig[position]["round"].size() != MIN_FIDUCIAL_POSITIONS) {
				this->eHandle->report(this->id + ": Round must be an array of size: " + std::to_string(MIN_FIDUCIAL_POSITIONS));
				continue;
			}

			auto roundPathHandles{ std::vector<Shared::File::PathHandle>(jsonConfig[position]["round"].size()) }; //
			for (auto i = 0; i < roundPathHandles.size(); i++) {
				if (!(result = this->loadPathHandleConfig(roundPathHandles[i], jsonConfig[position]["round"][i], CALLER))) {
					break;
				}
			}

			// Check that paths were specififed
			if (!result) {
				continue;
			}

			fiducialPositions.map[position] = roundPathHandles;

			validPoses++;
		}
	}
	
	// Must have at least 2 valid fiducial positions
	return validPoses >= MIN_FIDUCIAL_POSITIONS;
}

bool DynamicExtrinsicsJSONParser::loadDataSetsConfig(DE::Dataset & datasets, nlohmann::json & jsonConfig) {
	static const std::string CALLER{ typeid(&DynamicExtrinsicsJSONParser::loadDataSetsConfig).name() };

	std::string key{ };

	// Matrix storage order
	key = "matrix_storage_order";
    if (this->isKeySet(jsonConfig, key)) {
		datasets.isRowMajor = jsonConfig[key].get<std::string>() == Presets::DataSet::StorageOrder::ROW_MAJ;
	}
	else {
		this->eHandle->report(this->id + ": No <" + key + "> specified. Default: \"row-major\"");
		datasets.isRowMajor = true;
	}

	// Decide whether to remove pan tilts
	key = "strip_pan";
	if (this->isKeySet(jsonConfig, key)) {
	    if (!jsonConfig[key].is_array()) {
            this->eHandle->report(
                this->id + ": Invalid JSON. <" + key + "> must be in array format of size = 2",
                Shared::Error::Severity::WARN
            );
	    }
	    else if (!jsonConfig[key].get<std::vector<double>>().empty()) {
	        datasets.panRanges = jsonConfig[key].get<std::vector<double>>();
	    }
	    else {
	        this->eHandle->report(this->id + ": Not stripping pan angles.");
	    }
	}

    key = "strip_tilt";
    if (this->isKeySet(jsonConfig, key)) {
        if (!jsonConfig[key].is_array()) {
            this->eHandle->report(
                    this->id + ": Invalid JSON. <" + key + "> must be in array format of size = 2",
                    Shared::Error::Severity::WARN
            );
        }
        else if (!jsonConfig[key].get<std::vector<double>>().empty()) {
            datasets.tiltRanges = jsonConfig[key].get<std::vector<double>>();
        }
        else {
            this->eHandle->report(this->id + ": Not stripping pan angles.");
        }
    }

	return true;
}

bool DynamicExtrinsicsJSONParser::loadModelConfig(DE::Model &model, nlohmann::json &jsonConfig) {
	static const std::string CALLER{ typeid(&DynamicExtrinsicsJSONParser::loadModelConfig).name() };

	std::string key{};

    // REQUIRED
    // Output
    key = "output";
    if (!this->isKeySet(jsonConfig, key)) {
        this->eHandle->report(
                this->id + ": Invalid JSON. Model output file path not specified.",
                Shared::Error::Severity::KILL_AND_EXIT
        );
        return false;
    }
    if (!this->loadPathHandleConfig(model.output, jsonConfig["output"], CALLER)) {
        this->eHandle->report(
                this->id + ": Invalid JSON. Error reading output file name from JSON.",
                Shared::Error::Severity::KILL_AND_EXIT
        );
        return false;
    }

    // DIrectory/Files
    // Input
	key = "input";
    if (this->isKeySet(jsonConfig, key)) {
        this->eHandle->report(
                this->id + ": Using prespecified model file.",
                Shared::Error::Severity::INFO
        );

        if (!this->loadPathHandleConfig(model.input, jsonConfig[key], CALLER)) {
            this->eHandle->report(
                    this->id + ": Invalid JSON. Error reading input file name from JSON.",
                    Shared::Error::Severity::KILL_AND_EXIT
            );
            return false;
        }

        model.separateFiles = true;
    }

    return true;
}

void DynamicExtrinsicsJSONParser::loadAlgOptConfig(DE::Optimization::LFBGS &optimizationConfig, nlohmann::json &jsonConfig) {
    // Epsilon
    if (!jsonConfig["epsilon"].is_null()) {
        optimizationConfig.epsilon = jsonConfig["epsilon"].get<double>();
    }
    // Delta
    if (!jsonConfig["delta"].is_null()) {
        optimizationConfig.delta = jsonConfig["delta"].get<double>();
    }
    // Max Iterations
    if (!jsonConfig["max_iterations"].is_null()) {
        optimizationConfig.maxIterations = jsonConfig["max_iterations"].get<double>();
    }
    // Max Iteration in subspace minimizations
    if (!jsonConfig["max_submin"].is_null()) {
        optimizationConfig.maxSubMin = jsonConfig["max_submin"].get<double>();
    }
    // Max number of trials (linesearch)
    if (!jsonConfig["max_linesearch"].is_null()) {
        optimizationConfig.maxLineSearch = jsonConfig["max_linesearch"].get<double>();
    }
    // Min step length (linesearch)
    if (!jsonConfig["min_step"].is_null()) {
        optimizationConfig.minStep = jsonConfig["min_step"].get<double>();
    }
    // Max step length (linesearch)
    if (!jsonConfig["max_step"].is_null()) {
        optimizationConfig.maxStep = jsonConfig["max_step"].get<double>();
    }
}

void DynamicExtrinsicsJSONParser::loadExamples(DE::Examples &examplesConfig, nlohmann::json &json) {
    examplesConfig.runStaticModelFitting = (json["static"].is_null() ? true : json["static"].get<bool>());
    examplesConfig.runDynamicModelFitting = (json["dynamic"].is_null() ? true : json["dynamic"].get<bool>());
}

bool DynamicExtrinsicsJSONParser::validateLoadedJSON() {
	bool result{true};

    // Dataset verification
	for (auto & pair : this->fiducialPositions.map) {
		result &= this->verifyPaths(pair.second);
	}

	if (this->model.separateFiles) {
	    result = this->verifyPath(this->model.input);
	}

	return result && this->verifyPath(this->model.output, true);
}
