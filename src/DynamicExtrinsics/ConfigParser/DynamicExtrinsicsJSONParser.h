//
// Created by U'Ren, Mason R (VEN) on 6/25/20.
//

#ifndef METROLOGY2020_ALGORITHMS_DYNAMICEXTRINSICSCONFIGPARSER_H
#define METROLOGY2020_ALGORITHMS_DYNAMICEXTRINSICSCONFIGPARSER_H

#include <string>
#include <iostream>
#include <fstream>

// json
#include <nlohmann/json.hpp>

// HRL
#include <Shared/DynamicExtrinsicConfig.h>
#include <Shared/Presets.h>
#include <Shared/SharedStructs.h>

#include "JSONParser/JSONParser.h"

class DynamicExtrinsicsJSONParser : private JSONParser {
public:
	explicit DynamicExtrinsicsJSONParser(const std::string & configFile = Presets::JSON::DYNAMIC_EXTRINSICS, const bool isSkeletonClass = false) :
		JSONParser(Presets::JSON::DEFUALT_DIR, isSkeletonClass),
		fiducialPositions{},
		datasetConfig{},
		model{},
		algOptimization{},
		examples{}
	{
		JSONParser::configFile = configFile;
	}
    ~DynamicExtrinsicsJSONParser() = default;

    // Functions
	bool loadDynamicExtrinsicConfig();
    bool loadDynamicExtrinsicConfig(const std::string & filePath);

	// Variables
	DE::FiducialPositions fiducialPositions{};
	DE::Dataset datasetConfig{};
	DE::Model model{};
	DE::Optimization::LFBGS algOptimization{};
	DE::Examples examples{};

private:
    // Functions
	bool validateLoadedJSON() override;
	bool loadFiducialPositionsConfig(DE::FiducialPositions & fiducialPositions, nlohmann::json & json);
	bool loadDataSetsConfig(DE::Dataset & datasets, nlohmann::json & json);
    bool loadModelConfig(DE::Model & model, nlohmann::json & json);
    void loadAlgOptConfig(DE::Optimization::LFBGS & optimizationConfig, nlohmann::json & json);
    void loadExamples(DE::Examples & examplesConfig, nlohmann::json & json);
};


#endif //METROLOGY2020_ALGORITHMS_DYNAMICEXTRINSICSCONFIGPARSER_H


