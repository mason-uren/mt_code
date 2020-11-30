#ifndef METROLOGY2020_TRACKINGFIDUCIAL_CONFIGPARSER_H
#define METROLOGY2020_TRACKINGFIDUCIAL_CONFIGPARSER_H


#include <iostream>
#include <unordered_map>

// HRL
#include <Shared/Presets.h>
#include <Shared/SharedStructs.h>
#include <Shared/TrackingConfig.h>
#include <Shared/OSDefines.h>

#include "JSONParser/JSONParser.h"

class TrackingJSONParser : private JSONParser {
public:
	TrackingJSONParser(const std::string & configFile = Presets::JSON::TRACKING, const bool isSkeletonClass = false) : 
		JSONParser(Presets::JSON::DEFUALT_DIR, isSkeletonClass),
		models{},
		stagingPositions{},
		markerDetectionScales{},
		mode{},
		display{},
		input{},
		output{}
	
	{
		JSONParser::configFile = configFile;
	}
	~TrackingJSONParser() = default;

	// Functions
	bool loadTrackingConfig();
	bool loadTrackingConfig(const std::string & filePath);

	// Variables
	std::unordered_map<std::string, Shared::File::PathHandle> models{};
	std::unordered_map<std::string, std::vector<double>> stagingPositions{};
	std::unordered_map<std::string, double> markerDetectionScales{};


	Tracking::Visualization::Display display{};

	
	Tracking::Visualization::Video output{};
	

private:
	// Functions
	bool validateLoadedJSON() override;
	
	bool loadModelsConfig(std::unordered_map<std::string, Shared::File::PathHandle> & modelsConfig, nlohmann::json & json);
	bool loadStagingPositions(std::unordered_map<std::string, std::vector<double>> & stagingPositions, nlohmann::json & json);
	bool loadVisualConfig(Tracking::Visualization::Display & display, nlohmann::json & json);
	bool loadRecordingConfig(Tracking::Visualization::Video & input, Tracking::Visualization::Video & output, nlohmann::json & json);

	// Variables
	std::string mode{};
	Tracking::Visualization::Video input{};
};

#endif // METROLOGY2020_TRACKINGFIDUCIAL_CONFIGPARSER_H