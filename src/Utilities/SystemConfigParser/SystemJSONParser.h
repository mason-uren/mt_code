#ifndef METROLOGY2020_SYSTEMJSONPARSER_H
#define METROLOGY2020_SYSTEMJSONPARSER_H

#include <string>
#include <iostream>
#include <unordered_map>

// HRL
#include <Shared/Presets.h>
#include <Shared/OSDefines.h>
#include <Shared/SharedStructs.h>
#include <Shared/SystemConfig.h>

#include "JSONParser/JSONParser.h"

class SystemJSONParser : private JSONParser {
public:
	explicit SystemJSONParser(const std::string & configFile = Presets::JSON::SYSTEM, const bool isSkeletonClass = false) :
		JSONParser(Presets::JSON::DEFUALT_DIR, isSkeletonClass),
		mode{},
		pairs{}
	{
		JSONParser::configFile = configFile;
	}
	~SystemJSONParser() = default;

	// Functions
	bool loadSystemConfig();
	bool loadSystemConfig(const std::string & filePath);
	Presets::Mode::Enum runningMode(const std::string & mode);

	// Variables
	System::Mode mode{};
	System::Camera wFrame{};
	// Key: <pair name> Value: Associated camera/ptu/intrinsics
	std::unordered_map<std::string, System::Pair> pairs{};


private:
	// Functions
	bool validateLoadedJSON() override;
	bool loadRunningState(System::Mode & mode, nlohmann::json & json);
	bool loadEnvironment(const std::vector<std::string> & targetComps, nlohmann::json & json);
	bool loadCameraParams(System::Camera & camera, nlohmann::json & json);
	bool loadComponentPair(System::Pair & pair, nlohmann::json & json);
};

#endif // METROLOGY2020_SYSTEMJSONPARSER_H
