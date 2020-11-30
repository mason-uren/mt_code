#ifndef METROLOGY2020_MODELS_MODELSJSONPARSER_H
#define METROLOGY2020_MODELS_MODELSJSONPARSER_H

#include <string>

// HRL
#include <Shared/ModelsConfig.h>
#include <Shared/Presets.h>
#include <Shared/OSDefines.h>
#include <Shared/SharedStructs.h>

#include "JSONParser/JSONParser.h"

class ModelsJSONParser : JSONParser {
public:
	ModelsJSONParser(const std::string & configFile = Presets::JSON::MODELS, const bool isSkeletonClass = false) :
		JSONParser(Presets::JSON::DEFUALT_DIR, isSkeletonClass),
		chArUcoBoards(std::unordered_map<std::string, Model::Board>()),
		ximea(std::unordered_map<std::string, Model::Camera::Intrinsics>()),
		imperx(std::unordered_map<std::string, Model::Camera::Intrinsics>())
	{
		JSONParser::configFile = configFile;
	}
	~ModelsJSONParser() = default;

	// Functions
	bool loadModelsConfig();
	bool loadModelsConfig(const std::string & filePath);

	// Variables
	std::unordered_map<std::string, Model::Board> chArUcoBoards{};
	std::unordered_map<std::string, Model::Camera::Intrinsics> ximea{};
	std::unordered_map<std::string, Model::Camera::Intrinsics> imperx{};

private:
	// Functions
	bool validateLoadedJSON() override;

	bool loadChArUcoBoardDefinitions(std::unordered_map<std::string, Model::Board> & boardDefs, nlohmann::json & json);
	bool loadCameraIntrinsics(std::unordered_map<std::string, Model::Camera::Intrinsics> & intrinsics, nlohmann::json & json);

};

#endif // METROLOGY2020_MODELS_MODELSJSONPARSER_H