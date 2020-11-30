#ifndef METROLOGY2020_DATACOLLECTION_CONFIGPARSER_H
#define METROLOGY2020_DATACOLLECTION_CONFIGPARSER_H

#include <iostream>
#include <vector>

// HRL
#include <Shared/DataCollectionConfig.h>
#include <Shared/Presets.h>
#include <Shared/OSDefines.h>
#include <Shared/SharedStructs.h>

#include "JSONParser/JSONParser.h"

class DataCollectionJSONParser : private JSONParser {
public:
	DataCollectionJSONParser(const std::string & configFile = Presets::JSON::DATA_COLLECTION, const bool isSkeletonClass = false) :
		JSONParser(Presets::JSON::DEFUALT_DIR, isSkeletonClass),
		collection{},
		h5Handle{},
		scanPattern{},
		stagingPt{0, 0}
	{
		JSONParser::configFile = configFile;
	}
	~DataCollectionJSONParser() = default;

	// Functions
	bool loadDataCollectionConfig();
	bool loadDataCollectionConfig(const std::string & filePath);

	// Variables
	DataCollection::CollectionConfig collection{};
	DataCollection::H5Handle h5Handle{};
	DataCollection::ScanPattern scanPattern{};
	std::vector<double> stagingPt{};
	bool saveImages{};

private:
	// Functions
	bool validateLoadedJSON() override;
	bool parseAxisDomain(DataCollection::Domain & domain, nlohmann::json & json, const std::string & axis);
	bool parseScanPattern(DataCollection::ScanPattern & scanPattern, nlohmann::json & json);
	bool loadH5HandleConfig(DataCollection::H5Handle & h5Handle, nlohmann::json & json);
};
#endif // !METROLOGY2020_DATACOLLECTION_CONFIGPARSER_H
