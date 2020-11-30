#ifndef METROLOGY2020_CONFIGPARSER_H
#define METROLOGY2020_CONFIGPARSER_H

#include <string>
#include <fstream>
#include <iostream>
#include <unordered_map>

// HRL
#include <Shared/InterfaceConfig.h>
#include <Shared/Presets.h>
#include <Shared/OSDefines.h>
#include <Shared/CameraDefines.h>
#include <Shared/SharedStructs.h>

#include "JSONParser/JSONParser.h"

class InterfaceJSONParser: private JSONParser {
public:
	explicit InterfaceJSONParser(const std::string & configFile = Presets::JSON::INTERFACE, const bool isSkeletonClass = false) :
		JSONParser(Presets::JSON::DEFUALT_DIR, isSkeletonClass),
		cameras{},
		ptus{}
	{
		JSONParser::configFile = configFile;
	}
	~InterfaceJSONParser() = default;

	// Functions
	bool loadInterfaceConfig();
	bool loadInterfaceConfig(const std::string & filePath);
	bool hasWFOV(const std::string & ID);	

	// Variables
	std::vector<Interface::Imaging::Camera> cameras{};
	std::vector<Interface::PanTilt::PTU> ptus{};

private:
	// Functions
	bool validateLoadedJSON() override;

	void loadImageFormat(Interface::Imaging::Image & image, const std::string & format);
	void loadDownsamplingType(Interface::Imaging::Image & image, const std::string & dwnSmplType);
	bool loadNetworkProtocol(Interface::Network::Pattern & pattern, nlohmann::json & json, const std::string & key);
	void loadAxesContext(Interface::PanTilt::Axes & axes, nlohmann::json & json);
	void loadPIDController(Interface::PanTilt::PID & pid, nlohmann::json & json);

	// Variables
	std::vector<std::string> cameraIDs{};
};

#endif // METROLOGY2020_CONFIGPARSER_H