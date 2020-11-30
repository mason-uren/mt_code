#ifndef METROLOGY2020_CONFIGINTERFACE_H
#define METROLOGY2020_CONFIGINTERFACE_H

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <utility>

// json
#include "nlohmann/json.hpp"
// HRL
#include <Shared/OSDefines.h>
#include <Shared/Presets.h>
#include <Shared/SharedStructs.h>

#include "ErrorHandler/ErrorHandler.h"

class JSONParser {
public:
    explicit JSONParser(std::string  configPathRootDir = Presets::JSON::DEFUALT_DIR, const bool isSkeletonClass = false) :
        eHandle(ErrorHandler::getInstance())
	{
		// Is skelecton class... (only create objects, DO NOT source)
		if (isSkeletonClass) {
			eHandle->report("Creating skeleton JSON class.");
			eHandle->report("<path> : [UNSET]. Will need to manually set root JSON directory.");
		}
		else {
			// Default initialization...
			if (!hasFoundPath) {
				std::string root{ FILE_SEP + std::move(configPathRootDir) };
				struct stat info;

				// Get current file path
				char buf[FILENAME_MAX];
				std::string cwd{ GetCurrentDir(buf, FILENAME_MAX) };

				eHandle->report("Searching for <Config> directory...");
				std::cout << "Searching for Config/ directory..." << std::endl;

				// Search directory for Config/ (if doesn't exist)
				while (!(stat((path = cwd + root).c_str(), &info) == 0 && S_IFDIR)) {
					cwd = cwd.substr(0, cwd.find_last_of("/\\"));
				}

				eHandle->report("Found: " + path);
				std::cout << "Found: " << path << std::endl;

				// Save shared reference to root directory (static)
				sourceConfigDirectory(path);

				hasFoundPath = true;
			}
			else {
				path = foundPath;
				eHandle->report("Path previously sourced ( " + path + ")");
				std::cout << "Path previously sourced ( " << path << ")" << std::endl;
			}
		}
	}
	~JSONParser() = default;

protected:
	// Functions
	virtual bool validateLoadedJSON() = 0;

	void sourceConfigDirectory(const std::string & rootDir);
	bool openJSON(nlohmann::json & jsonConfig, const std::string & filePath) const;
	bool isKeySet(nlohmann::json & jsonConfig, const std::string & key);
	bool loadPathHandleConfig(Shared::File::PathHandle & pathHandle, nlohmann::json & jsonConfig, const std::string & caller);
	bool verifyPaths(const std::vector<Shared::File::PathHandle> & pathHandles, const bool allowedToCreateFile = false) const;
	bool verifyPath(const Shared::File::PathHandle & pathHandle, const bool allowedToCreateFile = false) const;

	// Variables
	ErrorHandler * eHandle;

	std::string path;
	static std::string foundPath;
	static bool hasFoundPath;

	// Set by children
	std::string id{};
	std::string configFile{};
};

#endif // METROLOGY2020_CONFIGINTERFACE_H

