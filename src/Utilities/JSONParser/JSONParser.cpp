#include "JSONParser.h"

// Static member variable initialization
bool JSONParser::hasFoundPath = false;
std::string JSONParser::foundPath = "";

void JSONParser::sourceConfigDirectory(const std::string & path) {
	JSONParser::foundPath = path;
}

bool JSONParser::openJSON(nlohmann::json & jsonConfig, const std::string & filePath) const {
	bool result{};

	std::ifstream fileHandle{ filePath };
	if (!(result = fileHandle.is_open())) {
		this->eHandle->report(
			"Failed to open file at path: [ " + filePath + "]",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return result;
	}

	try {
		fileHandle >> jsonConfig;
	}
	catch (const nlohmann::detail::out_of_range & error) {
		this->eHandle->report(
			error.what(),
			Shared::Error::Severity::KILL_AND_EXIT
		);
	}
	catch (...) {
		this->eHandle->report(
			this->id + ": Invalid JSON. Unable to parse <" + filePath + ">.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
	}

	return result;
}

bool JSONParser::isKeySet(nlohmann::json & jsonConfig, const std::string & key) {
	return jsonConfig.contains(key) && !jsonConfig[key].is_null();
}

bool JSONParser::loadPathHandleConfig(Shared::File::PathHandle & pathHandle, nlohmann::json & jsonConfig, const std::string & caller) {
	// Directory
	if (jsonConfig["directory"].is_null()) {
		this->eHandle->report(
			this->id + ": No directory specified for <" + caller + ">",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}

	// File(s) and Path(s)
	if (jsonConfig["file"].is_null()) {
		this->eHandle->report(
			this->id + ": No file(s) specified for <" + caller + ">",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}

	pathHandle.directory = jsonConfig["directory"].get<std::string>();
	pathHandle.file = jsonConfig["file"].get<std::string>();
	pathHandle.path = pathHandle.directory + FILE_SEP + pathHandle.file;

	return true;
}

bool JSONParser::verifyPaths(const std::vector<Shared::File::PathHandle> & pathHandles, const bool allowedToCreateFile) const {
	bool result{};

	for (auto & pathHandle : pathHandles) {
		if (!(result = verifyPath(pathHandle, allowedToCreateFile))) {
			break;
		}
	}

	return result;
}

bool JSONParser::verifyPath(const Shared::File::PathHandle & pathHandle, const bool allowedToCreateFile) const {
	bool result{};

	std::ifstream _fileHandle{ pathHandle.path };
	// Check if file exists
	if (!(result = _fileHandle.is_open())) {

		// No file - Are we allowed to create one
		if (allowedToCreateFile) {
			auto createdFile{ std::ofstream(pathHandle.path, std::ios::out) };

			// Check to see file was created
			if (!(result = createdFile.is_open())) {
				this->eHandle->report(
					this->id + ": Failed to verify file at path: [ " + pathHandle.path + "]",
					Shared::Error::Severity::KILL_AND_EXIT
				);
			}
		}
		else {
			this->eHandle->report(
				this->id + ": Failed to verify file at path: [ " + pathHandle.path + "]",
				Shared::Error::Severity::KILL_AND_EXIT
			);
		}
	}

	return result;
}