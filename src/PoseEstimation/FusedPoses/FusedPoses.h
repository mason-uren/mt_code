#ifndef METROLOGY2020_WORLDFRAME_H
#define METROLOGY2020_WORLDFRAME_H

#include <string>
#include <iostream>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <vector>

// HRL
#include <Shared/Presets.h>

// Utilities
#include "ErrorHandler/ErrorHandler.h"

#include "SixDOF.h"


class FusedPoses {
public:
	struct Context;
	static FusedPoses * getInstance() {
		static FusedPoses instance;
		return &instance;
	}

	// Functions
	bool addInstance(const std::string & id, Context & context);
	bool recordEstimatedPose(const std::string & instance, const SixDOF & sixDOF);
	bool fetchPose(const std::string & caller, SixDOF & sixDOF);
	void updateFiducialState(const std::string & caller, const bool isFound);


	// Strcuts
	struct Context {
	    Context() = default;
	    Context(const Presets::Device::Type & type,
	            const SixDOF & sixDOF,
	            const std::string & loggerID = Presets::Logger::DEFAULT_ID,
	            const bool isFiducialVisible = false) :
	            type(type), sixDOF(sixDOF), loggerID(loggerID), isFiducialVisible(isFiducialVisible) {}

		Presets::Device::Type type{};
		SixDOF sixDOF{};
		std::string loggerID{  };
		bool isFiducialVisible{};
	};

private:
	FusedPoses(ErrorHandler * eHandle = ErrorHandler::getInstance()) : 
		contexts{}, 
		wFramePtr{},
		eHandler(eHandle) {}
	~FusedPoses() = default;

	// Functions
	bool getContext(const std::string & caller, const std::string & loggerID);

	// Variables
	std::mutex contextMux{};
	std::unordered_map<std::string, Context> contexts{};
	std::string wFramePtr{};

	// Static
	ErrorHandler * eHandler;
};

#endif // METROLOGY2020_WORLDFRAME_H