#ifndef METROLOGY2020_ERRORHANDLER_H
#define METROLOGY2020_ERRORHANDLER_H

#include <vector>
#include <algorithm>
#include <mutex>


// HRL
#include <Shared/InterfaceConfig.h>

#include "Logger/Logger.h"

class ErrorHandler {
public:
	static ErrorHandler * getInstance() {
		static ErrorHandler instance;
		return &instance;
	}

	// Copy constuction & assignment
	ErrorHandler(ErrorHandler const &) = delete;
	void operator=(ErrorHandler const &) = delete;

	void report(const std::string &msg, 
				const Shared::Error::Severity &severity = Shared::Error::Severity::INFO, 
				const std::string & loggerID = Presets::Logger::DEFAULT_ID);
	void reset();
	bool shouldContinue();
	Shared::Error::Severity getSeverity();
	void setListeningLevel(const Shared::Error::Severity &severity);

private:
	ErrorHandler() :
		currLevel(Shared::Error::Severity::INFO),
		reportingLevel(Shared::Error::Severity::INFO),
		logger(Logger::getInstance())
	{
		if (!logger->isListening()) {
			logger->startListening();
		}
	}
	~ErrorHandler() = default;
	
	// Functions
	bool shouldIgnore(const Shared::Error::Severity &severity);

	// Variables
	std::mutex severityMux{};
	std::mutex reportMux{};

	int currLevel{};
	Shared::Error::Severity reportingLevel{};
	Logger * logger;
};

#endif // METROLOGY2020_ERRORHANDLER_H