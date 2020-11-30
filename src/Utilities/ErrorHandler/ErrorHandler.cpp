#include "ErrorHandler.h"

void ErrorHandler::report(const std::string &msg, const Shared::Error::Severity &severity, const std::string & loggerID) { 
	this->reportMux.lock();
	this->currLevel = std::max(this->currLevel, (int) (this->shouldIgnore(severity) ? Shared::Error::Severity::INFO : severity));
	this->reportMux.unlock();

	// Check recording level
    if (!this->shouldIgnore(severity)) {
        this->logger->logMsg( Shared::Log::Msg{ Shared::Log::severities[severity], msg }, loggerID); 
    }
}

void ErrorHandler::reset() {
    std::lock_guard<std::mutex> guard(this->reportMux);
	if (this->currLevel < (int) Shared::Error::Severity::KILL_AND_EXIT) {
		this->currLevel = (int)Shared::Error::Severity::INFO;
	}
}

bool ErrorHandler::shouldContinue() {
	std::lock_guard<std::mutex> guard(this->reportMux);
    return this->currLevel < (int) Shared::Error::Severity::KILL_AND_EXIT;
}

Shared::Error::Severity ErrorHandler::getSeverity() {
	std::lock_guard<std::mutex> guard(this->reportMux);
	return static_cast<Shared::Error::Severity>(this->currLevel);
}

void ErrorHandler::setListeningLevel(const Shared::Error::Severity &severity) {
    std::lock_guard<std::mutex> guard(this->severityMux);
    this->reportingLevel = severity;
}

bool ErrorHandler::shouldIgnore(const Shared::Error::Severity &severity) {
	std::lock_guard<std::mutex> guard(this->severityMux);
    return this->reportingLevel > severity;
}