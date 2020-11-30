#include "Logger.h"

// Static memeber initialization
std::unordered_map<std::string, std::thread> Logger::threads{};
std::unordered_map<std::string, std::ofstream> Logger::handles{};
std::unordered_map<std::string, std::queue<std::string>> Logger::msgBufferQueues{};
// END Static

void Logger::startListening() {
	if (!this->isThreadListening) {
		this->isThreadListening = true;

		this->setup(Presets::Logger::DEFAULT_ID, this->fileName);
		
		// Give time for thread to start
		Sleep(100);
	}
}

void Logger::stopListening() {
	this->isThreadListening = false;

	while (!this->readyToClose) {
		// Spin...
		Sleep(10);
	}

	for (auto & instance : Logger::threads) {
		this->close(instance.first);
	}
}

bool Logger::isListening() const {
	return this->isThreadListening;
}

bool Logger::addInstance(const std::string & loggerID) {
	// Report to main logger
	// Lambda
	auto report = [&](const std::string & msg) {
		std::cerr << msg << std::endl;
		this->logMsg(Shared::Log::Msg{ msg }, Presets::Logger::DEFAULT_ID);
	};
	// END lambda

	try {
		return this->setup(loggerID, std::string{FILE_SEP}.append(loggerID + ".out"));
	}
	catch (const std::bad_alloc & error) {
		auto sstream{ std::stringstream{} };
		sstream << "Unable to add logging instance <" << loggerID << ">." << std::endl;
		sstream << error.what() << std::endl;

		report(sstream.str());

		return false;
	}
	catch (...) {
		auto sstream{ std::stringstream{} };
		sstream << "An unknown error occured adding logging instance <" << loggerID << ">." << std::endl;

		report(sstream.str());

		return false;
	}
}

void Logger::logMsg(const Shared::Log::Msg & msg, const std::string & msgBufferID) {
    /**
     * EX: [ Time] ( Type:Severity) "Message"
     */
	std::lock_guard<std::mutex> guard(this->bufMux);
	try {
		Logger::msgBufferQueues.at(msgBufferID).emplace("\n[ " + std::to_string(time(NULL)) + "] ( " + msg.severity + ") " + msg.data);
	}
	catch (const std::out_of_range & error) {
		std::cerr << "Unable to log msg for instance key <" << msgBufferID << ">." << std::endl;
		std::cerr << error.what() << std::endl;
	}
}

bool Logger::flush(const std::string & handleID) {
	bool result{};

	std::unique_lock<std::mutex> bufferGuard(this->bufMux, std::defer_lock);
	std::unique_lock<std::mutex> handleGuard(this->handleMux, std::defer_lock);
	{
		// Ensure lock of both mutex(s)
		std::lock(bufferGuard, handleGuard);

		try {
			while (!Logger::msgBufferQueues.at(handleID).empty()) {
				Logger::handles.at(handleID) << Logger::msgBufferQueues.at(handleID).front();
				Logger::handles.at(handleID).flush();
				Logger::msgBufferQueues.at(handleID).pop();
			}
		}
		catch (const std::out_of_range & error) {
			std::cerr << "(Logger)" << error.what() << std::endl;
			return false;
		}
	}

	return true;
}

void Logger::close(const std::string & threadID) {
	this->handleMux.lock();
	for (auto & instance : Logger::handles) {
		if (instance.second.is_open()) {
			instance.second.close();
		}
	}
	this->handleMux.unlock();

	try {
		if (Logger::threads.at(threadID).joinable()) {
			Logger::threads.at(threadID).join();
		}
	}
	catch (std::system_error & error) {
		std::cout << "Unable to close Logger::thread" << std::endl;
		std::cout << error.what() << std::endl;
	}
	catch (std::out_of_range & error) {
		std::cerr << "Unable to close Logger::thread" << std::endl;
		std::cerr << error.what() << std::endl;
	}
}

bool Logger::setup(const std::string & ID, const std::string & file) {
	bool result{};

	std::unique_lock<std::mutex> bufferGuard(this->bufMux, std::defer_lock);
	std::unique_lock<std::mutex> handleGuard(this->handleMux, std::defer_lock);
	std::unique_lock<std::mutex> threadGuard(this->threadMux, std::defer_lock);
	{
		// Ensure lock of all mutexs
		std::lock(threadGuard, handleGuard, bufferGuard);

		try {
			// Initialize instance handle and message buffer..
            Logger::handles.insert(std::pair<std::string, std::ofstream>(ID, std::ofstream(this->directory + file, std::ios::out)));
            Logger::msgBufferQueues.insert( std::pair<std::string, std::queue<std::string>>(ID, std::queue<std::string>{})  );

			// Start thread instance...
			Logger::threads.insert(std::pair<std::string, std::thread>(ID, std::thread(&Logger::readBuffer, this, ID)));
		}
		catch (const std::bad_alloc & error) {
			std::cerr << "(Logger) Unable to add logger instance key <" << ID << ">." << std::endl;
			std::cerr << error.what() << std::endl;
			return false;
		}
	}

	return true;
}

void Logger::readBuffer(const std::string & bufferID) {
	while (this->isThreadListening) {
		this->flush(bufferID);
	}

	// Ensure everything is flushed from buffer
	this->flush(bufferID);

	this->readyToClose = true;
}