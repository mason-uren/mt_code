#ifndef METROLOGY2020_LOGGER_H
#define METROLOGY2020_LOGGER_H

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <mutex>
#include <ctime>
#include <utility>
#include <thread>
#include <queue>
#include <unordered_map>

// HRL
#include <Shared/SharedStructs.h>
#include <Shared/OSDefines.h>
#include <Shared/Presets.h>


class Logger {
public:
	static Logger * getInstance(
		const std::string & logDir = std::string{FILE_SEP}.append(Presets::Logger::DEFAULT_LOG_DIR), 
		const std::string & logFile = std::string{FILE_SEP}.append(Presets::Logger::DEFAULT_OUT_FILE))
    {
		static Logger instance(logDir, logFile);
		return &instance;
	}

	Logger(Logger const &) = delete;
	void operator=(Logger const &) = delete;

	void startListening();
	void stopListening();
	bool isListening() const;
	bool addInstance(const std::string & loggerID);
	void logMsg(const Shared::Log::Msg & msg, const std::string & msgBufferID = Presets::Logger::DEFAULT_ID);
	bool flush(const std::string & handleID = Presets::Logger::DEFAULT_ID);
	void close(const std::string & threadID = Presets::Logger::DEFAULT_ID);

private:
	struct stat info;
	Logger(std::string logDir, std::string logFile) :
		directory(std::move(logDir)),
		fileName(std::move(logFile)),
		isThreadListening(false),
		readyToClose(false)
	{
		// Get current file path
		char buf[FILENAME_MAX];	
		std::string cwd{GetCurrentDir(buf, FILENAME_MAX)};
		
		int result{};
		// Create </Logs> directory (if doesn't exist)
		if (!(stat((directory = cwd + directory).c_str(), &info) == 0 && S_IFDIR)) {

#ifdef WIN32
// Windows
            if (!CreateDirectoryA((LPCSTR) directory.c_str(), NULL))
#else 
//Linux & MacOS
            if ((result = mkdir(directory.c_str(), S_IRWXU | S_IRWXG)) )
#endif
			{
				std::cerr << "Error: Failed to create directory: " << result << std::endl;
			}
		}
	}
	~Logger() {
		stopListening();
	}

	// Functions
	bool setup(const std::string & ID = Presets::Logger::DEFAULT_ID, const std::string & fileName = Presets::Logger::DEFAULT_OUT_FILE);
	void readBuffer(const std::string & bufferID = Presets::Logger::DEFAULT_ID);

	// Variables
	bool isThreadListening{};
	bool readyToClose{};
	std::string directory{};
	std::string fileName{};

	std::mutex bufMux{};
	std::mutex handleMux{};
	std::mutex threadMux{};

	// Static
	static std::unordered_map<std::string, std::thread> threads;
	static std::unordered_map<std::string, std::ofstream> handles;
	static std::unordered_map<std::string, std::queue<std::string>> msgBufferQueues;
};

#endif // METROLOGY2020_LOGGER_H