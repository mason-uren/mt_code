#ifndef METROLOGY2020_THREADINTERFACE_H
#define METROLOGY2020_THREADINTERFACE_H

#include <string>
#include <iostream>
#include <sstream>
#include <memory>
#include <thread>
#include <functional>

// HRL
#include "ErrorHandler/ErrorHandler.h"
#include "Logger/Logger.h"
#include "JSONParser/JSONParser.h"
#include "ConfigInterface.h"

static constexpr int THREAD_TIMEOUT{ 1000 };

class ThreadInterface: public ConfigInterface {
public:
	ThreadInterface() :
		ConfigInterface("Default"),
		thread(), // Default initialization
		initialized(false),
		running(false),
		shouldDetach(false)
	{}

	ThreadInterface(const std::string name, const bool detach = false) :
	    ConfigInterface(std::move(name)),
		thread(), // Default initialization
		initialized(false),
		running(false),
		shouldDetach(detach)
	{}

	// Copy construction & assignment
	ThreadInterface(const ThreadInterface & obj) = delete;
	ThreadInterface & operator=(const ThreadInterface & obj) = delete;

	// Move construction & assignement
	ThreadInterface(ThreadInterface && obj)  noexcept :
		ConfigInterface(obj.name),
	    thread(std::move(obj.thread))
    {}
	ThreadInterface & operator=(ThreadInterface && obj)  noexcept {
		// Kill current process before assignment
		obj.stop();

		thread = std::move(obj.thread);
		return *this;
	}

	virtual ~ThreadInterface() = default;

	// Member
	virtual bool start(void * params = nullptr);
	virtual void stop();
	virtual void wait();
	virtual bool isInitialized() const;
	virtual bool isRunning() const;
	virtual std::string threadID() const;

protected:
	// Abstract
	virtual bool setup(void * params = nullptr) = 0;
	virtual void run() = 0;
	virtual bool cleanAndExit() = 0;

	// Variables
	bool initialized{};
	bool running{};
	bool shouldDetach{};

private:
	std::thread thread{};
};

#endif // METROLOGY2020_THREADINTERFACE_H
