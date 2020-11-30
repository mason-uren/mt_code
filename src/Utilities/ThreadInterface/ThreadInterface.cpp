#include "ThreadInterface.h"

bool ThreadInterface::start(void * params) {
	// Setup thread....
	if (!(this->initialized = this->setup(params))) {
		this->eHandler->report(
			"Failed to setup <" + this->name + ">.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}

	Sleep(100);

	// Start worker thread
	this->thread = std::thread(&ThreadInterface::run, this);

	Sleep(100);
	
	// Get Thread ID
	auto stream{ std::stringstream{} };
	stream << this->thread.get_id();

	if (this->shouldDetach) {
		this->thread.detach();
		this->eHandler->report("Thread ( " + this->name + ":" + stream.str() + "): Detached");
	}


	auto start{ std::chrono::steady_clock::now() };

	while (!this->running) {
		auto diff{ (std::chrono::steady_clock::now() - start).count() };
		if (THREAD_TIMEOUT < diff) {
			this->eHandler->report(
				"Thread ( " + this->name + "): Failed to start. Timed out! Set: " + std::to_string(THREAD_TIMEOUT) + "(ms), Elapsed: " + std::to_string(diff) + "(ms)",
				Shared::Error::Severity::KILL_AND_EXIT
			);
			return false;
		}
	}

	
	this->eHandler->report("Thread ( " + this->name + ":" + stream.str() + "): Created/Started.");

	return this->logger->isListening() ? true : this->logger->flush();
}

void ThreadInterface::stop() {
	// Check if thread is running
	if (!(this->running)) { // || this->initialized)) {
		//this->eHandler->report(this->name + ": Already closed.");
		return;
	}

	try {
		if (!this->cleanAndExit()) {
			// Failed to stop worker
			this->eHandler->report(
				"Thread ( " + this->name + "): Failed to stop worker.",
				Shared::Error::Severity::WARN
			);
		}
	}
	catch (...) {
		this->eHandler->report("Thread: ( " + this->name + ": Failed to execute <cleanAndExit>.");
		// TODO - what next??
	}

	try {
		if (this->thread.joinable()) {
			this->thread.join();
			this->eHandler->report("Thread ( " + this->name + "): Joined/Closed");
		}
	}
	catch (std::system_error &error) {
		this->eHandler->report(
			"Thread ( " + this->name + "): Failed to join thread ( " + this->name + "). \n" + error.what(),
			Shared::Error::Severity::WARN
		);
	}

	if (!this->logger->isListening()) {
		this->logger->flush();
	}
}

void ThreadInterface::wait() {
	try {
		if (this->thread.joinable()) {
			this->thread.join();
			this->eHandler->report("Thread ( " + this->name + "): Joined");
		}
	}
	catch (std::system_error &error) {
		this->eHandler->report(
			"Thread ( " + this->name + "): Failed to join thread ( " + this->name + "). \n" + error.what(),
			Shared::Error::Severity::WARN
		);
	}
}

bool ThreadInterface::isInitialized() const {
	return this->initialized;
}

bool ThreadInterface::isRunning() const {
	return this->running;
}

std::string ThreadInterface::threadID() const {
	auto sstream{ std::stringstream{} };
	sstream << this->thread.get_id();

	return sstream.str();
}
