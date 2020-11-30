#include "FusedPoses.h"

bool FusedPoses::addInstance(const std::string & instance, Context & context) {
	bool lockCatcher{};

	try {
		this->contextMux.lock();
		lockCatcher = true;

		if (this->contexts.find(instance) == this->contexts.end()) {
			this->contexts.insert({ instance, context });
			this->contexts.at(instance).isFiducialVisible = false;

			// Determine if Imperx camera
			if (context.type == Presets::Device::Type::IMPERX) {
				this->wFramePtr = instance;
			}
		}
		else {
			this->eHandler->report("(WorldFrame) Context instance <" + instance + "> already tracked.", Shared::Error::Severity::DEBUG, context.loggerID);
		}

		this->contextMux.unlock();
		lockCatcher = false;
	}
	catch (const std::bad_alloc & error) {
		this->eHandler->report(error.what(), Shared::Error::Severity::KILL_AND_EXIT, context.loggerID);
		return false;
	}
	catch (...) {
		this->eHandler->report("(WorldFrame) Error adding instance.", Shared::Error::Severity::KILL_AND_EXIT, context.loggerID);
		return false;
	}

	// Ensure key was added
	return this->contexts.find(instance) != this->contexts.end();
}

bool FusedPoses::recordEstimatedPose(const std::string & instance, const SixDOF & sixDOF) {
	bool lockCatcher{};

	try {
		this->contextMux.lock();
		lockCatcher = true;
		if (this->contexts.find(instance) != this->contexts.end()) {
			this->contexts.at(instance).sixDOF = sixDOF;
			this->contexts.at(instance).isFiducialVisible = true;
		}
		else {
			// TODO - how to unset <isFiducialVisible>
			//		- what does sixDOF look like when it isn't set
		}

		this->contextMux.unlock();
		lockCatcher = false;
	}
	catch (const std::out_of_range & error) {
		if (lockCatcher) {
			this->contextMux.unlock();
		}
		this->eHandler->report(error.what(), Shared::Error::Severity::KILL_AND_EXIT);
		return false;
	}
	catch (const std::bad_alloc & error) {
		if (lockCatcher) {
			this->contextMux.unlock();
		}
		this->eHandler->report(error.what(), Shared::Error::Severity::KILL_AND_EXIT);
		return false;
	}

	return true;
}

bool FusedPoses::fetchPose(const std::string & caller, SixDOF & sixDOF) {
	std::lock_guard<std::mutex> contextGuard(this->contextMux);

//	bool found{};
//	std::string ID{};
//	// 1. Check if other tracking instances can see fiducial...
//	for (const auto & _context : this->contexts) {
//		if (_context.first != caller && _context.second.isFiducialVisible) {
//			sixDOF = _context.second.sixDOF;
//			ID = _context.first;
//			found = true;
//			break;
//		}
//	}
//	if (!found) {
//		auto context{ this->contexts.find(this->wFramePtr) };
//		if (context != this->contexts.end() && context->second.isFiducialVisible) {
//			sixDOF = context->second.sixDOF;
//			ID = this->wFramePtr;
//			found = true;
//		}
//	}

    bool found{};

	// 1. Check if ImperX (world_frame) can see fiducial..
	auto context{ this->contexts.find(this->wFramePtr) };
	std::string ID{};
	if (context != this->contexts.end() && context->second.isFiducialVisible) {
		sixDOF = context->second.sixDOF;
		ID = this->wFramePtr;
		found = true;
	}
	// 2. Check if other tracking instances can see fiducial...
	else {
		for (const auto & _context : this->contexts) {
			if (_context.first != caller && _context.second.isFiducialVisible) {
				sixDOF = _context.second.sixDOF;
				ID = _context.first;
				found = true;
				break;
			}
		}

		if (!found) {
			this->eHandler->report("Failed to find a valid pose.", Shared::Error::Severity::WARN, caller);
			return false;
		}
	}

    this->eHandler->report(found ? "FusedPoses::fetchPose(): Found world frame context. ID: " + ID :
                                  "FusedPoses::fetchPose(): No world frame context found.",
                                  Shared::Error::Severity::DEBUG, caller);
	//return true;
	return found;
}

void FusedPoses::updateFiducialState(const std::string & caller, const bool isFound) {
	std::lock_guard<std::mutex> contextGuard(this->contextMux);
	try {
		auto context{ this->contexts.find(caller) };
		if (context != this->contexts.end()) {
			context->second.isFiducialVisible = isFound;
		}
		else {
			this->eHandler->report("Failed to update fiducial state, unable to find ID: " + caller, Shared::Error::Severity::WARN, caller);
		}
	} 
	catch (const std::out_of_range & error) {
		this->eHandler->report("Failed to update fiducial state. ID: " + caller, Shared::Error::Severity::WARN, caller);
		this->eHandler->report(error.what(), Shared::Error::Severity::WARN, caller);
	}
}
