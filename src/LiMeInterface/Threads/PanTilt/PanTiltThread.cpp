#include "PanTiltThread.h"

bool PanTiltThread::setup(void * params) {
	bool status{};
	if ((status = this->device->openTCPConnection())) {
		this->eHandler->report(this->name + ": Created TCP connection.");
	}
	else {
		this->eHandler->report(
			this->name + ": Failed to create/open TCP connection.",
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return status;
	}

	if ((status = this->device->openUDPConnection())) {
        this->eHandler->report(this->name + ": Created UDP connection.");
	}
	else {
        this->eHandler->report(
            this->name + ": Failed to create/open UDP connection.",
            Shared::Error::Severity::KILL_AND_EXIT
        );
        return status;
	}

	// Get/set current position
	auto position{std::vector<double>{Presets::PanTilt::Axis::Precision::BAD_ANGLE, Presets::PanTilt::Axis::Precision::BAD_ANGLE}};
	if (!(status = this->listen(&position, true))) {
		return status;
	}

	// Zero return PTU
	// Note: call run immediately after declaration (lambda func)
	std::thread zeroingThread = std::thread([&]() {
		if (!this->device->zeroReturn(this->getSensorID())) {
			this->eHandler->report(
				"Failed to zero-return PTU.",
				Shared::Error::Severity::KILL_AND_EXIT
			);
		}
	});
	zeroingThread.join();

	// Ensure hard stop (system should not be in motion)
	if (!(status = this->device->hardStop())) {
	    this->eHandler->report(
            "Failed to trigger hard-stop.",
            Shared::Error::Severity::KILL_AND_EXIT
        );
		return status;
	}

	return status;
}

void PanTiltThread::info(Shared::Device::Info * info) {
    strcpy_s(info->buffer[Shared::Device::Info::Setting::NAME], this->name.c_str());
    this->device->panTiltInfo(info);
}

bool PanTiltThread::listen(void * outRef, const bool sendQuery) {
	auto position{ static_cast<std::vector<double> *>(outRef) };
	if (sendQuery) {
		bool status{};
		if ((status = this->device->retrievePositionInDeg())) {
			*position = this->device->getCurrentAngle();
		}
		else {
			this->eHandler->report(
				"Failed to retrieve PAN/TILT current position.",
				Shared::Error::Severity::KILL_AND_EXIT
			);
		}

		return status;
	}
	else {
		*position = this->device->getCurrentAngle();
		return !position->empty();
	}
}

bool PanTiltThread::listen(cv::Vec2d & outRef) {
    bool result{};
    auto currentPT{std::vector<double>{outRef[0], outRef[1]}};
    if ((result = this->listen(&currentPT))) {
        outRef = cv::Vec2d{currentPT[0], currentPT[1]};
    }
    return result;
}

bool PanTiltThread::step(void * input) {
	// Will block for zeroing command
	bool status{};
    auto angles{ static_cast<std::vector<double> *>(input) };
	if (!(status = this->device->setTargetAngle(*angles))) {
		this->eHandler->report(
			this->getName() + ": Unable to set target angle. P/T " + this->device->displayPosition(*angles), 
			Shared::Error::Severity::WARN
		);
	}

    return status;
}

bool PanTiltThread::step(cv::Vec2d & input) {
    auto targetPT{std::vector<double>{input[0], input[1]}};
    return this->step(&targetPT);
}

void * PanTiltThread::getDevice() {
    return this->device;
}

Presets::Device::Type PanTiltThread::getType() const {
	return Presets::Device::Type::PTU;
}

void PanTiltThread::cliListener(const int & key) {
	static constexpr int POS = 1;
	static constexpr int NEG = -1;
	
	static bool endCommand{};
	if (key < 0 && endCommand) {
		endCommand = false;
		this->device->hardStop();
		return;
	}

	endCommand = true;
	
	struct AxisRotation {
		long voltage;
		Interface::PanTilt::Axis axis;
	};

	AxisRotation motion{};

	/**
	 * Developers' Note:
	 * This does not respect the flipped axis of motion that the rest of the system uses.
	 *  
	 *	 |  CLI  | System |
	 * ================
	 * A | left  | right
	 * S | down  | up
	 * D | right | left
	 * F | up	 | down
	 */
	switch ((char) key) {
	case 'a':
	case 'A':
		motion.voltage = POS;
		motion.axis = Interface::PanTilt::Axis::PAN;
		break;
	case 's':
	case 'S':
		motion.voltage = POS;
		motion.axis = Interface::PanTilt::Axis::TILT;
		break;
	case 'w':
	case 'W':
		motion.voltage = NEG;
		motion.axis = Interface::PanTilt::Axis::TILT;
		break;
	case 'd':
	case 'D':
		motion.voltage = NEG;
		motion.axis = Interface::PanTilt::Axis::PAN;
		break;
	default:
		
		break;
	}

	this->device->cliMotion(motion.voltage * Presets::PanTilt::Axis::Voltage::DEFAULT_CLI, motion.axis);
}

//void PanTiltThread::worker() {
void PanTiltThread::run() {
	// Ensure thread has been setup
	this->running = this->initialized;

	while (this->running && this->eHandler->shouldContinue()) {
		// Listen for PT positions in this loop
		bool status{};
		if (!(status = this->device->retrievePositionInDeg())) {
			this->eHandler->report(
				"Failed to retrieve PAN/TILT current position.",
				Shared::Error::Severity::KILL_AND_EXIT
			);
		}
		else {
			// Print position 
			//if (position.size()) {
			//	std::cout << "Current Position: " << position[0] << " " << position[1] << std::endl;
			//}

			// PID Controller
			if (!this->device->hasReachedTarget()) {
				this->device->advance();
			}
		}
		
		this->initialized = status; // block the main thread until we have at least one buffer stored.
	}
		
	Sleep(2000); // Not sure why waiting so long
	this->initialized = false;
}

bool PanTiltThread::cleanAndExit() {
	this->running = false;
	// 'initialized' will be set to false by the worker thread when the 'keepRunning' loop exits.
	// Be sure to set 'initialized' to false after your 'keepRunning' loop exits!!!
	// Block here until the worker thread is ready to terminate.

	auto status{ this->device->emergencyStop() };

	return !(this->initialized = false);
}