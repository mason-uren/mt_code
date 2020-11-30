#include "PanTiltController.h"

// Static function/variables 
long PanTiltController::transformVoltage(const long &voltage) {
	return PTU::DEFAULT_VOLTAGE + voltage;
}

std::string PanTiltController::UUID(PanTiltController * self) {
	static int instances{};

	if (self) {
		self->id = instances;
	}
	return std::to_string(instances++);
}
// END Static

std::string PanTiltController::getName() const {
	return this->name;
}

std::string PanTiltController::getID() const {
	return this->id;
}

bool PanTiltController::angleEquivalence(const std::vector<double> &currentAngles, const std::vector<double> &targetAngles, const bool absolute) {
	if (currentAngles.size() != targetAngles.size()) {
		return false;
	}
	for (auto i = 0; i < currentAngles.size(); i++) {
		if (!PanTiltController::angleEquivalence(currentAngles[i], targetAngles[i], absolute)) {
			return false; 
		}
	}
	return true;
}

bool PanTiltController::angleEquivalence(const double &current, const double &target, const bool absolute) {
	return std::fabs(current - target) <= (absolute ? 0 : this->axes.precision.epsilon);
}

bool PanTiltController::openUDPConnection() {
	bool status{};
	SOCKET socket{};

	// Create socket
	if (!(status = this->network.createSocket(Interface::Network::Protocol::UDP))) {
		return status;
	}

	try {
		socket = this->network.getHandle(Interface::Network::Protocol::UDP);
	}
	// NOTE: unsure if <runtime_error> is the most appropriate to listen (seems to general).
	//		 Consider creating custom exception
	catch (const std::runtime_error & error) {
		this->eHandler->report(error.what(), Shared::Error::Severity::KILL_AND_EXIT);
		return status = false;
	}

	if (!(status = this->network.bindSocketToServer(socket))) {
		return status;
	}

	return this->initialized = status;
}

bool PanTiltController::openTCPConnection() {
	bool status{};
	SOCKET socket{};

	// Create socket
	if (!(status = this->network.createSocket(Interface::Network::Protocol::TCP))) {
		return status;
	}

	try {
		socket = this->network.getHandle(Interface::Network::Protocol::TCP);
	}
	// NOTE: unsure if <runtime_error> is the most appropriate to listen (seems to general).
	//		 Consider creating custom exception
	catch (const std::runtime_error & error) {
		this->eHandler->report(error.what(), Shared::Error::Severity::KILL_AND_EXIT);
		return status = false;
	}

	if (!(status = this->network.connectSocket(socket))) {
		return status;
	}

	return this->initialized = status;
}

bool PanTiltController::retrievePositionInDeg() {
	static char * buffer = new char[BUFFER_LEN];

	bool result{};
	if (!(result = this->network.getDataPackets(&buffer))) {
		return result;
	}

	std::vector<int> encoder(Interface::PanTilt::NUMBER_OF_AXES);
	if (!this->deserialize(encoder, buffer)) {
		return false;
	}

	if (!this->recordAngles(encoder)) {
		return result;
	}
	return this->initialized = result;
}

void PanTiltController::panTiltInfo(Shared::Device::Info * info) {
	this->network.socketInfo(info);
}

bool PanTiltController::emergencyStop() {
	return !(this->initialized = !this->network.sendTCPCommand(PTU::exec(PTU::Operation::E_STOP).c_str()));
}

bool PanTiltController::hardStop() {
	return this->network.sendTCPCommand(PTU::exec(PTU::Operation::E_STOP).c_str());
}

bool PanTiltController::hasReachedTarget() {
	return this->hasAxisReachedTarget(Interface::PanTilt::Axis::PAN) && this->hasAxisReachedTarget(Interface::PanTilt::Axis::TILT);
}

void PanTiltController::advance() {
	bool sendStepCommand{true};
	auto isExternalPID{ this->pidController.type == Interface::PanTilt::PID_TYPE::EXTERNAL };
	auto accelFactors{ std::vector<double>{this->pidController.accelerationFactor.pan, this->pidController.accelerationFactor.tilt} };

	for (auto axis : Interface::PanTilt::AXES) {
	    auto _axis{static_cast<int>(axis)};
		if (this->isBusy[_axis] && this->angleEquivalence(this->currentAngles[_axis], this->targetAngles[_axis])) {
			// Trigger stop motion, if reached target
			if (++this->axisPolls[_axis] == 1) {
				auto result{ this->stopMotion(axis) };
			}
			// Verify that we remain at target pose
			this->isBusy[_axis] = this->axisPolls[_axis] <= Presets::PanTilt::Axis::Equivalence::POLLING_MIN;		
		}
		else if (isExternalPID) {
            /**
             * PID Controller
			 * - Should most likely be calibrated each time PTU is power-cycled
             */
            auto diff{std::fabs(this->targetAngles[_axis] - this->currentAngles[_axis])};
			auto devisor{Presets::PanTilt::Axis::Domain::DEFAULT_DOMAIN / accelFactors.at(_axis)};
			auto frac{ diff > devisor ? 1 : std::pow((diff / devisor), this->pidController.eFactor) };
			auto vel{ std::floor(frac * Presets::PanTilt::Axis::Voltage::DEFAULT_MAX) };

            this->velMux.lock();
			this->velocities[_axis] = vel < Presets::PanTilt::Axis::Voltage::DEFAULT_MIN ? Presets::PanTilt::Axis::Voltage::DEFAULT_MIN : (long)vel;
			this->velMux.unlock();

			// Reset axis polling
			this->axisPolls[_axis] = 0;
		}
		else {
			// Should be using INTERNAL PID
		}
	}

	if (isExternalPID && sendStepCommand) {
		this->serialize(this->targetAngles);
	}
}

bool PanTiltController::zeroReturn(const std::string & sensorID) {
	auto sstream{ std::stringstream{} };
	auto displayZeroReturnStatus = [&](const std::vector<double> position) {
		sstream.str("");
		sstream << "\rZero Return " << this->displayPosition(position);
		std::cout << sstream.str() << std::flush;
	};

	bool status{};
	// NOTE: if INTERNAL pid is selected, setting target will trigger movement
	// NOTE: ignores software softlimits
	this->ignoreSoftLimits = true;
	if (!(status = this->setTargetAngle({ 0, 0 }, true))) {
		return status;
	}

	displayZeroReturnStatus(this->currentAngles);
	this->eHandler->report(sstream.str(), Shared::Error::Severity::DEBUG);

	// FIxME - not a fan of this impl!!! Why do we need to wait...
	const auto QUERY_POSE_ON{100};
	int counter{};
	
	while (!this->hasReachedTarget()) {
		switch (this->pidController.type) {
		case Interface::PanTilt::PID_TYPE::INTERNAL:
			// Not needed since PanTiltController::setTargetAngle() hands control to internal PID
			if (!(++counter % QUERY_POSE_ON)) {
				this->retrievePositionInDeg();
			}
			break;
		case Interface::PanTilt::PID_TYPE::EXTERNAL:
			// Block until machine has reached home position
			this->retrievePositionInDeg();
			break;
		}
		this->advance();

		// Display position...
		displayZeroReturnStatus(this->currentAngles);
	}
	std::cout << std::endl;
	this->ignoreSoftLimits = false;

	this->eHandler->report(sstream.str(), Shared::Error::Severity::DEBUG);

	return status;
}

std::vector<double> PanTiltController::getCurrentAngle() {
	return this->currentAngles;
}

bool PanTiltController::setTargetAngle(std::vector<double> position, const bool shouldSend) {
	// Remove 'BAD_ANGLE' and substitute current
	this->preprocessTarget(position);

	// Don't spam PTU with same target
	if (this->angleEquivalence(position, this->currentAngles)) {
		this->eHandler->report(this->name + ": Target and current angles are equivalent. Targ: " + std::to_string(position[0]) + ", " + std::to_string(position[1]) + ")" + 
							   "Current: (" + std::to_string(this->currentAngles[0]) + ", " + std::to_string(this->currentAngles[1]) + ")",
								Shared::Error::Severity::DEBUG);
		return true;
	}

	// Ensure target is within axis soft limits
	if (!this->ignoreSoftLimits) {
		if (!this->isWithSoftLimits(position)) {
			this->eHandler->report(this->name + ": Proposed target angle not within software soft limits. See InterfaceConfig.json or Presets.h." +
				"\n\tTarget: P/T => " + this->displayPosition(position, true) +
				"\n\tSoft Limits (lowBound/highBound): Pan => " + this->displayPosition(this->axes.domain.pan, true) + " Tilt => " + this->displayPosition(this->axes.domain.tilt, true),
				Shared::Error::Severity::DEBUG
			);
			return false;
		}
	}

	bool status{ !shouldSend };
	if (shouldSend) {
		// Send Command to PTU
		status = this->serialize(position);
	}

	if (status) {
		this->angleMux.lock();
		this->targetAngles = position;
		this->angleMux.unlock();
	}

	// Inform system of new angles
	this->isBusy = { true, true };

	return status;
}

std::vector<double> PanTiltController::getTargetAngle() {
	return this->targetAngles;
}

bool PanTiltController::cliMotion(const long & voltage, const Interface::PanTilt::Axis & axis) {
	auto operation{ axis == Interface::PanTilt::Axis::PAN ? PTU::Operation::PAN : PTU::Operation::TILT };
	std::string command{ PTU::exec(operation, this->transformVoltage(voltage)) + "\r"};
	return this->network.sendTCPCommand(command.c_str());
}

bool PanTiltController::hasAxisReachedTarget(const Interface::PanTilt::Axis &axis) {
	return !this->isBusy[static_cast<int>(axis)];
}

bool PanTiltController::stopMotion(const Interface::PanTilt::Axis &axis) {
	PTU::Operation operation{};
	switch (axis) {
	    case Interface::PanTilt::Axis::PAN:
            operation = PTU::Operation::PAN;
            break;
	    case Interface::PanTilt::Axis::TILT:
            operation = PTU::Operation::TILT;
            break;
        default:
			this->eHandler->report(this->name + ": Failed to stop motion for < " + std::to_string(static_cast<int>(axis)), Shared::Error::Severity::KILL_AND_EXIT);
            return false;
	}
	// Passing no voltages uses DEFAULT_VOLTATGE (equiv. to stop)
	auto command{ PTU::exec(PTU::Operation::E_STOP) };

	return this->network.sendTCPCommand(command.c_str());
}

void PanTiltController::preprocessTarget(std::vector<double> &target) {
    auto PAN{static_cast<int>(Interface::PanTilt::Axis::PAN)};
    auto TILT{static_cast<int>(Interface::PanTilt::Axis::TILT)};

	target[PAN] = this->angleEquivalence(target[PAN], Presets::PanTilt::Axis::Precision::BAD_ANGLE) ? this->currentAngles[PAN] : target[PAN];
	target[TILT] = this->angleEquivalence(target[TILT], Presets::PanTilt::Axis::Precision::BAD_ANGLE) ? this->currentAngles[TILT] : target[TILT];
}

bool PanTiltController::serialize(const std::vector<double> &position) {
	auto command{ std::string{} };
	auto PAN{ static_cast<int>(Interface::PanTilt::Axis::PAN) };
	auto TILT{ static_cast<int>(Interface::PanTilt::Axis::TILT) };

	// Lambda expressions
	auto serializeCX = [&]() {
		auto cx{ PTU::CX{} };

		cx.pan = Presets::PanTilt::PID::DEG_TO_ENC(position[PAN], PAN);
		cx.tilt = Presets::PanTilt::PID::DEG_TO_ENC(position[TILT], TILT);

		// TODO - implement rest (not being listened to by PTU)
		return cx;
	};
	auto serializeDirectionalMotion = [&]() {
		std::vector<int> direction(Interface::PanTilt::NUMBER_OF_AXES);
		this->directionsOfMotion(direction, position);

		// Determine direction of motion
		return std::string{
			PTU::exec(PTU::Operation::PAN, this->transformVoltage(direction[PAN] * this->velocities[PAN])) + " " +
			PTU::exec(PTU::Operation::TILT, this->transformVoltage(direction[TILT] * this->velocities[TILT])) + "\r"
		};
	};

	switch (this->pidController.type) {
		case Interface::PanTilt::PID_TYPE::INTERNAL: {
			command = PTU::exec(serializeCX());
			break;
		}
		case Interface::PanTilt::PID_TYPE::EXTERNAL: {
			command = serializeDirectionalMotion();
			break;
		}
		default:
			// TODO - connect to error handler
			this->eHandler->report(
				this->name + ": Unrecognized pid-controller <" + std::to_string((int)this->pidController.type) + ">",
				Shared::Error::Severity::KILL_AND_EXIT
			);

			break;
	}
	
	return this->network.sendTCPCommand(command.c_str());
}

bool PanTiltController::deserialize(std::vector<int> &encoder, const char * data) {
	try {
        auto PAN{static_cast<int>(Interface::PanTilt::Axis::PAN)};
        auto TILT{static_cast<int>(Interface::PanTilt::Axis::TILT)};

		// Magic numbers lifted from Boeing 
		encoder[PAN] = 0 | ((unsigned char) data[2] << 16) | ((unsigned char) data[3] << 8) | (unsigned char) data[4];
		encoder[TILT] = 0 | ((unsigned char) data[5] << 16) | ((unsigned char) data[6] << 8) | (unsigned char) data[7];
		
		// Two's compliment
		if ((unsigned char) data[2] & 0x80) {
			encoder[PAN] = -(0xFFFFFF - encoder[PAN] + 1);
		}
		if ((unsigned char) data[5] & 0x80) {
			encoder[TILT] = -(0xFFFFFF - encoder[TILT] + 1);
		}

		return true;
	}
	catch (...) {
		this->eHandler->report(
			this->name + ": Error at deserialize(): Unable to parse data packets. Recieved: " + std::to_string(strlen(data)),
			Shared::Error::Severity::KILL_AND_EXIT
		);
		return false;
	}
}

bool PanTiltController::recordAngles(const std::vector<int> &encoder) {
	static const auto degPerEncoderCount{ 0.00045 }; // TODO - assume is correct

	try {
		std::lock_guard<std::mutex> guard(this->angleMux);
		
		for (auto axis : Interface::PanTilt::AXES) {
			this->currentAngles[static_cast<int>(axis)] = encoder[static_cast<int>(axis)] * degPerEncoderCount;
		}
	}
	catch (...) {
		this->eHandler->report(this->name + ": Error at recordAngles(): Unable to log received Pan/Tilt currentAngles.", Shared::Error::Severity::KILL_AND_EXIT);
		return false;
	}
	return true;
}

void PanTiltController::directionsOfMotion(std::vector<int> &directions, const std::vector<double> &target) {
	int sign{}; 
	// TODO - need to fix angles (ie 10 still goes up instead of down
	for (auto type : Interface::PanTilt::AXES) {
		// signbit -> Neg : 1, Pos : 0
		directions[static_cast<int>(type)] =
		        this->angleEquivalence(this->currentAngles[static_cast<int>(type)], target[static_cast<int>(type)]) ?
			        NO_DIRECTION :
	                (this->currentAngles[static_cast<int>(type)] < target[static_cast<int>(type)] ?
	                    1 : -1);
	}
}

bool PanTiltController::isWithSoftLimits(const std::vector<double> & target) {
	/**
	 * NOTE: the axis domains should always be sorted high to low
	 * low_bound <= pan <= high_bound
	 * low_bound <= tilt <= high_bound
	 */
	static constexpr int LOW_BOUND{ 0 };
	static constexpr int HIGH_BOUND{ 1 };
	return this->axes.domain.pan.at(LOW_BOUND) <= target.at((int) Interface::PanTilt::Axis::PAN) &&
		   target.at((int)Interface::PanTilt::Axis::PAN) <= this->axes.domain.pan.at(HIGH_BOUND) &&

		   this->axes.domain.tilt.at(LOW_BOUND) <= target.at((int)Interface::PanTilt::Axis::TILT) &&
		   target.at((int)Interface::PanTilt::Axis::TILT) <= this->axes.domain.tilt.at(HIGH_BOUND);
}


std::string PanTiltController::displayPosition(const std::vector<double> & position, const bool removeCallerStamp) {
	auto sstream{ std::stringstream{} };
	try {
		std::string idStamp{removeCallerStamp ? "" : "[ID: " + this->name + "] => P/T "};
		sstream << idStamp << "( " <<
			position.at((int)Interface::PanTilt::Axis::PAN) << ", " <<
			position.at((int)Interface::PanTilt::Axis::TILT) << ")";
	}
	catch (const std::out_of_range & error) {
		this->eHandler->report(error.what(), Shared::Error::Severity::KILL_AND_EXIT);
	}
	return sstream.str();
}