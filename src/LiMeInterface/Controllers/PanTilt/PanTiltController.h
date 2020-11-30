#ifndef METROLOGY2020_PANTILTCONTROLLER_H
#define METROLOGY2020_PANTILTCONTROLLER_H

#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <algorithm>
#include <limits>
#include <cfloat>
#include <cmath>
#include <thread>

// HRL
#include <Shared/InterfaceConfig.h>
#include <Shared/Presets.h>
#include <Shared/SharedStructs.h>

// Utilities
#include "ErrorHandler/ErrorHandler.h"

#include "Network/Tools.h"
#include "Network/Network.h"
#include "PTUCommands.h"


static constexpr int NO_DIRECTION = 0;

class PanTiltController
{
public:
	PanTiltController() = default;
	explicit PanTiltController(const Interface::PanTilt::PTU & ptuConfig, ErrorHandler * eHandler = ErrorHandler::getInstance()) :
		name("PanTiltController [ " + ptuConfig.id +"]"),
		id(ptuConfig.id),
		initialized(false),
		isBusy(Interface::PanTilt::NUMBER_OF_AXES, false),
		axisPolls(Interface::PanTilt::NUMBER_OF_AXES, 0),
		currentAngles(Interface::PanTilt::NUMBER_OF_AXES),
		targetAngles(Interface::PanTilt::NUMBER_OF_AXES),
		velocities(Interface::PanTilt::NUMBER_OF_AXES),
		network(Network{ ptuConfig.network }),
		axes(ptuConfig.axes),
		pidController(ptuConfig.pid),
		eHandler(eHandler),
		ignoreSoftLimits(false)
	{}
	explicit PanTiltController(const Network & network, 
							   const Interface::PanTilt::PID_TYPE & controller = Interface::PanTilt::PID_TYPE::INTERNAL,
							   ErrorHandler * eHandler = ErrorHandler::getInstance()) :
		name("PanTiltController [ " + UUID(this) + "]"),
		initialized(false),
		isBusy(Interface::PanTilt::NUMBER_OF_AXES, false),
		axisPolls(Interface::PanTilt::NUMBER_OF_AXES, 0),
		currentAngles(Interface::PanTilt::NUMBER_OF_AXES),
		targetAngles(Interface::PanTilt::NUMBER_OF_AXES),
		velocities(Interface::PanTilt::NUMBER_OF_AXES),
		network(network),
		axes(
			Interface::PanTilt::Axes{
				Interface::PanTilt::Domain{ 
					std::vector<double>{-Presets::PanTilt::Axis::Domain::DEFAULT_PAN, Presets::PanTilt::Axis::Domain::DEFAULT_PAN},
					std::vector<double>{-Presets::PanTilt::Axis::Domain::DEFAULT_TILT, Presets::PanTilt::Axis::Domain::DEFAULT_TILT}
				},
				Interface::PanTilt::Voltages{
					Presets::PanTilt::Axis::Voltage::DEFAULT_MAX,
					Presets::PanTilt::Axis::Voltage::DEFAULT_MIN,
					Presets::PanTilt::Axis::Voltage::DEFAULT_CLI,
				},
				Interface::PanTilt::Precision {Presets::PanTilt::Axis::Precision::EPSILON}
			}
		),
		pidController(Interface::PanTilt::PID{ controller, Presets::PanTilt::PID::EXPONENTIAL_FACTOR }),
		eHandler(eHandler),
					ignoreSoftLimits(false)
	{}
	PanTiltController(const Network & network, const Interface::PanTilt::Axes & axes, const Interface::PanTilt::PID_TYPE & controller = Interface::PanTilt::PID_TYPE::INTERNAL) :
		name("PanTiltController [ " + UUID(this) + "]"),
		initialized(false),
		isBusy(Interface::PanTilt::NUMBER_OF_AXES, false),
		axisPolls(Interface::PanTilt::NUMBER_OF_AXES, 0),
		currentAngles(Interface::PanTilt::NUMBER_OF_AXES),
		targetAngles(Interface::PanTilt::NUMBER_OF_AXES),
		velocities(Interface::PanTilt::NUMBER_OF_AXES),
		network(network),
		axes(axes),
		pidController(Interface::PanTilt::PID{controller, Presets::PanTilt::PID::EXPONENTIAL_FACTOR}),
		ignoreSoftLimits(false)
	{}
	~PanTiltController() = default;

	std::string getName() const;
	std::string getID() const;
	void panTiltInfo(Shared::Device::Info * info);

	// Networking 
	bool openUDPConnection();
	bool openTCPConnection();
	bool retrievePositionInDeg();
	// Motion
	bool emergencyStop();
	bool hardStop();
	bool hasReachedTarget();
    void advance();
	bool zeroReturn(const std::string & sensorID);

	std::vector<double> getCurrentAngle();
	bool setTargetAngle(std::vector<double> position, const bool shouldSend = true);
	std::vector<double> getTargetAngle();
	bool cliMotion(const long & voltage, const Interface::PanTilt::Axis & axis);
	std::string displayPosition(const std::vector<double> & position, const bool removeCallerStamp = false);

	// FIXME - was not intended to be exposed
	bool angleEquivalence(const std::vector<double> &currentAngles, const std::vector<double> &targetAngles, const bool absolute = false);

	// Variables
	bool ignoreSoftLimits{};

private:
    static long transformVoltage(const long &voltage);
	static std::string UUID(PanTiltController * self = nullptr);
	
	
    bool angleEquivalence(const double &current, const double &target, const bool absolute = false);
	bool hasAxisReachedTarget(const Interface::PanTilt::Axis &axis);
	bool stopMotion(const Interface::PanTilt::Axis &axis);
	void preprocessTarget(std::vector<double> &target);
	bool serialize(const std::vector<double> &position);
	bool deserialize(std::vector<int> &encoder, const char * data);
	bool recordAngles(const std::vector<int> &encoder);
	void directionsOfMotion(std::vector<int> &directions, const std::vector<double> &target);
	bool isWithSoftLimits(const std::vector<double> & target);


	std::mutex angleMux;
	std::mutex velMux;
	
	bool initialized{};
	bool isNewTarget{};
	char recvBuf[BUFFER_LEN]{};
	std::string name{};
	std::string id{};
	std::vector<bool> isBusy{};
	std::vector<int> axisPolls{};
	std::vector<double> currentAngles{};
	std::vector<double> targetAngles{};
	std::vector<long> velocities{};
	
	Interface::PanTilt::Axes axes{};
	Interface::PanTilt::PID pidController{};

	Network network;

	// static member
	ErrorHandler * eHandler;
};

#endif // METROLOGY2020_PANTILTCONTROLLER_H