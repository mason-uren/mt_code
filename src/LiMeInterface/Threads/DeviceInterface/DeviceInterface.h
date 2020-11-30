#ifndef METROLOGY2020_DEVICEINTERFACE_H
#define METROLOGY2020_DEVICEINTERFACE_H

#include <thread>
#include <iostream>
#include <mutex>
#include <utility>
#include <ctime>
#include <sstream>
#include <unordered_map>

// HRL
#include <Shared/InterfaceConfig.h>
#include <Shared/SystemConfig.h>
#include <Shared/OSDefines.h>
#include <Shared/ModelsConfig.h>

// Utilities
#include "ThreadInterface/ThreadInterface.h"

// Models
#include "Camera/CameraModel.h"

static constexpr int TIMEOUT{100};

class DeviceInterface: public ThreadInterface
{
public:
	explicit DeviceInterface(std::string  name, const bool detach = false) :
		ThreadInterface(name, detach)
	{}
	virtual ~DeviceInterface() = default;

	// Pure Abstract Member Functions
	virtual void info(Shared::Device::Info * info) = 0;
	virtual bool listen(void * outRef, const bool sendQuery = false) = 0;
	virtual bool step(void * input) = 0;
	// FIXME - return child pointers not parents
	virtual void * getDevice() = 0;
	virtual Presets::Device::Type getType() const = 0;
	virtual void cliListener(const int & key) = 0;

	// Member Functions
	void deviceInfo(bool consoleDisplay = false);
	std::string getSensorID() const;

	// Static Functions
	static void sourceEnvironment(const std::unordered_map<std::string, System::Pair> & targetPairs, const System::Camera & wFrame);
	static void sourceCameraIntrinsics(const std::unordered_map<std::string, Model::Camera::Intrinsics> & ximea, 
									   const std::unordered_map<std::string, Model::Camera::Intrinsics> & imperx);
	static bool addComponent(const std::string & id, DeviceInterface * comp);
	static std::vector<DeviceInterface *> & getActiveCameras(const bool reset = false);
	static std::vector<DeviceInterface *> & getActivePTUs(const bool reset = false);
	static bool initialize();
	static bool safeExit();

	// Static members
	static std::unordered_map<std::string, DeviceInterface *> components;
	static std::unordered_map<std::string, System::Pair> pairs;
	static System::Camera wFrame;
//	static std::unordered_map<std::string, Model::Camera::Intrinsics> ximea;
//	static std::unordered_map<std::string, Model::Camera::Intrinsics> imperx;
    static std::unordered_map<std::string, CameraModel<double>> ximea;
    static std::unordered_map<std::string, CameraModel<double>> imperx;

protected:
	std::string sensorID{};

private:
	static std::vector<DeviceInterface *> cameras;
	static std::vector<DeviceInterface *> ptus;
};

#endif // METROLOGY2020_DEVICEINTERFACE_H

