// PanTiltThread.h - Class for operating the pan-tilt unit in a thread
//

#ifndef METROLOGY2020_PANTILTTHREAD_H
#define METROLOGY2020_PANTILTTHREAD_H

#include <thread>
#include <iostream>

// OpenCV
#include <opencv2/core/core.hpp>

// HRL
#include "Threads/DeviceInterface/DeviceInterface.h"
#include "Controllers/PanTilt/PanTiltController.h"

#include "ConfigParser/InterfaceJSONParser.h"

class PanTiltThread : public DeviceInterface
{
public:
	explicit PanTiltThread(const Interface::PanTilt::PTU & ptuConfig, const std::string &name = "PanTiltController") :
		DeviceInterface(name + " [ " + ptuConfig.id + "]" ),
		device(new PanTiltController(ptuConfig))
	{
		DeviceInterface::sensorID = ptuConfig.id;
	}
	PanTiltThread(PanTiltController * device, const std::string & id, const std::string &name = "PanTiltController") :
		DeviceInterface(std::string{ name + " [ " + id + "]" }),
		device(device)
	{
		DeviceInterface::sensorID = id;
	}
	~PanTiltThread() override {
		// If we haven't already, initiate shutdown of the worker thread
		if (initialized) {
			this->stop();
		}
	}

	// Thread Interface
	bool setup(void * params = nullptr) override;
	void info(Shared::Device::Info * info) override;
	bool listen(void * outRef, const bool sendQuery = false) override;
	bool step(void * input) override;
	void * getDevice() override;
	Presets::Device::Type getType() const override;
	void cliListener(const int & key) override;

	bool step(cv::Vec2d & input);
	bool listen(cv::Vec2d & outRef);

protected:
	void run() override;
	bool cleanAndExit() override;

private:
	PanTiltController * device{};
};

#endif // METROLOGY2020_PANTILTTHREAD_H
