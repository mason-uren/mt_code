#ifndef METROLOGY2020_CAMERAINTERFACE_H
#define METROLOGY2020_CAMERAINTERFACE_H

#include <thread>
#include <iostream>
#include <vector>

// OpenCV
#include <opencv2/core/core.hpp>

// HRL
#include <Shared/InterfaceConfig.h>
#include <Shared/CameraDefines.h>

#include "Threads/DeviceInterface/DeviceInterface.h"
#include "Network/Tools.h"
#include "PhotoAlbum/PhotoAlbum.h"

// GenICam
#include "Adapters/GenICamInterface/GenICamAdapterInterface.h"

static const constexpr int BAD_FRAME = -1;

class CameraInterface : public DeviceInterface
{
public:
	CameraInterface(GenICamAdapterInterface * camera, const Interface::Imaging::Camera & cameraConfig) : 
		DeviceInterface(cameraConfig.name + " [ " + cameraConfig.id + "]" ),
		camera(camera),
		config(cameraConfig),
		imgBuffer(PhotoAlbum(cameraConfig.image))
	{
		DeviceInterface::sensorID = cameraConfig.id;
	}
	CameraInterface(GenICamAdapterInterface * camera,
		            const std::string & id,
					const Interface::Imaging::Type & type,
		            const std::string & name,
					const std::string & serialNo,
					const Interface::Imaging::Image & imgConfig,
					// Only for ximea 
					const int & pcieSlot = 0) :
		DeviceInterface(name + " [ " + id + "]" ),
		camera(camera),
		config(
			Interface::Imaging::Camera {
				type,
				id,
				Presets::Imaging::CIRCULAR_BUF_SIZE,
				serialNo,
				name,
				imgConfig,
				pcieSlot
			}
		),
		imgBuffer(PhotoAlbum(imgConfig))
	{
		DeviceInterface::sensorID = id;
	}
	virtual ~CameraInterface() override {
        // If we haven't already, initiate shutdown of the worker thread
	    if (initialized) {
			this->stop();
	    }
	};

	// Thread Interface
	bool setup(void * params = nullptr) override;

	// Device Interface
	bool listen(void * outRef, const bool sendQuery = false) override;
	bool step(void * input) override;
	void * getDevice() override;
	Presets::Device::Type getType() const override;
	void cliListener(const int & key) override;

	void setCameraSerialNumber(const std::string &serial);
	
protected:
	virtual bool createInstance() = 0;

	void triggerAcquisition(); 
	bool isReady(); // Is camera ready?
	void logGenIMsg();
	void displayLogMessage(GenICamAdapterInterface::LogMessageType &message);
	// TODO
	bool setThenValidateParam(); //

	Interface::Imaging::Camera config{};
	GenICamAdapterInterface * camera{};

	// Parameters
	enum class ParamType {
		INT = 0,
		FLOAT,
		BOOL,
		ENUM,
		STRING
	};
	struct Parameter {
		ParamType type;
		std::string key;
		std::string data;
		bool isValid;
	};
	std::vector <Parameter *> parameters{};
	

	// TODO - slated to be removed
	PhotoAlbum imgBuffer;

private:
    int getCurrentFrame(cv::Mat & img);
};

#endif //METROLOGY2020_CAMERAINTERFACE_H
