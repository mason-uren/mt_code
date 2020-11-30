// ImperxThread.h
//
// Derived class for operating an Imperx sensor. Leverages the GenICam interface.
#ifndef METROLOGY2020_IMPERXTHREAD_H
#define METROLOGY2020_IMPERXTHREAD_H

#include <list>
#include <memory.h>

// OpenCV
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// HRL
#include <Shared/Presets.h>

#include "Controllers/Camera/CameraInterface.h"

class ImperxThread : public CameraInterface
{
public:
	explicit ImperxThread(const Interface::Imaging::Camera & cameraConfig) :
		CameraInterface(GenICamAdapterInterface::createAdapter(GenICamAdapterInterface::CameraEnumType::IMPERX), cameraConfig)
	{}
	ImperxThread(GenICamAdapterInterface * camera, const std::string & id = 0, const std::string &name = "imperx") :
        CameraInterface(camera, id, Interface::Imaging::Type::IMPERX, name, std::to_string(Presets::Imaging::Image::SERIAL_NO),
			// FIXME - dummy values
			Interface::Imaging::Image {
				Presets::Imaging::Image::AUTO_WB,
				Presets::Imaging::Image::EXPOSURE,
				Presets::Imaging::Image::IMG_FORMAT,
				Presets::Imaging::Image::PIXEL_DEPTH,
				Presets::Imaging::Image::DOWNSAMPLING_TYPE,
				Presets::Imaging::Image::APERTURE,
				Presets::Imaging::Image::FOCAL_LENGTH
			}
		)
	{}
	~ImperxThread() override = default;

private:
	// Camera Interface
	bool createInstance() override;
	void info(Shared::Device::Info * info) override;

	// Thread Interface
	//void worker() override;
	void run() override;
	bool cleanAndExit() override;
};

#endif // METROLOGY2020_IMPERXTHREAD_H