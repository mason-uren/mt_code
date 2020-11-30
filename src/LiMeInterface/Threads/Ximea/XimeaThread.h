// XimeaThread.h - Derived class for operating a Ximea sensor. Leverages the Ximea API.
// 
// The class is set up so that multiple sensors may be run simultaneously.

#ifndef METROLOGY2020_XIMEATHREAD_H
#define METROLOGY2020_XIMEATHREAD_H

#include <memory>

// Ximea
//#include <xiApi.h>

// OpenCV
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// HRL
#include "Controllers/Camera/CameraInterface.h"


class XimeaThread : public CameraInterface
{
public:
	explicit XimeaThread(const Interface::Imaging::Camera & cameraConfig) :
		CameraInterface(GenICamAdapterInterface::createAdapter(GenICamAdapterInterface::CameraEnumType::XIMEA), cameraConfig)
	{
		CameraInterface::parameters = std::vector<Parameter *>{
			new Parameter{ParamType::INT, XI_PRM_EXPOSURE, std::to_string(cameraConfig.image.exposure) },
			new Parameter{ParamType::INT, XI_PRM_IMAGE_DATA_FORMAT, std::to_string(cameraConfig.image.imgFormat)},
			new Parameter{ParamType::INT, XI_PRM_AUTO_WB, std::to_string(cameraConfig.image.autoWhiteBalance) },
			new Parameter{ParamType::INT, XI_PRM_DOWNSAMPLING_TYPE, std::to_string(cameraConfig.image.downsamplingType) },
			new Parameter{ParamType::INT, XI_PRM_LENS_MODE, std::to_string(XI_ON) },
			new Parameter{ParamType::FLOAT, XI_PRM_LENS_APERTURE_VALUE, std::to_string(cameraConfig.image.aperture)}
			
			//{ XI_PRM_LENS_FOCUS_DISTANCE,	Parameter{ParamType::INT, false, std::to_string(cameraConfig.image.focalLength) } } // FIXME - suspect this will cause issues
		};
	}
	XimeaThread(GenICamAdapterInterface * camera, const int pcieSlot = 0, const std::string & id = "Default", const std::string &name = "ximea") :
        CameraInterface(camera, id, Interface::Imaging::Type::XIMEA, name, "null",
			Interface::Imaging::Image{
				Presets::Imaging::Image::AUTO_WB,
				Presets::Imaging::Image::EXPOSURE,
				Presets::Imaging::Image::IMG_FORMAT,
				Presets::Imaging::Image::PIXEL_DEPTH,
				Presets::Imaging::Image::DOWNSAMPLING_TYPE,
				Presets::Imaging::Image::APERTURE,
				Presets::Imaging::Image::FOCAL_LENGTH
			},
			pcieSlot
		)
	{
		CameraInterface::parameters = _parameters;
	}
	~XimeaThread() override = default;

private:
	// Camera Interface 
	bool createInstance() override;

	// Thread Interface
    void info(Shared::Device::Info * info) override;
	void run() override;
	bool cleanAndExit() override;

	// Variables
	std::vector<Parameter *> _parameters{
		new Parameter{ ParamType::INT, XI_PRM_EXPOSURE, std::to_string(Presets::Imaging::Image::EXPOSURE) },
		new Parameter{ ParamType::INT, XI_PRM_IMAGE_DATA_FORMAT, std::to_string(Presets::Imaging::Image::IMG_FORMAT) },
		new Parameter{ ParamType::INT, XI_PRM_AUTO_WB, std::to_string(Presets::Imaging::Image::AUTO_WB) },
		new Parameter{ ParamType::INT, XI_PRM_DOWNSAMPLING_TYPE, std::to_string(Presets::Imaging::Image::DOWNSAMPLING_TYPE) },
		new Parameter{ ParamType::INT, XI_PRM_LENS_MODE, std::to_string(XI_ON) },
		new Parameter{ ParamType::FLOAT, XI_PRM_LENS_APERTURE_VALUE, std::to_string(Presets::Imaging::Image::APERTURE) }

		//{ XI_PRM_LENS_FOCUS_DISTANCE,	Parameter{ParamType::INT, false, std::to_string(cameraConfig.image.focalLength) } } // FIXME - suspect this will cause issues
	};
};

#endif // METROLOGY2020_XIMEATHREAD_H