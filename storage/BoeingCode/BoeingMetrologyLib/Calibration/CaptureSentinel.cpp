#include "CaptureSentinel.h"
#include "json/reader.h"
#include <fstream>
#include "Utilities/Utilities.h"
#include <opencv2/imgproc.hpp>



BoeingMetrology::Calibration::CaptureSentinel::CaptureSentinel(std::vector<std::string> cameras)
{
	detectedCameras = cameras;
}

BoeingMetrology::Calibration::CaptureSentinel BoeingMetrology::Calibration::CaptureSentinel::Clone() const
{
	CaptureSentinel dst = CaptureSentinel(this->detectedCameras);
	return dst;
}

void BoeingMetrology::Calibration::CaptureSentinel::JsonDeserialize(const Json::Value &jsonNode)
{

	try
	{
		
		detectedCameras = {};

		for (Json::ArrayIndex i = 0; i < jsonNode["cameras"].size(); ++i)
		{
			detectedCameras.push_back(jsonNode["cameras"][i].asString());
		}
	}
	catch (...)
	{
		throw;
	}

}

void BoeingMetrology::Calibration::CaptureSentinel::JsonSerialize(Json::Value &jsonNode) const
{
	
	Json::Value cameras = Json::Value(Json::arrayValue);
	for (int i = 0; i < detectedCameras.size(); ++i)
	{
		cameras.append(detectedCameras[i]);
	}

	jsonNode["cameras"] = cameras;

}
