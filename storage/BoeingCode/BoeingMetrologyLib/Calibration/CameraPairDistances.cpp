#include "Calibration/CameraPairDistances.h"


void BoeingMetrology::Calibration::CameraPairDistances::JsonSerialize(Json::Value &jsonNode) const
{
	try
	{
		std::map<CAMERA_NAME_PAIR, bool> exists;

		for (const auto &cameraPD : this->cameraPairDistances)
		{
			
			std::string cameraName1 = cameraPD.first.first;
			std::string cameraName2 = cameraPD.first.second;
			double distance = cameraPD.second;
			if (0 == exists.count(std::make_pair(cameraName2, cameraName1)))
			{
				Json::Value pairNode;
				pairNode["cameraName1"] = cameraName1;
				pairNode["cameraName2"] = cameraName2;
				pairNode["distance"] = distance;
				jsonNode.append(pairNode);
				exists[std::make_pair(cameraName1, cameraName2)] = true;
			}
		}
	}
	catch (...)
	{
		throw;
	}
}

void BoeingMetrology::Calibration::CameraPairDistances::JsonDeserialize(const Json::Value &jsonNode)
{
	try
	{
		cameraPairDistances.clear();

		for (const Json::Value & cameraNode : jsonNode)
		{
			std::string cameraName1 = cameraNode["cameraName1"].asString();
			std::string cameraName2 = cameraNode["cameraName2"].asString();
			double distance = cameraNode["distance"].asDouble();

			this->cameraPairDistances[std::make_pair(cameraName1, cameraName2)] = distance;
			this->cameraPairDistances[std::make_pair(cameraName2, cameraName1)] = distance;
		}
	}
	catch (...)
	{
		throw;
	}
}
