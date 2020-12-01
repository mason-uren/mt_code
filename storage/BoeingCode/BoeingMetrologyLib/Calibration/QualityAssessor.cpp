#include "QualityAssessor.h"



namespace BoeingMetrology
{
namespace Calibration
{
	
	QualityAssessor::QualityAssessor(std::string cameraName, 
		std::string aPoseDataPath, std::string aTimestamp, std::string aQualityDataPath) :
		name(cameraName), qualityDataPath(aQualityDataPath), poseDataPath(aPoseDataPath), timestamp(aTimestamp)
	{
		
		intrinsicsInSetReprojection = 0.0;
		intrinsicsPosesKept = std::make_pair(0, 0);
		extrinsicsInSetReprojection = 0.0;
		extrinsicsInSetAllignmentError = 0.0;
		cameraDistsAvg = std::map<CAMERA_NAME, double>();
		cameraDistsDev = std::map<CAMERA_NAME, double>();
		outSetReconstructionL2Error = 0.0;
		intrinsicTimestamp = "";
		extrinsicTimestamp = "";

		intrinsicsReprojectionCriterion = 0;
		intrinsicsPoseCountCriterion = 0;
		extrinsicsReprojectionCriterion = 0;
		extrinsicsAllignmentCriterion = 0;
		reconstructionCriterion = 0;
	}

	void QualityAssessor::JsonDeserialize(const Json::Value &node)
	{
		try
		{
			name = node["name"].asString();
			poseDataPath = node["poseDataPath"].asString();
			qualityDataPath = node["qualityDataPath"].asString();
			intrinsicTimestamp = node["intrinsicTimestamp"].asString();
			extrinsicTimestamp = node["extrinsicTimestamp"].asString();
			timestamp = node["timestamp"].asString();

			intrinsicsInSetReprojection = node["intrinsicsInSetReprojection"].asDouble();
			intrinsicsPosesKept = { node["intrinsicsPosesKept"][0U].asInt(), node["intrinsicsPosesKept"][1].asInt() };
			
			extrinsicsInSetReprojection = node["extrinsicsInSetReprojection"].asDouble();
			extrinsicsInSetAllignmentError = node["extrinsicsInSetAllignmentError"].asDouble();

			Json::Value avgDistsArr = node["cameraDistsAvg"];
			for (Json::ArrayIndex i = 0; i < avgDistsArr.size(); i+=2)
			{
				cameraDistsAvg[avgDistsArr[i].asString()] = avgDistsArr[i + 1].asDouble();
			}

			Json::Value devDistsArr = node["cameraDistsDev"];
			for (Json::ArrayIndex i = 0; i < devDistsArr.size(); ++i)
			{
				cameraDistsDev[devDistsArr[i].asString()] = devDistsArr[i + 1].asDouble();
			}
			
			outSetReconstructionL2Error = node["outSetReconstructionL2Error"].asDouble();

			intrinsicsReprojectionCriterion = node["intrinsicsReprojectionCriterion"].asDouble();
			intrinsicsPoseCountCriterion = node["intrinsicsPoseCountCriterion"].asInt();
			extrinsicsReprojectionCriterion = node["extrinsicsReprojectionCriterion"].asDouble();
			extrinsicsAllignmentCriterion = node["extrinsicsAllignmentCriterion"].asDouble();
			reconstructionCriterion = node["reconstructionCriterion"].asDouble();

			passes = node["passes"].asBool();
		}
		catch (...)
		{
			throw;
		}
	}

	void QualityAssessor::JsonSerialize(Json::Value &node) const 
	{
		node["name"] = name;
		node["poseDataPath"] = poseDataPath;
		node["qualityDataPath"] = qualityDataPath;
		node["intrinsicTimestamp"] = intrinsicTimestamp;
		node["extrinsicTimestamp"] = extrinsicTimestamp;
		node["timestamp"] = timestamp;

		node["intrinsicsInSetReprojection"] = intrinsicsInSetReprojection;
		node["intrinsicsPosesKept"].append(intrinsicsPosesKept.first);
		node["intrinsicsPosesKept"].append(intrinsicsPosesKept.second);
		
		node["extrinsicsInSetReprojection"] = extrinsicsInSetReprojection;
		node["extrinsicsInSetAllignmentError"] = extrinsicsInSetAllignmentError;
		
		for (auto it = cameraDistsAvg.begin(); it != cameraDistsAvg.end(); ++it)
		{
			node["cameraDistsAvg"].append(it->first);
			node["cameraDistsAvg"].append(it->second);
		}

		for (auto it = cameraDistsDev.begin(); it != cameraDistsDev.end(); ++it)
		{
			node["cameraDistsDev"].append(it->first);
			node["cameraDistsDev"].append(it->second);
		}

		node["outSetReconstructionL2Error"] = outSetReconstructionL2Error;

		node["intrinsicsReprojectionCriterion"] = intrinsicsReprojectionCriterion;
		node["intrinsicsPoseCountCriterion"] = intrinsicsPoseCountCriterion;
		node["extrinsicsReprojectionCriterion"] = extrinsicsReprojectionCriterion;
		node["extrinsicsAllignmentCriterion"] = extrinsicsAllignmentCriterion;
		node["reconstructionCriterion"] = reconstructionCriterion;

		node["passes"] = passes;
	}

}//namespace Calibration
}//namespace BoeingMetrology