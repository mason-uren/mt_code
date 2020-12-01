#include "IntrinsicDataHistory.h"



namespace BoeingMetrology
{
namespace Calibration
{
	IntrinsicDataHistory::IntrinsicDataHistory()
	{
		sortedIntrinsics = std::map<TIMESTAMP, IntrinsicData>();
		name = "";
	}
	IntrinsicDataHistory::IntrinsicDataHistory(std::map<TIMESTAMP, IntrinsicData> sourceData)
	{
		sortedIntrinsics = sourceData;
		name = sourceData.begin()->second.name;
	}
	void IntrinsicDataHistory::addData(IntrinsicData data)
	{
		sortedIntrinsics[data.timestamp] = data;
		name = data.name;
	}
	enum csvOrder
	{
		timestamp = 0,
		rms = 1,
		startingPoses = 2,
		filteredPoses = 3,
		fx = 4,
		fy = 5,
		cx = 6,
		cy = 7,
		k1 = 8,
		k2 = 9,
		k3 = 10,
		k4 = 11,
		k5 = 12,
		sizeX = 13,
		sizeY = 14,
		csvSize
	};
	void IntrinsicDataHistory::WriteToCSV(std::ostream& csv)
	{

		csv << name << std::endl;

		std::vector<std::stringstream> vss;
		vss.resize((int)csvSize);
		vss[timestamp] << "timestamp";
		vss[rms] << "rms";
		vss[startingPoses] << "starting poses";
		vss[filteredPoses] << "filtered poses";
		vss[fx] << "fx";
		vss[fy] << "fy";
		vss[cx] << "cx";
		vss[cy] << "cy";
		vss[k1] << "k1";
		vss[k2] << "k2";
		vss[k3] << "k3";
		vss[k4] << "k4";
		vss[k5] << "k5";
		vss[sizeX] << "pixel size x";
		vss[sizeY] << "pixel size y";


		for (auto it = sortedIntrinsics.begin(); it != sortedIntrinsics.end(); ++it)
		{
			IntrinsicData &data = it->second;
			vss[timestamp] << ", " << data.timestamp;
			vss[rms] << ", " << data.rmsError;
			vss[startingPoses] << ", " << data.posesBeforeAndAfterFiltering.first;
			vss[filteredPoses] << ", " << data.posesBeforeAndAfterFiltering.second;
			vss[fx] << ", " << data.cameraMatrix(0,0);
			vss[fy] << ", " << data.cameraMatrix(1,1);
			vss[cx] << ", " << data.cameraMatrix(0, 2);
			vss[cy] << ", " << data.cameraMatrix(1,2);
			vss[k1] << ", " << data.distortionCoeffs.at<double>(0);
			vss[k2] << ", " << data.distortionCoeffs.at<double>(1);
			vss[k3] << ", " << data.distortionCoeffs.at<double>(2);
			vss[k4] << ", " << data.distortionCoeffs.at<double>(3);
			vss[k5] << ", " << data.distortionCoeffs.at<double>(4);
			vss[sizeX] << ", " << data.pixelSize.first;
			vss[sizeY] << ", " << data.pixelSize.second;
		}

		for (int i = 0; i < vss.size(); ++i)
		{
			csv << vss[i].str() << std::endl;
		}

	}


	void IntrinsicDataHistory::JsonDeserialize(const Json::Value &jsonNode)
	{
		for (Json::ArrayIndex i = 0; i < jsonNode.size(); ++i)
		{
			IntrinsicData data = IntrinsicData();
			data.JsonDeserialize(jsonNode[i]);
			addData(data);
		}
	}


	void IntrinsicDataHistory::JsonSerialize(Json::Value &jsonNode) const
	{
		jsonNode = Json::Value(Json::arrayValue);
		for (auto it = sortedIntrinsics.begin(); it != sortedIntrinsics.end(); ++it)
		{
			Json::Value val = Json::Value(Json::nullValue);
			it->second.JsonSerialize(val);
			jsonNode.append(val);
		}
	}
}//namespace Calibration
}//namespace BoeingMetrology