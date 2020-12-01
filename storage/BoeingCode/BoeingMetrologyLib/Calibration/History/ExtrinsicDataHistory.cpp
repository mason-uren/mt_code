#include "ExtrinsicDataHistory.h"



namespace BoeingMetrology
{
namespace Calibration
{
	ExtrinsicDataHistory::ExtrinsicDataHistory()
	{
		sortedExtrinsics = std::map<TIMESTAMP, ExtrinsicData>();
		name = "";
	}
	ExtrinsicDataHistory::ExtrinsicDataHistory(std::map<TIMESTAMP, ExtrinsicData> sourceData)
	{
		sortedExtrinsics = sourceData;
		name = sourceData.begin()->second.name;
	}
	void ExtrinsicDataHistory::addData(ExtrinsicData data)
	{
		sortedExtrinsics[data.timestamp] = data;
		name = data.name;
	}

	enum csvOrder
	{
		timestamp = 0,
		x = 1,
		y = 2,
		z = 3,
		rxx = 4,
		rxy = 5,
		rxz = 6,
		ryx = 7,
		ryy = 8,
		ryz = 9,
		rzx = 10,
		rzy = 11,
		rzz = 12,
		rms = 13,
		allignmentError = 14,
		csvSize
	};

	void ExtrinsicDataHistory::WriteToCSV(std::ostream& csv)
	{
		csv << name << std::endl;

		std::vector<std::stringstream> vss;
		vss.resize((int)csvSize);
		vss[timestamp] << "timestamp";
		vss[rms] << "rms";
		vss[x] << "x";
		vss[y] << "y";
		vss[z] << "z";
		vss[rxx] << "rx";
		vss[rxy] << "rxy";
		vss[rxz] << "rxz";
		vss[ryx] << "ryx";
		vss[ryy] << "ryy";
		vss[ryz] << "ryz";
		vss[rzx] << "rzx";
		vss[rzy] << "rzy";
		vss[rzz] << "rzz";
		vss[allignmentError] << "allignment error";

		for (auto it = sortedExtrinsics.begin(); it != sortedExtrinsics.end(); ++it)
		{
			ExtrinsicData &data = it->second;
			cv::Matx34d trans = data.transform;
			vss[timestamp] << ", " << data.timestamp;
			vss[rms] << ", " << data.rmsError;
			vss[x] << ", " << trans(0, 3);
			vss[y] << ", " << trans(1,3);
			vss[z] << ", " << trans(2,3);
			vss[rxx] << ", " << trans(0,0);
			vss[rxy] << ", " << trans(0, 1);
			vss[rxz] << ", " << trans(0, 2);
			vss[ryx] << ", " << trans(1,0);
			vss[ryy] << ", " << trans(1,1);
			vss[ryz] << ", " << trans(1,2);
			vss[rzx] << ", " << trans(2, 0);
			vss[rzy] << ", " << trans(2,1);
			vss[rzz] << ", " << trans(2,2);
			vss[allignmentError] << ", " << data.allignmentError;
		}
		for (int i = 0; i < vss.size(); ++i)
		{
			csv << vss[i].str() << std::endl;
		}
	}


	void ExtrinsicDataHistory::JsonDeserialize(const Json::Value &jsonNode)
	{
		for (Json::ArrayIndex i = 0; i < jsonNode.size(); ++i)
		{
			ExtrinsicData data = ExtrinsicData();
			data.JsonDeserialize(jsonNode[i]);
			addData(data);
		}
	}


	void ExtrinsicDataHistory::JsonSerialize(Json::Value &jsonNode) const
	{
		jsonNode = Json::Value(Json::arrayValue);
		for (auto it = sortedExtrinsics.begin(); it != sortedExtrinsics.end(); ++it)
		{
			Json::Value val = Json::Value(Json::nullValue);
			it->second.JsonSerialize(val);
			jsonNode.append(val);
		}
	}


}//namespace Calibration
}//namespace BoeingMetrology