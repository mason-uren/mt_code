#include "QualityAssessorHistory.h"

namespace BoeingMetrology
{
namespace Calibration
{
	QualityAssessorHistory::QualityAssessorHistory()
	{
		sortedAssessments = std::map<TIMESTAMP, QualityAssessor>();
		name = "";
	}
	QualityAssessorHistory::QualityAssessorHistory(std::map<TIMESTAMP, QualityAssessor> sourceData)
	{
		sortedAssessments = sourceData;
		name = sourceData.begin()->second.name;
		
	}
	void QualityAssessorHistory::addData(QualityAssessor data)
	{
		sortedAssessments[data.timestamp] = data;
		name = data.name;
	}

	enum csvOrder
	{
		timestamp = 0,
		intrinsicRMS = 1,
		extrinsicRMS = 2,
		preFilterPoses = 3,
		postFilterPoses = 4,
		allignmentError = 5,
		reconstructionError = 6,
		csvSize
	};

	void QualityAssessorHistory::WriteToCSV(std::ostream& csv)
	{

		csv << name << std::endl;

		std::vector<std::stringstream> vss;
		vss.resize((int)csvSize);
		vss[timestamp] << "timestamp";
		vss[intrinsicRMS] << "intrinsicRMS";
		vss[extrinsicRMS] << "extrinsicRMS";
		vss[preFilterPoses] << "preFilterPoses";
		vss[postFilterPoses] << "postFilterPoses";
		vss[allignmentError] << "allignmentError";
		vss[reconstructionError] << "reconstructionError";


		for (auto it = sortedAssessments.begin(); it != sortedAssessments.end(); ++it)
		{
			QualityAssessor &data = it->second;
			vss[timestamp] << ", " << data.timestamp;
			vss[intrinsicRMS] << ", " << data.intrinsicsInSetReprojection;
			vss[extrinsicRMS] << ", " << data.extrinsicsInSetReprojection;
			vss[preFilterPoses] << ", " << data.intrinsicsPosesKept.first;
			vss[postFilterPoses] << ", " << data.intrinsicsPosesKept.second;
			vss[allignmentError] << ", " << data.extrinsicsInSetAllignmentError;
			vss[reconstructionError] << ", " << data.outSetReconstructionL2Error;
		}

		for (int i = 0; i < vss.size(); ++i)
		{
			csv << vss[i].str() << std::endl;
		}
	}

	void QualityAssessorHistory::JsonDeserialize(const Json::Value &jsonNode)
	{
		for (Json::ArrayIndex i = 0; i < jsonNode.size(); ++i)
		{
			QualityAssessor data = QualityAssessor();
			data.JsonDeserialize(jsonNode[i]);
			addData(data);
		}
	}


	void QualityAssessorHistory::JsonSerialize(Json::Value &jsonNode) const
	{
		jsonNode = Json::Value(Json::arrayValue);
		for (auto it = sortedAssessments.begin(); it != sortedAssessments.end(); ++it)
		{
			Json::Value val = Json::Value(Json::nullValue);
			it->second.JsonSerialize(val);
			jsonNode.append(val);
		}
	}
}//namespace Calibration
}//namespace BoeingMetrology