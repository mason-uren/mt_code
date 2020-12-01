#include "PoseGraphAnalyzer.h"

void BoeingMetrology::Calibration::Pose::PoseGraphAnalyzerResults::JsonSerialize(Json::Value &jsonNode) const
{
	try
	{
		jsonNode["comparisonTolerance"] = this->comparisonTolerance;

		// Loop through poses in comparisonDistanceInformation
		for (auto &comp : comparisonDistanceInformation)
		{
            Json::Value pbase;
			const POSE_NAME poseName = comp.first;
            pbase["poseName"] = poseName;

			// Loop through camera pairs for this pose
			for (auto &cam : comp.second)
			{
                Json::Value ppose = pbase;
                ppose["camera1name"] = cam.first.first;
                ppose["camera2name"] = cam.first.second;
                ppose["comparisonDistanceInformation"] = cam.second;
                jsonNode["comparisonDistanceInformation"].append(ppose);
			}
		}

        // Loop through poses in absoluteDistancePerPoseInformation
        for (auto &comp : absoluteDistancePerPoseInformation)
        {
            Json::Value pbase;
            const POSE_NAME poseName = comp.first;
            pbase["poseName"] = poseName;

            // Loop through camera pairs for this pose
            for (auto &cam : comp.second)
            {
                Json::Value ppose = pbase;
                ppose["camera1name"] = cam.first.first;
                ppose["camera2name"] = cam.first.second;
                ppose["absoluteDistancePerPoseInformation"] = absoluteDistancePerPoseInformation.at(poseName).at(cam.first);
                ppose["positionVectorInformaton"].append(positionVectorInformation.at(poseName).at(cam.first).x);
                ppose["positionVectorInformaton"].append(positionVectorInformation.at(poseName).at(cam.first).y);
                ppose["positionVectorInformaton"].append(positionVectorInformation.at(poseName).at(cam.first).z);
                jsonNode["positionVectorInformaton"].append(ppose);
            }
        }

        // Loop through camera pairs with absolute distance measurements
        for (const auto & cameraPair : this->absoluteDistancePerCameraPairInformation)
        {
            const CAMERA_NAME_PAIR & pairName = cameraPair.first;
            Json::Value pbase;
            pbase["camera1name"] = pairName.first;
            pbase["camera2name"] = pairName.second;

            // Loop through poses for this camera pair
            for (const auto & pose : cameraPair.second)
            {
                const POSE_NAME & poseName = pose.first;
                Json::Value ppose = pbase;
                ppose["poseName"] = poseName;
                ppose["absoluteDistancePerCameraPairInformation"] = pose.second;
                jsonNode["absoluteDistancePerCameraPairInformation"].append(ppose);
            }
        }


		// Loop through poses in filterBytolerance
		for (auto &comp : filterBytolerance)
		{
			Json::Value cameraPairs;
			cameraPairs["poseName"] = comp.first;

			// Loop through camera pairs for this pose
			for (auto &cam : comp.second)
			{
				Json::Value p;
				p["camera1name"] = cam.first.first;
				p["camera2name"] = cam.first.second;
				p["filterBytolerance"] = cam.second;
				cameraPairs["cameraPairs"].append(p);
			}
			jsonNode["filterBytolerance"].append(cameraPairs);
		}

		// Loop through poses in filterByPoseTolerance
		for (auto &comp : filterByPoseTolerance)
		{
			Json::Value poseResult;
			poseResult["poseName"] = comp.first;
			poseResult["filterByPoseTolerance"] = comp.second;
			jsonNode["filterByPoseTolerance"].append(poseResult);
		}
	}
	catch (...)
	{
		throw;
	}
}

void BoeingMetrology::Calibration::Pose::PoseGraphAnalyzerResults::JsonDeserialize(const Json::Value &jsonNode)
{
	try
	{
		this->comparisonTolerance = jsonNode["comparisonTolerance"].asDouble();

		// Loop through poses
		for (const auto & pose : jsonNode["comparisonDistanceInformation"])
		{
			POSE_NAME poseName = pose["poseName"].asString();
			std::map<CAMERA_NAME_PAIR, double> comparisonDistanceThisPose;

			// Loop through camera pairs
			for (const auto & p : pose["comparisonDistanceInformation"])
			{
				CAMERA_NAME_PAIR camNamePair = { p["camera1name"].asString(), p["camera2name"].asString() };
				double compDist = p["comparisonDistanceInformation"].asDouble();
				comparisonDistanceThisPose[camNamePair] = compDist;
			}
			comparisonDistanceInformation[poseName] = comparisonDistanceThisPose;
		}

        // Loop through poses
        for (const auto & pose : jsonNode["positionVectorInformaton"])
        {
            POSE_NAME poseName = pose["poseName"].asString();
            std::map<CAMERA_NAME_PAIR, double> absDistanceThisPose;
            std::map<CAMERA_NAME_PAIR, cv::Point3d> positionVectorThisPose;

            // Loop through camera pairs
            for (const auto & p : pose["cameraPairs"])
            {
                CAMERA_NAME_PAIR camNamePair = { p["camera1name"].asString(), p["camera2name"].asString() };
                double absDist = p["absoluteDistancePerPoseInformation"].asDouble();
                absDistanceThisPose[camNamePair] = absDist;
                cv::Point3d positionVec(p["positionVectorInformaton"][0U].asDouble(),
                    p["positionVectorInformaton"][1].asDouble(), p["positionVectorInformaton"][2].asDouble());
                positionVectorThisPose[camNamePair] = positionVec;
            }
            absoluteDistancePerPoseInformation[poseName] = absDistanceThisPose;
            positionVectorInformation[poseName] = positionVectorThisPose;
        }

        // Loop through poses
        for (const auto & pose : jsonNode["absoluteDistancePerCameraPairInformation"])
        {
            POSE_NAME poseName = pose["poseName"].asString();
            CAMERA_NAME_PAIR camNamePair = { pose["camera1name"].asString(), pose["camera2name"].asString() };
            std::pair<POSE_NAME, double> newPair = { poseName, pose["absoluteDistancePerCameraPairInformation"].asDouble() };
            this->absoluteDistancePerCameraPairInformation[camNamePair].insert(newPair);
        }

		// Loop through poses
		for (const auto & pose : jsonNode["filterBytolerance"])
		{
			POSE_NAME poseName = pose["poseName"].asString();
			std::map<CAMERA_NAME_PAIR, bool> filterByToleranceThisPose;
			
			// Loop through camera pairs
			for (const auto & p : pose["cameraPairs"])
			{
				CAMERA_NAME_PAIR camNamePair = { p["camera1name"].asString(), p["camera2name"].asString() };
				filterByToleranceThisPose[camNamePair] = p["filterBytolerance"].asDouble();
			}
			filterBytolerance[poseName] = filterByToleranceThisPose;
		}

		// Loop through poses
		for (const auto & pose : jsonNode["filterByPoseTolerance"])
		{
			POSE_NAME poseName = pose["poseName"].asString();
			filterByPoseTolerance[poseName] = pose["filterByPoseTolerance"].asBool();
		}
	}
	catch (...)
	{
		throw std::runtime_error("PoseGraphAnalyzerResults::JsonDeserialize failed");
	}
}

void BoeingMetrology::Calibration::Pose::PoseGraphAnalyzerResults::SerializePerPose(std::ostream &strm) const
{
	strm << "PoseGraphAnalyzerResults CSV" << std::endl;
	strm << "Pose name,camera1,camera2,camera1_camera2,comparisonDistanceInformation,filterByTolerance,FilterByPoseTolerance,absoluteDistancePerPoseInformation,absoluteDistancePerCameraPairInformation" << std::endl;
	
	// Loop through poses
	for (auto &comp : comparisonDistanceInformation)
	{
		POSE_NAME poseName = comp.first;

		strm << poseName << ",";

		// Loop through camera pairs for this pose
		for (auto &cam : comp.second)
		{
			strm << cam.first.first << ","; 
			strm << cam.first.second << ",";
			strm << cam.first.first + "_" + cam.first.second << ",";
			strm << cam.second << ","; // Distance from comparisonDistanceInformation
			strm << filterBytolerance.at(poseName).at(cam.first)  << ",";
			strm << filterByPoseTolerance.at(poseName) << ",";
            strm << absoluteDistancePerPoseInformation.at(poseName).at(cam.first);
            strm << absoluteDistancePerCameraPairInformation.at({ cam.first.first, cam.first.second }).at(poseName) << std::endl;
		}
	}
}

void BoeingMetrology::Calibration::Pose::PoseGraphAnalyzerResults::SerializePerCameraPair(std::ostream &strm) const
{
    strm << "PoseGraphAnalyzerResults CSV" << std::endl;
    strm << "camera1,camera2,pose name,absoluteDistancePerCameraPairInformation" << std::endl;

    // Loop through camera pairs
    for (const auto &comp : absoluteDistancePerCameraPairInformation)
    {
        const CAMERA_NAME_PAIR & cameraNames = comp.first;

        // Loop through camera pairs
        for (const auto & p : comp.second)
        {
            const POSE_NAME & poseName = p.first;
            strm << cameraNames.first << "," << cameraNames.second << "," << poseName << ",";
            strm << p.second << std::endl;
        }
    }
}

void BoeingMetrology::Calibration::Pose::PoseGraphAnalyzerResults::SerializeSummary(std::ostream &strm) const
{
	try
	{
		if (this->filterByPoseTolerance.size() > 0)
		{
			strm << "PoseGraphAnalyzerResults Summary" << std::endl;

			// Results per pose
			int numPosesFiltered = 0;
			for (auto &comp : this->filterByPoseTolerance)
				if (comp.second)
					numPosesFiltered++;
			strm << numPosesFiltered << " / " << this->filterByPoseTolerance.size() << " poses filtered (tolerance = "
				<< this->comparisonTolerance << ")" << std::endl;
		}
	}
	catch (...)
	{
		throw std::runtime_error("PoseGraphAnalyzerResults::SerializeSummary failed");
	}
}


void BoeingMetrology::Calibration::Pose::PoseGraphAnalyzerResults::SerializeFile(const std::string &fileName) const
{
	try
	{
		Json::Value jsonNode;
		this->JsonSerialize(jsonNode);
		Json::StyledWriter styledWriter;
		std::ofstream writer(fileName, std::ifstream::binary);
		writer << styledWriter.write(jsonNode);
		writer.close();
	}
	catch (...)
	{
		throw std::runtime_error("PoseGraphAnalyzerResults::SerializeFile failed");
	}
}

void BoeingMetrology::Calibration::Pose::PoseGraphAnalyzerResults::DeserializeFile(const std::string &fileName)
{
	try
	{
		Json::Reader jsonReader;
		std::ifstream fReader(fileName, std::ifstream::binary);
		Json::Value root;
		if (jsonReader.parse(fReader, root))
		{
			this->JsonDeserialize(root);
		}
		else
			throw std::runtime_error("PoseGraphAnalyzerResults::DeserializeFile failed");
	}
	catch (...)
	{
		throw std::runtime_error("PoseGraphAnalyzerResults::DeserializeFile failed");
	}
}
