#include "MultiCameraQualityAssessor.h"
#include "Features/FeatureBasedPointCloud.h"
#include "Utilities/Utilities.h"

namespace BoeingMetrology
{
namespace Calibration
{
	MultiCameraQualityAssessor::MultiCameraQualityAssessor()
	{
		assessedIntrinsics = MultiCameraIntrinsicData();
		assessedExtrinsics = MultiCameraExtrinsicData();
		cameraData = std::map<CAMERA_NAME, QualityAssessor>();
	}//MultiCameraQualityAssessor()

	void MultiCameraQualityAssessor::JsonDeserialize(const Json::Value &jsonNode)
	{
		try
		{
			this->cameraData.clear();

			for (const Json::Value & camera : jsonNode)
			{
				QualityAssessor assessor;
				assessor.JsonDeserialize(camera);
				this->cameraData[assessor.name] = assessor;
			}
		}
		catch (...)
		{
			throw;
		}
	}//JsonDeserialize()

	void MultiCameraQualityAssessor::JsonSerialize(Json::Value &jsonNode) const
	{
		try
		{
			for (const auto &cameraAssessor : this->cameraData)
			{
				std::string cameraName = cameraAssessor.first;

				Json::Value individualCamera;
				cameraAssessor.second.JsonSerialize(individualCamera);
				jsonNode.append(individualCamera);
			}
		}
		catch (...)
		{
			throw;
		}
	}//JsonSerialize()

	// Populate cameraData with calibration metrics for each camera
	void MultiCameraQualityAssessor::populateCalibrationData(MultiCameraExtrinsicData &aExtrinsics, MultiCameraIntrinsicData &aIntrinsics)
	{
		assessedExtrinsics = aExtrinsics;
		assessedIntrinsics = aIntrinsics;
		cameraData = std::map<CAMERA_NAME, QualityAssessor>();

		// Loop through all cameras
		for (auto it = assessedExtrinsics.cameraData.begin(); it != assessedExtrinsics.cameraData.end(); ++it)
		{
			CAMERA_NAME camera = it->first;
			if (assessedExtrinsics.cameraData.count(camera) > 0)
			{
				// Point to intrinsics and extrinsics for the current camera
				IntrinsicData &camIntrinsics = assessedIntrinsics.cameraData[camera];
				ExtrinsicData &camExtrinsics = assessedExtrinsics.cameraData[camera];

				// Populate a cameraAssessor object for this camera
				QualityAssessor cameraAssessor = QualityAssessor(camera, camExtrinsics.poseDataPath);
				cameraAssessor.intrinsicsInSetReprojection = camIntrinsics.rmsError;
				cameraAssessor.intrinsicsPosesKept = camIntrinsics.posesBeforeAndAfterFiltering;
				cameraAssessor.intrinsicTimestamp = camIntrinsics.timestamp;

				cameraAssessor.extrinsicsInSetReprojection = camExtrinsics.rmsError;
				cameraAssessor.extrinsicsInSetAllignmentError = camExtrinsics.allignmentError;
				cameraAssessor.extrinsicTimestamp = camExtrinsics.timestamp;

				// Copy into cameraData
				cameraData[camera] = cameraAssessor;
			}	
		}
	} //MultiCameraQualityAssessor::populateCalibrationData()

	void MultiCameraQualityAssessor::populatePoseData(Pose::PoseGraphAnalyzer &poseAnalyzer)
	{
		std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> offsetsPerPose;
		std::map<CAMERA_NAME_PAIR, std::map<POSE_NAME, double>> cameraDists;
		poseAnalyzer.ComputeCameraOffsets(offsetsPerPose, cameraDists);

		for (auto cameraPair = cameraDists.begin(); cameraPair != cameraDists.end(); ++cameraPair)
		{

			CAMERA_NAME_PAIR distPair = cameraPair->first;
			
			if (cameraData.count(distPair.first) == 0 || cameraData.count(distPair.second) == 0)
			{
				continue;
			}

			std::vector<double> distances = {};
			for (auto poseDistance = cameraPair->second.begin(); poseDistance != cameraPair->second.end(); ++poseDistance)
			{
				distances.push_back(poseDistance->second);
			}
			double avgDist, stdDev;

			CameraCalibrationUtilities::ComputeDistributionStats(distances, avgDist, stdDev);

			cameraData[distPair.first].cameraDistsAvg[distPair.second] = avgDist;
			cameraData[distPair.first].cameraDistsDev[distPair.second] = stdDev;

			cameraData[distPair.second].cameraDistsAvg[distPair.first] = avgDist;
			cameraData[distPair.second].cameraDistsDev[distPair.first] = stdDev;
		}
	}//MultiCameraQualityAssessor::populatePoseData

	void MultiCameraQualityAssessor::populatePoseData(Pose::PoseGraphAnalyzerResults &analyzerResults)
	{
		std::map<CAMERA_NAME_PAIR, std::map<POSE_NAME, double>> &cameraDists = analyzerResults.absoluteDistancePerCameraPairInformation;
		for (auto cameraPair = cameraDists.begin(); cameraPair != cameraDists.end(); ++cameraPair)
		{

			CAMERA_NAME_PAIR distPair = cameraPair->first;

			if (cameraData.count(distPair.first) == 0 || cameraData.count(distPair.second) == 0)
			{
				continue;
			}

			std::vector<double> distances = {};
			for (auto poseDistance = cameraPair->second.begin(); poseDistance != cameraPair->second.end(); ++poseDistance)
			{
				distances.push_back(poseDistance->second);
			}
			double avgDist, stdDev;

			CameraCalibrationUtilities::ComputeDistributionStats(distances, avgDist, stdDev);

			cameraData[distPair.first].cameraDistsAvg[distPair.second] = avgDist;
			cameraData[distPair.first].cameraDistsDev[distPair.second] = stdDev;

			cameraData[distPair.second].cameraDistsAvg[distPair.first] = avgDist;
			cameraData[distPair.second].cameraDistsDev[distPair.first] = stdDev;
		}
	}//MultiCameraQualityAssessor::populatePoseData

	void MultiCameraQualityAssessor::computeQualityData(MultiPoseObservations &outOfSetObservations)
	{
		Features::MultiPosePointCloud reconstructions = Features::MultiPosePointCloud(outOfSetObservations, assessedIntrinsics, assessedExtrinsics);
		std::vector<POSE_NAME> sortedPoses = reconstructions.sortCloudsByParam(Features::ErrorType::P2PL2Error);

		for (Features::FeatureBasedPointCloud &cloud : reconstructions.mClouds)
		{
			std::map<CAMERA_NAME, std::vector<double>> cameral2errors;
			for (CAMERA_NAME camera : cloud.mCameras)
			{
				if (cameral2errors.count(camera) == 0)
				{
					cameral2errors[camera] = {};
				}
				double p2pL2, p2pmax;
				int maxErrorId;
				cloud.getP2PErrors(p2pmax, maxErrorId, p2pL2);
				cameral2errors[camera].push_back(p2pL2);
			}

			for (auto cameraErrorPair : cameral2errors)
			{
				CAMERA_NAME camera = cameraErrorPair.first;
				double avg = 0.0;
				for (double error : cameraErrorPair.second)
				{
					avg += error;
				}
				avg /= cameraErrorPair.second.size();
				if (cameraData.count(camera) != 0)
				{
					cameraData[camera].outSetReconstructionL2Error = avg;
					cameraData[camera].qualityDataPath = outOfSetObservations.folderNameOfObservations;
				}
			}
		}

	}//MultiCameraQualityAssessor::assessQualityData

	void MultiCameraQualityAssessor::validateCalibration(
		bool useIntrinsicsReprojection, double &intrinsicsReprojectionThreshold,
		bool useIntrinsicsPosesKept, int &intrinsicsPosesThreshold,
		bool useExtrinsicsReprojection, double &extrinsicsReprojectionThreshold,
		bool useExtrinsicsAllignment, double &extrinsicsAllignmentThreshold,
		bool useOutSetReconstruction, double &outSetReconstructionThreshold,
		MultiCameraIntrinsicData& validatedIntrinsics, MultiCameraExtrinsicData& validatedExtrinsics)
	{
		const std::string timestamp = Utilities::getCurrentTimeInSeconds();
		//iterate over each cameras quality assessment, and determine whether it passes the provided set of thresholds
		//if a metric is used, but no threshold provided (0 provided as threshold), use the defaults associated with
		//the quality assessment namespace

		std::cout << "Beginning calibration quality assessment using the following metrics..." << std::endl;
		if (useIntrinsicsReprojection)
		{
			if (intrinsicsReprojectionThreshold < 0.001)
			{
				intrinsicsReprojectionThreshold = QualityAssessment::MaxIntrinsicsRMS;
			}
			std::cout << "Intrinsics RMS error < " << intrinsicsReprojectionThreshold << std::endl;
		}
		if (useIntrinsicsPosesKept)
		{
			if (intrinsicsPosesThreshold < 1)
			{
				intrinsicsPosesThreshold = QualityAssessment::MinIntrinsicsPosesKept;
			}
			std::cout << "Intrinsics poses kept > " << intrinsicsPosesThreshold << std::endl;
		}
		if (useExtrinsicsReprojection)
		{
			if (extrinsicsReprojectionThreshold < 0.001)
			{
				extrinsicsReprojectionThreshold = QualityAssessment::MaxExtrinsicsRMS;
			}
			std::cout << "Extrinsics Reprojection error < " << extrinsicsReprojectionThreshold << std::endl;
		}
		if (useExtrinsicsAllignment)
		{
			if (extrinsicsAllignmentThreshold < 0.00001)
			{
				extrinsicsAllignmentThreshold = QualityAssessment::MaxExtrinsicsAllignmentError;
			}
			std::cout << "Extrinsics Allignment error < " << extrinsicsAllignmentThreshold << std::endl;
		}
		if (useOutSetReconstruction)
		{
			if (outSetReconstructionThreshold < 0.00001)
			{
				outSetReconstructionThreshold = QualityAssessment::MaxReconstructionError;
			}
			std::cout << "Test-Set Reconstruction error < " << outSetReconstructionThreshold << std::endl;
		}
		for (auto cameraAssessor = cameraData.begin(); cameraAssessor != cameraData.end(); ++cameraAssessor)
		{
			CAMERA_NAME camera = cameraAssessor->first;
			QualityAssessor &assessor = cameraAssessor->second;

			bool passes = true;
			if (useIntrinsicsReprojection)
			{
				assessor.intrinsicsReprojectionCriterion = intrinsicsReprojectionThreshold;
				if (assessor.intrinsicsInSetReprojection > intrinsicsReprojectionThreshold)
				{
					passes = false;
					std::cout << camera << " fails on intrinsics reprojection metric " << assessor.intrinsicsInSetReprojection << " > " << intrinsicsReprojectionThreshold << std::endl;
				}
				
			}
			if (useIntrinsicsPosesKept)
			{
				assessor.intrinsicsPoseCountCriterion = intrinsicsPosesThreshold;
				if (assessor.intrinsicsPosesKept.second < intrinsicsPosesThreshold)
				{
					passes = false;
					std::cout << camera << " fails on intrinsics poses count metric " << assessor.intrinsicsPosesKept.second << " < " << intrinsicsPosesThreshold << std::endl;
				}
			}
			if (useExtrinsicsReprojection)
			{
				assessor.extrinsicsReprojectionCriterion = extrinsicsReprojectionThreshold;
				if (assessor.extrinsicsInSetReprojection > extrinsicsReprojectionThreshold)
				{
					passes = false;
					std::cout << camera << " fails on extrinsics reprojection metric " << assessor.extrinsicsInSetReprojection << " > " << extrinsicsReprojectionThreshold << std::endl;
				}
			}
			if (useExtrinsicsAllignment)
			{
				assessor.extrinsicsAllignmentCriterion = extrinsicsAllignmentThreshold;
				if (assessor.extrinsicsInSetAllignmentError > extrinsicsAllignmentThreshold)
				{
					passes = false;
					std::cout << camera << " fails on extrinsics allignment metric " << assessor.extrinsicsInSetAllignmentError << " > " << extrinsicsAllignmentThreshold << std::endl;
				}
			}
			if (useOutSetReconstruction)
			{
				assessor.reconstructionCriterion = outSetReconstructionThreshold;
				if (assessor.outSetReconstructionL2Error > outSetReconstructionThreshold)
				{
					passes = false;
					std::cout << camera << " fails on test-set reconstruction metric " << assessor.outSetReconstructionL2Error << " > " << outSetReconstructionThreshold << std::endl;
				}
			}
			assessor.timestamp = timestamp;
			assessor.passes = passes;
			if (passes)
			{
				validatedExtrinsics.cameraData[camera] = assessedExtrinsics.cameraData[camera];
				validatedIntrinsics.cameraData[camera] = assessedIntrinsics.cameraData[camera];
				std::cout << camera << " passes on all metrics" << std::endl;
			}

		}
		
	}//validateCalibration()

}//namespace Calibration
}//namespace BoeingMetrology