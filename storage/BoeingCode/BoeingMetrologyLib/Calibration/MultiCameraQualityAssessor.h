#ifndef BOEINGMETROLOGYLIB_MULTICAMERAQUALITYASSESSOR_H
#define BOEINGMETROLOGYLIB_MULTICAMERAQUALITYASSESSOR_H

#include "TypeDefs.h"
#include <string>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include "TypeDefs.h"
#include "MultiCameraIntrinsicData.h"
#include "MultiCameraExtrinsicData.h"
#include "Pose/PoseGraphAnalyzer.h"

#include "QualityAssessor.h"

namespace BoeingMetrology
{
	namespace Calibration
	{
		// Container for sensor extrinsics in a reference camera's frame
		class BOEINGMETROLOGYLIB_API MultiCameraQualityAssessor : public Boeing::Interface::Serializer
		{
		public:
			MultiCameraExtrinsicData assessedExtrinsics;
			MultiCameraIntrinsicData assessedIntrinsics;

			std::map<CAMERA_NAME, QualityAssessor> cameraData;


			MultiCameraQualityAssessor();

			// Populate this object from the input Json.
			// Throws exception on failure.
			void JsonDeserialize(const Json::Value &node) override;

			// Populate the output Json from this object's data members.
			// Throws exception on failure.
			void JsonSerialize(Json::Value &node) const override;


			//store across the internal measures associated with the intrinsics and extrinsics
			void populateCalibrationData(MultiCameraExtrinsicData &aExtrinsics, MultiCameraIntrinsicData &aIntrinsics);

			//given a pose graph analyzer, compute the camera distances
			void populatePoseData(Pose::PoseGraphAnalyzer &poseAnalyzer);

			void populatePoseData(Pose::PoseGraphAnalyzerResults &analyzerResults);

			//given a set of quality data for out set validation, compute the reconstruction results
			void computeQualityData(MultiPoseObservations &outOfSetObservations);

			//validate each camera's error metrics against the thresholds provided
			void validateCalibration(
				bool useIntrinsicsReprojection, double &intrinsicsReprojectionThreshold,
				bool useIntrinsicsPosesKept, int &intrinsicsPosesThreshold,
				bool useExtrinsicsReprojection, double &extrinsicsReprojectionThreshold,
				bool useExtrinsicsAllignment, double &extrinsicsAllignmentThreshold,
				bool useOutSetReconstruction, double &outSetReconstructionThreshold,
				MultiCameraIntrinsicData& validatedIntrinsics, MultiCameraExtrinsicData& validatedExtrinsics);

		};//class MultiCameraQualityAssessor
	}//namespace Calibration
}//namespace BoeingMetrology

#endif