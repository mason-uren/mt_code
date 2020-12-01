#ifndef BOEINGMETROLOGYLIB_QUALITYASSESSOR_H
#define BOEINGMETROLOGYLIB_QUALITYASSESSOR_H

#include <string>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include "TypeDefs.h"


namespace BoeingMetrology
{
	namespace Calibration
	{
		namespace QualityAssessment
		{
			const double MaxIntrinsicsRMS = 1.0;
			const double MaxExtrinsicsRMS = 1.0;
			const double MaxExtrinsicsAllignmentError = 0.03;
			const double MaxReconstructionError = 0.003;
			const int MinIntrinsicsPosesKept = 50;
		}
		
		// Container for sensor extrinsics in a reference camera's frame
		class BOEINGMETROLOGYLIB_API QualityAssessor : public Boeing::Interface::Serializer
		{
		public:
			
			//name of the associated camera
			CAMERA_NAME name;
			//path to calibration images of board poses
			std::string poseDataPath;
			//path to out-of-set data to assess calibration quality.  SHOULD eventually be deprecated by in-software sampling
			std::string qualityDataPath;
			//timestamp (identifier for queries) of the intrinsics used to compute this quality assessment
			std::string intrinsicTimestamp;
			//timestamp (identifier for queries) of the extrinsics used to compute this quality assessment
			std::string extrinsicTimestamp;
			//timestamp (identifier for queries) of this quality assessment
			std::string timestamp;

			//reprojection error for the camera on the set of board-images it was calibrated against
			double intrinsicsInSetReprojection;
			//the counts of images used for intrinsics calibration (number of input poses, number of filtered poses)
			std::pair<int, int> intrinsicsPosesKept;
			//the reprojection error of the extrinsics on the set of board-images used for extrinsics calibration
			double extrinsicsInSetReprojection;
			//the 3d pose-allignment error of these extrinsics relative to surrounding cameras across shared poses used for calibration
			double extrinsicsInSetAllignmentError;
			//map describing the distance between this camera and other cameras with which it shares poses in the calibration data
			std::map<CAMERA_NAME, double> cameraDistsAvg;
			//same as cameraDistsAvg, but describing the standard deviation of those distances in the pose graph
			std::map<CAMERA_NAME, double> cameraDistsDev;
			//the average of the reconstruction errors of arucoboards drawn from non-calibration data (out of set)
			double outSetReconstructionL2Error;
			//indicator whether the above metrics satisfied the thresholds set by the user when performing quality assessment
			bool passes;
			
			//the thresholds for each of the equivalent parameters listed above.  All but intrinsicsPoseCountCriterion are upper bounds on errors
			double intrinsicsReprojectionCriterion;
			int intrinsicsPoseCountCriterion;
			double extrinsicsReprojectionCriterion;
			double extrinsicsAllignmentCriterion;
			double reconstructionCriterion;
			
			QualityAssessor(std::string cameraName = "", 
				std::string aPoseDataPath = "", std::string aTimestamp = "", std::string aQualityDataPath = "");
			
			// Populate this object from the input Json.
			// Throws exception on failure.
			void JsonDeserialize(const Json::Value &node) override;

			// Populate the output Json from this object's data members.
			// Throws exception on failure.
			void JsonSerialize(Json::Value &node) const override;

            const std::string& GetCameraName() const { return name; }
            const std::string& GetTimestamp() const { return timestamp; }

		};//class CalibrationQualityAssessor
	}//namespace Calibration
}//namespace BoeingMetrology






#endif