#ifndef BOEINGMETROLOGYLIB_MultiLensStateCameraObservations_H
#define BOEINGMETROLOGYLIB_MultiLensStateCameraObservations_H
#pragma warning( disable : 4503 )

#include <string>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include "Scanning/Configuration/LensState.h"
#include "Calibration/IntrinsicData.h"
#include "Calibration/Observation/CameraObservation.h"

namespace BoeingMetrology
{
	namespace Scanning
	{
		namespace Observation
		{
            using namespace Scanning::Configuration;
            using namespace Calibration::Observation;

			// For multiple cameras, contains multiple poses organized by lens state per camera
			class BOEINGMETROLOGYLIB_API MultiLensStateCameraObservations : public Boeing::Interface::Serializer
			{
            private:

                // For each camera, contains observations of a known calibration object across poses organized by lens state.  
                std::map<CAMERA_NAME, std::map<LensState, std::map<POSE_NAME, CameraObservation>>> observations;

                void SerializeStream(std::ostream &strm) const;

                void DeserializeStream(std::istream &strm);

            public:

                std::string poseDir = "";

                // Get the list of all camera names
                std::vector<std::string> GetCameraNames() const;

                // Get list of lens states for a camera
                // Throws exception if this camera does not exist
                std::vector<LensState> GetLensStates(const std::string & cameraName) const;

                // Add an observation
                void AddObservation(const std::string & cameraName, const LensState & lensState, const std::string & poseName, 
                    const CameraObservation & observation);

                // Get all observations for all lens states specific to a camera.
                // Throws an exception if this camera doesn't exist.
                void GetObservations(const std::string & cameraName, std::map<LensState, std::map<POSE_NAME, CameraObservation>> & observations) const;

                // Get all observations specific to a camera and lens state.  Filter the observations based on quality.
                // Throws exception if observations are empty!
                void GetObservations(const std::string & cameraName, const LensState & lensState, const Calibration::IntrinsicData & intrinsicData,
                    std::map<POSE_NAME, CameraObservation> & observations, const int & minNumCorners = 20, const double & maxReprojThresh = 1.0) const;

                // Get all observations specific to a camera and lens state.  Filter the observations based on quality.
                // Throws exception if observations are empty!
                void GetObservations(const std::string & cameraName, const LensState & lensState, const Calibration::IntrinsicData & intrinsicData,
                    std::vector<std::vector<cv::Point2f>> & imagePoints, std::vector<std::vector<cv::Point3f>> & objectPoints, std::vector<POSE_NAME> & poseNames,
                    const int & minNumCorners = 20, const double & maxReprojThresh = 1.0) const;

                // Get multi camera observations for multiple lens states organized by pose name
                void GetObservations(std::map<POSE_NAME, std::map<CAMERA_NAME, std::pair<LensState, CameraObservation>>> & observations) const;

				// Populate the output Json from this object's data.
				// Throws exception on failure.
                virtual void JsonSerialize(Json::Value &jsonNode) const override;

				// Populate this object from the input stream.
				// Throws exception on failure.
                virtual void JsonDeserialize(const Json::Value &jsonNode) override;

				// Write this class's contents to a json-formatted text file
                virtual void SerializeFile(const std::string &fileName) const override;

				// Read a json-formatted text file into this class
                virtual void DeserializeFile(const std::string &fileName) override;
			};
		}
	}
}

#endif
