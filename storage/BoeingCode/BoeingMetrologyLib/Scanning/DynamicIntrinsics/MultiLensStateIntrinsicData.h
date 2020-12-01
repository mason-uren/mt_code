#ifndef BOEINGMETROLOGYLIB_MULTILENSSTATEINTRINSICDATA_H
#define BOEINGMETROLOGYLIB_MULTILENSSTATEINTRINSICDATA_H
#pragma warning( disable : 4503 )
#include <set>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include "Calibration/IntrinsicData.h"
#include "Scanning/Observation/MultiLensStateCameraObservations.h"

namespace BoeingMetrology
{
	namespace Scanning
	{
        namespace DynamicIntrinsics
        {
            // For a single camera, IntrinsicData objects that are identified by lens state
            class BOEINGMETROLOGYLIB_API MultiLensStateIntrinsicData : public Boeing::Interface::Serializer
            {
            private:
                // Camera name
                std::string name = "";

                // Intrinsics indexed by lens state.  Iteration by increasing focus value.
                std::map<Scanning::Configuration::LensState, Calibration::IntrinsicData> intrinsics;

            public:

                // Constructor
                MultiLensStateIntrinsicData() {};

                // Constructor 
                MultiLensStateIntrinsicData(const std::string & name);

                // Compute intrinsics for multiple lens states
                // Multi-threaded
                MultiLensStateIntrinsicData(const Scanning::Observation::MultiLensStateCameraObservations & multiLensStateCameraObservations, 
                    const Calibration::IntrinsicData & initialIntrinsicData, const int & flags, const int & minNumCorners, 
                    const double & maxReprojThresh, std::set<CAMERA_NAME> & failedCameras);

                // Get camera name
                std::string GetCameraName() const;

                // Get timestamp common to all lens states
                std::string GetTimestamp() const;

                // Add a new IntrinsicData and associated lens state.  If this lens state already exists, the associated
                // intrinsic data will be updated.
                // Throws exception if camera name does not match
                void AddLensState(const Scanning::Configuration::LensState & lensState, const Calibration::IntrinsicData & intrinsic);

                // Get intrinsics associated with an exact lens state.  Exception thrown if none is found.
                Calibration::IntrinsicData GetIntrinsicDataExact(const Scanning::Configuration::LensState & lensState) const;

                // Get intrinsics associated with the closest match to the provided lens state.  A nearest neighbor must share the same zoom and aperture
                // state.  Exception thrown if none is found.
                int GetIntrinsicDataNearestNeighbor(const Scanning::Configuration::LensState & lensState, 
                    std::pair<Scanning::Configuration::LensState, Calibration::IntrinsicData> & nearestNeighbor) const;

                // Get intrinsics by linearly interpolating based on focus value.  Exception thrown if no neighbors are found that share 
                // the same zoom and aperture value and straddle the requested focus value. 
                void GetIntrinsicDataLinearlyInterpolate(const Scanning::Configuration::LensState & lensState,
                    std::pair<Scanning::Configuration::LensState, Calibration::IntrinsicData> & interpolatedState) const;

                // Populate this object from the input json.
                // Throws exception on failure.
                virtual void JsonDeserialize(const Json::Value &jsonNode) override;

                // Populate the Json node from this object's data members.
                // Throws exception on failure.
                virtual void JsonSerialize(Json::Value &jsonNode) const override;
            };
        }
	}
}

#endif