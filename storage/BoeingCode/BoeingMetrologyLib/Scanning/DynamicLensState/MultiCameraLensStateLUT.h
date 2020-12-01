#ifndef BOEINGMETROLOGYLIB_MultiCameraLensStateLUT_H
#define BOEINGMETROLOGYLIB_MultiCameraLensStateLUT_H
#pragma warning( disable : 4503 )
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include "Scanning/Configuration/LensState.h"
#include "TypeDefs.h"

namespace BoeingMetrology
{
	namespace Scanning
	{
        namespace DynamicLensState
        {
            // For a single camera, IntrinsicData objects that are identified by lens state
            class BOEINGMETROLOGYLIB_API MultiCameraLensStateLUT : public Boeing::Interface::Serializer
            {
            private:

                // Lens states indexed by camera name and physical focus distance
                std::map<CAMERA_NAME, std::map<double, Scanning::Configuration::LensState>> lensStates;

            public:

                // Get the lens state count for a camera
                size_t GetLensStateCount(const std::string & cameraName) const;

                // Add a lens state associated with a camera and physical focus distance
                void AddLensState(const std::string & cameraName, const double & focusDistance, const Scanning::Configuration::LensState & lensState);

                // Linearly interpolate (or extrapolate) adjacent lens states to match the provided physical focus distance for a specific
                // zoom and aperture setting.
                // If 0 or 1 lens states exist, throws an exception.
                void GetLensStateLinearlyInterpolate(const std::string & cameraName, const int & zoomValue, const double & apertureValue,
                    const double & focusDistance, std::pair<double, Scanning::Configuration::LensState> & lensState) const;

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