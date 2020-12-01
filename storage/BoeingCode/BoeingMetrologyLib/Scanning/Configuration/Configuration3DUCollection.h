#ifndef BOEINGMETROLOGYLIB_Configuration3DUCollection_H
#define BOEINGMETROLOGYLIB_Configuration3DUCollection_H

#include <string>

#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include "Scanning/Configuration/LensState.h"
#include "TypeDefs.h"

namespace BoeingMetrology
{
	namespace Scanning
	{
		namespace Configuration
		{
			// Parameters that represent a 3DU collection configuration corresponding
            // to a set of sensors that share an overlapping FOV with a single projector
            class BOEINGMETROLOGYLIB_API Configuration3DUCollection : public Boeing::Interface::Serializer
			{
            public:

                struct CameraSettings
                {
                    float exposureTime;
                    float gain;
                    Scanning::Configuration::LensState lenState;

                    CameraSettings() : exposureTime(0), gain(0) {}
                };

                using CameraSettingsMap = std::map<std::string, CameraSettings>;

            private:                

                // Collection time
                std::string timestamp;

                // The full path to the captured images
                std::string cameraImagePath;

                // Per camera settings
                CameraSettingsMap cameras;

                // Name of the projector
                std::vector<PROJECTOR_NAME> projectorNames;

                // Width, height of projected image in pixels
                cv::Size projectedImageSize;

                // Intensity of projector image pixels
                // 0 < projectorImageIntensity < 1, where 1 maps to an 8-bit intensity of 255
                double projectedImageIntensity;

                // The number of patterns in the time-encoded capture
                int numberOfPatterns = 0;

            public:
                // Default constructor does nothing
                Configuration3DUCollection() {};

                // Constructor generates a timestamp
                Configuration3DUCollection(
                    const std::string & cameraImagePath, 
                    const CameraSettingsMap& cameraSettings, 
                    const std::vector<PROJECTOR_NAME>& projectorNames, 
                    const cv::Size& projectedImageSize,
                    const double projectedImageIntensity, 
                    const int numberOfPatterns);

                // Get the camera details
                const CameraSettingsMap& GetCameraSettings() const;

				// Populate the output Json from this object's data.
				// Throws exception on failure.
                virtual void JsonSerialize(Json::Value &jsonNode) const override;

				// Populate this object from the input stream.
				// Throws exception on failure.
                virtual void JsonDeserialize(const Json::Value &jsonNode) override;
			};
		}
	}
}

#endif
