#ifndef BOEINGMETROLOGYLIB_Configuration3DUReconstruction_H
#define BOEINGMETROLOGYLIB_Configuration3DUReconstruction_H

#include <string>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include "Scanning/Configuration/Configuration3DUCollection.h"
#include "Calibration/MultiCameraIntrinsicData.h"
#include "Calibration/MultiCameraExtrinsicData.h"

namespace BoeingMetrology
{
	namespace Scanning
	{
		namespace Configuration
		{
			// Parameters that represent a 3DU reconstruction configuration corresponding
            // to multiple Configuration3DUCollection objects
            class BOEINGMETROLOGYLIB_API Configuration3DUReconstruction : public Boeing::Interface::Serializer
			{
            private:
                // Reconstruction time
                std::string timestamp = "";

                // A list of individual collections to be reconstructed as a group
                std::vector<Configuration3DUCollection> configuration3DUCollections;

                // Intrinsics for all cameras involved in reconstruction
                Calibration::MultiCameraIntrinsicData multiCameraIntrinsicData;

                // Extrinsics for all cameras involved in reconstruction, in a common world frame
                Calibration::MultiCameraExtrinsicData multiCameraExtrinsicData;

                // The path to reconstructed 3D data
                std::string reconstructionPath = "";

                // The 3D reconstruction black threshold
                double reconstructionBlackThreshold;

                // The 3D reconstruction white threshold
                double reconstructionWhiteThreshold;

            public:
                // Constructor is empty
                Configuration3DUReconstruction() {};

                // Constructor generates a timestamp
                Configuration3DUReconstruction(const std::vector<Configuration3DUCollection> & configuration3DUCollections, const Calibration::MultiCameraIntrinsicData & multiCameraIntrinsicData,
                    const Calibration::MultiCameraExtrinsicData & multiCameraExtrinsicData, const std::string & reconstructionPath, const double & reconstructionBlackThreshold, const double & reconstructionWhiteThreshold);

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
