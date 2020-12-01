#ifndef BOEINGMETROLOGYLIB_CaptureSentinel_H
#define BOEINGMETROLOGYLIB_CaptureSentinel_H

#include <string>
#include <cv.h>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Calibration
	{
		class BOEINGMETROLOGYLIB_API CaptureSentinel : public Boeing::Interface::Serializer
		{
		public:

			std::vector<std::string> detectedCameras;

			CaptureSentinel(std::vector<std::string> cameras);

			// Return a deep copy of this object
			CaptureSentinel Clone() const;

			
			// Populate this object from the input stream.
			// Throws exception on failure.
			void JsonDeserialize(const Json::Value &jsonNode) override;

			// Populate the output Json from this object's data members.
			// Throws exception on failure.
			void JsonSerialize(Json::Value &jsonNode) const override;
		};
	}
}

#endif
