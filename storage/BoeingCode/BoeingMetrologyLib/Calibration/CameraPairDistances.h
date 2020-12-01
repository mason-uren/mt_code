#pragma once
#include <string>
#include "json/value.h"
#include "TypeDefs.h"
#include <map>
#include "Interface/ICameraPairDistances.h"
#include "Common/Interface/Serializer.h"

#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Calibration
	{
        class BOEINGMETROLOGYLIB_API CameraPairDistances : public Boeing::Interface::Serializer, public Interface::ICameraPairDistances
		{
		protected:
			std::map<CAMERA_NAME_PAIR, double> cameraPairDistances;
		public:
			// Retrieve the distances between sensor pairs
			std::map<CAMERA_NAME_PAIR, double> getCameraPairDistance() const override
			{
				return cameraPairDistances;
			}

			void JsonSerialize(Json::Value &jsonNode) const override;
			void JsonDeserialize(const Json::Value &jsonNode) override;
		};
	}
}
