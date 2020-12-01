#pragma once
#include "json/reader.h"
#include "Common/Interface/Serializer.h"

#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Calibration
	{
		namespace Observation
		{
            class BOEINGMETROLOGYLIB_API ObservationAnalyzerResults : public Boeing::Interface::Serializer
			{
			public:
				void JsonSerialize(Json::Value &jsonNode) const override;
				void JsonDeserialize(const Json::Value &jsonValue) override;
			};
		}
	}
}
