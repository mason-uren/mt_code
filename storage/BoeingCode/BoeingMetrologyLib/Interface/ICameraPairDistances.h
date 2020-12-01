#pragma once
#include <string>
#include "json/value.h"
#include "TypeDefs.h"
#include <map>

#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Interface
	{
        class BOEINGMETROLOGYLIB_API ICameraPairDistances
		{
		public:
			virtual std::map<CAMERA_NAME_PAIR, double> getCameraPairDistance() const
			{
				throw std::runtime_error("ICameraPairDistances::getCameraPairDistance() is undefined.  Your inheriting class should needs to override.");
			}
		};
	}
}
