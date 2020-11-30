#ifndef METROLOGY2020_DATACOLLECTIONCONFIG_H
#define METROLOGY2020_DATACOLLECTIONCONFIG_H

#include <string>
#include <vector>

#include "SharedStructs.h"
#include "Presets.h"

namespace DataCollection {
	
	struct Domain {
		int slots;
		std::vector<double> range;
	};

	struct ScanPattern {
		std::string pattern;
		std::string origin;
	};

	struct CollectionConfig {
		Domain pan;
		Domain tilt;
	};

	struct H5Handle {
		Shared::File::PathHandle pathHandle{};
	};

	struct Positions {
		std::vector<double> pan{};
		std::vector<double> tilt{};
	};
}

#endif // METROLOGY2020_DATACOLLECTIONCONFIG_H
