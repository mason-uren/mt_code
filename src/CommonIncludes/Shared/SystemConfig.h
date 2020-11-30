#ifndef METROLOGY2020_SYSTEMCONFIG_H
#define METROLOGY2020_SYSTEMCONFIG_H

#include <string>
#include <vector>

#include "SharedStructs.h"
#include "Presets.h"

namespace System {

	struct Mode {
		std::string run{};
		std::vector<std::string> components{};
	};

	struct Camera {
		std::string id;
		std::string intrinsics;
	};

	struct Pair {		
		Camera camera;
		std::string ptu;
		std::string extrinsics;
	};
}

#endif // METROLOGY2020_SYSTEMCONFIG_H
