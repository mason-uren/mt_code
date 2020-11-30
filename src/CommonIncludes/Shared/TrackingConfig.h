#ifndef METROLOGY2020_TRACKINGCONFIG_H
#define METROLOGY2020_TRACKINGCONFIG_H

// HRL
#include "SharedStructs.h"

namespace Tracking {

	namespace Visualization {

		struct Display {
			bool visible{};
			double scale{};
		};

		struct Video {
			bool record{};
			Shared::File::PathHandle video{};
		};
	}
}
#endif // METROLOGY2020_TRACKINGCONFIG_H