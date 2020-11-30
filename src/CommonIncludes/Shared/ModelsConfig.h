#ifndef METROLOGY2020_MODELS_MODELSCONFIG_H
#define METROLOGY2020_MODELS_MODELSCONFIG_H

#include <Shared/SharedStructs.h>

namespace Model {
	// Charuco board 
	struct Board {
	    Board() = default;
	    Board(const double & squaresX, const double & squaresY,
              const double & squareLength, const double & markerLength) :
                  squaresX(squaresX), squaresY(squaresY),
                  squareLength(squareLength), markerLength(markerLength)
          {}
        double squaresX{};
        double squaresY{};
		double squareLength{};
		double markerLength{};
	};

	// Camera Intrinsics
	namespace Camera {

		struct Intrinsics {
			Shared::File::PathHandle cameraMatrix{};
			Shared::File::PathHandle distortionCoeff{};
		};
	}
}

#endif // METROLOGY2020_MODELS_MODELSCONFIG_H