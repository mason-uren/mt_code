//
// Created by U'Ren, Mason R (VEN) on 4/23/20.
//

#ifndef METROLOGY2020_ALGORITHMS_DYNAMICEXTRINSICCONFIG_H
#define METROLOGY2020_ALGORITHMS_DYNAMICEXTRINSICCONFIG_H

#include <memory>
#include <vector>
#include <unordered_map>

#include "Presets.h"
#include "SharedStructs.h"

// Dynamic Extrinsic
namespace DE {

	struct FiducialPositions {
		std::unordered_map<std::string, std::vector<Shared::File::PathHandle>> map{};
	};

	struct Dataset {
		bool isRowMajor{};
        std::vector<double> panRanges{};
		std::vector<double> tiltRanges{};
	};

    struct Model {
        bool saveInPlace{};
        bool separateFiles{};
		Shared::File::PathHandle input{};
		Shared::File::PathHandle output{};
    };

    namespace Optimization {

        struct LFBGS {
            double epsilon{};
            double delta{};
            double maxIterations{};
            double maxSubMin{};
            double maxLineSearch{};
            double minStep{};
            double maxStep{};
        };
    }

    struct Examples {
        bool runStaticModelFitting{};
        bool runDynamicModelFitting{};
    };

	// FIXME - not sure if I belong here??

    namespace Set {

        namespace Buffer {

            enum Type {
                VECTOR = 0,
                MATRIX_2d,
				MATRIX_3d
            };
        }
    }
}

#endif //METROLOGY2020_ALGORITHMS_DYNAMICEXTRINSICCONFIG_H
