//
// Created by U'Ren, Mason R (VEN) on 3/23/20.
//

#ifndef METROLOGY2020_INTERFACE_CONFIG_H
#define METROLOGY2020_INTERFACE_CONFIG_H

#include <unordered_map>
#include <utility>
#include <vector>
#include <iostream>
#include <sstream>

#include <Shared/SharedStructs.h>

namespace Interface {

    /**
     * Imaging Namespace
     */
    namespace Imaging {

        enum class Type {
            XIMEA = 0,
            IMPERX
        };

		struct Image {
			bool autoWhiteBalance;
			int exposure;
			int imgFormat;
			int pixelDepth;
			int downsamplingType;
			float aperture;
			float focalLength;
		};

		struct Camera {
			Interface::Imaging::Type type;
			std::string id;
			int imageBufferSize;
			std::string serialNo;
			std::string name;
			Image image;
			int pcieSlot;
		};
    }

	/**
	 * Network Namespace
	 */
	namespace Network {

		enum class Protocol {
			UDP = 0,
			TCP
		};

		struct Socket {
			std::string ip{};
			std::string port{};
		};

		struct Pattern {
			Socket client{};
			Socket server{};
		};
	}

    /**
     * PTU Namespace
     */
    namespace PanTilt {

		enum class Axis {
			PAN = 0,
			TILT
		};

		enum class PID_TYPE {
			INTERNAL = 0, // CX
			EXTERNAL
		};

		static constexpr Interface::PanTilt::Axis AXES[] = { Interface::PanTilt::Axis::PAN, Interface::PanTilt::Axis::TILT };
		static constexpr int NUMBER_OF_AXES = sizeof(AXES) / sizeof(*AXES);

		struct Voltages {
			int max;
			int min;
			int cli;
		};

		struct Precision {
		    double epsilon;
		};

		struct Acceleration {
			double pan;
			double tilt;
		};

		struct Domain {
			std::vector<double> pan;
			std::vector<double> tilt;
		};

		struct Axes {
			Domain domain;
			Voltages volt;
			Precision precision;
		};

		struct PID {
			PID_TYPE type;
			double eFactor;
			Acceleration accelerationFactor;
		};

		struct PTU {
			std::string id;
			Interface::Network::Pattern network{};
			Interface::PanTilt::Axes axes{};
			Interface::PanTilt::PID pid{};
		};
    }
}

#endif //METROLOGY2020_INTERFACE_CONFIG_H
