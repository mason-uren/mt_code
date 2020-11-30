#ifndef METROLOGY2020_PTUCOMMANDS_H
#define METROLOGY2020_PTUCOMMANDS_H

#include <iostream>
#include <string>

namespace PTU {

	// Provides no motion at designated voltage
	static const constexpr long DEFAULT_VOLTAGE = 16383;

	enum class Operation {
		PAN = 0,
		TILT,
		ZOOM,
		FOCUS,
		TELEVATOR,
		DOLLEY,
		IRIS,
		E_STOP,
		ZERO_POSITION
	};

	enum Direction {
		UP = 1,
		DOWN = -1, 
		LEFT = 1, 
		RIGHT = -1
	};

	// TODO - possibly create struct to house CX parameters
	struct CX {
		long int pan{};
		long int tilt{};
		unsigned int zoom{};
		unsigned int focus{};
		long int trolley{};
		long int aux{};
		unsigned int iris{};
		unsigned int trolleyMaster{};
		unsigned int fade{1};
	};

	/**
	 * @fn inline std::string exec(const CX & command)
	 * @brief Serializae the CX variable. Assumes all values are set
	 */
	inline std::string exec(const PTU::CX & command) {
		std::string buffer{"CX "};

		// Order matters!!
		buffer += std::to_string(command.pan) + " ";
		buffer += std::to_string(command.tilt) + " ";
		buffer += std::to_string(command.zoom) + " ";
		buffer += std::to_string(command.focus) + " ";
		buffer += std::to_string(command.trolley) + " ";
		buffer += std::to_string(command.aux) + " ";
		buffer += std::to_string(command.iris) + " ";
		buffer += std::to_string(command.trolleyMaster) + " ";
		buffer += std::to_string(command.fade);
		buffer += "\r";

		return buffer;
	}
	  
	inline std::string exec(const Operation &command, const long &voltage = DEFAULT_VOLTAGE) {
		std::string buffer{};
		switch (command) {
			case Operation::PAN:
				buffer = "P ";
				break;
			case Operation::TILT:
				buffer = "T ";
				break;
			case Operation::ZOOM:
				buffer = "Z ";
				break;
			case Operation::FOCUS:
				buffer = "F ";
				break;
			case Operation::TELEVATOR:
				buffer = "X ";
				break;
			case Operation::DOLLEY:
				buffer = "Y ";
				break;
			case Operation::IRIS:
				buffer = "I ";
				break;
			case Operation::E_STOP:
				return "R\r";
			case Operation::ZERO_POSITION:
				return "@CNTR 1010\r";
		}
		
		return buffer + std::to_string(voltage);
	}
}

#endif //  METROLOGY2020_PTUCOMMANDS_H