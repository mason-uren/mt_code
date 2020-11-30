
#ifndef METROLOGY2020_SHAREDSTRUCTS_H
#define METROLOGY2020_SHAREDSTRUCTS_H


#include <unordered_map>
#include <utility>
#include <vector>
#include <iostream>
#include <sstream>

namespace Shared {

	typedef int Key;

	/**
	 * File Namespace
	 */
	namespace File {

		struct PathHandle {
			std::string directory{};
			std::string file{};
			std::string path{};
		};
	}

	/**
	 * Device Namespace
	 */
	namespace Device {

		enum class Type {
			CAMERA = 0,
			PTU
		};

		struct Info {

			enum Setting {
				NAME = 0,
				MODEL_NO,
				SERIAL_N0,
				IP,
				PORT,

				// Must always be last
				END
			};

			std::unordered_map<int, char[256]> buffer = std::unordered_map<int, char[256]>(Setting::END);

			void display() {
				std::cout << toString() << std::endl;
			}

			std::string toString() {
				std::string config{ "\n*** Device ***" };
				for (auto &info : buffer) {
					std::string param{};
					switch (info.first) {
					case Setting::NAME:
						param += "Name: ";
						break;
					case Setting::MODEL_NO:
						param += "ModelNo.: ";
						break;
					case Setting::SERIAL_N0:
						param += "SerialNo.: ";
						break;
					case Setting::IP:
						param += "IP: ";
						break;
					case Setting::PORT:
						param += "Port: ";
						break;
					default:
						std::cerr << "Unhandled device setting <" << info.first << ">" << std::endl;
						break;
					}
					config += "\n\t" + param + info.second;
				}
				return config;
			}
		};
	}

	/**
	 * Error Namespace
	 */
	namespace Error {

		enum Severity {
			DEBUG = 0,
			INFO,
			WARN,
			KILL_AND_EXIT,

			END // Must always exist at end
		};
	}

	/**
	 * Msg Namespace
	 */
	namespace Log {


		static std::vector<std::string> severities = std::vector<std::string>{
				"Debug",
				"Info",
				"Warning",
				"Kill and Exit"
		};

		struct Msg {
			std::string data{};
			std::string severity{};

			explicit Msg(std::string  msg) : data(std::move(msg)), severity(severities.at(Error::Severity::INFO)) {}
			Msg(std::string severity, std::string msg) : severity(std::move(severity)), data(std::move(msg)) {}
		};
	}
}

#endif // METROLOGY2020_SHAREDSTRUCTS_H