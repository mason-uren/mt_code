#ifndef METROLOGY2020_PRESETS_H
#define METROLOGY2020_PRESETS_H

#include <utility>
#include <string>
#include <cfloat>
#include <memory>
#include <vector>
#include <cmath>
#include <iostream>

/**
 * @namespace Presets
 * @brief A place where all interface compile-time presets should live.
 */
namespace Presets {

	namespace System {

		namespace Pairs {
			static const std::string ALPHA{ "alpha" };
			static const std::string BETA{ "beta" };

			enum class Enum {
				ALPHA = 0,
				BETA
			};
		}
	}

	namespace Logger {
		static const std::string DEFAULT_ID{"Logger"};
		static const std::string DEFAULT_LOG_DIR{ "Logs" };
		static const std::string DEFAULT_OUT_FILE{ "report.out" };
	}

	namespace Schema {
		static const std::string RELEASE{ "release" };
		static const std::string DEBUG{ "debug" };
	}

    namespace JSON {
		// Directory 
		static const std::string DEFUALT_DIR{"Config"};

		// Files
        static const std::string INTERFACE{ "InterfaceConfig.json" };
        static const std::string DATA_COLLECTION{"DataCollectionConfig.json" };
        static const std::string DYNAMIC_EXTRINSICS{"DynamicExtrinsicsConfig.json" };
		static const std::string MODELS{"ModelsConfig.json"};
		static const std::string SYSTEM{"SystemConfig.json"};
        static const std::string TRACKING{"TrackingConfig.json" };
    }

	namespace Mode {

		static const std::string INTERFACE{ "interface" };
		static const std::string DATA_COLLECTION{ "data_collection" };
		static const std::string DYNAMIC_EXTRINSICS{ "dynamic_extrinsics" };
		static const std::string TRACKING{ "tracking" };
		static const std::string TEST{ "test" };
		static const std::string EXPERIMENTAL{ "experimental" };

		enum class Enum {
			INTERFACE = 0,
			DATA_COLLECTION,
			DYNAMIC_EXTRINSICS,
			TRACKING,
			TEST,
			EXPERIMENTAL
		};

	}

	namespace Device {
		static const int MAX_DEVICE_CREATE_ATTEMPTS{ 5 };

		enum class Type {
			BAD_DEVICE_TYPE = 0,
			XIMEA,
			IMPERX,
			PTU
		};
	}

	namespace DataCollection {

		namespace Origin {
			static const std::string TOP_LEFT{ "top_left" };
			static const std::string TOP_RIGHT{ "top_right" };
			static const std::string BOT_LEFT{ "bottom_left" };
			static const std::string BOT_RIGHT{ "bottom_right" };

			enum class Enum {
				TOP_LEFT = 0,
				TOP_RIGHT,
				BOT_LEFT,
				BOT_RIGHT
			};

		}

		namespace ScanningPattern {

			static const std::string HORIZONTAL_RASTER{ "horizontal_raster" };
			static const std::string VERTICAL_RASTER{ "vertical_raster" };
			static const std::string RANDOM{ "random" };

			enum class Enum {
				HORIZONTAL_RASTER = 0,
				VERTICAL_RASTER,
				RANDOM
			};
		}
	}

	namespace Tracking {

		namespace Mode{
			static const std::string LIVE{"live"};
			static const std::string VIDEO_STREAM{"video_stream"};
			static const std::string IMAGES{ "images" };

			enum class Enum {
				LIVE = 0,
				VIDEO_STREAM,
				IMAGES
			};
		}

		namespace Context {
			static const std::string WORLD_FRAME{ "world_frame" };
			static const std::string XIMEA_FRAME{"ximea"};
			static const std::string IMPERX_FRAME{"imperx"};
		}

		namespace Marker {
			static constexpr double XI_SCALE{0.2};
			static constexpr double IPX_SCALE{ 0.7 };
		}

		namespace StagingArea {
			static const std::string SHARED{ "shared" };
		}

		namespace DefaultDeviceKeys {
			static const std::string IMPERX_ID{"arbitrary-device-id-imperx"};
			static const std::string XIMEA_ID{ "arbitary-device-id-ximea" };
			static const std::string PTU_ID{ "aribitrary-device-id-ptu" };
		}
	}

	namespace FiducialPosition {

		static const std::string TOP_LEFT{ "top_left" };
		static const std::string TOP_RIGHT{ "top_right" };
		static const std::string BOT_LEFT{ "bottom_left" };
		static const std::string BOT_RIGHT{ "bottom_right" };

		enum class Enum {
			TOP_LEFT = 0,
			TOP_RIGHT,
			BOT_LEFT,
			BOT_RIGHT
		};
	}

	namespace DataSet {

        typedef std::string Key;

        constexpr bool isRowMajor = false;

        namespace StorageOrder {

            static const std::string ROW_MAJ{"row_major"};
            static const std::string COL_MAJ{"column_major"};
        }

        namespace Types {
            static const std::string COMBINED_EXT{"combined_ext"};
            static const std::string UV_CORNERS_XIMEA{"UV_corners_ximea"};
            static const std::string UV_CORNERS_IMPERX{"UV_corners_imperx"};
            static const std::string IMPERX_CHIDS{"imperx_chids"};
            static const std::string IMPERX_EXT{"impx_ext"};
            static const std::string XIMEA_CHIDS{"ximea_chids"};
            static const std::string XIMEA_EXT{"ximea_ext"};

            // Not in H5 file
            static const std::string PAN{"pan"};
            static const std::string TILT{"tilt"};
        }
        namespace PanTiltAngles {
            static const std::string DEG{"pan_tilt"};
            static const std::string RAD{"pan_tilt_rad"};
        }
    }

    namespace Model {

        static const std::string Key{"cad_model"};

        namespace Types {
            static const std::string X_CAMERA{"x_camera"};
            static const std::string Y_CAMERA{"y_camera"};
            static const std::string Z_CAMERA{"z_camera"};
            static const std::string RX_CAMERA{"rx_camera"};
            static const std::string RY_CAMERA{"ry_camera"};
            static const std::string RZ_CAMERA{"rz_camera"};
            static const std::string Z_OFFSET{"z_offset"};
            static const std::string RZ_OFFSET{"rz_offset"};
            static const std::string PAN_SCALE{"pan_scale"};
            static const std::string TILT_SCALE{"tilt_scale"};
            static const std::string X_PTU{"x_ptu"};
            static const std::string Y_PTU{"y_ptu"};
            static const std::string Z_PTU{"z_ptu"};
            static const std::string RX_PTU{"rx_ptu"};
            static const std::string RY_PTU{"ry_ptu"};
            static const std::string RZ_PTU{"rz_ptu"};
        }
    }

	namespace Imaging {

		namespace Image {
			// XIMEA
			static constexpr bool AUTO_WB = false;
			static constexpr int EXPOSURE = 100000; //
			static constexpr int IMG_FORMAT = 0;
			static constexpr int PIXEL_DEPTH = 1;
			static constexpr int DOWNSAMPLING_TYPE = 0;
			static constexpr float APERTURE = 2.8f;
			static constexpr float FOCAL_LENGTH = 200.0f;

			// IMPERX
			static constexpr int SERIAL_NO = 860064;
		}

		namespace Camera {
			static const std::string XIMEA{"ximea"};
			static const std::string IMPERX{"imperx"};

			namespace Ximea {
				static constexpr int IMAGE_WIDTH{7920};
				static constexpr int IMAGE_HEIGHT{6004};
			}
		}

		static constexpr int CIRCULAR_BUF_SIZE = 1;
	}

	namespace PanTilt {

		namespace PID {
			// Encoder count
			static const long int ENCODER_COUNT{ (long int)std::pow(2, 31) };

			static inline long int DEG_TO_ENC(const double & deg, const int & axis) {
				// Note: <axis> is dangerous; only expects 0 or 1 (0 = Pan, 1 = Tilt)
				return static_cast<long int>(deg * (-1) * (axis ? 15600 : 15560)); // 15564 FIXME - don't know exact converstion ratio (PanTilt::PID::ENCODER_COUNT / axisDomain);
			}

			static const std::string INTERNAL{ "internal" };
			static const std::string EXTERNAL{ "external" };

			constexpr double DEFAULT_ACCELERATION_FACTOR{5};
			constexpr int EXPONENTIAL_FACTOR{ 2 };
		}

		namespace Axis {

			namespace Type {
				static const std::string PAN{ "pan" };
				static const std::string TILT{ "tilt" };
			}

			namespace Precision {
				static constexpr double EPSILON = 0.04; // 0.05
				static constexpr double BAD_ANGLE = ((-1) * DBL_MAX);
			}

			namespace Equivalence {
				static constexpr int POLLING_MIN = 5;
			}

			namespace Voltage {
				// PTU tuning
				static constexpr unsigned int DEFAULT_MAX = 17500;
				static constexpr unsigned int DEFAULT_MIN = 500;
				// Manual 
				static constexpr unsigned int DEFAULT_CLI = 5000;
			}

			namespace Domain {
				static constexpr double DEFAULT_DOMAIN = 360;
				static constexpr double DEFAULT_PAN = 360;
				static constexpr double DEFAULT_TILT = 360;
			}
		}
	}
}

#endif // METROLOGY2020_PRESETS_H
