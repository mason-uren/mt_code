#ifndef BOEINGMETROLOGYLIB_MESHGENERATOROBSERVATIONS_H
#define BOEINGMETROLOGYLIB_MESHGENERATOROBSERVATIONS_H

#include <string>
#include <cv.h>
#include <map>
#include <set>
#include "json/value.h"
#include <fstream>
#include "Calibration/CalibrationObject/ArucoBoard.h"
#include "Calibration/Observation/MultiCameraObservation.h"
#include "Scanning/Observation/MultiLensStateCameraObservations.h"
#include "../Common/Interface/Serializer.h"
#include "Calibration/IntrinsicData.h"
#include <map>
#include "Calibration/Observation/MultiPoseObservations.h"


namespace BoeingMetrology
{
	namespace Calibration
	{
		using namespace CalibrationObject;
		namespace Observation
		{
			// Contains MultiCameraObservations spanning multiple poses
			class BOEINGMETROLOGYLIB_API MeshGeneratorObservations : public MultiPoseObservations
			{
			public:
				// Load data from a folder structure.  Missing detection results will be ignored.
				void ReadObservationData(
					const std::string &observationFolder, 
					const std::string &illuminatedFolder,
					const std::string &darkFolder,
					Scanning::Observation::MultiLensStateCameraObservations & multiLensStateCameraObservations,
					std::set<CAMERA_NAME> & failedCameras,
					AbstractObjectBase *detector = NULL);

				std::string illuminatedDir;
				std::string darkDir;

			};
		}
	}
}
#endif
