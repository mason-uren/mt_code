#ifndef BOEINGMETROLOGYLIB_EXTRINSICCALIBRATIONFROMPOINTS_H
#define BOEINGMETROLOGYLIB_EXTRINSICCALIBRATIONFROMPOINTS_H

#include "cv.h"
#include <vector>
#include <string>
#include <map>
#include "json/value.h"
#include "MultiCameraExtrinsicData.h"
#include "Pose/MultiPoseGraph.h"

namespace BoeingMetrology
{
    using namespace Calibration;

    class BOEINGMETROLOGYLIB_API ExtrinsicCalibrationFromPoints
    {
    public:

        // Get a pointer to the pose graph
        ExtrinsicCalibrationFromPoints(BoeingMetrology::Calibration::Pose::MultiPoseGraph & poseGraph);

        /**
        * Optimize the pose graph by updating extrinsics
        *
        * \return reprojectionError
        */
        double Calibrate(MultiCameraExtrinsicData & multiCameraExtrinsicData, const std::vector<std::string> & lockedCameraNames);

    protected:

        // A pointer to a pose graph object
        BoeingMetrology::Calibration::Pose::MultiPoseGraph * _poseGraph = nullptr;
    };
}

#endif
