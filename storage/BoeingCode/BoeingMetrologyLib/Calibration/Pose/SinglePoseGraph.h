#ifndef BOEINGMETROLOGYLIB_SINGLEPOSEGRAPH_H
#define BOEINGMETROLOGYLIB_SINGLEPOSEGRAPH_H

#include "PoseGraphBase.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
    namespace Calibration
    {
        namespace Pose
        {
            class BOEINGMETROLOGYLIB_API SinglePoseGraph : public PoseGraphBase
            {
            public:
                // an edge connects a camera and pattern
                SinglePoseGraph()
                {
                    _flags = 0;
                    _verbose = 1;
                    _criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-7);
                }

                // Compute the initial estimate of the 3D position and orientation of each calibration object
                // at each pose for each camera that saw at least N markers.  Uses the camera intrinsics
                // provided as input.
                void initialPoseEstimator(MultiCameraObservation & observations, MultiCameraIntrinsicData & multiCameraIntrinsicData, const double & reprojThreshold, const int & minimumPointsPerView);

            };
        }
    }
}

#endif
