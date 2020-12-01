#ifndef BOEINGMETROLOGYLIB_MULTIPOSEGRAPH_H
#define BOEINGMETROLOGYLIB_MULTIPOSEGRAPH_H

#include "PoseGraphBase.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Calibration
	{
		namespace Pose
		{
            class BOEINGMETROLOGYLIB_API  MultiPoseGraph : public PoseGraphBase
			{
			public:
				MultiPoseGraph()
				{
					_flags = 0;
					_verbose = 1;
					_criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-7);
				}

				// Compute the initial estimate of the 3D position and orientation of each calibration object 
				// at each pose for each camera that saw at least N markers.  Uses the camera intrinsics 
				// provided as input. 
                void initialPoseEstimator(MultiPoseObservations & observations, MultiCameraIntrinsicData & multiCameraIntrinsicData, const double & reprojThreshold,
                    const int & minimumPointsPerView, const std::map<POSE_NAME, bool> * cameraPoseFilter = nullptr, const std::map<POSE_NAME, std::map<CAMERA_NAME, bool>> * cameraObsFilter = nullptr);

                // Add new vertices and edges to the existing pose graph without modifying existing vertices and edges.  MultiPoseObservations passed in should
                // contain the union of all observations from which the current pose graph was constructed as well as the new ones.  Similarly, 
                // MultiCameraIntrinsicData should also contain the union of all cameras.  
                void addMultiPoseObservations(const MultiPoseObservations & observations, MultiCameraIntrinsicData & multiCameraIntrinsicData, const double & reprojThreshold,
                    const int & minimumPointsPerView, const int & flag, const std::map<POSE_NAME, bool> * cameraPoseFilter = nullptr, const std::map<POSE_NAME, std::map<CAMERA_NAME, bool>> * cameraObsFilter = nullptr);

                
			};
		}
	}
}

#endif
