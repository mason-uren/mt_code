#ifndef BOEINGMETROLOGYLIB_ProjectorCalibration_H
#define BOEINGMETROLOGYLIB_ProjectorCalibration_H

#include <string>
#include <cv.h>
#include "BoeingMetrologyLib_API.h"
#include "Calibration/Observation/CameraObservation.h"

namespace BoeingMetrology
{
	namespace Calibration
	{
        // A static class with methods for using structured light and traditional camera calibration methods to 
        // derive calibration board observations in the projector's coordinate system that can be used to 
        // compute intrinsics and extrinsics for a projector-camera system.  
        // Techniques are adapted from http://mesh.brown.edu/calibration/.
        class BOEINGMETROLOGYLIB_API ProjectorCalibration
		{
        private:
            static inline bool INVALID(float value) { return _isnan(value) > 0; }
            static inline bool INVALID(const cv::Vec2f & pt) { return (_isnan(pt[0]) > 0) || (_isnan(pt[1]) > 0); }

		public:
			// For a single camera and projector, get calibration object projector observations using structured light correspondences 
            // for a single pose
            static void ComputeProjectorObservations(const Observation::CameraObservation& cameraObs, const cv::Mat& patternImage, 
                const cv::Mat& minMaxImage, Observation::CameraObservation& projectorObs, const unsigned& threshold = 0.3, const unsigned& WINDOW_SIZE = 30);
		};
	}
}

#endif
