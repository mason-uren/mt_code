#pragma once
#include "cv.h"
#include "TypeDefs.h"

namespace BoeingMetrology
{
    namespace Calibration
    {
        namespace Pose
        {
#define HEAD -1
#define INVALID -2
            // an edge connects a camera and pattern
            struct edge
            {
                int cameraVertex;   // vertex index for camera in this edge
                int photoVertex;    // vertex index for pattern in this edge
                int photoIndex;     // photo index among photos for this camera
                cv::Mat transform;      // transform from pattern to camera
                CAMERA_NAME cameraName;
                std::string poseName;
                edge(int cv, int pv, int pi, const cv::Mat& trans, const CAMERA_NAME& cameraNameIn, const std::string& poseNameIn)
                {
                    cameraVertex = cv;
                    photoVertex = pv;
                    photoIndex = pi;
                    transform = trans.clone();
                    cameraName = cameraNameIn;
                    poseName = poseNameIn;
                }
            };

            struct vertex
            {
                // relative pose to the first camera. For camera vertex, it is the
                // transform from the first camera to this camera, for pattern vertex,
                // it is the transform from pattern to the first camera
                cv::Mat pose;

                // timestamp of photo, only available for photo vertex
                int timestamp;

                // The name of the camera (if this vertex represents a camera).  Will be empty if this vertex represents a calibration board.
                std::string name = "";

                // The name of the pose (unique identifier) along with "timestamp"
                std::string poseName = "";

                // The list of cameras that observed this pose
                std::vector<std::string> observingCameras;

                // Constructor for a calibration board
                vertex(const cv::Mat& po, int ts, const std::string & pn, const std::string & observingCameraName)
                {
                    pose = po.clone();
                    timestamp = ts;
                    poseName = pn;
                    observingCameras.push_back(observingCameraName);
                }

                // Constructor for a camera
                vertex(const CAMERA_NAME & nameIn)
                {
                    pose = cv::Mat::eye(4, 4, CV_32F);
                    timestamp = -1;
                    name = nameIn;
                    poseName = "";
                }

                // Copy constructor
                vertex(const vertex& other) :
                    pose(other.pose.clone()),
                    timestamp(other.timestamp),
                    name(other.name),
                    poseName(other.poseName),
                    observingCameras(other.observingCameras)
                {
                }
            };

        }
    }
}
