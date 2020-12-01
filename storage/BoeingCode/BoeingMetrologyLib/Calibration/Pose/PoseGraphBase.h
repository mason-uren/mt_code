#ifndef BOEINGMETROLOGYLIB_POSEGRAPHBASE_H
#define BOEINGMETROLOGYLIB_POSEGRAPHBASE_H

#include "cv.h"
#include <vector>
#include <string>
#include <map>
#include "json/value.h"
#include "Calibration/MultiCameraIntrinsicData.h"
#include "Calibration/MultiCameraExtrinsicData.h"
#include "Calibration/Observation/CameraObservation.h"
#include "Calibration/Observation/MultiPoseObservations.h"
#include "Common/Interface/Serializer.h"
#include "PoseGraphEdgeVertex.h"
#include <mutex>
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
    namespace Calibration
    {
        namespace Pose
        {
            class BOEINGMETROLOGYLIB_API PoseGraphBase : public Boeing::Interface::Serializer
            {
            public:
                //************************************
                // Method:    PoseGraphBase
                // FullName:  BoeingMetrology::Calibration::Pose::PoseGraphBase::PoseGraphBase
                // Access:    public 
                // Returns:   
                // Qualifier: A PoseGraph is the joining of intrinsic information with observation data.
                //            This allows estimation of the extrinsic data and the optimization of that data
                //************************************
                PoseGraphBase() :
                    _criteria( cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-7)),
                    _verbose(1),
                    _nCamera(0),
                    _nPose(0),
                    _flags(0),
                    _initialized(false),
                    _isConnected(false)
                {
					_intrinsicTimestamps = std::map<CAMERA_NAME, std::string>();
                }
                void initialize(const bool & announceConnectivity = true);

                // Stack the pose XYZWPR into a flattened format _extrinParam.  Initialize a mask used for locking extrinsics
                // of particular cameras.
                void initializeExtrinParam(const std::vector<std::string> & lockedCameraNames);

                void populatervectvecFromExtrinParam();

                bool IsInitialized()
                {
                    return _initialized;
                }

                bool IsConnected()
                {
                    return _isConnected;
                }

                // Package _outrvec, _outtvec as multiCameraExtrinsicData
                void packageMultiCameraExtrinsicData(MultiCameraExtrinsicData & multiCameraExtrinsicData);

                // Populate this object from the input stream.
                // Throws exception on failure.
                void JsonDeserialize(const Json::Value &jsonNode) override;

                // Populate the output Json from this object's data members.
                // Throws exception on failure.
                void JsonSerialize(Json::Value &jsonNode) const override;

				double optimizeExtrinsics(const std::vector<std::string> & lockedCameraNames, std::map<CAMERA_NAME, double> &perCamera2dErrors, std::map<CAMERA_NAME, double> &perCamera3dErrors, const bool reportIterResults = false);

                void vector2parameters(const cv::Mat& parameters, std::vector<cv::Vec3f>& rvecVertex, std::vector<cv::Vec3f>& tvecVertexs) const;
                void parameters2vector(const std::vector<cv::Vec3f>& rvecVertex, const std::vector<cv::Vec3f>& tvecVertex, cv::Mat& parameters) const;

                // Clear data members
                void Clear();

                // Return true if the camera is currently in the pose graph, false otherwise
                bool CameraExists(const std::string & cameraName) const;

                // Get the vertex index of a camera vertex
                int GetCameraVertexIndex(const std::string & cameraName) const;

                // Get the list of camera names
                std::vector<CAMERA_NAME> GetCameraNames() const;

                // Get the list of pose names
                std::vector<POSE_NAME> GetPoseNames() const;

                // Allocate a blank entry for a new camera.  Return the index to the camera.  Will throw 
                // exception if the camera already exists.  
                // Note: _extrinParam remains unallocated!
                void AllocateNewCamera(const std::string & cameraName, const IntrinsicData & intrinsicData, int & vertexIndex);

                // Used for 3D back-projection.  This should eventually replace _objectPointsForEachCamera and _imagePointsForEachCamera.
                std::map<POSE_NAME, std::map<CAMERA_NAME, ObservationPoints<float>>> _poseObservationPoints;

                // Known geometry: 3D coordinates of chessboard corners in calibration board coordinates.
                // Map indexed by camera, inner vector indexed by pose id
                std::map<CAMERA_NAME, std::vector<cv::Mat>> _objectPointsForEachCamera;

                // Extracted image coordinates of chessboard corners associated with _objectPointsForEachCamera
                // Map indexed by camera, inner vector indexed by pose id
                std::map<CAMERA_NAME, std::vector<cv::Mat>> _imagePointsForEachCamera;

                // Intrinsic information of each camera
                std::map<CAMERA_NAME, cv::Mat> _cameraMatrix;

                // Intrinsic information of each camera
                std::map<CAMERA_NAME, cv::Mat> _distortCoeffs;

				// Intrinsic timestamps of each camera
				std::map<CAMERA_NAME, std::string> _intrinsicTimestamps;

                // An edge for each (camera, pose id) pair across all cameras and poses
                std::vector<edge> _edgeList;

                // Each vertex in the connectivity graph.  Each vertex represents either a calibration board that connects
                // cameras or a camera itself.  
                std::vector<vertex> _vertexList;

                //output rotation and translation vectors of size _nCamera
                std::map<CAMERA_NAME, cv::Vec3f> _outrvec, _outtvec;

                // A flattened vector with XYZRPW for each vertex
                cv::Mat _extrinParam;

                // A flattened vector with the initial extrinsics for each camera and the associated mask
                cv::Mat _extrinParamFixed;
                cv::Mat _extrinParamFixedMask;

                // The list of vertices (indexes into _vertexList) that require initialization
                std::vector<int> newVertices;

                cv::TermCriteria _criteria;
                int _verbose;
                int _nCamera;
                std::string poseDataPath;

            protected:

                void computeJacobianExtrinsic(const cv::Mat& extrinsicParams, cv::Mat& JTJ_inv, cv::Mat& JTE);

                double computeProjectError(cv::Mat& parameters, std::map<POSE_NAME, double> & perPoseMetric) const;

                void computePhotoCameraJacobian(const cv::Mat& rvecPhoto, const cv::Mat& tvecPhoto, const cv::Mat& rvecCamera,
                    const cv::Mat& tvecCamera, cv::Mat& rvecTran, cv::Mat& tvecTran, const cv::Mat& objectPoints,
                    const cv::Mat& imagePoints, const cv::Mat& K, const cv::Mat& distort, cv::Mat& jacobianPhoto, cv::Mat& jacobianCamera,
                    cv::Mat& E);
                void compose_motion(cv::InputArray _om1, cv::InputArray _T1, cv::InputArray _om2, cv::InputArray _T2, cv::Mat& om3,
                    cv::Mat& T3, cv::Mat& dom3dom1, cv::Mat& dom3dT1, cv::Mat& dom3dom2, cv::Mat& dom3dT2, cv::Mat& dT3dom1,
                    cv::Mat& dT3dT1, cv::Mat& dT3dom2, cv::Mat& dT3dT2);

                // Initialize _vertexList.  photoVertex is the index into _vertexList of each pose that's identified by the timestamp. 
                int getPhotoVertex(int timestamp, const std::string & poseName, const std::string & observingCameraName);

                void graphTraverse(const cv::Mat& G, int begin, std::vector<int>& order, std::vector<int>& pre);
                void findRowNonZero(const cv::Mat& row, cv::Mat& idx);

                int _nPose;
                int _flags;

                bool _initialized = false;
                bool _isConnected = false;

                // Add 3D pose results for a single camera as vertices and edges in the pose graph
                void AddPoseVertices(const std::vector<std::vector<int>> & poseIds, const std::vector<cv::Mat> & rotMat,
                    const std::vector<cv::Mat> & transMat, const std::vector<std::string> & poseNames, const std::string & cameraName, std::recursive_mutex & dataLock,
                    const std::vector<std::vector<cv::Point3f>> & cameraObjectPoints, const std::vector<std::vector<cv::Point2f>> & cameraImagePoints, 
                    const std::vector<std::vector<MARKER_IDENTIFIER>> & markerIds);

                // Perform 3D pose estimation for multiple poses of a single camera and populate the pose graph 
                // with the results.  These initial results have not been optimized in any way.  
                // Optionally, update the intrinsics as part of the pose estimation process.  
                void populateInitialPoseGraph(const CAMERA_NAME &cameraName, const cv::Size &imgSize,
                    MultiCameraIntrinsicData & multiCameraIntrinsicData, std::recursive_mutex & dataLock,
                    const std::vector<std::vector<cv::Point3f>> &cameraObjectPointsInitial,
                    const std::vector<std::vector<cv::Point2f>> &cameraImagePointsInitial,
                    const std::vector<std::vector<int>> &poseIdsInitial,
                    const std::vector<std::string> & poseNamesInitial,
                    const std::vector<std::vector<MARKER_IDENTIFIER>> &markerIdsInitial,
                    std::vector<std::string> &failedCameras, const int & flag, const double & reprojThreshold,
                    const int & minimumPointsPerView, const bool & reportBeforeAfter = true);

                // Per-pose 3D back-projection metric.  Given a set of extrinsics and observation points, at each pose, perform back-projection of observation points to 3d, 
                // transform them into a common coordinate system using provided extrinsics, and then compare their 3D positions overall.
                static void ComparePerPoseBackProjectedObservations(const MultiCameraIntrinsicData & multiCameraIntrinsicData, const MultiCameraExtrinsicData & multiCameraExtrinsicData,
                    const std::map<POSE_NAME, std::map<CAMERA_NAME, ObservationPoints<float>>> & poseObservationPoints, std::map<POSE_NAME, double> & perPoseMetric, double & overallMetric);

            };
        }
    }
}

#endif
