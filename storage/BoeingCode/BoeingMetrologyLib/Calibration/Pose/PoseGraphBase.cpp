#include <iostream>
#include <thread>
#include <mutex>
#include <limits>
#include <queue>
#include <cassert>

#include "json/value.h"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"

#include "PoseGraphBase.h"
#include "Calibration/CameraCalibrationUtilities.h"

using namespace BoeingMetrology::Calibration::Observation;

#define STRINGIFY(name) #name

namespace
{
    /**
     * Obtains the sign of a number. This has been defined in CSIRO::Application as well, but we
     * have duplicated it here temporarily to avoid requiring a new installation of Workspace.
     *
     * \tparam T  The numeric data type in question (or a type implicitly convertible to a numeric type)
     * \param val The value of which we're obtaining the sign.
     * \return -1 if the number is negative, or 1 if the number is positive.
     */
    template <typename T>
    inline int sign(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    /**
     * Clamps value to a specific minimum precision (minPrec). If fabs(value) is less than minPrec,
     * minPrec will be returned, otherwise value is returned.
     *
     * This has been defined in CSIRO::Application as well, but we
     * have duplicated it here temporarily to avoid requiring a new installation of Workspace.
     *
     * \param value    The value to clamp
     * \param minPrec  The minimum precision allowed for value. If the fabs(value) is less than this
     *                 value, minPrec will be returned.
     * \return If fabs(value) is less than minPrec, minPrec will be returned, otherwise value is returned.
     */
    inline double clampMinPrec(double value, double minPrec = std::numeric_limits<double>::min())
    {
        return std::fabs(value) < minPrec ? sign(value) * minPrec : value;
    }

    // JSON tags for overarching categories.
    const std::string CAMERAS_JSON_TAG = "cameras";
    const std::string VERTICES_JSON_TAG = "vertices";
    const std::string EDGES_JSON_TAG = "edges";

    // Vertex section JSON tags
    const std::string VERTEX_POSE_TAG = "pose";
    const std::string VERTEX_TIMESTAMP_TAG = "timestamp";
    const std::string VERTEX_NAME_TAG = "name";
    const std::string VERTEX_POSE_NAME_TAG = "poseName";
    const std::string VERTEX_OBSERVING_CAMERAS_TAG = "observingCameras";

    // Edge section JSON tags
    const std::string EDGE_CAMERA_VERTEX_TAG = "cameraVertex";
    const std::string EDGE_PHOTO_VERTEX_TAG = "photoVertex";
    const std::string EDGE_PHOTO_INDEX_TAG = "photoIndex";
    const std::string EDGE_TRANSFORM_TAG = "transform";

    // Criteria object JSON tags
    const std::string CRITERIA_TYPE_TAG = "type";
    const std::string CRITERIA_MAX_COUNT_TAG = "maxCount";
    const std::string CRITERIA_EPSILON_TAG = "epsilon";
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::packageMultiCameraExtrinsicData(MultiCameraExtrinsicData & multiCameraExtrinsicData)
{
    // Add each camera's extrinsics to the list
    const std::string refCameraName = _vertexList[0].name;
    for (const auto & name : this->GetCameraNames())
    {
        // Set camera names
        ExtrinsicData extrinsicData;
        extrinsicData.name = name;
        extrinsicData.refCameraName = refCameraName;
        CameraCalibrationUtilities::CleanFileName(poseDataPath, extrinsicData.poseDataPath);

        // Set position
        extrinsicData.SetTranslationVector(_outtvec[name]);

        // Rodrigues angles to 3x3 matrix
        cv::Mat rotMat(3, 3, CV_64FC1);
        cv::Rodrigues(_outrvec[name], rotMat);

        // Set rotation components
        extrinsicData.SetRotationMatrix(rotMat);

        // Append to the list
        multiCameraExtrinsicData.cameraData[name] = extrinsicData;
    }
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::JsonDeserialize(const Json::Value &jsonNode)
{
    Clear();

    // Read camera data: { CAMERA_NAME: { ... } }
    auto& cameras = jsonNode[CAMERAS_JSON_TAG];
    auto cameraNames = cameras.getMemberNames();
    for (auto cIter = cameraNames.begin(); cIter != cameraNames.end(); ++cIter)
    {
        // Camera root node
        const auto& cameraName = *cIter;
        auto& camera = cameras[cameraName];

        // Read _objectPointsForEachCamera: [ [x,y,z], ... ]
        {
            auto& pointVec = _objectPointsForEachCamera[cameraName];
            auto& points = camera[STRINGIFY(_objectPointsForEachCamera)];
            pointVec.resize(points.size(), cv::Mat(1, 3, CV_32F));
            for (Json::ArrayIndex i = 0; i < points.size(); ++i)
            {
                auto& coords = pointVec[i];
                auto& jsonCoords = points[i];
                for (int j = 0; j < 3; ++j)
                {
                    coords.at<double>(j) = jsonCoords[j].asDouble();
                }
            }
        }

        // Read _imagePointsForEachCamera: [ [x,y], ... ]
        {
            auto& pointVec = _imagePointsForEachCamera[cameraName];
            auto& points = camera[STRINGIFY(_imagePointsForEachCamera)];
            pointVec.resize(points.size(), cv::Mat(1, 3, CV_32F));
            for (Json::ArrayIndex i = 0; i < points.size(); ++i)
            {
                auto& coords = pointVec[i];
                auto& jsonCoords = points[i];
                for (int j = 0; j < 2; ++j)
                {
                    coords.at<double>(j) = jsonCoords[j].asDouble();
                }
            }
        }

        // Read _cameraMatrix: [ m00, m01, m02, m10, m11, m12, m20, m21, m22 ]
        {
            auto& cameraMat = _cameraMatrix[cameraName];
            cameraMat = cv::Mat(cv::Matx33d());
            auto& cameraMatJson = camera[STRINGIFY(_cameraMatrix)];
            for (int i = 0; i < 9; ++i)
            {
                cameraMat.at<double>(i) = cameraMatJson[i].asDouble();
            }
        }

        // Read _distortCoeffs: [ k1, k2, p1, p2, k3 ]
        {
            auto& distortCoeffs = _distortCoeffs[cameraName];
            distortCoeffs = cv::Mat(1, 5, CV_32F);
            auto& distortCoeffsJson = camera[STRINGIFY(_distortCoeffs)];
            for (int i = 0; i < 5; ++i)
            {
                distortCoeffs.at<double>(i) = distortCoeffsJson[i].asDouble();
            }
        }

        // Read _outrvec: [ x, y, z ]
        {
            auto& outrvec = _outrvec[cameraName];
            auto& outrvecJson = camera[STRINGIFY(_outrvec)];
            for (int i = 0; i < 3; ++i)
            {
                outrvec[i] = outrvecJson[i].asFloat();
            }
        }

        // Read _outtvec: [ x, y, z ]
        {
            auto& outtvec = _outtvec[cameraName];
            auto& outtvecJson = camera[STRINGIFY(_outtvec)];
            for (int i = 0; i < 3; ++i)
            {
                outtvec[i] = outtvecJson[i].asFloat();
            }
        }
    }

    // Read _vertexList: [ vertex1, ... ]
    auto& verticesJson = jsonNode[VERTICES_JSON_TAG];
    int numVerts = (int)verticesJson.size();
    for (int i = 0; i < numVerts; ++i)
    {
        auto& vertexJson = verticesJson[i];
        auto cameraName = vertexJson[VERTEX_NAME_TAG].asString();
        if (!cameraName.empty())
        {
            // The case where the vertex is a camera node
            _vertexList.push_back(vertex(cameraName));
            continue;
        }

        auto& poseJson = vertexJson[VERTEX_POSE_TAG];
        cv::Mat pose = cv::Mat::eye(4, 4, CV_32F);
        for (int j = 0; j < 16; ++j)
        {
            pose.at<float>(j) = poseJson[j].asFloat();
        }

        auto timestamp = vertexJson[VERTEX_TIMESTAMP_TAG].asInt();
        auto poseName = vertexJson[VERTEX_POSE_NAME_TAG].asString();
        auto& observingCamerasJson = vertexJson[VERTEX_OBSERVING_CAMERAS_TAG];
        if (observingCamerasJson.empty())
        {
            // If it's empty, indicative of corrupt data file.
            continue;
        }

        // Pose-based constructor requires a single observing camera
        _vertexList.push_back(vertex(pose, timestamp, poseName, observingCamerasJson[0].asString()));
        auto& vertex = _vertexList.back();
        for (int j = 1; j < (int)observingCamerasJson.size(); ++j)
        {
            // Now we can add all the remaining observing cameras
            vertex.observingCameras.push_back(observingCamerasJson[j].asString());
        }
    }

    // Read _edgeList: [ edge1, ... ]
    auto& edgesJson = jsonNode[EDGES_JSON_TAG];
    int numEdges = (int)edgesJson.size();
    for (int i = 0; i < numEdges; ++i)
    {
        // Temporaries needed as we can't construct a default object.
        int cameraVertex = 0;
        int photoVertex = 0;
        int photoIndex = 0;
        cv::Mat transform = cv::Mat::eye(4, 4, CV_32F);

        auto& edgeJson = edgesJson[i];
        cameraVertex = edgeJson[EDGE_CAMERA_VERTEX_TAG].asInt();
        photoVertex = edgeJson[EDGE_PHOTO_VERTEX_TAG].asInt();
        photoIndex = edgeJson[EDGE_PHOTO_INDEX_TAG].asInt();
        auto& poseJson = edgeJson[EDGE_TRANSFORM_TAG];
        for (int j = 0; j < 16; ++j)
        {
            transform.at<float>(j) = poseJson[j].asFloat();
        }

        // Get the camera name and pose name from the existing vertices, since we don't store these.
        auto cameraName = _vertexList[cameraVertex].name;
        auto poseName = _vertexList[photoVertex].poseName;
        _edgeList.push_back(edge(cameraVertex, photoVertex, photoIndex, transform.clone(), cameraName, poseName));
    }

    // Read _extrinParam
    _extrinParam = cv::Mat(1, ((int)_vertexList.size() - 1) * 6, CV_32F);
    auto& extrinParamJson = jsonNode[STRINGIFY(_extrinParam)];
    for (int i = 0; i < (int)extrinParamJson.size(); ++i)
    {
        _extrinParam.at<float>(i) = extrinParamJson[i].asFloat();
    }

    // Read _extrinParamFixed
    _extrinParamFixed = _extrinParam.clone();
    auto& extrinParamFixedJson = jsonNode[STRINGIFY(_extrinParamFixed)];
    for (int i = 0; i < (int)extrinParamFixedJson.size(); ++i)
    {
        _extrinParamFixed.at<float>(i) = extrinParamFixedJson[i].asFloat();
    }

    // Read _extrinparamFixedMask
    _extrinParamFixedMask = cv::Mat::zeros(_extrinParam.size(), CV_8UC1);
    auto& extrinParamFixedMaskJson = jsonNode[STRINGIFY(_extrinParamFixedMask)];
    for (int i = 0; i < (int)extrinParamFixedMaskJson.size(); ++i)
    {
        _extrinParamFixedMask.at<uchar>(i) = (uchar)extrinParamFixedMaskJson[i].asUInt();
    }

    // Read newVertices
    auto& newVertsJson = jsonNode[STRINGIFY(newVertices)];
    int numNewVerts = (int)newVertsJson.size();
    newVertices.resize(numNewVerts);
    for (int i = 0; i < numNewVerts; ++i)
    {
        newVertices[i] = newVertsJson[i].asInt();
    }

    // Read _criteria
    auto termCriteria = jsonNode[STRINGIFY(_criteria)];
    _criteria.type = termCriteria[CRITERIA_TYPE_TAG].asInt();
    _criteria.maxCount = termCriteria[CRITERIA_MAX_COUNT_TAG].asInt();
    _criteria.epsilon = termCriteria[CRITERIA_EPSILON_TAG].asDouble();

    // Read other flags
    _verbose = jsonNode[STRINGIFY(_verbose)].asInt();
    _nCamera = jsonNode[STRINGIFY(_nCamera)].asInt();
    poseDataPath = jsonNode[STRINGIFY(poseDataPath)].asString();
    _nPose = jsonNode[STRINGIFY(_nPose)].asInt();
    _flags = jsonNode[STRINGIFY(_flags)].asInt();
    _initialized = jsonNode[STRINGIFY(_initialized)].asBool();
    _isConnected = jsonNode[STRINGIFY(_isConnected)].asBool();
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::JsonSerialize(Json::Value &jsonNode) const
{
    // Note: The floating point numbers here have a habit of being smaller than the
    // smallest possible double value. We avoid writing these out at anything smaller
    // than this precision, as reading anything smaller than DBL_MIN will fail (although for
    // some reason we're able to write them out). We use clampMinPrec for this.

    // Store cameras: { CAMERA_NAME: { ... } }
    Json::Value cameras(Json::objectValue);
    auto cameraNames = GetCameraNames();
    for (auto cIter = cameraNames.begin(); cIter != cameraNames.end(); ++cIter)
    {
        // Camera root node
        const auto& cameraName = *cIter;
        Json::Value camera(Json::objectValue);

        // Store _objectPointsForEachCamera: [ [x,y,z], ... ]
        {
            const auto& pointVec = _objectPointsForEachCamera.at(cameraName);
            Json::Value points(Json::arrayValue);
            for (auto pIter = pointVec.begin(); pIter != pointVec.end(); ++pIter)
            {
                const auto& mat = *pIter;
                Json::Value coords(Json::arrayValue);
                // Confirm that each cv::Mat in _objectPointsForEachCamera contains 3 values [x, y, z]. 
                // We should be able to safely assume this, given this is in the definition of the PoseGraphBase
                // data structure. If we can't assume this, this should be changed from an assert to an if-statement.
                assert(mat.rows * mat.cols == 3);
                for (int i = 0; i < 3; ++i)
                {
                    coords.append(clampMinPrec(mat.at<float>(i)));
                }
                points.append(coords);
            }
            camera[STRINGIFY(_objectPointsForEachCamera)] = points;
        }


        // Store _imagePointsForEachCamera: [ [x,y], ... ]
        {
            const auto& pointVec = _imagePointsForEachCamera.at(cameraName);
            Json::Value points(Json::arrayValue);
            for (auto pIter = pointVec.begin(); pIter != pointVec.end(); ++pIter)
            {
                const auto& mat = *pIter;
                Json::Value coords(Json::arrayValue);
                for (int i = 0; i < 2; ++i)
                {
                    coords.append(clampMinPrec(mat.at<float>(i)));
                }
                points.append(coords);
            }
            camera[STRINGIFY(_imagePointsForEachCamera)] = points;
        }

        // Store _cameraMatrix: [ m00, m01, m02, m10, m11, m12, m20, m21, m22 ]
        {
            const auto& mat = _cameraMatrix.at(cameraName);
            Json::Value coords(Json::arrayValue);
            for (int i = 0; i < 9; ++i)
            {
                coords.append(clampMinPrec(mat.at<double>(i)));
            }
            camera[STRINGIFY(_cameraMatrix)] = coords;
        }

        // Store _distortCoeffs: [ k1, k2, p1, p2, k3 ]
        {
            const auto& mat = _distortCoeffs.at(cameraName);
            Json::Value coords(Json::arrayValue);
            for (int i = 0; i < 5; ++i)
            {
                coords.append(clampMinPrec(mat.at<double>(i)));
            }
            camera[STRINGIFY(_distortCoeffs)] = coords;
        }

        // Store _outrvec: [ x, y, z ]
        {
            const auto& mat = _outrvec.at(cameraName);
            Json::Value coords(Json::arrayValue);
            for (int i = 0; i < 3; ++i)
            {
                coords.append(clampMinPrec(mat[i]));
            }
            camera[STRINGIFY(_outrvec)] = coords;
        }

        // Store _outtvec: [ x, y, z ]
        {
            const auto& mat = _outtvec.at(cameraName);
            Json::Value coords(Json::arrayValue);
            for (int i = 0; i < 3; ++i)
            {
                coords.append(clampMinPrec(mat[i]));
            }
            camera[STRINGIFY(_outtvec)] = coords;
        }

        cameras[cameraName] = camera;
    }
    jsonNode[CAMERAS_JSON_TAG] = cameras;

    // Store _vertexList: [ vertex1, ... ]
    Json::Value vertices(Json::arrayValue);
    for (auto& vertex : _vertexList)
    {
        Json::Value jsonVertex(Json::objectValue);
        if (!vertex.name.empty())
        {
            // Only the name is relevant if the node is a camera
            jsonVertex[VERTEX_NAME_TAG] = Json::Value(vertex.name);
            vertices.append(jsonVertex);
            continue;
        }

        // The node is a pose
        Json::Value poseMat(Json::arrayValue);
        for (int i = 0; i < 16; ++i)
        {
            poseMat[i] = clampMinPrec(vertex.pose.at<float>(i));
        }
        jsonVertex[VERTEX_POSE_TAG] = poseMat;
        jsonVertex[VERTEX_TIMESTAMP_TAG] = Json::Value(vertex.timestamp);
        jsonVertex[VERTEX_POSE_NAME_TAG] = Json::Value(vertex.poseName);
        Json::Value observingCameras(Json::arrayValue);
        for (auto& str : vertex.observingCameras)
        {
            observingCameras.append(Json::Value(str));
        }
        jsonVertex[VERTEX_OBSERVING_CAMERAS_TAG] = observingCameras;
        vertices.append(jsonVertex);
    }
    jsonNode[VERTICES_JSON_TAG] = vertices;

    // Store _edgeList: [ edge1, ... ]
    Json::Value edges(Json::arrayValue);
    for (auto& edge : _edgeList)
    {
        Json::Value jsonEdge(Json::objectValue);
        jsonEdge[EDGE_CAMERA_VERTEX_TAG] = edge.cameraVertex;
        jsonEdge[EDGE_PHOTO_VERTEX_TAG] = edge.photoVertex;
        jsonEdge[EDGE_PHOTO_INDEX_TAG] = edge.photoIndex;
        Json::Value transformMat(Json::arrayValue);
        for (int i = 0; i < 16; ++i)
        {
            transformMat[i] = clampMinPrec(edge.transform.at<float>(i));
        }
        jsonEdge[EDGE_TRANSFORM_TAG] = transformMat;
        edges.append(jsonEdge);
    }
    jsonNode[EDGES_JSON_TAG] = edges;

    // _extrinParam
    Json::Value extrinParam(Json::arrayValue);
    for (int i = 0; i < _extrinParam.rows; ++i)
    {
        for (int j = 0; j < _extrinParam.cols; ++j)
        {
            extrinParam.append(clampMinPrec(_extrinParam.at<float>(i,j)));
        }
    }
    jsonNode[STRINGIFY(_extrinParam)] = extrinParam;

    // _extrinParamFixed
    Json::Value extrinParamFixed(Json::arrayValue);
    for (int i = 0; i < _extrinParamFixed.rows; ++i)
    {
        for (int j = 0; j < _extrinParamFixed.cols; ++j)
        {
            extrinParamFixed.append(clampMinPrec(_extrinParamFixed.at<float>(i,j)));
        }
    }
    jsonNode[STRINGIFY(_extrinParamFixed)] = extrinParamFixed;

    // _extrinparamFixedMask
    Json::Value extrinParamFixedMask(Json::arrayValue);
    for (int i = 0; i < _extrinParamFixedMask.rows; ++i)
    {
        for (int j = 0; j < _extrinParamFixedMask.cols; ++j)
        {
            extrinParamFixedMask.append(_extrinParamFixedMask.at<uchar>(i,j));
        }
    }
    jsonNode[STRINGIFY(_extrinParamFixedMask)] = extrinParamFixedMask;

    // newVertices
    Json::Value newVerts(Json::arrayValue);
    for (auto i : newVertices)
    {
        newVerts.append(i);
    }
    jsonNode[STRINGIFY(newVertices)] = newVerts;

    // _criteria
    Json::Value termCriteria(Json::objectValue);
    termCriteria[CRITERIA_TYPE_TAG] = _criteria.type;
    termCriteria[CRITERIA_MAX_COUNT_TAG] = _criteria.maxCount;
    termCriteria[CRITERIA_EPSILON_TAG] = _criteria.epsilon;
    jsonNode[STRINGIFY(_criteria)] = termCriteria;

    // remaining internal flags
    jsonNode[STRINGIFY(_verbose)] = Json::Value(_verbose);
    jsonNode[STRINGIFY(_nCamera)] = Json::Value(_nCamera);
    jsonNode[STRINGIFY(poseDataPath)] = Json::Value(poseDataPath);
    jsonNode[STRINGIFY(_nPose)] = Json::Value(_nPose);
    jsonNode[STRINGIFY(_flags)] = Json::Value(_flags);
    jsonNode[STRINGIFY(_initialized)] = Json::Value(_initialized);
    jsonNode[STRINGIFY(_isConnected)] = Json::Value(_isConnected);
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::computeJacobianExtrinsic(const cv::Mat& extrinsicParams, cv::Mat& JTJ_inv, cv::Mat& JTE)
{
    // 6 * _nVertex
    int nParam = (int)extrinsicParams.total();

    // All the edges between cameras and poses
    int nEdge = (int)this->_edgeList.size();

    std::vector<int> pointsLocation(nEdge + 1, 0);

    for (int edgeIdx = 0; edgeIdx < nEdge; ++edgeIdx)
    {
        int nPoints = (int)this->_objectPointsForEachCamera[this->_edgeList[edgeIdx].cameraName][this->_edgeList[edgeIdx].photoIndex].total();
        pointsLocation[edgeIdx + 1] = pointsLocation[edgeIdx] + nPoints * 2;
    }

    JTJ_inv = cv::Mat(nParam, nParam, CV_64F);
    JTE = cv::Mat(nParam, 1, CV_64F);

    cv::Mat J = cv::Mat::zeros(pointsLocation[nEdge], nParam, CV_64F);
    cv::Mat E = cv::Mat::zeros(pointsLocation[nEdge], 1, CV_64F);

    for (int edgeIdx = 0; edgeIdx < nEdge; ++edgeIdx)
    {
        int photoVertex = this->_edgeList[edgeIdx].photoVertex;
        int photoIndex = this->_edgeList[edgeIdx].photoIndex;
        int cameraVertex = this->_edgeList[edgeIdx].cameraVertex;
        const std::string cameraName = this->_edgeList[edgeIdx].cameraName;

        cv::Mat objectPoints = this->_objectPointsForEachCamera[cameraName][photoIndex];
        cv::Mat imagePoints = this->_imagePointsForEachCamera[cameraName][photoIndex];

        cv::Mat rvecTran, tvecTran;
        cv::Mat R = this->_edgeList[edgeIdx].transform.rowRange(0, 3).colRange(0, 3);
        tvecTran = this->_edgeList[edgeIdx].transform.rowRange(0, 3).col(3);
        cv::Rodrigues(R, rvecTran);

        cv::Mat rvecPhoto = extrinsicParams.colRange((photoVertex - 1) * 6, (photoVertex - 1) * 6 + 3);
        cv::Mat tvecPhoto = extrinsicParams.colRange((photoVertex - 1) * 6 + 3, (photoVertex - 1) * 6 + 6);

        cv::Mat rvecCamera, tvecCamera;
        if (cameraVertex > 0)
        {
            rvecCamera = extrinsicParams.colRange((cameraVertex - 1) * 6, (cameraVertex - 1) * 6 + 3);
            tvecCamera = extrinsicParams.colRange((cameraVertex - 1) * 6 + 3, (cameraVertex - 1) * 6 + 6);
        }
        else
        {
            rvecCamera = cv::Mat::zeros(3, 1, CV_32F);
            tvecCamera = cv::Mat::zeros(3, 1, CV_32F);
        }


        cv::Mat jacobianPhoto, jacobianCamera, error;
        computePhotoCameraJacobian(rvecPhoto, tvecPhoto, rvecCamera, tvecCamera, rvecTran, tvecTran,
            objectPoints, imagePoints, this->_cameraMatrix[cameraName], this->_distortCoeffs[cameraName],
            jacobianPhoto, jacobianCamera, error);
        if (cameraVertex > 0)
        {
            jacobianCamera.copyTo(J.rowRange(pointsLocation[edgeIdx], pointsLocation[edgeIdx + 1]).
                colRange((cameraVertex - 1) * 6, cameraVertex * 6));
        }
        jacobianPhoto.copyTo(J.rowRange(pointsLocation[edgeIdx], pointsLocation[edgeIdx + 1]).
            colRange((photoVertex - 1) * 6, photoVertex * 6));

        error.copyTo(E.rowRange(pointsLocation[edgeIdx], pointsLocation[edgeIdx + 1]));
    }
    //flog << "E=" << E << std::endl;
    //std::cout << J.t() * J << std::endl;
    JTJ_inv = (J.t() * J + 1e-10).inv();
    JTE = J.t() * E;

}

double BoeingMetrology::Calibration::Pose::PoseGraphBase::computeProjectError(cv::Mat& parameters, std::map<POSE_NAME, double> & perPoseMetric) const
{
    int nVertex = (int)this->_vertexList.size();
    CV_Assert((int)parameters.total() == (nVertex - 1) * 6 && parameters.depth() == CV_32F);
    int nEdge = (int)this->_edgeList.size();

    std::map<POSE_NAME, double> denom;

    // recompute the transform between photos and cameras

    std::vector<Calibration::Pose::edge> edgeList = this->_edgeList;
    std::vector<cv::Vec3f> rvecVertex, tvecVertex;
    this->vector2parameters(parameters, rvecVertex, tvecVertex);

    double totalError = 0.0;
    int totalNPoints = 0;
    for (int edgeIdx = 0; edgeIdx < nEdge; ++edgeIdx)
    {
        cv::Mat RPhoto, RCamera, TPhoto, TCamera, transform;
        const std::string cameraName = edgeList[edgeIdx].cameraName;
        int cameraVertex = edgeList[edgeIdx].cameraVertex;
        int photoVertex = edgeList[edgeIdx].photoVertex;
        int PhotoIndex = edgeList[edgeIdx].photoIndex;
        TPhoto = cv::Mat(tvecVertex[photoVertex - 1]).reshape(1, 3);

        //edgeList[edgeIdx].transform = Mat::ones(4, 4, CV_32F);
        transform = cv::Mat::eye(4, 4, CV_32F);
        cv::Rodrigues(rvecVertex[photoVertex - 1], RPhoto);
        if (cameraVertex == 0)
        {
            // Reference camera
            RPhoto.copyTo(transform.rowRange(0, 3).colRange(0, 3));
            TPhoto.copyTo(transform.rowRange(0, 3).col(3));
        }
        else
        {
            TCamera = cv::Mat(tvecVertex[cameraVertex - 1]).reshape(1, 3);
            cv::Rodrigues(rvecVertex[cameraVertex - 1], RCamera);
            cv::Mat(RCamera*RPhoto).copyTo(transform.rowRange(0, 3).colRange(0, 3));
            cv::Mat(RCamera * TPhoto + TCamera).copyTo(transform.rowRange(0, 3).col(3));
        }

        transform.copyTo(edgeList[edgeIdx].transform);
        cv::Mat rvec, tvec;
        cv::Rodrigues(transform.rowRange(0, 3).colRange(0, 3), rvec);
        transform.rowRange(0, 3).col(3).copyTo(tvec);

        cv::Mat objectPoints, imagePoints, proImagePoints;
        objectPoints = this->_objectPointsForEachCamera.at(cameraName)[PhotoIndex];
        imagePoints = this->_imagePointsForEachCamera.at(cameraName)[PhotoIndex];

        cv::projectPoints(objectPoints, rvec, tvec, this->_cameraMatrix.at(cameraName), this->_distortCoeffs.at(cameraName),
            proImagePoints);

        cv::Mat error = imagePoints - proImagePoints;
        cv::Vec2f* ptr_err = error.ptr<cv::Vec2f>();
        double edgeError = 0;
        for (int i = 0; i < (int)error.total(); ++i)
        {
            edgeError += sqrt(ptr_err[i][0] * ptr_err[i][0] + ptr_err[i][1] * ptr_err[i][1]);
        }
        totalError += edgeError;
        totalNPoints += (int)error.total();

        if (perPoseMetric.count(edgeList[edgeIdx].poseName) == 0)
        {
            perPoseMetric[edgeList[edgeIdx].poseName] = 0.0;
            denom[edgeList[edgeIdx].poseName] = 0.0;
        }
        perPoseMetric[edgeList[edgeIdx].poseName] += edgeError;
        denom[edgeList[edgeIdx].poseName] += (int)error.total();
    }
    double meanReProjError = totalError / totalNPoints;

    for (auto & pose : perPoseMetric)
        pose.second /= denom[pose.first];

    return meanReProjError;
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::computePhotoCameraJacobian(const cv::Mat& rvecPhoto, const cv::Mat& tvecPhoto,
    const cv::Mat& rvecCamera, const cv::Mat& tvecCamera, cv::Mat& rvecTran, cv::Mat& tvecTran, const cv::Mat& objectPoints,
    const cv::Mat& imagePoints, const cv::Mat& K, const cv::Mat& distort, cv::Mat& jacobianPhoto, cv::Mat& jacobianCamera, cv::Mat& E)
{
    cv::Mat drvecTran_drecvPhoto, drvecTran_dtvecPhoto,
        drvecTran_drvecCamera, drvecTran_dtvecCamera,
        dtvecTran_drvecPhoto, dtvecTran_dtvecPhoto,
        dtvecTran_drvecCamera, dtvecTran_dtvecCamera;

    BoeingMetrology::Calibration::Pose::PoseGraphBase::compose_motion(rvecPhoto, tvecPhoto, rvecCamera, tvecCamera, rvecTran, tvecTran,
        drvecTran_drecvPhoto, drvecTran_dtvecPhoto, drvecTran_drvecCamera, drvecTran_dtvecCamera,
        dtvecTran_drvecPhoto, dtvecTran_dtvecPhoto, dtvecTran_drvecCamera, dtvecTran_dtvecCamera);

    if (rvecTran.depth() == CV_64F)
    {
        rvecTran.convertTo(rvecTran, CV_32F);
    }
    if (tvecTran.depth() == CV_64F)
    {
        tvecTran.convertTo(tvecTran, CV_32F);
    }

    cv::Mat imagePoints2, jacobian, dx_drvecCamera, dx_dtvecCamera, dx_drvecPhoto, dx_dtvecPhoto;

    cv::projectPoints(objectPoints, rvecTran, tvecTran, K, distort, imagePoints2, jacobian);

    if (objectPoints.depth() == CV_32F)
    {
        cv::Mat(imagePoints - imagePoints2).convertTo(E, CV_64FC2);
    }
    else
    {
        E = imagePoints - imagePoints2;
    }
    E = E.reshape(1, (int)imagePoints.total() * 2);

    dx_drvecCamera = jacobian.colRange(0, 3) * drvecTran_drvecCamera + jacobian.colRange(3, 6) * dtvecTran_drvecCamera;
    dx_dtvecCamera = jacobian.colRange(0, 3) * drvecTran_dtvecCamera + jacobian.colRange(3, 6) * dtvecTran_dtvecCamera;
    dx_drvecPhoto = jacobian.colRange(0, 3) * drvecTran_drecvPhoto + jacobian.colRange(3, 6) * dtvecTran_drvecPhoto;
    dx_dtvecPhoto = jacobian.colRange(0, 3) * drvecTran_dtvecPhoto + jacobian.colRange(3, 6) * dtvecTran_dtvecPhoto;

    jacobianCamera = cv::Mat(dx_drvecCamera.rows, 6, CV_64F);
    jacobianPhoto = cv::Mat(dx_drvecPhoto.rows, 6, CV_64F);

    dx_drvecCamera.copyTo(jacobianCamera.colRange(0, 3));
    dx_dtvecCamera.copyTo(jacobianCamera.colRange(3, 6));
    dx_drvecPhoto.copyTo(jacobianPhoto.colRange(0, 3));
    dx_dtvecPhoto.copyTo(jacobianPhoto.colRange(3, 6));
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::compose_motion(cv::InputArray _om1, cv::InputArray _T1, cv::InputArray _om2,
    cv::InputArray _T2, cv::Mat& om3, cv::Mat& T3, cv::Mat& dom3dom1, cv::Mat& dom3dT1, cv::Mat& dom3dom2, cv::Mat& dom3dT2,
    cv::Mat& dT3dom1, cv::Mat& dT3dT1, cv::Mat& dT3dom2, cv::Mat& dT3dT2)
{
    cv::Mat om1, om2, T1, T2;
    _om1.getMat().convertTo(om1, CV_64F);
    _om2.getMat().convertTo(om2, CV_64F);
    _T1.getMat().reshape(1, 3).convertTo(T1, CV_64F);
    _T2.getMat().reshape(1, 3).convertTo(T2, CV_64F);
    /*Mat om2 = _om2.getMat();
    Mat T1 = _T1.getMat().reshape(1, 3);
    Mat T2 = _T2.getMat().reshape(1, 3);*/

    //% Rotations:
    cv::Mat R1, R2, R3, dR1dom1(9, 3, CV_64FC1), dR2dom2;
    cv::Rodrigues(om1, R1, dR1dom1);
    cv::Rodrigues(om2, R2, dR2dom2);
    /*JRodriguesMatlab(dR1dom1, dR1dom1);
    JRodriguesMatlab(dR2dom2, dR2dom2);*/
    dR1dom1 = dR1dom1.t();
    dR2dom2 = dR2dom2.t();

    R3 = R2 * R1;
    cv::Mat dR3dR2, dR3dR1;
    //dAB(R2, R1, dR3dR2, dR3dR1);
    matMulDeriv(R2, R1, dR3dR2, dR3dR1);
    cv::Mat dom3dR3;
    cv::Rodrigues(R3, om3, dom3dR3);
    //JRodriguesMatlab(dom3dR3, dom3dR3);
    dom3dR3 = dom3dR3.t();

    dom3dom1 = dom3dR3 * dR3dR1 * dR1dom1;
    dom3dom2 = dom3dR3 * dR3dR2 * dR2dom2;
    dom3dT1 = cv::Mat::zeros(3, 3, CV_64FC1);
    dom3dT2 = cv::Mat::zeros(3, 3, CV_64FC1);

    //% Translations:
    cv::Mat T3t = R2 * T1;
    cv::Mat dT3tdR2, dT3tdT1;
    //dAB(R2, T1, dT3tdR2, dT3tdT1);
    matMulDeriv(R2, T1, dT3tdR2, dT3tdT1);

    cv::Mat dT3tdom2 = dT3tdR2 * dR2dom2;
    T3 = T3t + T2;
    dT3dT1 = dT3tdT1;
    dT3dT2 = cv::Mat::eye(3, 3, CV_64FC1);
    dT3dom2 = dT3tdom2;
    dT3dom1 = cv::Mat::zeros(3, 3, CV_64FC1);
}

int BoeingMetrology::Calibration::Pose::PoseGraphBase::getPhotoVertex(int timestamp, const std::string & poseName, const std::string & observingCameraName)
{
    int photoVertex = INVALID;

    // find in existing photo vertex
    for (int i = 0; i < (int)_vertexList.size(); ++i)
    {
        if (_vertexList[i].poseName == poseName)
        {
            photoVertex = i;
            break;
        }
    }

    // add a new photo vertex
    if (photoVertex == INVALID)
    {
        _vertexList.push_back(vertex(cv::Mat::eye(4, 4, CV_32F), timestamp, poseName, observingCameraName));
        photoVertex = (int)_vertexList.size() - 1;
        this->newVertices.push_back(photoVertex);
    }

    return photoVertex;
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::initialize(const bool & announceConnectivity)
{
    _intrinsicTimestamps = std::map<CAMERA_NAME, std::string>();
    int nVertices = (int)_vertexList.size();
    int nEdges = (int)_edgeList.size();

    //this->_verbose = true;

    // build graph
    cv::Mat G = cv::Mat::zeros(nVertices, nVertices, CV_32S);
    for (int edgeIdx = 0; edgeIdx < nEdges; ++edgeIdx)
    {
        //std::cout << "edge" << edgeIdx << " cam" << this->_edgeList[edgeIdx].cameraVertex
        //    << " photo" << this->_edgeList[edgeIdx].photoVertex << std::endl;
        G.at<int>(this->_edgeList[edgeIdx].cameraVertex, this->_edgeList[edgeIdx].photoVertex) = edgeIdx + 1;
    }
    G = G + G.t();

    // traverse the graph
    std::vector<int> pre, order;
    graphTraverse(G, 0, order, pre);

    // Find the connected vertices
    std::vector<int> connectedVertices;
    for (int i = 0; i < (int)order.size(); ++i)
        connectedVertices.push_back(order[i]);

    // Find the disconnected vertices
    std::vector<bool> disconnectedVertices;
    for (int i = 0; i < nVertices; i++)
        disconnectedVertices.push_back(true);
    for (const auto & cv : connectedVertices)
        disconnectedVertices[cv] = false;

    // Determine if all the cameras are connected
    _isConnected = true;
    for (int i = 0; i < nVertices; i++)
    {
        if (disconnectedVertices[i])
        {
            if (this->_vertexList[i].name != "")
            {
                // This camera is not connected!
                if (announceConnectivity)
                    std::cout << "ERROR: Camera " << this->_vertexList[i].name << " is not connected" << std::endl;
                _isConnected = false;
            }

            // Mark unused poses because they may be connected by additional cameras added in the future
            if (i > 0)
                this->newVertices.push_back(i);
        }
    }

    // Loop through connected (not dangling) vertices
    for (int i = 1; i < (int)order.size(); ++i)
    {
        int vertexIdx = order[i];
        const auto & it = std::find(this->newVertices.begin(), this->newVertices.end(), vertexIdx);
        if (it != this->newVertices.end())
        {
            // This vertex is newly connected and we need to initialize its state
            cv::Mat prePose = this->_vertexList[pre[vertexIdx]].pose;
            int edgeIdx = G.at<int>(vertexIdx, pre[vertexIdx]) - 1;
            cv::Mat transform = this->_edgeList[edgeIdx].transform;

            //std::cout << "vertexId=" << vertexIdx << " preVertex = " << pre[vertexIdx] << std::endl;

            if (this->_vertexList[vertexIdx].name != "")
            {
                // This is a camera vertex
                this->_vertexList[vertexIdx].pose = transform * prePose.inv();
                this->_vertexList[vertexIdx].pose.convertTo(this->_vertexList[vertexIdx].pose, CV_32F);
                if (_verbose)
                {
                    std::cout << "initial pose for camera " << this->_vertexList[vertexIdx].name << "(vertexIdx " << vertexIdx << ") is " << std::endl;
                    std::cout << this->_vertexList[vertexIdx].pose << std::endl;
                }
            }
            else
            {
                // This is a board pose
                this->_vertexList[vertexIdx].pose = prePose.inv() * transform;
                this->_vertexList[vertexIdx].pose.convertTo(this->_vertexList[vertexIdx].pose, CV_32F);
            }

            // This vertex is now initialized
            this->newVertices.erase(it);
        }
    }

    _initialized = true;
}


void BoeingMetrology::Calibration::Pose::PoseGraphBase::initializeExtrinParam(const std::vector<std::string> & lockedCameraNames)
{
    // get om, t vector
    int nVertex = (int)this->_vertexList.size();

    // Initialize the flattened vector
    _extrinParam = cv::Mat(1, (nVertex - 1) * 6, CV_32F);

    // Initialize the mask for locked extrinsics
    _extrinParamFixedMask = cv::Mat::zeros(_extrinParam.size(), CV_8UC1);

    int offset = 0;
    // the pose of the vertex[0] is eye
    for (int i = 1; i < nVertex; ++i)
    {
        // Pull out r and t for this pose
        cv::Mat rvec, tvec;
        cv::Rodrigues(this->_vertexList[i].pose.rowRange(0, 3).colRange(0, 3), rvec);
        this->_vertexList[i].pose.rowRange(0, 3).col(3).copyTo(tvec);

        // Copy XYZWPR into the flattened vector
        rvec.reshape(1, 1).copyTo(_extrinParam.colRange(offset, offset + 3));
        tvec.reshape(1, 1).copyTo(_extrinParam.colRange(offset + 3, offset + 6));

        // If this vertex represents a camera, see if it needs to be locked
        if (std::find(lockedCameraNames.begin(), lockedCameraNames.end(), this->_vertexList[i].name) != lockedCameraNames.end())
        {
            _extrinParamFixedMask.at<uchar>(offset) = 1;
            _extrinParamFixedMask.at<uchar>(offset + 1) = 1;
            _extrinParamFixedMask.at<uchar>(offset + 2) = 1;
            _extrinParamFixedMask.at<uchar>(offset + 3) = 1;
            _extrinParamFixedMask.at<uchar>(offset + 4) = 1;
            _extrinParamFixedMask.at<uchar>(offset + 5) = 1;
        }

        offset += 6;
    }

    // Deep copy initial extrinsics
    this->_extrinParamFixed = _extrinParam.clone();
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::populatervectvecFromExtrinParam()
{
    std::vector<cv::Vec3f> rvecVertex, tvecVertex;
    this->vector2parameters(this->_extrinParam, rvecVertex, tvecVertex);
    for (int verIdx = 1; verIdx < (int)this->_vertexList.size(); ++verIdx)
    {
        auto & v = this->_vertexList[verIdx];
        cv::Mat R;
        cv::Mat pose = cv::Mat::eye(4, 4, CV_32F);
        Rodrigues(rvecVertex[verIdx - 1], R);
        R.copyTo(pose.colRange(0, 3).rowRange(0, 3));
        cv::Mat(tvecVertex[verIdx - 1]).reshape(1, 3).copyTo(pose.rowRange(0, 3).col(3));
        v.pose = pose;
        if (this->_verbose && v.name != "")
        {
            std::cout << "final pose for camera " << v.name << "(vertexIdx " << verIdx << ") is " << std::endl;
            std::cout << pose << std::endl;
        }

        if (v.name != "")
        {
            this->_outrvec[v.name] = rvecVertex[verIdx - 1];
            this->_outtvec[v.name] = tvecVertex[verIdx - 1];
        }
    }

    const std::string refCameraName = this->_vertexList[0].name;
    Rodrigues(cv::Mat::eye(3, 3, CV_32F), this->_outrvec[refCameraName]);
    this->_outtvec[refCameraName] = cv::Vec3f(0, 0, 0);

    if (_verbose)
        std::cout << "Reference camera is " << _vertexList.front().name << std::endl;
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::graphTraverse(const cv::Mat& G, int begin, std::vector<int>& order, std::vector<int>& pre)
{
    CV_Assert(!G.empty() && G.rows == G.cols);
    int nVertex = G.rows;
    order.resize(0);
    pre.resize(nVertex, INVALID);
    pre[begin] = -1;
    std::vector<bool> visited(nVertex, false);
    std::queue<int> q;
    visited[begin] = true;
    q.push(begin);
    order.push_back(begin);

    while (!q.empty())
    {
        int v = q.front();
        q.pop();
        cv::Mat idx;
        // use my findNonZero maybe
        findRowNonZero(G.row(v), idx);
        for (int i = 0; i < (int)idx.total(); ++i)
        {
            int neighbor = idx.at<int>(i);
            if (!visited[neighbor])
            {
                visited[neighbor] = true;
                q.push(neighbor);
                order.push_back(neighbor);
                pre[neighbor] = v;
            }
        }
    }
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::findRowNonZero(const cv::Mat& row, cv::Mat& idx)
{
    CV_Assert(!row.empty() && row.rows == 1 && row.channels() == 1);
    cv::Mat _row;
    std::vector<int> _idx;
    row.convertTo(_row, CV_32F);
    for (int i = 0; i < (int)row.total(); ++i)
    {
        if (_row.at<float>(i) != 0)
        {
            _idx.push_back(i);
        }
    }
    idx.release();
    idx.create(1, (int)_idx.size(), CV_32S);
    for (int i = 0; i < (int)_idx.size(); ++i)
    {
        idx.at<int>(i) = _idx[i];
    }
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::vector2parameters(const cv::Mat& parameters, std::vector<cv::Vec3f>& rvecVertex, std::vector<cv::Vec3f>& tvecVertexs) const
{
    int nVertex = (int)_vertexList.size();
    CV_Assert((int)parameters.channels() == 1 && (int)parameters.total() == 6 * (nVertex - 1));
    CV_Assert(parameters.depth() == CV_32F);
    parameters.reshape(1, 1);

    rvecVertex.reserve(0);
    tvecVertexs.resize(0);

    for (int i = 0; i < nVertex - 1; ++i)
    {
        rvecVertex.push_back(cv::Vec3f(parameters.colRange(i * 6, i * 6 + 3)));
        tvecVertexs.push_back(cv::Vec3f(parameters.colRange(i * 6 + 3, i * 6 + 6)));
    }
}


void BoeingMetrology::Calibration::Pose::PoseGraphBase::parameters2vector(const std::vector<cv::Vec3f>& rvecVertex, const std::vector<cv::Vec3f>& tvecVertex, cv::Mat& parameters) const
{
    CV_Assert(rvecVertex.size() == tvecVertex.size());
    int nVertex = (int)rvecVertex.size();
    // the pose of the first camera is known
    parameters.create(1, 6 * (nVertex - 1), CV_32F);

    for (int i = 0; i < nVertex - 1; ++i)
    {
        cv::Mat(rvecVertex[i]).reshape(1, 1).copyTo(parameters.colRange(i * 6, i * 6 + 3));
        cv::Mat(tvecVertex[i]).reshape(1, 1).copyTo(parameters.colRange(i * 6 + 3, i * 6 + 6));
    }
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::Clear()
{
    this->_poseObservationPoints.clear();
    this->_objectPointsForEachCamera.clear();
    this->_imagePointsForEachCamera.clear();
    this->_cameraMatrix.clear();
    this->_distortCoeffs.clear();
    this->_edgeList.clear();
    this->_vertexList.clear();
    this->newVertices.clear();
    this->_outrvec.clear();
    this->_outtvec.clear();
    this->_extrinParam = cv::Mat();
    this->_initialized = false;
    this->_isConnected = false;
}

bool BoeingMetrology::Calibration::Pose::PoseGraphBase::CameraExists(const std::string & cameraName) const
{
    if (this->_cameraMatrix.count(cameraName) != 0)
        return true;
    else
        return false;
}

int BoeingMetrology::Calibration::Pose::PoseGraphBase::GetCameraVertexIndex(const std::string & cameraName) const
{
    int results = -1;
    for (int i = 0; i < (int)this->_vertexList.size(); i++)
    {
        if (this->_vertexList[i].name == cameraName)
        {
            results = i;
            break;
        }
    }
    return results;
}

std::vector<BoeingMetrology::CAMERA_NAME> BoeingMetrology::Calibration::Pose::PoseGraphBase::GetCameraNames() const
{
    std::vector<CAMERA_NAME> cameras;
    for (const auto & item : this->_cameraMatrix)
    {
        cameras.push_back(item.first);
    }
    return cameras;
}

std::vector<BoeingMetrology::POSE_NAME> BoeingMetrology::Calibration::Pose::PoseGraphBase::GetPoseNames() const
{
    std::vector<POSE_NAME> poses;
    for (const auto& pair : _poseObservationPoints)
    {
        poses.push_back(pair.first);
    }
    return poses;
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::AllocateNewCamera(const std::string & cameraName, const IntrinsicData & intrinsicData, int & vertexIndex)
{
    // Throw exception if camera already exists
    if (this->CameraExists(cameraName))
        throw std::runtime_error("PoseGraphBase::AllocateNewCamera failed because camera " + cameraName + " already exists");
    this->_cameraMatrix[cameraName] = cv::Mat(intrinsicData.cameraMatrix).clone();
    this->_distortCoeffs[cameraName] = intrinsicData.distortionCoeffs.clone();

    this->_nCamera = (int)this->_cameraMatrix.size() + 1;
    //this->_extrinParam = cv::Mat();

    vertexIndex = (int)this->_vertexList.size();
    this->_vertexList.push_back(vertex(cameraName));
    this->newVertices.push_back(vertexIndex);
}

double BoeingMetrology::Calibration::Pose::PoseGraphBase::optimizeExtrinsics(const std::vector<std::string> & lockedCameraNames, std::map<CAMERA_NAME, double> &perCamera2dErrors, std::map<CAMERA_NAME, double> &perCamera3dErrors, const bool reportIterResults /* = false */)
{
    this->initializeExtrinParam(lockedCameraNames);

    double scaledEpsilon = this->_criteria.epsilon * GetCameraNames().size();

    std::string timestamp = Utilities::getCurrentTimeInSeconds();

    std::map<POSE_NAME, std::vector<double>> perPoseMetrics3d;
    std::map<POSE_NAME, std::vector<double>> perPoseMetrics2d;

    //double error_pre = computeProjectError(extrinParam);
    // optimization
    const double alpha_smooth = 0.01;
    double change = 1;
    double error_old;
    for (int iter = 0;; ++iter)
    {
        if ((this->_criteria.type == 1 && iter >= this->_criteria.maxCount) ||
            (this->_criteria.type == 2 && change <= scaledEpsilon) ||
            (this->_criteria.type == 3 && (change <= scaledEpsilon || iter >= this->_criteria.maxCount)))
            break;
        double alpha_smooth2 = 1 - std::pow(1 - alpha_smooth, (double)iter + 1.0);
        cv::Mat JTJ_inv, JTError;
        this->computeJacobianExtrinsic(this->_extrinParam, JTJ_inv, JTError);

        cv::Mat G = alpha_smooth2 * JTJ_inv * JTError;
        if (G.depth() == CV_64F)
        {
            G.convertTo(G, CV_32F);
        }

        double kk = 1;

        // Update tentative extrinsics.  Ignore update to locked cameras.
        cv::Mat tentativeParam = this->_extrinParam + G.reshape(1, 1);
        for (int r = 0; r < this->_extrinParam.rows; r++)
        {
            for (int c = 0; c < this->_extrinParam.cols; c++)
            {
                if (this->_extrinParamFixedMask.at<uchar>(r, c) > 0)
                    tentativeParam.at<float>(r, c) = this->_extrinParamFixed.at<float>(r, c);
            }
        }

        std::map<POSE_NAME, double> perPoseMetric2d;
        double error = computeProjectError(tentativeParam, perPoseMetric2d);
        error_old = error;

        // Avoid increasing the error at this iteration
        for (int j = 0; j < 10; j++)
        {
            if (iter == 0 || error < error_old)
            {
                break;
            }
            kk *= 0.5;

            // Update tentative extrinsics.  Ignore update to locked cameras.
            tentativeParam = this->_extrinParam + kk*G.reshape(1, 1);
            for (int r = 0; r < this->_extrinParam.rows; r++)
            {
                for (int c = 0; c < this->_extrinParam.cols; c++)
                {
                    if (this->_extrinParamFixedMask.at<uchar>(r, c) > 0)
                        tentativeParam.at<float>(r, c) = this->_extrinParamFixed.at<float>(r, c);
                }
            }
            perPoseMetric2d.clear();
            error = computeProjectError(tentativeParam, perPoseMetric2d);
        }

        if (iter == 0 || error < error_old)
        {
            // Accept current results
            this->_extrinParam = tentativeParam;

            change = norm(G) / norm(this->_extrinParam);


            error_old = error;

            // Export current results
            this->populatervectvecFromExtrinParam();
            MultiCameraExtrinsicData multiCameraExtrinsicData;
            this->packageMultiCameraExtrinsicData(multiCameraExtrinsicData);
            const std::string fname = this->poseDataPath + "/accumulate/" + timestamp + "_optimize_iter_" + Utilities::zeroPadNumber(iter, 3) + ".json";
            multiCameraExtrinsicData.SerializeFile(fname);

            // 3D metric
            MultiCameraIntrinsicData multiCameraIntrinsicData(this->_cameraMatrix, this->_distortCoeffs);
            std::map<POSE_NAME, double> perPoseMetric3d;
            double overallMetric = 0.0;
            PoseGraphBase::ComparePerPoseBackProjectedObservations(multiCameraIntrinsicData, multiCameraExtrinsicData, this->_poseObservationPoints, perPoseMetric3d, overallMetric);

            for (const auto & pose : perPoseMetric3d)
                perPoseMetrics3d[pose.first].push_back(pose.second);

            for (const auto & pose : perPoseMetric2d)
                perPoseMetrics2d[pose.first].push_back(pose.second);

            if (reportIterResults)
            {
                std::cout << "iter = " << iter << " 2d error = " << error << "; 3d error = " << overallMetric << std::endl;
                //for (const auto & pose : perPoseMetric3d)
                //{
                //    std::cout << pose.first << ", " << pose.second << std::endl;
                //}
            }

        }
        else
        {
            break;
        }

    }
    std::vector<CAMERA_NAME> cameras = GetCameraNames();
    std::map<CAMERA_NAME, int> perCameraCount = std::map<CAMERA_NAME, int>();
    for (CAMERA_NAME camera : cameras)
    {
        perCamera3dErrors[camera] = 0.0;
        perCamera2dErrors[camera] = 0.0;
        perCameraCount[camera] = 0;
    }

    // Report out progress at each iteration with a row for each pose
    for (const auto & pose : perPoseMetrics3d)
    {
        std::cout << pose.first;
        for (const auto & metric : pose.second)
            std::cout << ", " << metric;
        std::cout << std::endl;
        for (const auto & metric : perPoseMetrics2d[pose.first])
            std::cout << ", " << metric;
        std::cout << std::endl;

        for (auto cameraObservationPair : _poseObservationPoints[pose.first])
        {
            perCameraCount[cameraObservationPair.first] += 1;
            perCamera2dErrors[cameraObservationPair.first] += perPoseMetrics2d[pose.first].back();
            perCamera3dErrors[cameraObservationPair.first] += pose.second.back();
        }
    }

    std::map<POSE_NAME, double> perPoseMetric2d;
    double error = computeProjectError(this->_extrinParam, perPoseMetric2d);

    for (CAMERA_NAME camera : cameras)
    {
        perCamera2dErrors[camera] /= perCameraCount[camera];
        perCamera3dErrors[camera] /= perCameraCount[camera];
    }


    this->populatervectvecFromExtrinParam();

    return error;
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::AddPoseVertices(const std::vector<std::vector<int>> &poseIds, const std::vector<cv::Mat> & rotMat,
    const std::vector<cv::Mat> & transMat, const std::vector<std::string> & poseNames, const std::string & cameraName, std::recursive_mutex & dataLock, 
    const std::vector<std::vector<cv::Point3f>> & cameraObjectPoints, const std::vector<std::vector<cv::Point2f>> &cameraImagePoints, const std::vector<std::vector<MARKER_IDENTIFIER>> & markerIds)
{
    // Loop through poses for this camera
    for (int poseIdx = 0; poseIdx < (int)poseIds.size(); ++poseIdx)
    {
        // Convert the calib object's position+orientation from 3x1 rotation angles and 3x1 translations to 4x4 matrix
        cv::Mat transform = cv::Mat::eye(4, 4, CV_32F);
        cv::Mat R, T;
        cv::Rodrigues(rotMat[poseIdx], R);
        T = (transMat[poseIdx]).reshape(1, 3);
        R.copyTo(transform.rowRange(0, 3).colRange(0, 3));
        T.copyTo(transform.rowRange(0, 3).col(3));

        {
            std::lock_guard<std::recursive_mutex> locker(dataLock);

            // Pose identifier.  Initialize item in _vertexList
            int timestamp = poseIds[poseIdx][0];
            int photoVertex = this->getPhotoVertex(timestamp, poseNames[poseIdx], cameraName);

            // Add the edge from camera to pose
            this->_edgeList.push_back(edge(this->GetCameraVertexIndex(cameraName), photoVertex, poseIdx, transform, cameraName, poseNames[poseIdx]));

            // Convert the data so that the marker index is in the cv::Mat 
            // Push the corresponding points to this camera, pose, marker
            cv::Mat objectpoints_i, imagepoints_i;
            int _depth = CV_32F;
            cv::Mat(cameraObjectPoints[poseIdx]).convertTo(objectpoints_i, CV_MAKETYPE(_depth, 3));
            cv::Mat(cameraImagePoints[poseIdx]).convertTo(imagepoints_i, CV_MAKETYPE(_depth, 2));
            _objectPointsForEachCamera[cameraName].push_back(objectpoints_i);
            _imagePointsForEachCamera[cameraName].push_back(imagepoints_i);
            
            // Add pose observation info
            ObservationPoints<float> obs;
            obs.cameraName = cameraName;
            obs.controlPoints = cameraObjectPoints[poseIdx];
            obs.observedPoints = cameraImagePoints[poseIdx];
            obs.markerIdentifier = markerIds[poseIdx];
            _poseObservationPoints[poseNames[poseIdx]].insert({ cameraName, obs });
        }

    } // End loop through poses
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::populateInitialPoseGraph(const CAMERA_NAME &cameraName,
    const cv::Size &imgSize, MultiCameraIntrinsicData & multiCameraIntrinsicData, std::recursive_mutex & dataLock,
    const std::vector<std::vector<cv::Point3f>> &cameraObjectPointsInitial, const std::vector<std::vector<cv::Point2f>> &cameraImagePointsInitial,
    const std::vector<std::vector<int>> &poseIdsInitial, const std::vector<std::string> & poseNamesInitial,
    const std::vector<std::vector<MARKER_IDENTIFIER>> &markerIdsInitial, std::vector<std::string> &failedCameras,
    const int & flag, const double & reprojThreshold, const int & minimumPointsPerView, const bool & reportBeforeAfter)
{
    try
    {
        // Use pre-existing intrinsic data
        cv::Mat cameraMatrix = cv::Mat::zeros(cv::Size(3, 3), CV_64F);
        cv::Mat distortionCoeffs = cv::Mat::zeros(cv::Size(5, 1), CV_64F);
        {
            // Store results in map indexed by camera name
            std::lock_guard<std::recursive_mutex> locker(dataLock);

            if (multiCameraIntrinsicData.cameraData.find(cameraName) == multiCameraIntrinsicData.cameraData.end())
            {
                throw std::runtime_error("Failed to find intrinsics for " + cameraName);
            }

            // Deep copy temporary data, since cv::calibrateCamera will modify the intrinsics
            cameraMatrix = cv::Mat(multiCameraIntrinsicData.cameraData.at(cameraName).cameraMatrix).clone();
            _cameraMatrix[cameraName] = cameraMatrix;
            distortionCoeffs = multiCameraIntrinsicData.cameraData.at(cameraName).distortionCoeffs.clone();
            _distortCoeffs[cameraName] = distortionCoeffs;
            if (multiCameraIntrinsicData.cameraData[cameraName].timestamp != "")
            {
                _intrinsicTimestamps[cameraName] = multiCameraIntrinsicData.cameraData[cameraName].timestamp;
            }
        }

        // Filter observations by reprojection error
        std::vector<std::vector<cv::Point3f>> cameraObjectPoints = cameraObjectPointsInitial;
        std::vector<std::vector<cv::Point2f>> cameraImagePoints = cameraImagePointsInitial;
        std::vector<std::vector<int>> poseIds = poseIdsInitial;
        std::vector<std::string> poseNames = poseNamesInitial;
        std::vector<std::vector<MARKER_IDENTIFIER>> markerIds = markerIdsInitial;
        size_t n_before = cameraImagePoints.size();
        MultiPoseObservations::FilterObservationsBasedOnReproj(cameraObjectPoints, cameraImagePoints, imgSize,
            cameraMatrix, distortionCoeffs, flag, reprojThreshold, minimumPointsPerView, &poseIds, &poseNames, &markerIds);

        if (cameraObjectPoints.size() == 0)
        {
            throw std::runtime_error("All poses were filtered out!");
        }

        {
            // Have to do this again
            std::lock_guard<std::recursive_mutex> locker(dataLock);
            cameraMatrix = cv::Mat(multiCameraIntrinsicData.cameraData.at(cameraName).cameraMatrix).clone();
            distortionCoeffs = multiCameraIntrinsicData.cameraData.at(cameraName).distortionCoeffs.clone();
        }

        std::vector<cv::Mat> rotMat(poseIds.size()), transMat(poseIds.size());


        bool useSolvePnp = false;
        if (useSolvePnp)
        {
            for (int poseIdx = 0; poseIdx < (int)poseIds.size(); ++poseIdx)
            {
                cv::solvePnP(cameraObjectPoints[poseIdx], cameraImagePoints[poseIdx], cameraMatrix, distortionCoeffs, rotMat[poseIdx], transMat[poseIdx], 0);
            }
        }
        else
        {
            // Get the 3D position/orientation estimates for each pose for this camera.  The 3D estimates
            // relate the calibration object's local coordinate system (in which object points are defined)
            // to the camera.

            // TBD tangential distortion parameters are being modified by this function
            double reprojectError = cv::calibrateCamera(cameraObjectPoints, cameraImagePoints, imgSize, cameraMatrix, distortionCoeffs, rotMat, transMat, flag, _criteria);
            if (reportBeforeAfter)
            {
                std::cout << cameraName << " before=" << n_before << " after=" << cameraImagePoints.size() << " reproj error=" << reprojectError << std::endl;
            }
        }

        // Add vertices and edges for these poses
        this->AddPoseVertices(poseIds, rotMat, transMat, poseNames, cameraName, dataLock, cameraObjectPoints, cameraImagePoints, markerIds);

        std::lock_guard<std::recursive_mutex> locker(dataLock);
        if (_verbose)
        {
            std::cout << std::endl << "**** initialization results for camera " << cameraName << std::endl;
            //std::cout << "     rms = " << reprojectError << std::endl;
            std::cout << "     camera matrix = " << cameraMatrix << std::endl;
            std::cout << "     distortion parameters = " << distortionCoeffs << std::endl;
            std::cout << "**** initialization results for camera " << cameraName << std::endl << std::endl;
        }
    }
    catch (std::exception & e)
    {
        std::lock_guard<std::recursive_mutex> locker(dataLock);
        std::string blah = e.what();
        if (reportBeforeAfter)
            std::cout << "ERROR: initialPoseEstimator(): " << cameraName << " failed with std::exception - " << e.what() << std::endl;
        failedCameras.push_back(cameraName);
    }
    catch (...)
    {
        std::lock_guard<std::recursive_mutex> locker(dataLock);
        if (reportBeforeAfter)
            std::cout << "ERROR: initialPoseEstimator(): " << cameraName << " failed with unknown exception." << std::endl;
        failedCameras.push_back(cameraName);
    }
}

void BoeingMetrology::Calibration::Pose::PoseGraphBase::ComparePerPoseBackProjectedObservations(const MultiCameraIntrinsicData & multiCameraIntrinsicData,
    const MultiCameraExtrinsicData & multiCameraExtrinsicData, const std::map<POSE_NAME, std::map<CAMERA_NAME, ObservationPoints<float>>> & poseObservationPoints,
    std::map<POSE_NAME, double> & perPoseMetric, double & overallMetric)
{
    // Initialize overall result, which is an average of the pose-dependent errors
    overallMetric = 0.0;

    // Get the reference camera name
    std::string refName;
    multiCameraExtrinsicData.GetCommonReferenceCameraName(refName);

    // Loop through poses
    for (const auto & pose : poseObservationPoints)
    {
        // This will hold all the points in a common reference system
        std::map<MARKER_IDENTIFIER, std::vector<cv::Point3d>> backProjectedPts;

        // Get all the cameras that observed this pose
        POSE_NAME poseName = pose.first;

        // For each camera, compute the back-projected 3D points in that camera's coordinate system after rectifying the observations.  
        // Then transform all points into a common frame indexed by marker id
        std::map<CAMERA_NAME, std::map<MARKER_IDENTIFIER, cv::Point3d>> corners3d;
        std::map<CAMERA_NAME, cv::Mat> boardToCameraRot, boardToCameraTrans;
        for (const auto & camera : pose.second)
        {
            // Backproject in this camera's coordinate frame
            camera.second.BackprojectTo3d(multiCameraIntrinsicData.cameraData.at(camera.first), corners3d[camera.first], boardToCameraRot[camera.first], boardToCameraTrans[camera.first], true);

            // Contains transform from this camera to reference camera frame
            const ExtrinsicData & extrinsicData = multiCameraExtrinsicData.cameraData.at(camera.first);

            // Transform each point to the reference frame
            for (const auto & marker : corners3d[camera.first])
            {
                cv::Point3d ptInRefFrame = extrinsicData.TransformFromThisCameraToWorldFrameInverse(marker.second);
                backProjectedPts[marker.first].push_back(ptInRefFrame);
                //std::cout << ptInRefFrame.x << ", " << ptInRefFrame.y << ", " << ptInRefFrame.z << std::endl;
            }
        }

        // For this pose, this will be the mean across markers of the mean across cameras
        double errorThisPose = 0.0;

        // Loop through markers
        for (const auto & marker : backProjectedPts)
        {
            // Compute mean across cameras of this marker
            cv::Point3d mean(0.0, 0.0, 0.0);
            for (const auto & cam : marker.second)
                mean += cam;
            mean /= (double)marker.second.size();

            // Compute the mean absolute distance from mean
            double meanNorm = 0.0;
            for (const auto & cam : marker.second)
                meanNorm += cv::norm(cam - mean);
            meanNorm /= (double)marker.second.size();

            // Update the average error across markers
            errorThisPose += meanNorm;
        }
        errorThisPose /= (double)backProjectedPts.size();

        // Update outputs
        perPoseMetric[poseName] = errorThisPose;
        overallMetric += errorThisPose;

    } // End loop through poses

    overallMetric /= (double)poseObservationPoints.size();
}

