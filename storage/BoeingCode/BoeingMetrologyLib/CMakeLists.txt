cmake_minimum_required(VERSION 3.0)
project(BoeingMetrologyLib)

include_directories(${BoeingMetrologyLib_SOURCE_DIR})
if (NOT ${BoeingMetrologyLib_SOURCE_DIR} STREQUAL ${BoeingMetrologyLib_BINARY_DIR}) 
    include_directories(${BoeingMetrologyLib_BINARY_DIR})
endif()

if (NOT CSIRO_WORKSPACE_VERSION_MAJOR)

    link_directories(${OpenCV_DIR} ${COMMON_CORE_DIR} ${COMMON_CORE_DIR}/SupportLibs/Zlib_1_2_8x64) 
    include_directories(${OpenCV_INCLUDE_DIRS} ) 


    INCLUDE_DIRECTORIES(
      ${BoeingMetrologyLib_SOURCE_DIR}
    )

    LINK_DIRECTORIES(${OPENCV_LIB} ${COMMON_CORE_DIR}) 
endif()
message(STATUS ${OpenCV_DIR}/include/opencv2)

if (UNIX AND APPLE)
  include_directories(${OpenCV_DIR}/../../include/opencv2)
else()
  include_directories(${OpenCV_DIR}/include/opencv2)
endif()

include_directories(${COMMON_UI_DIR} ${COMMON_UI_DIR}/OpenGL3DUI ${COMMON_CORE_DIR})

if (MSVC_VERSION EQUAL 1800)
# Add other Qt modules as required by your plugin - eg. QtNetwork, QtOpenGL, QtSql
find_package(Qt5Core)
find_package(Qt5Widgets)
set(QT_LIBRARIES Qt5::Core;Qt5::Widgets)
endif()

set(HEADERS
    ${BoeingMetrologyLib_SOURCE_DIR}/TypeDefs.h
    ${BoeingMetrologyLib_SOURCE_DIR}/BoeingMetrologyLib_API.h
    ${BoeingMetrologyLib_SOURCE_DIR}/PointCloudDataset.h
    
    ${BoeingMetrologyLib_SOURCE_DIR}/DeviceIO/FocusController/LC2/LensControls.h
    ${BoeingMetrologyLib_SOURCE_DIR}/DeviceIO/RangeFinderObservation.h

    ${BoeingMetrologyLib_SOURCE_DIR}/ImageProcessing/ImageCharacteristics.h
    ${BoeingMetrologyLib_SOURCE_DIR}/ImageProcessing/BrightnessMeasure.h
    ${BoeingMetrologyLib_SOURCE_DIR}/ImageProcessing/skeletonization.h
    ${BoeingMetrologyLib_SOURCE_DIR}/ImageProcessing/rochade.h
    ${BoeingMetrologyLib_SOURCE_DIR}/ImageProcessing/voronoi.h
    ${BoeingMetrologyLib_SOURCE_DIR}/ImageProcessing/ImagePlanePoints.h

    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/aruco/aruco.hpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/aruco/charuco.hpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CalibrationDatabaseManager.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CirclesGrid/CirclesGrid.hpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CirclesGrid/precomp.hpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CirclesGrid/BlobDetector.hpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CameraCalibrationUtilities.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/ExtrinsicCalibrationFromPoints.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/ExtrinsicStereoPairCalibrationFromPoints.h 
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/MultiRangeFinderObservation.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/ExtrinsicData.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/IntrinsicData.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CaptureSentinel.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/MultiCameraIntrinsicData.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/MultiCameraExtrinsicData.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/QualityAssessor.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/MultiCameraQualityAssessor.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CalibrationQualityComparator.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CameraPairDistances.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/SensorInfo.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/SensorInfoArray.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CalibrationObject/AbstractObjectBase.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CalibrationObject/ArucoBoard.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CalibrationObject/ArucoCirclesBoard.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/History/DataHistory.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/History/ExtrinsicDataHistory.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/History/IntrinsicDataHistory.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/History/QualityAssessorHistory.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/CameraObservation.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/LaserDotObservationPoints.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/MultiCameraObservation.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/MultiPoseObservations.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/ObservationPoints.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/ObservationAngles.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/ObservationAnalyzer.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/ObservationAnalyzerResults.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/LaserDotDetector.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Pose/PoseGraphBase.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Pose/MultiPoseGraph.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Pose/SinglePoseGraph.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Pose/PoseGraphAnalyzer.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Pose/PoseGraphEdgeVertex.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Pose/PoseGraphAnalyzerResults.h
    ${BoeingMetrologyLib_SOURCE_DIR}/MeshGenerator/MeshGeneratorObservations.h

    ${BoeingMetrologyLib_SOURCE_DIR}/Interface/ICameraPairDistances.h

    ${BoeingMetrologyLib_SOURCE_DIR}/Utilities/Utilities.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Utilities/Units.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Utilities/rotMatrixToEuler.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Utilities/Histogram.h

    ${BoeingMetrologyLib_SOURCE_DIR}/Features/FeatureBasedPointCloud.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Features/InspectionFeatures.h

    ${BoeingMetrologyLib_SOURCE_DIR}/2d23d/2d23d.h
    
    ${COMMON_CORE_DIR}/../Common/Interface/Serializer.h
    ${COMMON_CORE_DIR}/../Common/Manifold/IO/PlyMeshReader.h
    ${COMMON_CORE_DIR}/../Common/Manifold/Geometry/Mesh/UVTriangleMesh.h
    ${COMMON_CORE_DIR}/MeasurementHelper.h
    ${COMMON_CORE_DIR}/../Common/Manifold/Geometry/Vertex3f.h
    ${COMMON_CORE_DIR}/../Common/Manifold/Geometry/Vertex2f.h
    ${COMMON_CORE_DIR}/../Common/Manifold/Texture/InkbotsTexture.h
    ${COMMON_CORE_DIR}/../Common/Manifold/Texture/TexturePoint.h
    ${COMMON_CORE_DIR}/../Common/Manifold/Containers/MeshOctree.h
    ${COMMON_CORE_DIR}/../Common/Manifold/Containers/Octree.h
    ${COMMON_CORE_DIR}/../Common/Manifold/Geometry/FacePoint3f.h
    ${COMMON_CORE_DIR}/../Common/Communication/Socket/PracticalSocket.h
    ${COMMON_CORE_DIR}/ZlibIO.h
    ${COMMON_CORE_DIR}/SupportLibs/nanosvg/src/nanosvg.h
    ${COMMON_CORE_DIR}/SupportLibs/nanosvg/src/nanosvgrast.h
    ${COMMON_CORE_DIR}/SVGHelper.h
    ${COMMON_CORE_DIR}/StringHelper.h
    ${COMMON_CORE_DIR}/ImageCompression.h
    ${COMMON_CORE_DIR}/SupportLibs/Zlib_1_2_8x64/include/zlib.h

    #${BoeingMetrologyLib_SOURCE_DIR}/Features/FeatureBundleAdjuster.h
    #${BoeingMetrologyLib_SOURCE_DIR}/Features/FeatureBasedPointCloud.h

    ${BoeingMetrologyLib_SOURCE_DIR}/StereoReconstruct3d/StereoReconstruct3d.h

    ${BoeingMetrologyLib_SOURCE_DIR}/Scanning/Configuration/LensState.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Scanning/Configuration/Configuration3DUCollection.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Scanning/Configuration/Configuration3DUReconstruction.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Scanning/Configuration/SensorFOVGroups.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Scanning/DynamicIntrinsics/MultiLensStateIntrinsicData.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Scanning/Observation/MultiLensStateCameraObservations.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Scanning/DynamicLensState/MultiCameraLensStateLUT.h

    #Communication
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/RouterInterface.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/RouterWriteToFile.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/RouterWriteToZMQ.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/RouterManager.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/FormatterInterface.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/FormatterJpegToMat.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/FormatterJpegToPng.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/FormatterRawToPng.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/FormatterRawToMat.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/RoutingInstruction.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/RoutingTask.h
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/RouterDefs.h
)

set(SOURCES    
    ${BoeingMetrologyLib_SOURCE_DIR}/PointCloudDataset.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/TypeDefs.cpp
    
    ${BoeingMetrologyLib_SOURCE_DIR}/DeviceIO/FocusController/LC2/LensControls.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/DeviceIO/RangeFinderObservation.cpp

    ${BoeingMetrologyLib_SOURCE_DIR}/ImageProcessing/ImageCharacteristics.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/ImageProcessing/BrightnessMeasure.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/ImageProcessing/skeletonization.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/ImageProcessing/voronoi.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/ImageProcessing/rochade.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/ImageProcessing/ImagePlanePoints.cpp

    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/aruco/aruco.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/aruco/charuco.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CalibrationDatabaseManager.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CirclesGrid/CirclesGrid.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CirclesGrid/BlobDetector.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CameraCalibrationUtilities.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/ExtrinsicCalibrationFromPoints.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/ExtrinsicStereoPairCalibrationFromPoints.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/QualityAssessor.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/MultiCameraQualityAssessor.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CalibrationQualityComparator.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/ExtrinsicData.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/IntrinsicData.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CaptureSentinel.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/MultiCameraIntrinsicData.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/MultiCameraExtrinsicData.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CameraPairDistances.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/SensorInfo.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/SensorInfoArray.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CalibrationObject/AbstractObjectBase.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CalibrationObject/ArucoBoard.cpp    
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/CalibrationObject/ArucoCirclesBoard.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/History/ExtrinsicDataHistory.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/History/IntrinsicDataHistory.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/History/QualityAssessorHistory.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/CameraObservation.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/LaserDotObservationPoints.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/MultiCameraObservation.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/MultiPoseObservations.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/ObservationPoints.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/ObservationAngles.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/ObservationAnalyzer.cpp    
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/ObservationAnalyzerResults.cpp 
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/LaserDotDetector.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Pose/PoseGraphBase.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Pose/MultiPoseGraph.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Pose/SinglePoseGraph.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Pose/PoseGraphAnalyzer.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Pose/PoseGraphAnalyzerResults.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Pose/PoseGraphEdgeVertex.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Calibration/Observation/MultiRangeFinderObservation.cpp

    ${BoeingMetrologyLib_SOURCE_DIR}/MeshGenerator/MeshGeneratorObservations.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Interface/ICameraPairDistances.cpp

    ${BoeingMetrologyLib_SOURCE_DIR}/Utilities/Utilities.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Utilities/Units.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Utilities/Histogram.cpp

    ${BoeingMetrologyLib_SOURCE_DIR}/Features/FeatureBasedPointCloud.cpp    
    ${BoeingMetrologyLib_SOURCE_DIR}/Features/InspectionFeatures.cpp

    ${BoeingMetrologyLib_SOURCE_DIR}/2d23d/2d23d.cpp

    ${COMMON_CORE_DIR}/../Common/Interface/Serializer.cpp
#    ${COMMON_CORE_DIR}/../Common/Manifold/IO/PlyMeshReader.cpp
#    ${COMMON_CORE_DIR}/../Common/Manifold/Geometry/Mesh/UVTriangleMesh.cpp
    ${COMMON_CORE_DIR}/../Common/Manifold/Geometry/Vertex2f.cpp
    ${COMMON_CORE_DIR}/../Common/Manifold/Geometry/Vertex3f.cpp
    ${COMMON_CORE_DIR}/../Common/Manifold/Texture/InkbotsTexture.cpp
    ${COMMON_CORE_DIR}/../Common/Manifold/Texture/TexturePoint.cpp
#    ${COMMON_CORE_DIR}/../Common/Manifold/Containers/MeshOctree.cpp
#    ${COMMON_CORE_DIR}/../Common/Manifold/Containers/octree.cpp
#    ${COMMON_CORE_DIR}/../Common/Manifold/Geometry/FacePoint3f.cpp
    ${COMMON_CORE_DIR}/../Common/Communication/Socket/PracticalSocket.cpp
    ${COMMON_CORE_DIR}/ZlibIO.cpp
    ${COMMON_CORE_DIR}/MeasurementHelper.cpp
    ${COMMON_CORE_DIR}/SVGHelper.cpp
    ${COMMON_CORE_DIR}/StringHelper.cpp
    ${COMMON_CORE_DIR}/ImageCompression.cpp

    #${BoeingMetrologyLib_SOURCE_DIR}/Features/FeatureBundleAdjuster.cpp
    #${BoeingMetrologyLib_SOURCE_DIR}/Features/FeatureBasedPointCloud.cpp

    ${BoeingMetrologyLib_SOURCE_DIR}/StereoReconstruct3d/StereoReconstruct3d.cpp

    ${BoeingMetrologyLib_SOURCE_DIR}/Scanning/Configuration/LensState.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Scanning/Configuration/Configuration3DUCollection.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Scanning/Configuration/Configuration3DUReconstruction.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Scanning/Configuration/SensorFOVGroups.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Scanning/DynamicIntrinsics/MultiLensStateIntrinsicData.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Scanning/Observation/MultiLensStateCameraObservations.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Scanning/DynamicLensState/MultiCameraLensStateLUT.cpp

    #Communication
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/RouterInterface.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/RouterWriteToFile.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/RouterWriteToZMQ.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/RouterManager.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/FormatterInterface.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/FormatterJpegToMat.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/FormatterJpegToPng.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/FormatterRawToPng.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/FormatterRawToMat.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/RoutingInstruction.cpp
    ${BoeingMetrologyLib_SOURCE_DIR}/Communication/Router/RoutingTask.cpp
)

include(RailControlAutomation/CMakeLists.txt)

if (UNIX AND APPLE)
    set(JSONCPP_LIBRARIES libjsoncpp.a)
    find_package(ZLIB)
elseif(UNIX)
    set(JSONCPP_LIBRARIES libjsoncpp.a)
else()
    set(JSONCPP_LIBRARIES jsoncpp.lib)
    set(ZLIB_LIBRARIES ${COMMON_CORE_DIR}/SupportLibs/Zlib_1_2_8x64/lib/zdll.lib)
endif()

#find_package(Ceres REQUIRED)
#include_directories(${CERES_INCLUDE_DIRS})

add_library(BoeingMetrologyLib ${HEADERS} ${SOURCES})

if (MSVC_VERSION EQUAL 1800)
    target_link_libraries(BoeingMetrologyLib ${QT_LIBRARIES})
endif()
if (UNIX AND NOT APPLE)
    target_link_libraries(BoeingMetrologyLib stdc++fs)
endif()
target_link_libraries(BoeingMetrologyLib ${OpenCV_LIBS} ${JSONCPP_LIBRARIES} ${CERES_LIBRARIES} ${ZMQ_LIBRARIES} ${ZLIB_LIBRARIES} ${BOOST_LIBRARIES})

add_dependencies(BoeingMetrologyLib jsoncpp)

set_target_properties(BoeingMetrologyLib PROPERTIES DEFINE_SYMBOL BOEINGMETROLOGYLIB_EXPORT)

if (CSIRO_WORKSPACE_VERSION_MAJOR)
    # Part of a Workspace plugin build
    setTargetOutputDirectory(BoeingMetrologyLib ${CSIRO_INSTALL_AREA}/bin)
else()
    message (WARNING "${OpenCV_LIBS}")
endif()
    
foreach(inFile ${HEADERS} ${SOURCES})
    file(RELATIVE_PATH relativePath ${BoeingMetrologyLib_SOURCE_DIR}/.. ${inFile})
    get_filename_component(relativeDir ${relativePath} DIRECTORY)
    file(TO_NATIVE_PATH "${relativeDir}" groupName)
    source_group(${groupName} FILES ${inFile})
    unset(relativePath)
    unset(relativeDir)
    unset(groupName)
endforeach(inFile)
