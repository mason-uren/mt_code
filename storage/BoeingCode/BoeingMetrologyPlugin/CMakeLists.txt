cmake_minimum_required(VERSION 3.0)
if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()
project(BOEINGMETROLOGYPLUGIN)


if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    find_path(WORKSPACE_CMAKE_ROOT Workspace_CMake_Root.txt
              HINTS ${CMAKE_SOURCE_DIR} ENV WORKSPACE_CMAKE_ROOT
              DOC "Directory containing the Workspace_CMake_Root.txt file")
    if (NOT WORKSPACE_CMAKE_ROOT)
        message(FATAL_ERROR "Please set WORKSPACE_CMAKE_ROOT to the directory containing the file Workspace_CMake_Root.txt")
    endif()
    include( ${WORKSPACE_CMAKE_ROOT}/CMakeLists.txt NO_POLICY_SCOPE )
endif()


include_directories(${BoeingMetrologyLib_SOURCE_DIR} ${COMMON_CORE_DIR})

include_directories(${MetaCloudLib_SOURCE_DIR})

include_directories(${BOEINGMETROLOGYPLUGIN_SOURCE_DIR})
if (NOT ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR} STREQUAL ${BOEINGMETROLOGYPLUGIN_BINARY_DIR})
    include_directories(${BOEINGMETROLOGYPLUGIN_BINARY_DIR})
endif()

option(EMULATE_LENSCONTROLLER_FOR_TESTING "Emulate a fake reading inside the Lens Controller operation" OFF)
if (EMULATE_LENSCONTROLLER_FOR_TESTING)
    add_definitions(-DEMULATE_LENSCONTROLLER_FOR_TESTING)
endif()

set(BOEINGMETROLOGYPLUGIN_VERSION 0.1.0)
string(REGEX MATCH "^[0-9]+" BOEINGMETROLOGYPLUGIN_SOVERSION ${BOEINGMETROLOGYPLUGIN_VERSION})

# Add other Qt modules as required by your plugin - eg. QtNetwork, QtOpenGL, QtSql
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Network)
set(QT_LIBRARIES Qt5::Core;Qt5::Widgets;Qt5::Network)

find_package(CUDA ${OpenCV_CUDA_VERSION})
include_directories(${CUDA_INCLUDE_DIRS})

if (WIN32)
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /bigobj")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
endif()

set(HEADERS
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/railmoverequest.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/mporefine.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/availablecudadevice.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/databaseconnectionsettings.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/pointcloudreader.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/assesscalibrationquality.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/CollectCalibrationHistory.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Utilities.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/removeblobs.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/cornervisualize.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/verifycheckerboard.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/removenonloop.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/generatearucoboardmask.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/rochadelocalthresholding.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/rochadeRefine.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/combineSaddlePoints.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/findSaddlePoints.h
    #${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/IntrinsicsExperiment.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/rochadeResize.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/skeleton.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Scharr.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/morphBlackHat.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/morphTopHat.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/morphGradient.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/morphClose.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/morphOpen.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/fillHolesAndNotches.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/multiposegraphinfo.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/charucoboardgeneratorwidget.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/arucocirclesdetector.h
    #${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiposeArucoCirclesDetection.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/generatearucocirclesboard.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/VisualizePoseErrors.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/charucoboard3dviz.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/charucoboard3dvizoptions.h
    #${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/multicam3dreconstruct.h
    #${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ProjectPoseObservations.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ChArucoEnums.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/detectiongraph.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/detectiongraphinfo.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/detectiongraphtojsonadaptor.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/updatedetectiongraph.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/comparedetectiongraphs.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/intrinsiccalibrationflags.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/boeingmetrologytypes.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/boeingmetrologyplugin_api.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/boeingmetrologyplugin.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ChArucoBoardGenerator.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ChArucoBoardDetector.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ChArucoBoardMultiDetector.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/CollectiveCameraCalibration.h    
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/CameraIntrinsicComputation.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiLensStateCalibrationIntrinsicData.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ExtrinsicCalibrationFromPointsOperation.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/FeatureExtraction.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ReadMultiPoseObservationsFromFile.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ReadMeshGeneratorObservationsFromFile.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/UndistortObservations.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiCameraExtrinsicComparison.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiPoseGraphGenerator.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiPoseGraphAddObservations.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiPoseObservationsFilter.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/op2d23d.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/SinglePoseExtrinsicEstimator.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/PoseGraphAnalyzerGenerator.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/PoseGraphAnalyzerFilter.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/PoseGraphAnalyzerResultsLoader.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/PoseGraphAnalyzerResultsWriter.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/postfilemultipart.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/postfilemultipartimpl.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/projectimage.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/LaserDotDetectorOp.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/StereoReconstruct3dOperation.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/GetObservationAngles.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/createcapturemetadata.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/LensControlApertureOp.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/LensControlFocusOp.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/LensControlGetState.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/LensStateMultiCameraSerializeFile.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Focuser.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/RangeFinderObservationSerializeFile.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/RangeFinderCameraExtrinsics.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/BrightnessMeasureOp.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/GenerateArucoDetectionsFromImages.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiCameraLensStateLUTGenerator.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiCameraLensStateSelector.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiPoseObservationsBestLensState.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/SetMultiCameraLensState.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ProjectorFocuser.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/IntrinsicDataDatabaseFilter.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ExtrinsicDataDatabaseFilter.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/QualityAssessorDatabaseFilter.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/CalibrationDatabaseManagerOp.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Configuration3DUCollectionDeserializeFile.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Configuration3DUCollectionGenerator.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Configuration3DUReconstructionGenerator.h
    
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/InitializeRouterManager.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/UpdateRouterManagerQueue.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/RouterManagerObjectGroup.h
    
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/GenerateImagesFromArucoBoard.h
	
	${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MetaCloudLibTests.h

)

set(INSTALL_HEADERS
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/railmoverequest.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/mporefine.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/availablecudadevice.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/databaseconnectionsettings.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/pointcloudreader.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/assesscalibrationquality.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/CollectCalibrationHistory.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Utilities.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/removeblobs.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/cornervisualize.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/verifycheckerboard.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/removenonloop.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/generatearucoboardmask.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/rochadelocalthresholding.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/rochadeRefine.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/combineSaddlePoints.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/findSaddlePoints.h
    #${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/IntrinsicsExperiment.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/rochadeResize.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/skeleton.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Scharr.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/morphBlackHat.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/morphTopHat.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/morphGradient.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/morphClose.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/morphOpen.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/fillHolesAndNotches.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/multiposegraphinfo.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/charucoboardgeneratorwidget.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/arucocirclesdetector.h
    #${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiposeArucoCirclesDetection.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/generatearucocirclesboard.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/VisualizePoseErrors.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/charucoboard3dviz.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/charucoboard3dvizoptions.h
    #${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/multicam3dreconstruct.h
    #${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ProjectPoseObservations.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ChArucoEnums.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/detectiongraph.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/detectiongraphinfo.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/detectiongraphtojsonadaptor.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/updatedetectiongraph.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/comparedetectiongraphs.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/intrinsiccalibrationflags.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/boeingmetrologytypes.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/boeingmetrologyplugin_api.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/boeingmetrologyplugin.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ChArucoBoardGenerator.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ChArucoBoardDetector.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ChArucoBoardMultiDetector.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/CollectiveCameraCalibration.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/CameraIntrinsicComputation.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiLensStateCalibrationIntrinsicData.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ExtrinsicCalibrationFromPointsOperation.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ReadMultiPoseObservationsFromFile.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ReadMeshGeneratorObservationsFromFile.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/UndistortObservations.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiCameraExtrinsicComparison.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiPoseGraphGenerator.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiPoseGraphAddObservations.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiPoseObservationsFilter.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/op2d23d.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/SinglePoseExtrinsicEstimator.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/PoseGraphAnalyzerGenerator.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/PoseGraphAnalyzerFilter.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/PoseGraphAnalyzerResultsLoader.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/PoseGraphAnalyzerResultsWriter.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/postfilemultipart.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/projectimage.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/LaserDotDetectorOp.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/StereoReconstruct3dOperation.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/GetObservationAngles.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/createcapturemetadata.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/LensControlApertureOp.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/LensControlFocusOp.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/LensControlGetState.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/LensStateMultiCameraSerializeFile.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Focuser.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/RangeFinderObservationSerializeFile.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/RangeFinderCameraExtrinsics.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/BrightnessMeasureOp.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/GenerateArucoDetectionsFromImages.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiCameraLensStateLUTGenerator.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiCameraLensStateSelector.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiPoseObservationsBestLensState.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/SetMultiCameraLensState.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ProjectorFocuser.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/IntrinsicDataDatabaseFilter.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ExtrinsicDataDatabaseFilter.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/QualityAssessorDatabaseFilter.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/CalibrationDatabaseManagerOp.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Configuration3DUCollectionDeserializeFile.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Configuration3DUCollectionGenerator.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Configuration3DUReconstructionGenerator.h

    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/InitializeRouterManager.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/UpdateRouterManagerQueue.h
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/RouterManagerObjectGroup.h
    
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/GenerateImagesFromArucoBoard.h
	
		${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MetaCloudLibTests.h

)

set(SOURCES
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/railmoverequest.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/mporefine.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/availablecudadevice.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/databaseconnectionsettings.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/pointcloudreader.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/assesscalibrationquality.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/CollectCalibrationHistory.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Utilities.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/removeblobs.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/cornervisualize.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/verifycheckerboard.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/removenonloop.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/generatearucoboardmask.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/rochadelocalthresholding.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/rochadeRefine.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/combineSaddlePoints.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/findSaddlePoints.cpp
    #${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/IntrinsicsExperiment.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/rochadeResize.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/skeleton.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Scharr.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/morphBlackHat.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/morphTopHat.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/morphGradient.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/morphClose.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/morphOpen.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/fillHolesAndNotches.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/multiposegraphinfo.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/charucoboardgeneratorwidget.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/arucocirclesdetector.cpp
    #${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiposeArucoCirclesDetection.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/generatearucocirclesboard.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/VisualizePoseErrors.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/charucoboard3dviz.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/charucoboard3dvizoptions.cpp
    #${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/multicam3dreconstruct.cpp
    #${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ProjectPoseObservations.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ChArucoEnums.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/detectiongraph.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/detectiongraphinfo.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/updatedetectiongraph.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/comparedetectiongraphs.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/detectiongraphtojsonadaptor.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/intrinsiccalibrationflags.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/boeingmetrologytypes.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/boeingmetrologyplugin.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ChArucoBoardGenerator.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ChArucoBoardDetector.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ChArucoBoardMultiDetector.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/CollectiveCameraCalibration.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/CameraIntrinsicComputation.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiLensStateCalibrationIntrinsicData.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ExtrinsicCalibrationFromPointsOperation.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/FeatureExtraction.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ReadMultiPoseObservationsFromFile.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ReadMeshGeneratorObservationsFromFile.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/UndistortObservations.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiCameraExtrinsicComparison.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiPoseGraphGenerator.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiPoseGraphAddObservations.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiPoseObservationsFilter.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/op2d23d.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/postfilemultipart.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/postfilemultipartimpl.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/projectimage.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/SinglePoseExtrinsicEstimator.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/PoseGraphAnalyzerGenerator.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/PoseGraphAnalyzerFilter.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/PoseGraphAnalyzerResultsLoader.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/PoseGraphAnalyzerResultsWriter.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/LaserDotDetectorOp.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/StereoReconstruct3dOperation.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/createcapturemetadata.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/GetObservationAngles.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/LensControlApertureOp.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/LensControlFocusOp.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/LensControlGetState.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/LensStateMultiCameraSerializeFile.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Focuser.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/RangeFinderObservationSerializeFile.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/RangeFinderCameraExtrinsics.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/BrightnessMeasureOp.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/GenerateArucoDetectionsFromImages.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiCameraLensStateLUTGenerator.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiCameraLensStateSelector.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MultiPoseObservationsBestLensState.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/SetMultiCameraLensState.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ProjectorFocuser.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/IntrinsicDataDatabaseFilter.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/ExtrinsicDataDatabaseFilter.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/QualityAssessorDatabaseFilter.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/CalibrationDatabaseManagerOp.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Configuration3DUCollectionDeserializeFile.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Configuration3DUCollectionGenerator.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Configuration3DUReconstructionGenerator.cpp

    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/InitializeRouterManager.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/UpdateRouterManagerQueue.cpp
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/RouterManagerObjectGroup.cpp
    
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/GenerateImagesFromArucoBoard.cpp
	
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/MetaCloudLibTests.cpp
)

set(UI_SOURCES
    ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/charucoboardgeneratorwidget.ui
)

list(APPEND RESOURCES ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/charucoboardgeneratorwidget_resources.qrc)

add_definitions(-DBOEINGMETROLOGYPLUGIN_VERSION=${BOEINGMETROLOGYPLUGIN_VERSION})

# The next line is used by the simple application generator wizard
# add_subdirectory(${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/Application)

# The below line can be used to import sub-directories
# include( ${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/SubDir/CMakeLists.txt )

# qtx macros are defined in the ${WORKSPACE_CMAKE_ROOT}/CMakeLists.txt included at the top of this file
# to support both Qt4 and Qt5 builds.
qtx_wrap_ui(UIC_SOURCES ${UI_SOURCES})
qtx_add_resources(RES_SOURCES ${RESOURCES})

add_library(boeingmetrologyplugin ${SOURCES} ${HEADERS} ${UIC_SOURCES} ${RES_SOURCES})
target_link_libraries(boeingmetrologyplugin workspace ${QT_LIBRARIES} ${OpenCV_LIBS} ${CUDA_CUDART_LIBRARY} BoeingMetrologyLib opencvplugin MetaCloudLib)
add_dependencies(boeingmetrologyplugin jsoncpp BoeingMetrologyLib TinyThread opencvplugin MetaCloudLib)

set_target_properties(boeingmetrologyplugin PROPERTIES
    DEFINE_SYMBOL BOEINGMETROLOGYPLUGIN_EXPORT
    VERSION ${BOEINGMETROLOGYPLUGIN_VERSION}
    SOVERSION ${BOEINGMETROLOGYPLUGIN_SOVERSION}
)

setTargetOutputDirectory(boeingmetrologyplugin ${CSIRO_INSTALL_AREA}/lib/Plugins)
configure_file(pkg-boeingmetrologyplugin.cmake ${CSIRO_INSTALL_AREA}/cmake/Exports/pkg-boeingmetrologyplugin.cmake @ONLY)

# Copy our install headers into the install directory so that others can build against our plugin.
foreach(inFile ${INSTALL_HEADERS})
    string(REGEX REPLACE "(${BOEINGMETROLOGYPLUGIN_SOURCE_DIR}/)(.*)" "${CSIRO_INSTALL_AREA}/include/BoeingMetrologyPlugin/\\2" outFile "${inFile}")
    configure_file(${inFile} ${outFile} COPYONLY)
endforeach(inFile)

add_subdirectory(Tests)
add_subdirectory(Designer)
