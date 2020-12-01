/*============================================================================

  Copyright 2017 by:

  The Boeing Company

  ============================================================================*/

#include "ChArucoBoardMultiDetector.h"
#include "Workspace/DataExecution/InputOutput/simpleoperationio.h"
#include "boeingmetrologytypes.h"
#include "Workspace/Application/System/systemutilities.h"
#include "Workspace/Application/LanguageUtils/streamqstring.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include "Workspace/DataExecution/InputOutput/input.h"
#include <mutex>
#include "Workspace/DataExecution/DataObjects/objectdictionary.h"

#include <thread>
#include "json/writer.h"
#include "Calibration/CalibrationObject/ArucoBoard.h"
#include "Calibration/Observation/CameraObservation.h"
#include "Calibration/Observation/ObservationAngles.h"
#include "Workspace/Application/logmanager.h"
#include "Calibration/CaptureSentinel.h"

namespace BoeingMetrology
{
    using namespace CSIRO;
    using namespace CSIRO::DataExecution;


    /**
     * \internal
     */
    class ChArucoBoardMultiDetectorImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(CSIRO::OpenCV::ChArucoBoardMultiDetectorImpl)

    public:
        ChArucoBoardMultiDetector&  op_;

        // Inputs
        CSIRO::DataExecution::SimpleInput< ObjectDictionary > inputImages_;
        CSIRO::DataExecution::SimpleInput< BoeingMetrology::Calibration::CalibrationObject::ArucoBoard > arucoBoard_;
        CSIRO::DataExecution::SimpleInput< QString > accumulatedTargetDir_;
        CSIRO::DataExecution::SimpleInput< QString > targetResultsJsonDir_;
        CSIRO::DataExecution::SimpleInput< double > reprojectionErrorThreshold_;
        CSIRO::DataExecution::SimpleInput<int> numRequiredDetectedTargets_;
        CSIRO::DataExecution::SimpleInput<bool> writeResultsToDisk_;
        CSIRO::DataExecution::SimpleInput< BoeingMetrology::Calibration::MultiCameraIntrinsicData > multiCameraIntrinsicData_;

        // Outputs
        CSIRO::DataExecution::SimpleOutput< ObjectDictionary > targetImageMasks_;
        CSIRO::DataExecution::SimpleOutput< ObjectDictionary > detectedTargetMask_;
        CSIRO::DataExecution::SimpleOutput< int > numDetections_;
        CSIRO::DataExecution::SimpleOutput< BoeingMetrology::Calibration::MultiCameraObservation > cameraPosePoints_;
        CSIRO::DataExecution::SimpleOutput< ObjectDictionary > observationPoints_;
        CSIRO::DataExecution::SimpleOutput< ObjectDictionary > observationAngles_;

        // Data objects
        std::recursive_mutex detectionLock;

        ChArucoBoardMultiDetectorImpl(ChArucoBoardMultiDetector& op);
        void DetectTargetThreaded(const QString &indexString, const std::string& accumulatedTargetDir, const std::string& targetResultsJsonDir);

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    ChArucoBoardMultiDetectorImpl::ChArucoBoardMultiDetectorImpl(ChArucoBoardMultiDetector& op) :
        op_(op),
        inputImages_("InputImages", op_),
        arucoBoard_("Aruco Board", op_),
        accumulatedTargetDir_("AccumulatedTargetDir", op_),
        targetResultsJsonDir_("TargetResultsJsonDir", op_),
        reprojectionErrorThreshold_("ReprojectionErrorThreshold", 0.0, op_),
        numRequiredDetectedTargets_("numRequiredDetectedTargets", 20, op_),
        writeResultsToDisk_("Write results to disk", true, op_),
        multiCameraIntrinsicData_("MultiCameraIntrinsics", op_),        
        targetImageMasks_("TargetImageMasks", op_),
        detectedTargetMask_("DetectTargetMask", op_),
        numDetections_("NumDetections", op_),
        cameraPosePoints_("PoseData", op_),
        observationPoints_("OutputObservations", op_),
        observationAngles_("ObservationAngles", op_)
    {
        //op_.ensureHasData();

        // Operation description
        op_.setDescription(tr("Perform Aruco board detection for multiple cameras viewing the same board."));

        // Inputs description
        inputImages_.input_.setDescription(tr("An ObjectDictionary of cv::Mat images indexed by camera name (QString).  These are the source images for detection processing."));
        arucoBoard_.input_.setDescription(tr("ArucoBoard class instance.  Describes the board geometry."));
        accumulatedTargetDir_.input_.setDescription(tr("Directory where chessboard corner detections accumulated over time are cached."));
        targetResultsJsonDir_.input_.setDescription(tr("Directory where json-serialized detection results and corresponding board geometry are cached."));
        reprojectionErrorThreshold_.input_.setDescription(tr("Maximum reprojection error (in pixels) allowed for a valid detection."));
        numRequiredDetectedTargets_.input_.setDescription(tr("Minimum number of chessboard corners to be considered a valid detection."));
        writeResultsToDisk_.input_.setDescription(tr("Write results to disk. Uncheck to not change any file on disk."));
        multiCameraIntrinsicData_.input_.setDescription(tr("MultiCameraIntrinsicData class instance.  These may just be initial estimates -- they're used for 3D pose estimation to populate ObservationAngles output and to not effect detection results"));

        // Outputs description
        targetImageMasks_.output_.setDescription(tr("An ObjectDictionary of cv::Mat images indexed by camera name (QString).  These are the cumulative images with red overlaid charuco IDs."));
        detectedTargetMask_.output_.setDescription(tr("An ObjectDictionary of bools indexed by camera name (QString).  These indicate a detection."));
        numDetections_.output_.setDescription(tr("The total number of cameras that detected the board."));
        cameraPosePoints_.output_.setDescription(tr("MultiCameraObservation class instance.  TBD I don't think this is actually populated."));
        observationPoints_.output_.setDescription(tr("An ObjectDictionary of CameraObservation class instances indexed by camera name (QString).  "));
        observationAngles_.output_.setDescription(tr("An ObjectDictionary of ObservationAngles class instances indexed by camera name (QString).  These are accumulated over time.  "));
    }

    void ChArucoBoardMultiDetectorImpl::DetectTargetThreaded(const QString &cameraName, const std::string& accumulatedTargetDir, const std::string& targetResultsJsonDir)
    {
        const auto& inputImages = *inputImages_;
        auto& targetImageMasks = *targetImageMasks_;
        auto& detectedTargetMask = *detectedTargetMask_;
        auto& numDetections = *numDetections_;
        // auto writeResultsToDisk = *writeResultsToDisk_; // This variable is not used
        Calibration::CalibrationObject::ArucoBoard &arucoBoard = *arucoBoard_;
        BoeingMetrology::Calibration::MultiCameraIntrinsicData & cameraIntrinsics = *multiCameraIntrinsicData_;
        auto &observationPts = *observationPoints_;


        if (!inputImages.getItem(cameraName)->isType<cv::Mat>())
        {
            Application::LogManager::logLine(LOG_ERROR, QString("Input image %1 is not of type cv::Mat").arg(cameraName));
            return;
        }

        // Get references to dictionary elements for this camera
        const auto& inputImage = inputImages.getItem(cameraName)->getRawData<cv::Mat>();
        auto& targetImageMask = targetImageMasks.getItem(cameraName)->getRawData<cv::Mat>();
        auto& detectedTarget = detectedTargetMask.getItem(cameraName)->getRawData<bool>();
        auto& observationPtsThisCamera = observationPts.getItem(cameraName)->getRawData<Calibration::Observation::CameraObservation>();
        auto& observationAnglesThisCamera = observationAngles_->getItem(cameraName)->getRawData<Calibration::Observation::ObservationAngles>();

        // Attempt to detect the calibration board.  If it's detected, estimate the 3d pose of the board using the provided intrinsics.  
        // Record results to disk. 
        try
        {
            cv::Mat markerCornerOverlay;
            detectedTarget = arucoBoard.DetectCornersAndEstimatePose(inputImage, cameraName.toStdString(), cameraIntrinsics.cameraData.at(cameraName.toStdString()),
                *numRequiredDetectedTargets_, *reprojectionErrorThreshold_, accumulatedTargetDir_->toStdString(), targetResultsJsonDir,
                observationPtsThisCamera, targetImageMask, markerCornerOverlay, observationAnglesThisCamera);
        }
        catch (std::exception& e)
        {
            op_.logLine(CSIRO::LOG_ERROR, tr("exception in CalibTargetDetector: %1").arg(static_cast<const char*>(e.what())));
            detectedTarget = false;
            return;
        }

        if (detectedTarget)
        {
            std::lock_guard<std::recursive_mutex> locker(detectionLock);
            numDetections++;
            std::cout << "ChArucoDetector: " << cameraName.toStdString() << " distance to board = " << observationPtsThisCamera.GetRangeToTarget() << " meters" << std::endl;
        }
    }

    /**
    *
    */
    bool ChArucoBoardMultiDetectorImpl::execute()
    {
        auto& inputImages = *inputImages_;
        auto& targetImageMasks = *targetImageMasks_;
        auto& detectedTargetMask = *detectedTargetMask_;
        auto& observedPts = *observationPoints_;

        // Clear some items
        targetImageMasks.clear();
        detectedTargetMask.clear();
        observedPts.clear();
        observationAngles_->clear();
        *numDetections_ = 0;

        // Get list of camera names
        QStringList nameList = inputImages.getNames();

        // Validate ArucoBoard
        if (!arucoBoard_->isValid())
        {
            op_.logLine(LOG_ERROR, tr("Invalid board passed through input: %1").arg(arucoBoard_.input_.getName()));
            return false;
        }

        // Storing json results is critical.  Return early if the path is incorrect.
        if (targetResultsJsonDir_->size() == 0)
        {
            op_.logLine(LOG_ERROR, tr("targetResultsJsonDir is empty"));
            return false;
        }

        // Create output paths if necessary
        QString accumulatedTargetDir = CSIRO::System::Utilities::resolveAbsoluteFilePath(*accumulatedTargetDir_);
        QString targetResultsJsonDir = CSIRO::System::Utilities::resolveAbsoluteFilePath(*targetResultsJsonDir_);
        QDir(accumulatedTargetDir).mkpath(".");
        QDir(targetResultsJsonDir).mkpath(".");

        // Loop through cameras
        for (const QString &str : nameList)
        {
            // Create Output dictionary so that the threading does not have to do it
            detectedTargetMask.addItem<bool>(str);
            targetImageMasks.addItem<cv::Mat>(str);
            observedPts.addItem<Calibration::Observation::CameraObservation>(str);
            observationAngles_->addItem<Calibration::Observation::ObservationAngles>(str);
        }

        // Launch detection for each camera in parallel
        std::vector<std::thread*> threads(inputImages.count());
        for (int t = 0; t < threads.size(); t++)
        {
            threads[t] = new std::thread(&ChArucoBoardMultiDetectorImpl::DetectTargetThreaded, this,
                inputImages.getName(t),
                accumulatedTargetDir.toStdString(),
                targetResultsJsonDir.toStdString());
        }

        // Wait for threads to finish
        std::for_each(threads.begin(), threads.end(), [](std::thread* x){ x->join(); });

        // Free threads
        for (unsigned iter = 0; iter < threads.size(); iter++) delete threads[iter];

        // Clear any entries from failed threads
		std::vector<std::string> cameras = {};
        for (const QString& name : nameList)
        {
            auto& observationPtsThisCamera = observedPts.getItem(name)->getRawData<Calibration::Observation::CameraObservation>();
            if (observationPtsThisCamera.cameraName == "")
            {
                detectedTargetMask.removeItem(name);
                targetImageMasks.removeItem(name);
                observedPts.removeItem(name);
                observationAngles_->removeItem(name);
            }
			else
			{
				cameras.push_back(name.toStdString());
			}
        }

		
		for (CAMERA_NAME cam : cameras)
		{
			Calibration::IntrinsicData intrinsics = multiCameraIntrinsicData_->cameraData[cam];

			std::string intrinsicFileName = targetResultsJsonDir.toStdString() + "/" + cam + ".json";

			intrinsics.SerializeFile(intrinsicFileName);
			
		}

		Calibration::CaptureSentinel sentinel = Calibration::CaptureSentinel(cameras);

		std::string sentinelFileName = targetResultsJsonDir.toStdString() + "/sentinel.json";

		sentinel.SerializeFile(sentinelFileName);


        return true;
    }



    /**
     *
     */
    ChArucoBoardMultiDetector::ChArucoBoardMultiDetector() :
        CSIRO::DataExecution::Operation(
        CSIRO::DataExecution::OperationFactoryTraits< ChArucoBoardMultiDetector >::getInstance(),
        tr("Multi-Camera ChAruco Board Detection"))
    {
        pImpl_ = new ChArucoBoardMultiDetectorImpl(*this);
        ensureHasData();
    }

    /**
     *
     */
    ChArucoBoardMultiDetector::~ChArucoBoardMultiDetector()
    {
        delete pImpl_;
    }

    /**
     *
     */
    bool  ChArucoBoardMultiDetector::execute()
    {
        return pImpl_->execute();
    }
}


using namespace BoeingMetrology;
DEFINE_WORKSPACE_OPERATION_FACTORY_NAMED(
    ChArucoBoardMultiDetector,
    BoeingMetrologyPlugin::getInstance(),
    CSIRO::DataExecution::Operation::tr("BoeingMetrology"),
    "ChArucoBoardMultiDetector")

