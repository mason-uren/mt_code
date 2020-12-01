/*============================================================================

  Copyright 2017 by:

  The Boeing Company

  ============================================================================*/

#include "GenerateArucoDetectionsFromImages.h"
#include "Workspace/DataExecution/InputOutput/simpleoperationio.h"
#include "Workspace/Application/System/systemutilities.h"
#include "Workspace/DataExecution/InputOutput/input.h"
#include "Calibration/CalibrationObject/ArucoBoard.h"
#include "Calibration/MultiCameraIntrinsicData.h"

namespace BoeingMetrology
{
	using namespace CSIRO::DataExecution;

	class GenerateArucoDetectionsFromImagesImpl
	{
		// Allow string translation to work properly
		Q_DECLARE_TR_FUNCTIONS(BoeingMetrology::GenerateArucoDetectionsFromImagesImpl)

	public:
		GenerateArucoDetectionsFromImages&  op_;

		// Inputs and outputs
		CSIRO::DataExecution::SimpleInput<QString> filePath_;
		CSIRO::DataExecution::SimpleInput<int> numRequiredDetectedMarkers_;
        CSIRO::DataExecution::SimpleInput<double> maxReprojThreshold_;
        CSIRO::DataExecution::SimpleInput<Calibration::MultiCameraIntrinsicData> multiCameraIntrinsicData_;
		CSIRO::DataExecution::SimpleInput<Calibration::ArucoBoard> boardFormat_;

		GenerateArucoDetectionsFromImagesImpl(GenerateArucoDetectionsFromImages& op);

		bool  execute();
		void  logText(const QString& msg)   { op_.logText(msg); }
	};


	/**
	 *
	 */
	GenerateArucoDetectionsFromImagesImpl::GenerateArucoDetectionsFromImagesImpl(GenerateArucoDetectionsFromImages& op) :
		op_(op),
		filePath_("File Path to Pose Data", op_),
		numRequiredDetectedMarkers_("Num Required Detected Markers", 6, op_),
        maxReprojThreshold_("Max Reprojection Threshold", 1.0, op_),
        multiCameraIntrinsicData_("Multi Camera Intrinsic Data", op_),
		boardFormat_("Aruco Board Format", op_)
	{
        op_.setDescription(tr("Perform ChAruco detection on multiple image poses for multiple cameras.  Filter results based on reprojection error."));
        filePath_.input_.setDescription(tr("Top-level directory containing pose subfolders"));
        numRequiredDetectedMarkers_.input_.setDescription(tr("Minimum number of required markers to load an observation"));
        boardFormat_.input_.setDescription(tr("Deprecated -- ChAruco board description"));
	}

	/**
	 *  Main
	 */
	bool GenerateArucoDetectionsFromImagesImpl::execute()
	{
        std::set<CAMERA_NAME> failedCameras;
		
		// Read all the pose observations from the input path
		try
		{
            if (boardFormat_->isValid())
                Calibration::MultiCameraIntrinsicData::GenerateObservationData(filePath_->toStdString(), *numRequiredDetectedMarkers_, *multiCameraIntrinsicData_, failedCameras, &(*boardFormat_), *maxReprojThreshold_);
            else
                Calibration::MultiCameraIntrinsicData::GenerateObservationData(filePath_->toStdString(), *numRequiredDetectedMarkers_, *multiCameraIntrinsicData_, failedCameras, nullptr, *maxReprojThreshold_);

            if (failedCameras.size() > 0)
            {
                std::cout << "ReadMultiPoseObservationFromFile: Failed to read observation data for at least one camera" << std::endl;
                return false;
            }
			
            std::cout << "GenerateArucoDetectionsFromImages: Successfully generated observation data in \n" << filePath_->toStdString() << std::endl;
			return true;
		}
		catch (...)
		{
            std::cout << "GenerateArucoDetectionsFromImages: Failed to generate observation data in \n" << filePath_->toStdString() << std::endl;
			return false;
		}
	}

	/**
	 *
	 */
	GenerateArucoDetectionsFromImages::GenerateArucoDetectionsFromImages() :
		CSIRO::DataExecution::Operation(
		CSIRO::DataExecution::OperationFactoryTraits< GenerateArucoDetectionsFromImages >::getInstance(),
		tr("GenerateArucoDetectionsFromImages"))
	{
		pImpl_ = new GenerateArucoDetectionsFromImagesImpl(*this);
	}

	/**
	 *
	 */
	GenerateArucoDetectionsFromImages::~GenerateArucoDetectionsFromImages()
	{
		delete pImpl_;
	}

	/**
	 *
	 */
	bool  GenerateArucoDetectionsFromImages::execute()
	{
		return pImpl_->execute();
	}
}

using namespace BoeingMetrology;
DEFINE_WORKSPACE_OPERATION_FACTORY_NAMED(
	GenerateArucoDetectionsFromImages,
	BoeingMetrologyPlugin::getInstance(),
	CSIRO::DataExecution::Operation::tr("BoeingMetrology"),
	"GenerateArucoDetectionsFromImages")

