/*============================================================================

  Copyright 2017 by:

  The Boeing Company

  ============================================================================*/

#include "CameraIntrinsicComputation.h"
#include "Workspace/DataExecution/InputOutput/simpleoperationio.h"
#include "Workspace/Application/System/systemutilities.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Workspace/DataExecution/InputOutput/input.h"
#include "opencv2/calib3d.hpp"
#include "Calibration/MultiCameraIntrinsicData.h"
#include "Calibration/Observation/MultiPoseObservations.h"
#include "Calibration/Pose/PoseGraphAnalyzerResults.h"
#include "intrinsiccalibrationflags.h"
#include "Calibration/CalibrationObject/ArucoBoard.h"

namespace BoeingMetrology
{
	using namespace BoeingMetrology::Calibration;

	class CameraIntrinsicComputationImpl
	{

	public:
		CameraIntrinsicComputation&  op_;

		// Inputs and outputs
		CSIRO::DataExecution::SimpleInput<Calibration::MultiPoseObservations> multiPoseObservations_;
		CSIRO::DataExecution::SimpleInput<Calibration::Pose::PoseGraphAnalyzerResults> poseGraphAnalyzerResults_;
		CSIRO::DataExecution::SimpleInput<MultiCameraIntrinsicData> initialMultiCameraIntrinsicData_;
        CSIRO::DataExecution::SimpleInput<double> reprojThreshold_;
        CSIRO::DataExecution::SimpleInput<int> minimumPointsPerView_;
		CSIRO::DataExecution::SimpleInput<IntrinsicCalibrationFlags> computeIntrinsicDataFlags_;
        CSIRO::DataExecution::SimpleInput<ArucoBoard> arucoBoard_;


		CSIRO::DataExecution::SimpleOutput<MultiCameraIntrinsicData> multiCameraIntrinsicData_;
        CSIRO::DataExecution::SimpleOutput< QString> timestamp_;

		CameraIntrinsicComputationImpl(CameraIntrinsicComputation& op);

		bool  execute();
		void  logText(const QString& msg)   { op_.logText(msg); }
	};


	/**
	 *
	 */
	CameraIntrinsicComputationImpl::CameraIntrinsicComputationImpl(CameraIntrinsicComputation& op) :
		op_(op),
		multiPoseObservations_("MultiPoseObservations Instance", op_),
		poseGraphAnalyzerResults_("PoseGraphAnalyzerResults Instance", op_),
		initialMultiCameraIntrinsicData_("Initial Intrinsic Guess", op_),
        reprojThreshold_("Acceptable reprojection error in pixels (applied per pose)", 1.0, op_),
        minimumPointsPerView_("Minimum number of chessboard corners detected (applied per pose", 20, op_),
		computeIntrinsicDataFlags_("Compute Intrinsic Data Flags", op_),
        arucoBoard_("Aruco Board", op_),
        multiCameraIntrinsicData_("Intrinsics Of Each Camera", op_),
        timestamp_("Time Stamp", op_)
	{
	}

	/**
	 *  
	 */
	bool CameraIntrinsicComputationImpl::execute()
	{
        try
        {
            // Handles to inputs
            MultiPoseObservations& multiPoseObservations = *multiPoseObservations_;
            const Pose::PoseGraphAnalyzerResults & poseGraphAnalyzerResults = *poseGraphAnalyzerResults_;
            const MultiCameraIntrinsicData & initialMultiCameraIntrinsicData = *initialMultiCameraIntrinsicData_;
            const double & reprojThreshold = *reprojThreshold_;
            const int & minimumPointsPerView = *minimumPointsPerView_;
            const IntrinsicCalibrationFlags  &setFlags = *computeIntrinsicDataFlags_;
            int flags =
                setFlags.getCV_CALIB_FIX_ASPECT_RATIO() * CV_CALIB_FIX_ASPECT_RATIO +
                setFlags.getCV_CALIB_FIX_K1() * CV_CALIB_FIX_K1 +
                setFlags.getCV_CALIB_FIX_K2() * CV_CALIB_FIX_K2 +
                setFlags.getCV_CALIB_FIX_K3() * CV_CALIB_FIX_K3 +
                setFlags.getCV_CALIB_FIX_K4() * CV_CALIB_FIX_K4 +
                setFlags.getCV_CALIB_FIX_K5() * CV_CALIB_FIX_K5 +
                setFlags.getCV_CALIB_FIX_K6() * CV_CALIB_FIX_K6 +
                setFlags.getCV_CALIB_FIX_PRINCIPAL_POINT() * CV_CALIB_FIX_PRINCIPAL_POINT +
                setFlags.getCV_CALIB_RATIONAL_MODEL() * CV_CALIB_RATIONAL_MODEL +
                setFlags.getCV_CALIB_USE_INTRINSIC_GUESS() * CV_CALIB_USE_INTRINSIC_GUESS +
                setFlags.getCV_CALIB_ZERO_TANGENT_DIST() * CV_CALIB_ZERO_TANGENT_DIST;
            MultiCameraIntrinsicData& multiCameraIntrinsicData = *multiCameraIntrinsicData_;

            const std::map<POSE_NAME, bool> * poseFilter = nullptr;
            if (poseGraphAnalyzerResults.filterByPoseTolerance.size() > 0)
                poseFilter = &poseGraphAnalyzerResults.filterByPoseTolerance;

            if (initialMultiCameraIntrinsicData.cameraData.size() == 0)
                std::cout << "CameraIntrinsicComputation: NO INITIAL INTRINSICS PROVIDED.  RESULTS MAY BE VERY POOR" << std::endl;

            // Construct new MultiCameraIntrinsicData object
            multiCameraIntrinsicData = MultiCameraIntrinsicData(multiPoseObservations, flags, reprojThreshold, minimumPointsPerView, *arucoBoard_, &initialMultiCameraIntrinsicData, poseFilter);

            // Set timestamp of each document
            const std::string timestamp = Utilities::getCurrentTimeInSeconds();
            *timestamp_ = QString::fromStdString(timestamp);
            for (auto & intrinsicData : multiCameraIntrinsicData.cameraData)
                intrinsicData.second.timestamp = timestamp;

            return true;
        }
        catch (cv::Exception & e)
        {
            std::cout << "CameraIntrinsicComputation: " << e.what() << std::endl;
            return false;
        }
        catch (std::exception & e)
        {
            std::cout << "CameraIntrinsicComputation: " << e.what() << std::endl;
            return false;
        }
        catch (...)
        {
            std::cout << "CameraIntrinsicComputation: FAILED" << std::endl;
            return false;
        }
	}

	/**
	 *
	 */
	CameraIntrinsicComputation::CameraIntrinsicComputation() :
		CSIRO::DataExecution::Operation(
		CSIRO::DataExecution::OperationFactoryTraits< CameraIntrinsicComputation >::getInstance(),
		tr("Camera Intrinsic Data Calibration"))
	{
		pImpl_ = new CameraIntrinsicComputationImpl(*this);
	}

	/**
	 *
	 */
	CameraIntrinsicComputation::~CameraIntrinsicComputation()
	{
		delete pImpl_;
	}

	/**
	 *
	 */
	bool  CameraIntrinsicComputation::execute()
	{
		return pImpl_->execute();
	}
}


using namespace BoeingMetrology;
DEFINE_WORKSPACE_OPERATION_FACTORY_NAMED(
	CameraIntrinsicComputation,
	BoeingMetrologyPlugin::getInstance(),
	CSIRO::DataExecution::Operation::tr("BoeingMetrology"),
	"CameraIntrinsicComputation")

