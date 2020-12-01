#include <cassert>

#include "Workspace/Application/textlogger.h"
#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/InputOutput/inputarray.h"
#include "Workspace/DataExecution/InputOutput/simpleoperationio.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"

#include "ExtrinsicCalibrationFromPointsOperation.h"
#include "Calibration/ExtrinsicCalibrationFromPoints.h"
#include <iostream>
#include "Utilities/Utilities.h"
#include <fstream>
#include "Calibration/MultiCameraExtrinsicData.h"
#include "Calibration/Pose/MultiPoseGraph.h"

namespace BoeingMetrology
{
    using namespace CSIRO::DataExecution;

    using namespace BoeingMetrology::Calibration;

    /**
    * \internal
    */
    class ExtrinsicCalibrationFromPointsOperationImpl : public CSIRO::Application::TextLogger
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(BoeingMetrology::ExtrinsicCalibrationFromPointsOperationImpl)

    public:

        ExtrinsicCalibrationFromPointsOperation&  op_;

        // Inputs and outputs
        SimpleInput<QString> cameraRegex_;
        SimpleInput<QString> referenceCamera_;
        SimpleInputOutput< Calibration::Pose::MultiPoseGraph > poseGraph_;
        SimpleOutput< MultiCameraExtrinsicData > outputExtrinsics_;
        SimpleOutput< double > reprojectionError_;
        SimpleOutput< QString> timestamp_;

        ExtrinsicCalibrationFromPointsOperationImpl(ExtrinsicCalibrationFromPointsOperation& op);

        bool  execute();

    };


    /**
    *
    */
    ExtrinsicCalibrationFromPointsOperationImpl::ExtrinsicCalibrationFromPointsOperationImpl(ExtrinsicCalibrationFromPointsOperation& op) :
        op_(op),
        poseGraph_("MultiPoseGraph Instance", op_),
        cameraRegex_("RegEx of Locked Cameras", op_),
        referenceCamera_("Reference Camera Name", op_),
        outputExtrinsics_("Output Extrinsics", op_),
        reprojectionError_("Reprojection error", op_),
        timestamp_("Time Stamp", op_)
    {
        // Make sure all of our inputs have data by default. If your operation accepts a
        // large data structure as input, you may wish to remove this call and replace it
        // with constructors for each input in the initialisation list above.
        op_.ensureHasData();

        // Recommend setting a description of the operation and each input / output here:
        op_.setDescription(tr("Compute extrinsics for multiple cameras."));
        poseGraph_.input_.setDescription(tr("An instance of MultiPoseGraph.  It will be modified in place and output."));
        cameraRegex_.input_.setDescription(tr("A regular expression string used to match camera names that should be locked.  E.g. \"(Nikon).(*)\" Will lock all Nikons."));
        referenceCamera_.input_.setDescription(tr("The name of the camera to use as the reference camera."));
        outputExtrinsics_.output_.setDescription(tr("An instance of MultiCameraExtrinsicsData."));
        timestamp_.output_.setDescription(tr("A timestamp in the format YYYYMMDD_HHMMSS."));
    }

    /**
    *
    */
    bool ExtrinsicCalibrationFromPointsOperationImpl::execute()
    {
        Pose::MultiPoseGraph& poseGraph = *poseGraph_;
        MultiCameraExtrinsicData& outputExtrinsics = *outputExtrinsics_;

        try
        {
            // Construct calibration object
            BoeingMetrology::ExtrinsicCalibrationFromPoints calib(poseGraph);

            // Verify that the reference camera requested is in the pose graph
            const auto & camNames = poseGraph.GetCameraNames();
            if (std::find(camNames.begin(), camNames.end(), referenceCamera_->toStdString()) == camNames.end())
            {
                std::cout << "ERROR: ExtrinsicCalibrationFromPoints: Invalid reference camera input " << referenceCamera_->toStdString() << std::endl;
                return false;
            }

            // Determine the cameras that should be locked
            std::vector<std::string> lockedCameraNames;
            for (const auto & name : poseGraph.GetCameraNames())
            {
                if (cameraRegex_->toStdString() != "" && BoeingMetrology::Utilities::RegexMatch(cameraRegex_->toStdString(), name))
                    lockedCameraNames.push_back(name);
            }

            // Run calibration
            *reprojectionError_ = calib.Calibrate(outputExtrinsics, lockedCameraNames);

            // Update reference camera
            outputExtrinsics.RedefineReferenceCamera(referenceCamera_->toStdString());

            // Set timestamp of each document
            const std::string timestamp = Utilities::getCurrentTimeInSeconds();
            *timestamp_ = QString::fromStdString(timestamp);
            for (auto & extrinsicData : outputExtrinsics.cameraData)
                extrinsicData.second.timestamp = timestamp;

            return true;
        }
        catch (std::exception & e)
        {
            std::cout << "ERROR: ExtrinsicCalibrationFromPoints exception - " << e.what() << std::endl;
            return false;
        }
        catch (...)
        {
            std::cout << "ERROR: ExtrinsicCalibrationFromPoints - unknown exception." << std::endl;
            return false;
        }
    }


    /**
    *
    */
    ExtrinsicCalibrationFromPointsOperation::ExtrinsicCalibrationFromPointsOperation() :
        CSIRO::DataExecution::Operation(
        CSIRO::DataExecution::OperationFactoryTraits< ExtrinsicCalibrationFromPointsOperation >::getInstance(),
        tr("ExtrinsicCalibrationFromPoints"))
    {
        pImpl_ = new ExtrinsicCalibrationFromPointsOperationImpl(*this);
    }


    /**
    *
    */
    ExtrinsicCalibrationFromPointsOperation::~ExtrinsicCalibrationFromPointsOperation()
    {
        delete pImpl_;
    }


    /**
    *
    */
    bool  ExtrinsicCalibrationFromPointsOperation::execute()
    {
        return pImpl_->execute();
    }
}



using namespace BoeingMetrology;
DEFINE_WORKSPACE_OPERATION_FACTORY_NAMED(ExtrinsicCalibrationFromPointsOperation,
    BoeingMetrologyPlugin::getInstance(),
    CSIRO::DataExecution::Operation::tr("BoeingMetrology"), "ExtrinsicCalibrationFromPoints")

