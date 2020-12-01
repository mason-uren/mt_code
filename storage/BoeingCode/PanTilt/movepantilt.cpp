#include <cassert>

#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/InputOutput/inputarray.h"
#include "Workspace/DataExecution/InputOutput/simpleoperationio.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"


#include "pantiltplugin.h"
#include "movepantilt.h"
#include "TelemetricClient/PanTiltController.h"
#include <iostream>


namespace BoeingMetrology
{
    /**
     * \internal
     */
    class MovePanTiltImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(BoeingMetrology::MovePanTiltImpl)

    public:
        MovePanTilt&  op_;

        // Data objects


        // Inputs and outputs
		CSIRO::DataExecution::SimpleInput< QString > panTiltIpAddress_;
		CSIRO::DataExecution::SimpleInput< QString > clientIpAddress_;
		CSIRO::DataExecution::SimpleInput< QString > tcpPort_;
		CSIRO::DataExecution::SimpleInput< QString > udpPort_;
		CSIRO::DataExecution::SimpleInput< float > panAngle_;
		CSIRO::DataExecution::SimpleInput< float > tiltAngle_;
        CSIRO::DataExecution::SimpleInput< int > angleRate_;
        CSIRO::DataExecution::SimpleInput< float > positionTolerance_;
		CSIRO::DataExecution::SimpleOutput< float > outputPanAngle_;
		CSIRO::DataExecution::SimpleOutput< float > outputTiltAngle_;
		CSIRO::DataExecution::SimpleOutput< QString > outputAngleStr_;


        MovePanTiltImpl(MovePanTilt& op);
		PanTiltController * panTiltUnit = nullptr;

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    MovePanTiltImpl::MovePanTiltImpl(MovePanTilt& op) :
        op_(op),
		panTiltIpAddress_("PanTiltIpAddress", op_),
		clientIpAddress_("ClientIpAddress", op_),
		tcpPort_("TcpPort", op_),
		udpPort_("UdpPort", op_),
		panAngle_("PanAngle", op_),
		tiltAngle_("TiltAngle", op_),
        angleRate_("AngleRate", op_),
        positionTolerance_("PositionTolerance", op_),
		outputPanAngle_("OutputPanAngle", op_),
		outputTiltAngle_("OutputTiltAngle", op_),
		outputAngleStr_("OutputAngleString", op_)
    {
        // Make sure all of our inputs have data by default. If your operation accepts a
        // large data structure as input, you may wish to remove this call and replace it
        // with constructors for each input in the initialisation list above.
        op_.ensureHasData();

        // Recommend setting a description of the operation and each input / output here:
        op_.setDescription(tr("Control the motion of the pan tilt unit. Encoder value of 16383 is zero movement, +/- controls speed and direction."));
        // input1_.input_.setDescription(tr("Used for such and such."));
        // output1_.output_.setDescription(tr("Results of the blah-di-blah."));
    }

    /**
     *
     */
    bool MovePanTiltImpl::execute()
    {
		std::string tcpPort = (*tcpPort_).toUtf8().constData();
		std::string udpPort = (*udpPort_).toUtf8().constData();
		std::string panTiltIpAddress = (*panTiltIpAddress_).toUtf8().constData();
		std::string clientIpAddress = (*clientIpAddress_).toUtf8().constData();
		const float& panAngle = *panAngle_;
		const float& tiltAngle = *tiltAngle_;
		float& outputPanAngle = *outputPanAngle_;
		float& outputTiltAngle = *outputTiltAngle_;
        int& angleRate = *angleRate_;
        float& positionTolerance = *positionTolerance_;
		QString& outputAngleStr = *outputAngleStr_;
		double DEGPERCOUNT = 0.00045;

		bool success = true;
		if (panTiltUnit == nullptr)
		{
			panTiltUnit = new PanTiltController(panTiltIpAddress, clientIpAddress, tcpPort, udpPort, success);
		}

		if (success)
		{
			panTiltUnit->SetUdpPort(udpPort);
			panTiltUnit->SetAngles(panAngle, tiltAngle, outputPanAngle, outputTiltAngle, angleRate, positionTolerance);
            std::string temp = std::to_string(outputPanAngle) + "_" + std::to_string(outputTiltAngle);
			outputAngleStr = QString::fromStdString (temp);
			return true;
		}
		else
		{
			return false;
		}
    }


    /**
     *
     */
    MovePanTilt::MovePanTilt() :
        CSIRO::DataExecution::Operation(
            CSIRO::DataExecution::OperationFactoryTraits< MovePanTilt >::getInstance(),
            tr("MovePanTilt"))
    {
        pImpl_ = new MovePanTiltImpl(*this);
    }


    /**
     *
     */
    MovePanTilt::~MovePanTilt()
    {
		delete pImpl_->panTiltUnit;
        delete pImpl_;
    }


    /**
     *
     */
    bool  MovePanTilt::execute()
    {
        return pImpl_->execute();
    }
}


using namespace BoeingMetrology;
DEFINE_WORKSPACE_OPERATION_FACTORY(MovePanTilt, 
                                   BoeingMetrology::PanTiltPlugin::getInstance(),
                                   CSIRO::DataExecution::Operation::tr("PanTilt"))

