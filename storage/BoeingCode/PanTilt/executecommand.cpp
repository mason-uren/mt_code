#include <cassert>

#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/InputOutput/inputarray.h"
#include "Workspace/DataExecution/InputOutput/simpleoperationio.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"


#include "pantiltplugin.h"
#include "executecommand.h"
#include "TelemetricClient/PanTiltController.h"

namespace BoeingMetrology
{
    /**
     * \internal
     */
    class ExecuteCommandImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(BoeingMetrology::ExecuteCommandImpl)

    public:
        ExecuteCommand&  op_;

        // Data objects


        // Inputs and outputs
		CSIRO::DataExecution::SimpleInput< QString > panTiltIpAddress_;
		CSIRO::DataExecution::SimpleInput< QString > clientIpAddress_;
		CSIRO::DataExecution::SimpleInput< QString > tcpPort_;
		CSIRO::DataExecution::SimpleInput< QString > udpPort_;
        CSIRO::DataExecution::SimpleInput< QString > command_;
        CSIRO::DataExecution::SimpleOutput< QString > results_;


        ExecuteCommandImpl(ExecuteCommand& op);

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    ExecuteCommandImpl::ExecuteCommandImpl(ExecuteCommand& op) :
        op_(op),
		panTiltIpAddress_("PanTiltIpAddress", op_),
		clientIpAddress_("ClientIpAddress", op_),
		tcpPort_("TcpPort", op_),
		udpPort_("UdpPort", op_),
        command_("Command",  op_),
        results_("Results",  op_)
    {
        // Make sure all of our inputs have data by default. If your operation accepts a
        // large data structure as input, you may wish to remove this call and replace it
        // with constructors for each input in the initialisation list above.
        op_.ensureHasData();

        // Recommend setting a description of the operation and each input / output here:
        // op_.setDescription(tr("My operation does this, that and this other thing."));
		command_.input_.setDescription(tr("e.g. 'p t'"));
        // output1_.output_.setDescription(tr("Results of the blah-di-blah."));
    }


    /**
     *
     */
    bool ExecuteCommandImpl::execute()
    {
		std::string tcpPort = (*tcpPort_).toUtf8().constData();
		std::string udpPort = (*udpPort_).toUtf8().constData();
		std::string panTiltIpAddress = (*panTiltIpAddress_).toUtf8().constData();
		std::string clientIpAddress = (*clientIpAddress_).toUtf8().constData();
		std::string command = (*command_).toUtf8().constData() + '\r';
        QString& results   = *results_;
        
		bool success;
		PanTiltController panTiltUnit(panTiltIpAddress, clientIpAddress, tcpPort, udpPort, success);

		//command = "p t\r";
		char *sendbuf = (char*)(command).c_str();

		std::cout << "Sent: " << command << std::endl;

		std::string bufferString = panTiltUnit.Execute(sendbuf, success);
		results = QString::fromStdString(bufferString);

		if (success) return true;
		else return false;
    }


    /**
     *
     */
    ExecuteCommand::ExecuteCommand() :
        CSIRO::DataExecution::Operation(
            CSIRO::DataExecution::OperationFactoryTraits< ExecuteCommand >::getInstance(),
            tr("ExecuteCommand"))
    {
        pImpl_ = new ExecuteCommandImpl(*this);
    }


    /**
     *
     */
    ExecuteCommand::~ExecuteCommand()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  ExecuteCommand::execute()
    {
        return pImpl_->execute();
    }
}


using namespace BoeingMetrology;
DEFINE_WORKSPACE_OPERATION_FACTORY(ExecuteCommand, 
                                   BoeingMetrology::PanTiltPlugin::getInstance(),
                                   CSIRO::DataExecution::Operation::tr("PanTilt"))

