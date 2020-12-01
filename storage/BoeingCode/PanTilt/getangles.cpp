#include <cassert>

#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/InputOutput/inputarray.h"
#include "Workspace/DataExecution/InputOutput/simpleoperationio.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"


#include "pantiltplugin.h"
#include "getangles.h"
#include "TelemetricClient/PanTiltController.h"
#include <iostream>



namespace BoeingMetrology
{
    /**
     * \internal
     */
    class GetAnglesImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(BoeingMetrology::GetAnglesImpl)

    public:
        GetAngles&  op_;

        // Data objects


        // Inputs and outputs
		CSIRO::DataExecution::SimpleInput< QString > panTiltIpAddress_;
		CSIRO::DataExecution::SimpleInput< QString > clientIpAddress_;
		CSIRO::DataExecution::SimpleInput< QString > tcpPort_;
		CSIRO::DataExecution::SimpleInput< QString > udpPort_;
		CSIRO::DataExecution::SimpleOutput< float > panAngle_;
		CSIRO::DataExecution::SimpleOutput< float > tiltAngle_;


        GetAnglesImpl(GetAngles& op);
		PanTiltController * panTiltUnit = nullptr;

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    GetAnglesImpl::GetAnglesImpl(GetAngles& op) :
        op_(op),
		panTiltIpAddress_("PanTiltIpAddress", op_),
		clientIpAddress_("ClientIpAddress", op_),
		tcpPort_("TcpPort", op_),
		udpPort_("UdpPort", op_),
		panAngle_("panAngle", op_),
        tiltAngle_("tiltAngle",  op_)
    {
        // Make sure all of our inputs have data by default. If your operation accepts a
        // large data structure as input, you may wish to remove this call and replace it
        // with constructors for each input in the initialisation list above.
        op_.ensureHasData();

        // Recommend setting a description of the operation and each input / output here:
        // op_.setDescription(tr("My operation does this, that and this other thing."));
        // input1_.input_.setDescription(tr("Used for such and such."));
        // output1_.output_.setDescription(tr("Results of the blah-di-blah."));
    }

	template<typename Out>
	void split(const std::string &s, char delim, Out result) {
		std::stringstream ss;
		ss.str(s);
		std::string item;
		while (std::getline(ss, item, delim)) {
			*(result++) = item;
		}
	}

	std::vector<std::string> split(const std::string &s, char delim) {
		std::vector<std::string> elems;
		split(s, delim, std::back_inserter(elems));
		return elems;
	}

    /**
     *
     */
    bool GetAnglesImpl::execute()
    {
		std::string tcpPort = (*tcpPort_).toUtf8().constData();
		std::string udpPort = (*udpPort_).toUtf8().constData();
		std::string panTiltIpAddress = (*panTiltIpAddress_).toUtf8().constData();
		std::string clientIpAddress = (*clientIpAddress_).toUtf8().constData();
		float& panAngle = *panAngle_;
        float& tiltAngle = *tiltAngle_;
		double DEGPERCOUNT = 0.00045;       

		bool success = true;
		if (panTiltUnit == nullptr)
		{
			panTiltUnit = new PanTiltController(panTiltIpAddress, clientIpAddress, tcpPort, udpPort, success);
		}

		if (success)
		{
			success = panTiltUnit->GetAngles(panAngle, tiltAngle);
			if (success)
			{
				return true;
			}
			else
			{
				std::cout << "Error parsing angles from UDP stream!" << std::endl;
				return false;
			}
		}
		else
		{
			std::cout << "Error initializing the PanTilt Connection!" << std::endl;
			return false;
		}
    }


    /**
     *
     */
    GetAngles::GetAngles() :
        CSIRO::DataExecution::Operation(
            CSIRO::DataExecution::OperationFactoryTraits< GetAngles >::getInstance(),
            tr("GetAngles"))
    {
        pImpl_ = new GetAnglesImpl(*this);
    }


    /**
     *
     */
    GetAngles::~GetAngles()
    {
		delete pImpl_->panTiltUnit;
        delete pImpl_;
    }


    /**
     *
     */
    bool  GetAngles::execute()
    {
        return pImpl_->execute();
    }
}


using namespace BoeingMetrology;
DEFINE_WORKSPACE_OPERATION_FACTORY(GetAngles, 
                                   BoeingMetrology::PanTiltPlugin::getInstance(),
                                   CSIRO::DataExecution::Operation::tr("PanTilt"))

