/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    RouterInterface.cpp
*  @author  Tom Childress
*
*  @brief Implementation file for RouterInterface class
*
*****************************************************************************/
#include <map>
#include <string>
#include "RouterInterface.h"
#include "RouterWriteToFile.h"
#include "RouterWriteToZMQ.h"
using namespace std;

namespace BoeingMetrology
{
    namespace Communication
    {

        const string RouterInterface::WRITE_TO_FILE_ROUTER_LABEL = "WriteToFile";
        const string RouterInterface::WRITE_TO_ZMQ_ROUTER_LABEL = "WriteToZMQ";
        const string RouterInterface::ANNOUNCE_ZMQ_ROUTER_LABEL = "SendNotificationZMQ";

        map<string, uint32_t> RouterInterface::routerLabelMap =
        {
            { WRITE_TO_FILE_ROUTER_LABEL, WRITE_TO_FILE_ROUTER },
            { WRITE_TO_ZMQ_ROUTER_LABEL, WRITE_TO_ZMQ_ROUTER },
            { ANNOUNCE_ZMQ_ROUTER_LABEL, ANNOUNCE_ZMQ_ROUTER }
        };

        RouterInterface::RouterInterface()
        {
        }

        RouterInterface::~RouterInterface()
        {
        }

		string RouterInterface::getRouteInfo(Json::Value destinationInfo)
		{
			return "Routing information undefined";
		}

		RouterInterface* RouterInterface::createRouter(const string routeID_Label)
        {
            RouterInterface *retVal = nullptr;
            //make sure this is a valid route ID label
            if (routerLabelMap.count(routeID_Label) > 0)
            {
                const uint32_t routeID = routerLabelMap[routeID_Label];
                switch (routeID)
                {
                    case WRITE_TO_FILE_ROUTER:
                        retVal = new RouterWriteToFile();
                        break;
                    case WRITE_TO_ZMQ_ROUTER:
                        retVal = new RouterWriteToZMQ();
                        break;
                    case ANNOUNCE_ZMQ_ROUTER:
                    default:
                        break;
                }
            }
            return retVal;
        }
    }
}
