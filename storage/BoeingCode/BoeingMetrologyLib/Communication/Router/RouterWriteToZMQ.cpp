/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    RouterWriteToZMQ.cpp
*  @author  Tom Childress
*
*  @brief Implementation file for RouterWriteToZMQ class
*
*****************************************************************************/
#include <sstream>
#include "RouterWriteToZMQ.h"
#include "Communication/Router/RouterDefs.h"
#include "json/value.h"
#include "zmq.hpp"
using namespace std;

namespace BoeingMetrology
{
    namespace Communication
    {
        const int RouterWriteToZMQ::RECEIVE_TIMEOUT_MS = 1000;

        const string RouterWriteToZMQ::OK_RESPONSE = "OK";

        RouterWriteToZMQ::RouterWriteToZMQ()
        {
        }

        RouterWriteToZMQ::~RouterWriteToZMQ()
        {
        }

        void RouterWriteToZMQ::route(Json::Value destinationInfo, vector<uint8_t> & data)
        {
            //make sure we have the info we need
            if (destinationInfo.isMember(Router::ADDRESS_KEY) && destinationInfo.isMember(Router::PORT_KEY))
            {
                string address = destinationInfo[Router::ADDRESS_KEY].asString();
                int port = destinationInfo[Router::PORT_KEY].asInt();
                //set up socket
                zmq::context_t context(1);
                zmq::socket_t socket(context, ZMQ_REQ);
                socket.setsockopt(ZMQ_RCVTIMEO, &RECEIVE_TIMEOUT_MS, sizeof(RECEIVE_TIMEOUT_MS));
                //build connection string
                stringstream ss;
                ss << "tcp://" << address << ":" << port;
                socket.connect(ss.str());
                //build and send data
                zmq::message_t sendMessage(data.data(), data.size());
                socket.send(sendMessage);
                //wait for reply
                zmq::message_t receiveMessage;
                socket.recv(&receiveMessage);
            }
            else
            {
                throw runtime_error("Address and/or Port key missing from destination info!");
            }
        }
    }
}
