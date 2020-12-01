/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    RouterWriteToZMQ.h
*  @author  Tom Childress
*
*  @brief Class derived from RouterInterface that provides unique implementation
*         of route() method.
*
*****************************************************************************/
#pragma once
#include <string>
#include <vector>
#include "RouterInterface.h"
#include "json/value.h"

namespace BoeingMetrology
{
    namespace Communication
    {
        class RouterWriteToZMQ : public RouterInterface
        {
            public:
                /************************************************************************
                @brief Constructor

                This is the default constuctor
                ************************************************************************/
                RouterWriteToZMQ();
                /************************************************************************
                @brief Destructor

                This is the default destructor
                ************************************************************************/
                virtual ~RouterWriteToZMQ();
                /************************************************************************
                @brief Implements unique routing behavior
                @param[in] destinationInfo Information specific to this router

                This method implements the abstract interface of RouterInterface.
                Specifically, this routes data to a ZMQ message
                ************************************************************************/
                virtual void route(Json::Value destinationInfo, vector<uint8_t> & data);

            protected:

                static const int RECEIVE_TIMEOUT_MS;
                static const string OK_RESPONSE;
        };
    }
}

