/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    RouterWriteToFile.h
*  @author  Tom Childress
*
*  @brief Class derived from RouterInterface that provides unique implementation
*         of route() method.
*
*****************************************************************************/

#pragma once
#include "RouterInterface.h"
#include "json/value.h"

namespace BoeingMetrology
{
    namespace Communication
    {
        class RouterWriteToFile : public RouterInterface
        {
            public:

                /************************************************************************
                @brief Constructor

                This is the default constuctor
                ************************************************************************/
                RouterWriteToFile();
                /************************************************************************
                @brief Destructor

                This is the default destructor
                ************************************************************************/
                virtual ~RouterWriteToFile();
                /************************************************************************
                @brief Implements unique routing behavior
                @param[in] destinationInfo Information specific to this router

                This method implements the abstract interface of RouterInterface.
                Specifically, this routes data to a file
                ************************************************************************/
                virtual void route(Json::Value destinationInfo, vector<uint8_t> &data);
				/************************************************************************
				@brief Method that retrieves a string form of the provided routing task
				@param[in] destinationInfo Information specific to unique implementation of this method

				This method implements the abstract interface for getting routing info
				************************************************************************/
				virtual string getRouteInfo(Json::Value destinationInfo);
        };
    }
}

