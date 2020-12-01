/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    RouterInterface.h
*  @author  Tom Childress
*
*  @brief Abstract class that exposes the pure virtual route() method
*
*****************************************************************************/
#pragma once
#include <string>
#include <cstdint>
#include <vector>
#include <map>
#include <string>
#include "json/value.h"
using namespace std;

namespace BoeingMetrology
{
    namespace Communication
    {
        class RouterInterface
        {
            public:
                /************************************************************************
                @brief Constructor

                This is the default constuctor
                ************************************************************************/
                RouterInterface();
                /************************************************************************
                @brief Destructor

                This is the default destructor
                ************************************************************************/
                virtual ~RouterInterface();
                /************************************************************************
                @brief Pure virtual method that defines the routing interface
                @param[in] destinationInfo Information specific to unique implementation of this method

                This method exposes an abstract interface for routing data
                ************************************************************************/
                virtual void route(Json::Value destinationInfo, vector<uint8_t> &data) = 0;
				/************************************************************************
				@brief Method that retrieves a string form of the provided routing task
				@param[in] destinationInfo Information specific to unique implementation of this method

				This method exposes an abstract interface for getting routing info
				************************************************************************/
				virtual string getRouteInfo(Json::Value destinationInfo);
                /************************************************************************
                @brief Constructs unique derived instances of RouterInterface
                @param[in] routeID_Label Label used to determine which instance to return

                This static method serves as the factory interface for objects that implement the RouterInterface
                ************************************************************************/
                static RouterInterface* createRouter(string routeID_Label);

                //Specific labels that map to routerID
                static const string WRITE_TO_FILE_ROUTER_LABEL;
                static const string WRITE_TO_ZMQ_ROUTER_LABEL;
                static const string ANNOUNCE_ZMQ_ROUTER_LABEL;

            protected:

                //route IDs
                enum
                {
                    WRITE_TO_FILE_ROUTER,
                    WRITE_TO_ZMQ_ROUTER,
                    ANNOUNCE_ZMQ_ROUTER,
                    NUM_ROUTERS
                };
                //map for labels to ids
                static map<string, uint32_t> routerLabelMap;
        };
    }
}

