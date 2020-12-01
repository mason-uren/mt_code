/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    RoutingTask.h
*  @author  Tom Childress
*
*  @brief An individual routing task
*
*****************************************************************************/
#pragma once
#include "BoeingMetrologyLib_API.h"
#include "json/value.h"

namespace BoeingMetrology
{
    namespace Communication
    {
        class BOEINGMETROLOGYLIB_API RoutingTask
        {
            public:
                /************************************************************************
                @brief Constructor

                This is the default constructor
                ************************************************************************/
                RoutingTask() = default;
                /************************************************************************
                @brief Constructor
                @param[in] taskInfo Specific information to be associated with this task 

                This constructor accepts and stores information associated with a task
                ************************************************************************/
                RoutingTask(Json::Value taskInfo);
                /************************************************************************
                @brief Destructor

                This is the default destructor
                ************************************************************************/
                virtual ~RoutingTask() = default;
                /************************************************************************
                @brief Copy constructor
                @param[in] other Instance to perform deep copy of

                This constructor constructs a copy of the provided object
                ************************************************************************/
                RoutingTask(const RoutingTask &other);
                /************************************************************************
                @brief Assignment operator
                @param[in] rhs Instance to perform deep copy of
                @return Reference to this object

                This constructor assigns a copy of the provided object
                ************************************************************************/
                RoutingTask& operator=(const RoutingTask &rhs);
                /************************************************************************
                @brief Accessor

                This is single accessor for the underlying routing information associated
                with the task
                ************************************************************************/
                virtual Json::Value getInfo() const;


            protected:
                //underylying task information
                Json::Value taskInfo;
        };
    }
}
