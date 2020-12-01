/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    RoutingTask.cpp
*  @author  Tom Childress
*
*  @brief Implementation file for RoutingTask class
*
*****************************************************************************/
#include "RoutingTask.h"
#include "json/value.h"

namespace BoeingMetrology
{
    namespace Communication
    {
        RoutingTask::RoutingTask(Json::Value taskInfo)
        {
            this->taskInfo = taskInfo;
        }

        RoutingTask::RoutingTask(const RoutingTask &other)
        {
            this->taskInfo = other.taskInfo;
        }

        RoutingTask& RoutingTask::operator=(const RoutingTask &rhs)
        {
            this->taskInfo = rhs.taskInfo;
            return *this;
        }

        Json::Value RoutingTask::getInfo() const
        {
            return taskInfo;
        }
    }
}