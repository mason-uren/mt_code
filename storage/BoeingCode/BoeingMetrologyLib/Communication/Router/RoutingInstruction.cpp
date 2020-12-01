/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    RoutingInstruction.cpp
*  @author  Tom Childress
*
*  @brief Implementation of the RoutingInstruction class
*
*****************************************************************************/
#include "RoutingInstruction.h"

namespace BoeingMetrology
{
    namespace Communication
    {
        void RoutingInstruction::JsonDeserialize(const Json::Value &jsonNode)
        {
            for (const Json::Value & ops : jsonNode)
            {
                RoutingTask routingTask(ops);
                tasks.push_back(routingTask);
            }
        }

        void RoutingInstruction::JsonSerialize(Json::Value &jsonNode) const
        {
            for (const RoutingTask &task : tasks)
            {
                Json::Value taskJson = task.getInfo();
                jsonNode.append(taskJson);
            }
        }

        bool RoutingInstruction::getNextTask(RoutingTask &task)
        {
            bool retVal = false;
            if (!tasks.empty())
            {
                task = tasks.front();
                retVal = true;
            }
            return retVal;
        }

        void RoutingInstruction::popNextTask()
        {
            if (!tasks.empty())
            {
                tasks.pop_front();
            }
        }
    }
}