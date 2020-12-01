/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    RoutingInstruction.h
*  @author  Tom Childress
*
*  @brief An individual routing instruction that contains 1 or more RoutingTasks
*
*****************************************************************************/
#pragma once
#include <cstdint>
#include <string>
#include <mutex>
#include <deque>
#include "RoutingTask.h"
#include "Common/Interface/Serializer.h"
using namespace std;

namespace BoeingMetrology
{
    namespace Communication
    {
        class BOEINGMETROLOGYLIB_API RoutingInstruction : public Boeing::Interface::Serializer
        {
        public:
            /************************************************************************
            @brief Constructor

            This is the default constructor
            ************************************************************************/
            RoutingInstruction() = default;
            /************************************************************************
            @brief Destructor

            This is the default destructor
            ************************************************************************/
            virtual ~RoutingInstruction() = default;
            /************************************************************************
            @brief Deserialization method that populates this object with provided info.
            @param[in] jsonNode Contains object state

            This method populates this object with the information contained inside
            the provided JSON object.
            ************************************************************************/
            virtual void JsonDeserialize(const Json::Value &jsonNode) override;
            /************************************************************************
            @brief Serialization method that populates the provide JSON object with
            the state of this instance.
            @param[out] jsonNode Contains object state

            This method populates JSON Object with the information contained inside
            this class instance.
            ************************************************************************/
            virtual void JsonSerialize(Json::Value &jsonNode) const override;
            /************************************************************************
            @brief Retrieves the next task in the queue
            @param[out] task Task object to populate

            This method populates a task object with the next task in the queue.
            ************************************************************************/
            virtual bool getNextTask(RoutingTask &task);
            /************************************************************************
            @brief Removes the next task in the queue

            This method pops the next task out of the queue.
            ************************************************************************/
            virtual void popNextTask();

        protected:
            //queue type
            typedef deque<RoutingTask> RoutingTaskQueueType;
            //queue of tasks
            RoutingTaskQueueType tasks;
        };
    }
}