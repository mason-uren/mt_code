/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    RouterManager.h
*  @author  Tom Childress
*
*  @brief Class that serves as the handler for routing instructions and how 
*         to route data based on those instructions
*
*****************************************************************************/
#pragma once
#include <map>
#include <deque>
#include <mutex>
#include <string>
#include "RouterInterface.h"
#include "FormatterInterface.h"
#include "RoutingInstruction.h"
#include "BoeingMetrologyLib_API.h"
#include "json/value.h"
using namespace std;

namespace BoeingMetrology
{
    namespace Communication
    {
        class BOEINGMETROLOGYLIB_API RouterManager
        {
            public:
                /************************************************************************
                @brief Constructor

                This is the default constuctor
                ************************************************************************/
				RouterManager() = default;
                /************************************************************************
                @brief Destructor

                This is the default destructor
                ************************************************************************/
				virtual ~RouterManager() = default;
                /************************************************************************
                @brief Copy constructor

                This is the default copy constructor
                ************************************************************************/
                RouterManager(const RouterManager &other);
                /************************************************************************
                @brief Assignment operator

                This is the default assigment operator definition
                ************************************************************************/
                RouterManager& operator=(const RouterManager &rhs);
                /************************************************************************
                @brief Initial setup

                This method sets up the router and formatter tables
                ************************************************************************/
                virtual void setupTables();
                /************************************************************************
                @brief Release resources

                This method frees up the router and formatter tables
                ************************************************************************/
                virtual void releaseTables();
                /************************************************************************
                @brief Pushes instruction to back of queue
                @param[in] instruction Routing information

                This method is intended to be called when a new instruction needs to be
                added to the route instruction queue. Calls to this method are thread safe.
                ************************************************************************/
                virtual void pushNewInstruction(RoutingInstruction &instruction);
                /************************************************************************
                @brief Pops instruction on front of queue and returns it
                @return Returns next routing instruction

                This method pulls the next instruction on the queue, then pops it off the
                queue. Calls to this method are thread safe.
                ************************************************************************/
                virtual bool getNextInstruction(RoutingInstruction &instruction);
                /************************************************************************
                @brief Pops instruction off front of queue

                This method pops the next instruction off the queue. Calls to this method
                are thread safe.
                ************************************************************************/
                virtual void popInstruction();
                /************************************************************************
                @brief Determines how to route the provided data
                @param[in] routerID_Label Used to determine which router to use from router table
                @param[in] destinationInfo Information specific to the chosen router
                @param[in] data Data that is to be routed

                This method makes the determination on how to route the provided data
                ************************************************************************/
                virtual void routeData(string routerID_Label, Json::Value destinationInfo, vector<uint8_t> &data);
                /************************************************************************
                @brief Determines how to format the provided data
                @param[in] formatterID_Label Used to determine which formatter to use from formatter table
                @param[in] inData Data to format
                @param[in] outData Data that has been formatted

                This method makes the determination on how to format the provided data
                ************************************************************************/
                virtual void formatData(string formatterID_Label, FormatterInterface::ImageType &inData, FormatterInterface::ImageType &outData);
                
                /* @brief Log message category type */
                typedef enum
                {
                    LOG_INFO,
                    LOG_DEBUG,
                    LOG_ERROR,
                    NUM_LOG_TYPES
                } LogMessageCategoryType;

                /* @brief Log message type */
                typedef struct
                {
                    LogMessageCategoryType type;
                    string contents;
                } LogMessageType;
                /************************************************************************
                @brief Get next log
                @param[out] logMessage message to populate
                @return If log is not empty

                This method retrieves the next log on the log queue
                ************************************************************************/
                virtual bool getNextLog(LogMessageType &logMessage);
                /************************************************************************
                @brief Pop next log

                This method removes the next log from the log queue
                ************************************************************************/
                virtual void popNextLog();
                /************************************************************************
                @brief Clear the instruction queue

                This method clears the instruction queue
                ************************************************************************/
                virtual void clearInstructionQueue();

            protected:
                /************************************************************************
                @brief Sets up the router table

                This method makes creation calls to RouterInterface in order to populate
                its router table
                ************************************************************************/
                virtual void createRouters();
                /************************************************************************
                @brief Sets up formatter table

                This method makes creation calls to FormatterInterface in order to populate
                its router table
                ************************************************************************/
                virtual void createFormatters();
                /************************************************************************
                @brief Deletes routers from router table

                This method frees the router objects contained in the router table.
                ************************************************************************/
                virtual void deleteRouters();
                /************************************************************************
                @brief Deletes formatters from formatter table

                This method frees the formatter objects contained in the formatter table.
                ************************************************************************/
                virtual void deleteFormatters();
                /************************************************************************
                @brief Log data
                @param[in] type Severity type
                @param[in] message Contents of message

                This method allows for logging information of varying severity
                ************************************************************************/
                virtual void log(LogMessageCategoryType type, string message);

                typedef map<string, RouterInterface*> RouterTableType;
                typedef map<string, FormatterInterface*> FormatterTableType;
                typedef deque<RoutingInstruction> RoutingInstructionQueueType;
                //key is routerID ("task")
                RouterTableType routerTable;
                //key is formatterID ("format")
                FormatterTableType formatterTable;
                //routing instruction queue and associated mutex
                RoutingInstructionQueueType instructionQueue;
                mutex instructionQueueMutex;
                //log queue
                mutex logQueueMutex;
                typedef deque<LogMessageType> LogMessageQueueType;
                LogMessageQueueType logs;
        };
    }
}

