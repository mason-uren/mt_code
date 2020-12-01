/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    RouterManager.cpp
*  @author  Tom Childress
*
*  @brief Implementation file for RouterManager class
*
*****************************************************************************/
#include <string>
#include <sstream>
#include <exception>
#include "RouterManager.h"
#include "RouterInterface.h"
#include "FormatterInterface.h"
#include "RoutingInstruction.h"
#include "json/value.h"
using namespace std;

namespace BoeingMetrology
{
    namespace Communication
    {
        RouterManager::RouterManager(const RouterManager &other)
        {
            this->routerTable = other.routerTable;
            this->formatterTable = other.formatterTable;
            this->instructionQueue = other.instructionQueue;
        }

        RouterManager& RouterManager::operator=(const RouterManager &rhs)
        {
            this->routerTable = rhs.routerTable;
            this->formatterTable = rhs.formatterTable;
            this->instructionQueue = rhs.instructionQueue;
            return *this;
        }

        void RouterManager::setupTables()
        {
            //set up tables
            createRouters();
            createFormatters();
        }

        void RouterManager::releaseTables()
        {
            //free up tables
            deleteRouters();
            deleteFormatters();
        }

        bool RouterManager::getNextLog(LogMessageType & logMessage)
        {
            lock_guard<mutex> guard(logQueueMutex);
            bool retVal = false;
            if (!logs.empty())
            {
                logMessage = logs.front();
                retVal = true;
            }
            return retVal;
        }

        void RouterManager::popNextLog()
        {
            lock_guard<mutex> guard(logQueueMutex);
            if (!logs.empty())
            {
                logs.pop_front();
            }
        }

        void RouterManager::clearInstructionQueue()
        {
            lock_guard<mutex> guard(instructionQueueMutex);
            instructionQueue.clear();
        }

        void RouterManager::createRouters()
        {
            routerTable[RouterInterface::WRITE_TO_FILE_ROUTER_LABEL] = RouterInterface::createRouter(RouterInterface::WRITE_TO_FILE_ROUTER_LABEL);
            routerTable[RouterInterface::WRITE_TO_ZMQ_ROUTER_LABEL] = RouterInterface::createRouter(RouterInterface::WRITE_TO_ZMQ_ROUTER_LABEL);
            routerTable[RouterInterface::ANNOUNCE_ZMQ_ROUTER_LABEL] = RouterInterface::createRouter(RouterInterface::ANNOUNCE_ZMQ_ROUTER_LABEL);
        }

        void RouterManager::createFormatters()
        {
            formatterTable[FormatterInterface::JPEG_TO_MAT_FORMATTER_LABEL] = FormatterInterface::createFormatter(FormatterInterface::JPEG_TO_MAT_FORMATTER_LABEL);
            formatterTable[FormatterInterface::JPEG_TO_PNG_FORMATTER_LABEL] = FormatterInterface::createFormatter(FormatterInterface::JPEG_TO_PNG_FORMATTER_LABEL);
			formatterTable[FormatterInterface::RAW_TO_MAT_FORMATTER_LABEL] = FormatterInterface::createFormatter(FormatterInterface::RAW_TO_MAT_FORMATTER_LABEL);
            formatterTable[FormatterInterface::RAW_TO_PNG_FORMATTER_LABEL] = FormatterInterface::createFormatter(FormatterInterface::RAW_TO_PNG_FORMATTER_LABEL);
        }

        void RouterManager::deleteRouters()
        {
            for (RouterTableType::iterator it = routerTable.begin(); it != routerTable.end(); ++it)
            {
                RouterInterface *router = it->second;
                delete router;
            }
            routerTable.clear();
        }

        void RouterManager::deleteFormatters()
        {
            for (FormatterTableType::iterator it = formatterTable.begin(); it != formatterTable.end(); ++it)
            {
                FormatterInterface *formatter = it->second;
                delete formatter;
            }
            formatterTable.clear();
        }

        void RouterManager::pushNewInstruction(RoutingInstruction &instruction)
        {
            lock_guard<mutex> guard(instructionQueueMutex);
            instructionQueue.push_back(instruction);
        }

        void RouterManager::routeData(string routerID_Label, Json::Value destinationInfo, vector<uint8_t> &data)
        {
            try
            {
                if (routerTable.count(routerID_Label) > 0)
                {
                    RouterInterface *router = routerTable[routerID_Label];
					log(LOG_INFO, router->getRouteInfo(destinationInfo));
                    router->route(destinationInfo, data);
                }
                else
                {
                    stringstream ss;
                    ss << "Invalid router label: " << routerID_Label;
                    log(LOG_ERROR, ss.str());
                }
            }
            catch (exception &ex)
            {
                log(LOG_ERROR, ex.what());
            }
        }

        void RouterManager::formatData(string formatterID_Label, FormatterInterface::ImageType &inData, FormatterInterface::ImageType &outData)
        {
            try
            {
                if (formatterTable.count(formatterID_Label) > 0)
                {
                    FormatterInterface *formatter = formatterTable[formatterID_Label];
                    formatter->format(inData, outData);
                }
                else
                {
                    stringstream ss;
                    ss << "Invalid formatter label: " << formatterID_Label;
                    //log error
                    log(LOG_ERROR, ss.str());
                }
            }
            catch (exception &ex)
            {
                log(LOG_ERROR, ex.what());
            }
        }

        void RouterManager::log(LogMessageCategoryType type, string message)
        {
            lock_guard<mutex> guard(logQueueMutex);
            LogMessageType logMessage = { type, message };
            logs.push_back(logMessage);
        }

        bool RouterManager::getNextInstruction(RoutingInstruction &instruction)
        {
            lock_guard<mutex> guard(instructionQueueMutex);
            bool retVal = false;
            if (!instructionQueue.empty())
            {
                instruction = instructionQueue.front();
                retVal = true;
            }
            return retVal;
        }

        void RouterManager::popInstruction()
        {
            lock_guard<mutex> guard(instructionQueueMutex);
            if (!instructionQueue.empty())
            {
                instructionQueue.pop_front();
            }
        }

    }
}
