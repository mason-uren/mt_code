/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    RouterDefs.h
*  @author  Tom Childress
*
*  @brief Contains the keys used to deconstruct routing instructions
*
*****************************************************************************/
#pragma once
#include <string>
using namespace std;

namespace BoeingMetrology
{
    namespace Communication
    {
        namespace Router
        {
            const string ROUTE_INSTRUCTION_LIST_KEY = "routeInstructionList";
            const string ROUTE_INSTRUCTION_KEY      = "routeInstruction";
            const string DEVICE_SERIAL_NUMBER_KEY   = "deviceSerialNumber";
            const string FORMAT_KEY                 = "format";
            const string TASK_KEY                   = "task";
            const string ADDRESS_KEY                = "address";
            const string PORT_KEY                   = "port";
            const string FILENAME_KEY               = "filename";
        }
    }
}