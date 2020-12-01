/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    RouterWriteToFile.cpp
*  @author  Tom Childress
*
*  @brief Implementation file for RouterWriteToFile class
*
*****************************************************************************/
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>

#include "RouterWriteToFile.h"
#include "Communication/Router/RouterDefs.h"
#include "json/value.h"
using namespace std;

namespace BoeingMetrology
{
	namespace Communication
	{
		RouterWriteToFile::RouterWriteToFile()
		{
		}

		RouterWriteToFile::~RouterWriteToFile()
		{
		}

		void RouterWriteToFile::route(Json::Value destinationInfo, vector<uint8_t> &data)
		{
			//make sure we have the info we need
			if (destinationInfo.isMember(Router::FILENAME_KEY))
			{
				string filename = destinationInfo[Router::FILENAME_KEY].asString();
				string dirPath = filename.substr(0, filename.find_last_of("/\\"));
				//create the directory
				stringstream commandStr;
				commandStr << "mkdir \"" << dirPath << "\"";
				//check if directory creation succeeds
				if (system(commandStr.str().c_str()) != 0)
				{
					struct stat dirStat;
					//check to see if directory exists
					if (stat(dirPath.c_str(), &dirStat) != 0)
					{
						stringstream ss;
						ss << "Failed to create directory " << dirPath;
						throw runtime_error(ss.str());
					}
				}
				//create file writer
				ofstream fileWriter = ofstream(filename, ios::out | ios::binary);
				//write to file
				fileWriter.write((char*)data.data(), data.size());
				//check for exceptions
				bool isError = fileWriter.fail();
				//close file
				fileWriter.close();
				if (isError)
				{
					stringstream ss;
					ss << "Failed to write to file " << filename << " : " << strerror(errno);
					throw runtime_error(ss.str());
				}
			}
			else
			{
				throw runtime_error("Filename key not present in task information!");
			}
		}

		string RouterWriteToFile::getRouteInfo(Json::Value destinationInfo)
		{
			stringstream ss;
			//make sure we have the info we need
			if (destinationInfo.isMember(Router::FILENAME_KEY))
			{
				ss << "Writing data to file: " << destinationInfo[Router::FILENAME_KEY].asString();				
			}
			else
			{
				throw runtime_error("Filename key not present in task information!");
			}
			return ss.str();
		}
	}
}
