
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <thread> //C++ 11 dependent 
#include <utility>
#include <iterator>
#include <chrono>
#include <cstring>
#include <fstream>
#include <algorithm>
#include "Common/Communication/Socket/PracticalSocket.h"
#include "DeviceIO/FocusController/LC2/LensControls.h"

using namespace BoeingMetrology::Scanning::Configuration;
using namespace Boeing::Communication;

const int BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::RESP_MAX = 1024;

BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::LensControls(const std::string & deviceName, const std::string & ipAddress, const int & timeoutMilliseconds)
{
    this->CONTROLLER_IP = ipAddress;
    this->timeoutms = timeoutMilliseconds;
    this->lensState = LensState(deviceName, ipAddress);
}

void BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::Connect()
{	
    if (this->CONTROLLER_IP == "")
    {
        std::cout << "LensControls::Connect failed because IP address is empty" << std::endl;
        connected = false;
        return;
    }

	try
	{
		UDPSocket sock;
		
        // Send and receive will not block any longer than these timeouts
        sock.setSendTimeout(this->timeoutms);
        sock.setRecvTimeout(this->timeoutms);

		// Ping the server
        sock.sendTo(PING, (int)strlen(PING), this->CONTROLLER_IP, this->CONTROLLER_PORT);

		//Ping responds with 7 packages 
		for (int packageReceived = 1; packageReceived < 8; packageReceived++)
		{
			// Receive a response
			char respBuffer[RESP_MAX + 1];       // Buffer for echoed string + \0
			int respStringLen;                  // Length of received response
			respStringLen = sock.recv(respBuffer, RESP_MAX);

			respBuffer[respStringLen] = '\0';             // Terminate the string!
			//std::cout << "Received: " << respBuffer << std::endl;   // Print the echoed arg

			//check response content based on API
			if (packageReceived == 1){ this->lensState.SetZoomRange(GetIntRange(respBuffer)); }
			if (packageReceived == 2){ this->lensState.SetFocusRange(GetIntRange(respBuffer)); }
			if (packageReceived == 3){ this->lensState.SetApertureRange(GetDoubleRange(respBuffer)); }
			if (packageReceived == 4){ GetCurrentParams(respBuffer); }
			if (packageReceived == 5){ this->automaticFocus = GetAutomaticFocus(respBuffer); }
			if (packageReceived == 6){ this->IS = GetIS(respBuffer); }
            if (packageReceived == 7){ this->IsActive = GetISactive(respBuffer); }
    	}

		// Software version installed 
		sock.sendTo(VERSION, (int)strlen(VERSION), CONTROLLER_IP, CONTROLLER_PORT);
		// Receive version
		char verBuffer[RESP_MAX + 1];      // Buffer for echoed string + \0
		int verStringLen;                  // Length of received response
		verStringLen = sock.recv(verBuffer, RESP_MAX);
		verBuffer[verStringLen] = '\0';    // Terminate the string!
		this->lensState.SetSoftwareVersion(verBuffer);
		
		std::cout << "Connected to controller IP " << CONTROLLER_IP << ":" << CONTROLLER_PORT << " Firmware: " << verBuffer << std::endl;
		connected = true;		
	}
	catch (SocketException &e) 
	{
		std::cout << e.what() << std::endl;
		connected = false;
	}
}

void BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::SetAperture(const double targetAperture, double &actualAperture, const int & sleepTimeMs)
{
	try
	{
		UDPSocket sock;
		std::string setMessage = SET_APER + std::to_string(targetAperture);
		const char* message = setMessage.c_str();

        // Send and receive will not block any longer than these timeouts
        sock.setSendTimeout(this->timeoutms);
        sock.setRecvTimeout(this->timeoutms);

		// Send the string to the server
		sock.sendTo(message, (int)strlen(message), CONTROLLER_IP, CONTROLLER_PORT);

		// Receive a response
		char respBuffer[RESP_MAX + 1];       // Buffer for echoed string + \0
		int respStringLen;                  // Length of received response
		respStringLen = sock.recv(respBuffer, RESP_MAX);

		respBuffer[respStringLen] = '\0';             // Terminate the string!
		//std::cout << "Received: " << respBuffer << std::endl;   // Print the echoed arg

		if (checkResponse(respBuffer, "Iris"))
		{
			std::string stringValue = respBuffer;
			size_t key = stringValue.find('=');
			actualAperture = (double)atof((stringValue.substr(key + 1)).c_str());
            this->lensState.SetApertureValue(actualAperture);
		}
	}
	catch (SocketException &e)
	{
		std::cout << e.what() << std::endl;
	}
}


void BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::SetFocus(const int targetFocus, int &actualFocus, const int & sleepTimeMs)
{
	try
	{
		if (targetFocus <  0)
			printf("focus value: %d for setFocus must be positive, default is 0", targetFocus);

		UDPSocket sock;

        // Send and receive will not block any longer than these timeouts
        sock.setSendTimeout(this->timeoutms);
        sock.setRecvTimeout(this->timeoutms);

		std::string setMessage = SET_FOCUS + std::to_string(targetFocus);
		const char* message = setMessage.c_str();

		// Send the string to the server
		sock.sendTo(message, (int)strlen(message), CONTROLLER_IP, CONTROLLER_PORT);

		// Receive a response
		char respBufferValue[RESP_MAX + 1];       // Buffer for echoed string + \0
		int respStringLen;                  // Length of received response
        respStringLen = sock.recv(respBufferValue, RESP_MAX);

        respBufferValue[respStringLen] = '\0';             // Terminate the string!
        //std::cout << "Received: " << respBufferValue << std::endl;   // Print the echoed arg

        if (checkResponse(respBufferValue, "Focus"))
		{
            // Parse the achieved focus
            std::string stringValue = respBufferValue;
            size_t key = stringValue.find('=');
            if (key == std::string::npos)
                throw std::runtime_error("SetFocus: failed to parse focus result message");
            int parsedFocusResult = atoi((stringValue.substr(key + 1)).c_str());

            // Verify movement is completed
            char respBufferStatus[RESP_MAX + 1];
            respStringLen = sock.recv(respBufferStatus, RESP_MAX);
            respBufferStatus[respStringLen] = '\0';
            if (isFocusSet(respBufferStatus))
			{
                // Focus state update complete
                actualFocus = parsedFocusResult;
                this->lensState.SetFocusValue(actualFocus);
                return;
			}
			else
                throw std::runtime_error("SetFocus: focusDone message not received");
		}
        throw std::runtime_error("SetFocus Failed");
	}
    catch (...)
    {
        throw;
    }
}

void BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::SetFocusIncrement(const int focusDelta, int &actualFocus)
{
    try
    {
        UDPSocket sock;

        // Send and receive will not block any longer than these timeouts
        sock.setSendTimeout(this->timeoutms);
        sock.setRecvTimeout(this->timeoutms);

        std::string setMessage = MOVE_FOCUS + std::to_string(focusDelta);
        const char* message = setMessage.c_str();

        // Send the string to the server
        sock.sendTo(message, (int)strlen(message), CONTROLLER_IP, CONTROLLER_PORT);

        // Receive a response
        char respBufferValue[RESP_MAX + 1];       // Buffer for echoed string + \0
        int respStringLen;                  // Length of received response
        respStringLen = sock.recv(respBufferValue, RESP_MAX);

        respBufferValue[respStringLen] = '\0';             // Terminate the string!
        //std::cout << "Received: " << respBufferValue << std::endl;   // Print the echoed arg

        if (checkResponse(respBufferValue, "Focus"))
        {
            // Parse the achieved focus
            std::string stringValue = respBufferValue;
            size_t key = stringValue.find('=');
            if (key == std::string::npos)
                throw std::runtime_error("SetFocus: failed to parse focus result message");
            // Focus state update complete
            actualFocus = atoi((stringValue.substr(key + 1)).c_str());
            this->lensState.SetFocusValue(actualFocus);
            return;
        }
        throw std::runtime_error("SetFocusIncrement Failed");
    }
    catch (...)
    {
        throw;
    }
}

bool BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::isFocusSet(const std::string stringValue)
{
	if (stringValue != "focusDone")
	{
        printf("expected 'focusDone' in response not: %s", stringValue.c_str());
		return false;
	}
	else
		return true;
}

//Parses response from controller for current values
void BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::GetCurrentParams(std::string stringValue)
{
	size_t key, firstComma, secondComma;

	if (!stringValue.empty())
	{
		key = stringValue.find('=');
		if (stringValue.substr(0, key) != "Current")
		{
			printf("expected key name: 'Current' in response not: %s", stringValue.substr(0, key).c_str());
			connected = false;
			return;
		}

		std::string valuesStr = stringValue.substr(key + 1);		//"XXX,YYY,ZZZ"
		firstComma = valuesStr.find(',');

		//zoom
		std::string zoomStr = valuesStr.substr(0, firstComma);
		this->lensState.SetZoomValue(atoi(zoomStr.c_str()));

		//focus
		std::string valuesBreakdown = valuesStr.substr(firstComma + 1);	//"YYY,ZZZ"
		secondComma = valuesBreakdown.find(',');
		std::string focStr = valuesBreakdown.substr(0, secondComma);
		this->lensState.SetFocusValue(atoi(focStr.c_str()));

		//aperture
		std::string aperStr = valuesBreakdown.substr(secondComma + 1);
		this->lensState.SetApertureValue((double)atof(aperStr.c_str()));
	}
	else
		printf("empty string response for Current values");
}


bool BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::checkResponse(const std::string stringValue, const std::string controlType)
{
	size_t key = stringValue.find('=');

	std::string keyStr = stringValue.substr(0, key);
	if (keyStr != controlType)
	{
        printf("expected key name: %s in response not: %s", controlType.c_str(), keyStr.c_str());
		return false;
	}
	return true;
}


//Parses response from controller for range values
std::pair<int, int> BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::GetIntRange(const std::string stringValue)
{
	size_t key, firstComma;
    std::pair<int, int> rangePair = { 0, 0 };

	if (!stringValue.empty())
	{
		key = stringValue.find('=');
		std::string keyStr = stringValue.substr(0, key);

		if (keyCheck(keyStr))
		{
			std::string valuesStr = stringValue.substr(key + 1);		//"min,max"
			firstComma = valuesStr.find(',');

			std::string minStr = valuesStr.substr(0, firstComma);
			std::string maxStr = valuesStr.substr(firstComma + 1);

			int minRange = atoi(minStr.c_str());
			int maxRange = atoi(maxStr.c_str());
			rangePair = std::make_pair(minRange, maxRange);
		}
	}
    return rangePair;
}

//Parses response from controller for range values
std::pair<double, double> BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::GetDoubleRange(const std::string stringValue)
{
	size_t key, firstComma;
    std::pair<double, double> rangePair = { 0.0, 0.0 };

	if (!stringValue.empty())
	{
		key = stringValue.find('=');
		std::string keyStr = stringValue.substr(0, key);

		if (keyCheck(keyStr))
		{
			std::string valuesStr = stringValue.substr(key + 1);		//"min,max"
			firstComma = valuesStr.find(',');

			std::string minStr = valuesStr.substr(0, firstComma);
			std::string maxStr = valuesStr.substr(firstComma + 1);

            double minRange = (double)atof(minStr.c_str());
            double maxRange = (double)atof(maxStr.c_str());
			rangePair = std::make_pair(minRange, maxRange);
		}
	}
    return rangePair;
}

//Checks key names for range responses
//Returns true for expected key values
bool BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::keyCheck(const std::string keyStr)
{
	if (keyStr != "zRange")
		if (keyStr != "aRange")
			if (keyStr != "fRange")
			{
				printf("expected  a range key name, not: %s/n", keyStr.c_str());
				return false;
			}

	return true;
}

//Parses response from controller for automatic focus boolean
bool BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::GetAutomaticFocus(const std::string respStr)
{
	size_t key;
	key = respStr.find('=');
	if (respStr.substr(0, key) != "AF")
	{
        printf("expected key name: 'AF' in response not: %s", respStr.substr(0, key).c_str());
		return false;
	}

	std::string boolStr = respStr.substr(key + 1);
	if (respStr.substr(key + 1) == "1")
		return true;
	else if (respStr.substr(key + 1) == "0")
		return false;
	else
	{
		printf("empty string response for Automatic Focus");
		return false;
	}
}

//Parses response from controller for IS boolean
bool BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::GetIS(const std::string respStr)
{
	size_t key;
	key = respStr.find('=');
	if (respStr.substr(0, key) != "IS")
	{
        printf("expected key name: 'IS' in response not: %s", respStr.substr(0, key).c_str());
		return false;
	}

	std::string boolStr = respStr.substr(key + 1);
	if (respStr.substr(key + 1) == "1")
		return true;
	else if (respStr.substr(key + 1) == "0")
		return false;
	else
	{
		printf("empty string response for IS");
		return false;
	}
}

bool BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::GetISactive(const std::string & respStr)
{
    size_t key;
    key = respStr.find('=');
    if (respStr.substr(0, key) != "ISactive")
    {
        printf("expected key name: 'ISactive' in response not: %s", respStr.substr(0, key).c_str());
        return false;
    }

    std::string boolStr = respStr.substr(key + 1);
    if (respStr.substr(key + 1) == "1")
        return true;
    else if (respStr.substr(key + 1) == "0")
        return false;
    else
    {
		printf("empty string response for ISactive");
        return false;
    }
}

int BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::getFocusMin()
{
    std::pair<int, int> range = lensState.GetFocusRange();
    return range.first;
}

int BoeingMetrology::DeviceIO::FocusController::LC2::LensControls::getFocusMax()
{
    std::pair<int, int> range = lensState.GetFocusRange();
    return range.second;
}


/*

bool GetFocusRange()
{
	std::cout << "Sent: " << DETECT_FOCUS_RANGE << std::endl;

	if (sendto(s, DETECT_FOCUS_RANGE, strlen(DETECT_FOCUS_RANGE), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
	{
		printf("GetFocusRange() failed with error code : %d/n", WSAGetLastError());
		return false;
	}

	this->focusRange = GetIntRange(GetCommandResponse());

	return true;
}

bool HardRestart()
{
	std::cout << "Sent: " << RESTART << std::endl;

	if (sendto(s, RESTART, strlen(RESTART), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
	{
		printf("RestartController() failed with error code : %d/n", WSAGetLastError());
		return false;
	}

	if (GetCommandResponse() == "OK") return true;
	else return false;
}

void SoftRestart()
{
	std::cout << "Sent: " << SOFT_RESTART << std::endl;

	if (sendto(s, SOFT_RESTART, strlen(SOFT_RESTART), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
	{
		printf("RestartController() failed with error code : %d/n", WSAGetLastError());
	}
}

//Stops setFocus command and expects no response
void StopFocus()
{
	std::cout << "Sent: " << STOP_FOCUS << std::endl;

	if (sendto(s, STOP_FOCUS, strlen(STOP_FOCUS), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
	{
		printf("StotFocus() failed with error code : %d/n", WSAGetLastError());
		return;
	}

	std::string resp = GetCommandResponse();
	if (resp != "")
	{ 
		printf("StotFocus() did not expect a response and recevied : %s/n", resp);;
		return;
	}
}


*/
