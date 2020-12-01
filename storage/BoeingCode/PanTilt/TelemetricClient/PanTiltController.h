#pragma once

#include "stdafx.h"
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <thread> //C++ 11 dependent 
#include <mutex>

class __declspec(dllexport) PanTiltController
{
public:
	PanTiltController(std::string PanTiltIpAddr, std::string ClientIpAddr, std::string TcpPort, std::string UdpPort, bool& Success);
	~PanTiltController();
	 
	// Returns the successful string, or an error message
	std::string Execute(char * CommandBuffer, bool & Success);
	bool GetAngles(float& PanAngle, float& TiltAngle); // Parses angle from UDP stream
    bool SetAngles(float PanAngle, float TiltAngle, float& OutputPanAngle, float& OutputTiltAngle, const int& AngleRate, const float& PositionTolerance); // Use control loop to set the angles
	void SetMovementVelocity(int Velocity); // Set the motion speed of the joint movements, default 200.
	void SetUdpPort(std::string Port);

private:
	const int bufferLength = 512;
	const float degreesPerEncoderCount = (float)0.00045;

	SOCKET ConnectSocket = INVALID_SOCKET;
	struct addrinfo *result = NULL, hints;
	bool isTcpOpen = false;
	bool isUdpOpen = false;

	const int zeroVelocity = 16383;
	int movementVelocity = 200;
	float angleThreshold = 0.5;

	std::thread * threadHolder;
	char* latestUdpResults = new char[bufferLength];
	std::mutex udpMutex;

	std::string clientIpAddr, panTiltIpAddr, tcpPort, udpPort;
	SOCKET udpSocket;

	float latestPanEncoder, latestTiltEncoder;

	bool OpenTcpConnection();
	bool OpenUdpConnection();
	bool CloseTcpConnection();
	bool SendTcpCommand(char * CommandBuffer);
	bool GetTcpResults(std::string& Results);
	void GetUdpResults(char* Results);
	bool SetVelocities(int PanVelocity, int TiltVelocity, std::string & ResultsMessage);
	bool StopMotion();

	float getActualPanEndAngle(float panStartAngle, float panIntendedEndAngle);
	float getActualTiltEndAngle(float tiltStartAngle, float tiltIntendedEndAngle);

};

