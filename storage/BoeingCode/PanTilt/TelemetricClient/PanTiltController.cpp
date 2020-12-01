#include "stdafx.h"
#include "PanTiltController.h"
#define WIN32_LEAN_AND_MEAN

#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <thread> //C++ 11 dependent 
#include <utility>
#include <iterator>
#include <chrono>

// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")

PanTiltController::PanTiltController(std::string PanTiltIpAddr, std::string ClientIpAddr, std::string TcpPort, std::string UdpPort, bool& Success)
{
	WSADATA wsaData;
	int iResult;

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		std::cout << "WSAStartup failed with error: " << iResult << std::endl;
		Success = false;
		return;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the client address and port
	iResult = getaddrinfo(PanTiltIpAddr.c_str(), TcpPort.c_str(), &hints, &result);
	if (iResult != 0) {
		std::cout << "getaddrinfo failed with error: " << iResult << std::endl;
		Success = false;
		return;
	}

	panTiltIpAddr = PanTiltIpAddr;
	clientIpAddr = ClientIpAddr;
	tcpPort = TcpPort;
	udpPort = UdpPort;

	bool udpSuccess = OpenUdpConnection();
    if (!udpSuccess)
    {
        Success = false;
        std::cout << "Error in OpenUdpConnection" << std::endl;
        return;
    }

	threadHolder = new std::thread(&PanTiltController::GetUdpResults, this, latestUdpResults);
	threadHolder->detach(); //detach from the main thread so it can run in parallel

	// Test TCP connection TODO
	//std::string command = "p t\r";;
	//char *sendbuf = (char*)(command).c_str();
	//std::string bufferString = Execute(sendbuf, Success);
	//if (!Success)
	//{
	//	std::cout << "Error: Cannot open TCP connection " << std::endl;
	//}

	Success = true;
}

template<typename Out>
void split(const std::string &s, char delim, Out result) {
	std::stringstream ss;
	ss.str(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		*(result++) = item;
	}
}

std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, std::back_inserter(elems));
	return elems;
}

PanTiltController::~PanTiltController()
{
    StopMotion();
	delete latestUdpResults;
	freeaddrinfo(result);
	if (udpSocket != INVALID_SOCKET)
	{
		closesocket(udpSocket);
		udpSocket = INVALID_SOCKET;
	}

	if (ConnectSocket != INVALID_SOCKET)
		closesocket(ConnectSocket);
	WSACleanup();
}

bool PanTiltController::OpenTcpConnection()
{
	// Create a SOCKET for connecting to server
	ConnectSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (ConnectSocket == INVALID_SOCKET) {
		std::cout << "socket failed with error: " << WSAGetLastError() << std::endl;
		return false;
	}

	// Connect to server.
	auto iResult = connect(ConnectSocket, result->ai_addr, (int)result->ai_addrlen);
	if (iResult == SOCKET_ERROR) {
		closesocket(ConnectSocket);
		std::cout << "Unable to connect to server!" << std::endl;
		ConnectSocket = INVALID_SOCKET;
		return false;
	}

	isTcpOpen = true;
	return true;
}

bool PanTiltController::OpenUdpConnection()
{
	struct sockaddr_in server;


	/* Open a datagram socket */
	udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
	if (udpSocket == INVALID_SOCKET)
	{
		std::cout << "Error: Could not create socket.\n" << std::endl;
		return false;
	}

	/* Clear out server struct */
	memset((void *)&server, '\0', sizeof(struct sockaddr_in));

	/* Set family and port */
	server.sin_family = AF_INET;
	server.sin_port = htons(std::atoi(udpPort.c_str()));

	auto splitIpAddr = split(clientIpAddr, '.');
	server.sin_addr.S_un.S_un_b.s_b1 = (unsigned char)(std::atoi(splitIpAddr[0].c_str()));
	server.sin_addr.S_un.S_un_b.s_b2 = (unsigned char)(std::atoi(splitIpAddr[1].c_str()));
	server.sin_addr.S_un.S_un_b.s_b3 = (unsigned char)(std::atoi(splitIpAddr[2].c_str()));
	server.sin_addr.S_un.S_un_b.s_b4 = (unsigned char)(std::atoi(splitIpAddr[3].c_str()));

	/* Bind address to socket */
	if (bind(udpSocket, (struct sockaddr *)&server, sizeof(struct sockaddr_in)) == -1)
	{
		std::cout << "Error: Could not bind name to socket.\n" << std::endl;
		closesocket(udpSocket);
		udpSocket = INVALID_SOCKET;
		return false;
	}

	return true;
}

bool PanTiltController::CloseTcpConnection()
{
	// shutdown the connection since no more data will be sent
	auto iResult = shutdown(ConnectSocket, SD_SEND);
	if (iResult == SOCKET_ERROR) {
		std::cout << "shutdown failed with error: " << WSAGetLastError() << std::endl;
		closesocket(ConnectSocket);
		ConnectSocket = INVALID_SOCKET;
		return false;
	}

	isTcpOpen = false;
	return true;
}

bool PanTiltController::SendTcpCommand(char * CommandBuffer)
{
	// Send an initial buffer
	auto iResult = send(ConnectSocket, CommandBuffer, (int)strlen(CommandBuffer), 0);
	if (iResult == SOCKET_ERROR) {
		std::cout << "send failed with error: " << WSAGetLastError() << std::endl;
		closesocket(ConnectSocket);
		ConnectSocket = INVALID_SOCKET;
		return false;
	}

	return true;
}

std::string PanTiltController::Execute(char * CommandBuffer, bool & Success)
{
	Success = OpenTcpConnection();
	if (!Success) return "Error In OpenConnection";

	Success = SendTcpCommand(CommandBuffer);
	if (!Success) return "Error In SendCommand";

	Success = CloseTcpConnection();
	if (!Success) return "Error In CloseConnection";

	std::string results;
	Success = GetTcpResults(results);

	return results;
}

bool PanTiltController::GetAngles(float& PanAngle, float& TiltAngle)
{ 
	try
	{
		int counter = 0;
		while (!isUdpOpen)  // Wait for UDP to begin streaming
		{ 
			std::this_thread::sleep_for(std::chrono::milliseconds(20)); 
			counter++;
			if (counter >= 200)
			{
				std::cout << "Timeout waiting for UDP to open" << std::endl;
				return false;
			}
		}

		// Pan and Tilt Encoder bytes
		int panEncoder, tiltEncoder;
		{
			std::lock_guard<std::mutex> guard(this->udpMutex);

			panEncoder = 0 | ((unsigned char)latestUdpResults[2] << 16) | ((unsigned char)latestUdpResults[3] << 8) | (unsigned char)latestUdpResults[4];
			tiltEncoder = 0 | ((unsigned char)latestUdpResults[5] << 16) | ((unsigned char)latestUdpResults[6] << 8) | (unsigned char)latestUdpResults[7];

			// Negative value (convert from twos compliment)
			if ((unsigned char)latestUdpResults[2] & 0x80)
			{
				panEncoder = -(0xFFFFFF - panEncoder + 1);
			}
			if ((unsigned char)latestUdpResults[5] & 0x80)
			{
				tiltEncoder = -(0xFFFFFF - tiltEncoder + 1);
			}
		}

		PanAngle = (float)panEncoder * degreesPerEncoderCount;
		TiltAngle = (float)tiltEncoder * degreesPerEncoderCount;

		return true;
	}
	catch (...)
	{
		std::cout << "Unable to parse UDP stream: " << latestUdpResults << std::endl;
		return false;
	}
}

bool PanTiltController::SetAngles(float PanAngle, float TiltAngle, float& OutputPanAngle, float& OutputTiltAngle, const int& AngleRate, const float& PositionTolerance)
{
    // Get current angle
	float currentPanAngle, currentTiltAngle, PanCorrectedAngle, TiltCorrectedAngle;
	bool success = GetAngles(currentPanAngle, currentTiltAngle);

    movementVelocity = 800;
    if (AngleRate > 0 && AngleRate < 1500)
        movementVelocity = AngleRate;
    else
        std::cout << "WARNING: Angle Rate should be between 0 and 800.  Using default = 800" << std::endl;

    angleThreshold = 0.05f;
    if (PositionTolerance > 0.0f && PositionTolerance < 1.0f)
        angleThreshold = PositionTolerance;
    else
        std::cout << "WARNING: Position tolerance should be between 0 and 1.  Using default = 0.5" << std::endl;
	
	//Adjust Pan and Tilt End Angles for minimal offset
	PanCorrectedAngle = getActualPanEndAngle(currentPanAngle, PanAngle);
	TiltCorrectedAngle = getActualTiltEndAngle(currentTiltAngle, TiltAngle);

	if (success)
	{

		float panAngleDiff = std::abs(currentPanAngle - PanCorrectedAngle);
		float tiltAngleDiff = std::abs(currentTiltAngle - TiltCorrectedAngle);

		// Move until both joints are in correct position
		bool isPanDone = panAngleDiff < angleThreshold;
		bool isTiltDone = tiltAngleDiff < angleThreshold;

		GetAngles(currentPanAngle, currentTiltAngle);

		// Plan Movements, Move each joint left or right
		int panVelocity = 0, tiltVelocity = 0;
		if (!isPanDone)
		{
			panVelocity = (currentPanAngle - PanCorrectedAngle) > 0 ? -movementVelocity : movementVelocity;
		}
		if (!isTiltDone)
		{
			tiltVelocity = (currentTiltAngle - TiltCorrectedAngle) > 0 ? -movementVelocity : movementVelocity;
		}

		// Execute Movements
		std::string resultsMessage;
		SetVelocities(panVelocity, tiltVelocity, resultsMessage);

		int counter = 0;
		while (!isPanDone || !isTiltDone)
		{
			if (counter % 30000000 == 0)
			{
				std::cout << "pan " << std::to_string(currentPanAngle) << " tilt " << std::to_string(currentTiltAngle) << std::endl;
			}
			counter++;

			// Update
			success = GetAngles(currentPanAngle, currentTiltAngle);
			if (!success)
			{
				std::cout << "Error: Failed to get Angles while moving." << std::endl;
				StopMotion();
				return false;
			}

			panAngleDiff = std::abs(currentPanAngle - PanCorrectedAngle);
			tiltAngleDiff = std::abs(currentTiltAngle - TiltCorrectedAngle);

			// Stop the Pan Joint when good
			if (panAngleDiff < angleThreshold && !isPanDone)
			{
				isPanDone = true;
				panVelocity = 0;
				SetVelocities(panVelocity, tiltVelocity, resultsMessage);
			}
			// Stop the Tilt Joint when good
			if (tiltAngleDiff < angleThreshold && !isTiltDone)
			{
				isTiltDone = true;
				tiltVelocity = 0;
				SetVelocities(panVelocity, tiltVelocity, resultsMessage);
			}
		}

        // Sleep so motion can complete
        bool bRetVal = GetAngles(OutputPanAngle, OutputTiltAngle);
        std::cout << "(pre-sleep) final pan " << std::to_string(OutputPanAngle) << " final tilt " << std::to_string(OutputTiltAngle) << std::endl;
        Sleep(300);

		bRetVal = GetAngles(OutputPanAngle, OutputTiltAngle);
        std::cout << "final pan " << std::to_string(OutputPanAngle) << " final tilt " << std::to_string(OutputTiltAngle) << std::endl;
        return bRetVal;
	}
	else
	{
		std::cout << "Error: Failed getting angles from unit" << std::endl;
		return false;
	}
}

void PanTiltController::SetMovementVelocity(int Velocity)
{
	if (Velocity > 0 && Velocity < 2 * zeroVelocity)
	{
		movementVelocity = Velocity;
	}
}

bool PanTiltController::SetVelocities(int PanVelocity, int TiltVelocity, std::string& ResultsMessage )
{
	std::string command = "P " + std::to_string(zeroVelocity + PanVelocity) + " T " + std::to_string(zeroVelocity + TiltVelocity) + "\r";
	char *sendbuf = (char*)(command).c_str();

	std::cout << "Sent: " << command << std::endl;

	bool success;
	ResultsMessage = Execute(sendbuf, success);

	return success;
}

void PanTiltController::SetUdpPort(std::string Port)
{
	if (udpPort != Port)
	{
		std::cout << "Changing UDP Port" << std::endl;
		udpPort = Port;
		isUdpOpen = false;

		bool udpSuccess = OpenUdpConnection();
		if (!udpSuccess) std::cout << "Error in OpenUdpConnection" << std::endl;

		delete threadHolder;
		threadHolder = new std::thread(&PanTiltController::GetUdpResults, this, latestUdpResults);
		threadHolder->detach(); //detach from the main thread so it can run in parallel
	}
}

bool PanTiltController::StopMotion()
{
	std::string resultsMessage;
	return SetVelocities(0, 0, resultsMessage);
}

bool PanTiltController::GetTcpResults(std::string &Results)
{
	char recvbuf[512];
	bool success = false;
	Results = "No Message from PanTilt";
	// Receive until the peer closes the connection
	int numBytesReturned;
	do {

		numBytesReturned = recv(ConnectSocket, recvbuf, bufferLength, 0);
		if (numBytesReturned > 0)
		{
			std::cout << "Bytes received: " << numBytesReturned << std::endl;
			if (numBytesReturned > 4)
			{
				Results = recvbuf;
				success = true;
			}
		}
		else if (numBytesReturned == 0)
			std::cout << "Connection closed" << std::endl;
	} while (numBytesReturned > 0);

	// cleanup
	closesocket(ConnectSocket);
	ConnectSocket = INVALID_SOCKET;

	return success;
}

void PanTiltController::GetUdpResults(char* Results)
{
	char* temp = new char[bufferLength];
	do
	{
		struct sockaddr_in client;
		int client_length = (int)sizeof(struct sockaddr_in);
		int numBytesReturned = recvfrom(udpSocket, temp, bufferLength, 0,
				(struct sockaddr *)&client, &client_length);

		if (numBytesReturned < 0)
		{
			std::cout << "Could not receive datagram" << std::endl;
			closesocket(udpSocket);
			udpSocket = INVALID_SOCKET;
			delete[] temp;
			return;
		}
		else if (numBytesReturned != 27)
		{
			std::cout << "Did not receive correct number of bytes (27): " << numBytesReturned << std::endl;
			closesocket(udpSocket);
			udpSocket = INVALID_SOCKET;
			delete[] temp;
			return;
		}

		isUdpOpen = true;

		{
			// Lock and deep copy into the output variable
			std::lock_guard<std::mutex> guard(this->udpMutex);
			std::copy(temp, temp + bufferLength, Results);
		}

	} while (isUdpOpen);
	delete[] temp;
}

float  PanTiltController::getActualPanEndAngle(float panStartAngle, float panIntendedEndAngle)
{
	/* Attaching corresponding stats model summary (just for reference)
	> summary(panModel1499) #Pan Velocity set to 1499

	Call:
	lm(formula = finalModelPan, data = combinedData1499)

	Residuals:
	Min        1Q    Median        3Q       Max
	-0.148859 -0.054392  0.000295  0.052881  0.145976

	Coefficients:
	Estimate Std. Error t value Pr(>|t|)
	(Intercept)            -0.0791706  0.0148833  -5.319 4.46e-07 ***
	movingForwardPan1       0.1951205  0.0116754  16.712  < 2e-16 ***
	netPanIntendedMovement -0.0011435  0.0007568  -1.511    0.133
	---
	Signif. codes:  0 ‘***’ 0.001 ‘**’ 0.01 ‘*’ 0.05 ‘.’ 0.1 ‘ ’ 1

	Residual standard error: 0.06707 on 129 degrees of freedom
	Multiple R-squared:  0.6858,	Adjusted R-squared:  0.6809
	F-statistic: 140.8 on 2 and 129 DF,  p-value: < 2.2e-16
	*/

	float movingForwardPan = 0;
	if (panStartAngle <= panIntendedEndAngle) movingForwardPan = 1;
	const float netPanMovement = std::abs(panIntendedEndAngle - panStartAngle);

	const float panRegCoeff_intercept = -0.0791706f;
	const float panRegCoeff_moveForward = 0.1951205f;
	const float panRegCoeff_netMovement = -0.0011435f;

	const float panOffset = panRegCoeff_intercept +
		panRegCoeff_moveForward*movingForwardPan +
		panRegCoeff_netMovement*netPanMovement;

	return (panOffset + panIntendedEndAngle);
}

float  PanTiltController::getActualTiltEndAngle(float tiltStartAngle, float tiltIntendedEndAngle)
{
	/* Attaching corresponding stats model summary (just for reference)
	> summary(tiltModel1499) #Tilt velocity set to 1499

	Call:
	lm(formula = finalModelTilt, data = combinedData1499)

	Residuals:
	Min       1Q   Median       3Q      Max
	-1.83329 -0.40755 -0.02844  0.45950  1.68559

	Coefficients:
	Estimate Std. Error t value Pr(>|t|)
	(Intercept)                             -0.79244    0.20742  -3.820 0.000207 ***
	againstGravity1                          1.46667    0.29334   5.000 1.85e-06 ***
	netTiltIntendedMovement                  0.03267    0.01147   2.849 0.005105 **
	againstGravity1:netTiltIntendedMovement -0.06224    0.01622  -3.838 0.000194 ***
	---
	Signif. codes:  0 ‘***’ 0.001 ‘**’ 0.01 ‘*’ 0.05 ‘.’ 0.1 ‘ ’ 1

	Residual standard error: 0.7185 on 128 degrees of freedom
	Multiple R-squared:  0.1774,	Adjusted R-squared:  0.1582
	F-statistic: 9.204 on 3 and 128 DF,  p-value: 1.473e-05
	*/

	const float againstGravity = (tiltStartAngle > tiltIntendedEndAngle) ? 1 : 0;
	const float netTiltMovement = std::abs(tiltIntendedEndAngle - tiltStartAngle);

	const float tiltRegCoeff_intercept = -0.79244f;
	const float tiltRegCoeff_againstGravity = 0.00763f;
	const float tiltRegCoeff_netMovement = 0.00388f;
	const float tiltRegCoeff_againstGravityNetMovement = 0.00298f;

	const float tiltOffset = tiltRegCoeff_intercept +
		tiltRegCoeff_againstGravity * againstGravity +
		tiltRegCoeff_netMovement * netTiltMovement +
		tiltRegCoeff_againstGravityNetMovement * againstGravity * netTiltMovement;

	return (tiltOffset + tiltIntendedEndAngle);
}


