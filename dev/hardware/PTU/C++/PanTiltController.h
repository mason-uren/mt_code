#pragma once

#include <iostream>
#include <string>
#include <cstring>
#include <sstream>
#include <vector>
#include <iterator>
#include <thread> //C++ 11 dependent 
#include <mutex>
#include <cmath>
#include <cstdio>

#ifdef __unix__

    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <netdb.h>
    #include <unistd.h>

    #ifndef closesocket
    #define closesocket close
    #endif

    #ifndef SOCKET
    #define SOCKET int
    #endif

    #ifndef SOCKET_ERROR
    #define SOCKET_ERROR -1
    #endif

    #ifndef INVALID_SOCKET
    #define INVALID_SOCKET -1
    #endif

    #ifndef SD_SEND
    #define SD_SEND SHUT_WR
    #endif

    #define WSAGetLastError() errno
    #define Sleep sleep

#elif defined(_WIN32) || defined(_WIN64)

#define WIN32_LEAN_AND_MEAN
    #include <winsock2.h>
    #include <ws2tcpip.h>

    // Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
    #pragma comment (lib, "Ws2_32.lib")
    #pragma comment (lib, "Mswsock.lib")
    #pragma comment (lib, "AdvApi32.lib")

#endif

class PanTiltController
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
    bool SetVelocities(int PanVelocity, int TiltVelocity, std::string & ResultsMessage);
    bool StopMotion();
    float getActualPanEndAngle(float panStartAngle, float panIntendedEndAngle);
    float getActualTiltEndAngle(float tiltStartAngle, float tiltIntendedEndAngle);

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
};

