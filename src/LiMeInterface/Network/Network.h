#ifndef METROLOGY2020_NETWORK_H
#define METROLOGY2020_NETWORK_H

#include <iostream>
#include <string>
#include <cstring>
#include <vector>
#include <mutex>

// HRL
#include <Shared/InterfaceConfig.h>
#include <Shared/SharedStructs.h>
#include <Shared/OSDefines.h>

#include "ConnectionInterface.h"
#include "Tools.h"

static const constexpr int BUFFER_LEN = 64;

class Network
{
public:
	Network(const Interface::Network::Pattern & networkConfig) :
		clientAddr(networkConfig.client.ip, networkConfig.client.port),
		serverAddr(networkConfig.server.ip, networkConfig.server.port),
		UDPhandle(INVALID_SOCKET),
		TCPhandle(INVALID_SOCKET)
	{}
	Network(const ConnectionInterface & client, const ConnectionInterface & server) :
		clientAddr(client),
		serverAddr(server),
		UDPhandle(INVALID_SOCKET),
		TCPhandle(INVALID_SOCKET)
	{}
	~Network() = default;
		
	bool createSocket(const Interface::Network::Protocol &protocol);
	bool connectSocket(SOCKET & socket);
	bool bindSocketToServer(SOCKET & socket);
	bool getDataPackets(char ** recvBuf);
	bool sendUDPCommand(const char * sendBuf);
	bool sendTCPCommand(const char * sendBuf);
	void socketInfo(Shared::Device::Info * info);

	void resetSocket(SOCKET &socket);
	void resetReceiver(sockaddr_in &server);
	SOCKET & getHandle(const Interface::Network::Protocol & protocol);

private:
	bool verifyWinSockLib();
	bool getSocketAddrInfo(struct addrinfo ** result, struct addrinfo &hints, ConnectionInterface &connection);
	void initSocket(addrinfo * addr, const int &family = AF_INET, const int &socketType = SOCK_DGRAM, const int &protocol = IPPROTO_UDP);
	void initSocket(addrinfo * addr);
	void initServer();
	void parseIP();
	void socketCleanUp();

	ConnectionInterface serverAddr{};
	ConnectionInterface clientAddr{};

	// Sockets 
#ifdef WIN32
	WSADATA wsaData{};
#endif
	SOCKET UDPhandle{};
	SOCKET TCPhandle{};
	
	struct sockaddr_in server;
	struct addrinfo *result = nullptr,
					*ptr = nullptr,
					hints;
};

#endif // METROLOGY2020_NETWORK_H