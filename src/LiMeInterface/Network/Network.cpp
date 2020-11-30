#include "Network.h"

bool Network::createSocket(const Interface::Network::Protocol &protocol) {
	bool status{};

#ifdef WIN32
	if (!(status = this->verifyWinSockLib())) {
		return !status;
	}
#endif

	// Address pointers
	addrinfo * addr{};
	SOCKET * handle{};
	switch (protocol) {
		case Interface::Network::Protocol::UDP: {
			this->initSocket(&this->hints, AF_INET, SOCK_DGRAM, 0);

			// Resolve socket address and port
			if (!(status = this->getSocketAddrInfo(&this->result, this->hints, this->clientAddr))) {
				return !status;
			}

			handle = &this->UDPhandle;
			break;
		}
		case Interface::Network::Protocol::TCP: {	
			this->initSocket(&this->hints, AF_UNSPEC, SOCK_STREAM, IPPROTO_TCP);

			// Resolve socket address and port
			if (!(status = this->getSocketAddrInfo(&this->result, this->hints, this->clientAddr))) {
				return !status;
			}
			handle = &this->TCPhandle;
			break;
		}

	}

	addr = this->result;
	*handle = socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
	if (*handle == INVALID_SOCKET) {
		std::cerr << "Error at socket(): " << WSAGetLastError() << std::endl;
		this->socketCleanUp();
		return false;
	}

	return status;
}

bool Network::connectSocket(SOCKET & socket) {
	bool status{};

	if (status = (SOCKET_ERROR == connect(socket, this->result->ai_addr, (int) this->result->ai_addrlen))) {
		this->resetSocket(socket);
	}

	Sleep(500);

	if (socket == INVALID_SOCKET) {
		this->socketCleanUp();
		std::cerr << "Unable to connect to PTU" << std::endl;
	}

	return !status; 
}

bool Network::bindSocketToServer(SOCKET & socket) {
	this->initServer();

	bool status{};
	// Bind socket (tell bind we want to receive datagrams from IP)
	if (status = (SOCKET_ERROR == bind(socket, (struct sockaddr *) &this->server, sizeof(this->server)))) {
		std::cerr << "Error at bind(): Unable to bind name to socket." << std::endl;
		this->resetSocket(this->UDPhandle);
	}

	return !status;
}

bool Network::getDataPackets(char ** recvBuf) {
	auto bytes{ (socklen_t) sizeof(this->server) };
	auto result{ recvfrom(this->UDPhandle, *recvBuf, (int)BUFFER_LEN, 0, (sockaddr *) &this->server, &bytes) };
	if (result < 0) {
		std::cerr << "Error at getDataPackets(): Unable to receive datagram" << std::endl;
		this->resetSocket(this->UDPhandle);
	}
 	else if (result == 0) {
		std::cerr << "Error at getDataPackets(): Connection issue, peer has disconnected." << std::endl;
		this->resetSocket(this->UDPhandle);
	}

	return result > 0;
}

bool Network::sendUDPCommand(const char * sendBuf) {
	auto bytes{ (socklen_t) sizeof(this->server) };
	bool status{};
	auto error{sendto(this->UDPhandle, sendBuf, (int) strlen(sendBuf), 0, (struct sockaddr *) &this->server, sizeof(this->server)) };
	if (!(status = (error != SOCKET_ERROR))) {
		std::cerr << "Error at sendUDPCommand(): Unable to send datagram" << std::endl;
	}
	else {
		std::cout << "Command -> " << sendBuf << std::endl;
		std::cout << "Package len ( " << strlen(sendBuf) << ")\n"
			<< "Bytes sent ( " << error << ")" << std::endl;
	}
	return status;
}

bool Network::sendTCPCommand(const char * sendBuf) {
	bool status{};
	auto error{send(this->TCPhandle, sendBuf, (int) strlen(sendBuf), 0)};
	if (!(status = (error != SOCKET_ERROR))) {
		std::cerr << "Error at sendTCPCommand(): Unable to send datagram" << std::endl;
	}

	//std::cout << "Sent: " << sendBuf << std::endl;

	return status;
}

void Network::socketInfo(Shared::Device::Info * info) {
	struct sockaddr_in address {};
	auto bytes{ (socklen_t) sizeof(address) };
	getsockname(this->UDPhandle, (struct sockaddr *) &address, &bytes);

    strcpy_s(info->buffer[Shared::Device::Info::Setting::IP], inet_ntoa(address.sin_addr));
    strcpy_s(info->buffer[Shared::Device::Info::Setting::PORT], std::to_string(htons(address.sin_port)).c_str());
}

void Network::resetSocket(SOCKET &socket) {
	closesocket(socket);
	socket = INVALID_SOCKET;
}

void Network::resetReceiver(sockaddr_in &server) {

}

SOCKET & Network::getHandle(const Interface::Network::Protocol & protocol) {
	switch (protocol) {
		case Interface::Network::Protocol::UDP:
			return this->UDPhandle;
		case Interface::Network::Protocol::TCP:
			return this->TCPhandle;
		default:
			throw std::runtime_error("Unrecognized protocol <" + std::to_string((int)protocol) + ">.");
	}
}

bool Network::verifyWinSockLib() {
	/*
	 * Magic numbers pulled from Microsoft Docs:
	 * https://docs.microsoft.com/en-us/windows/win32/winsock/initializing-winsock
	 * MAKEWORD(2, 2) - refers to WinSock version
	 */
	static bool verified{}; // Correct initialization returns - 0
	int status{};
	if (verified) {
		// If already verified
		return status;
	}

#ifdef WIN32
	if (status = WSAStartup(MAKEWORD(2, 2), &this->wsaData)) {
		std::cerr << "WSA Start-up failed: " << status << std::endl;
	}
#endif
	return !status;
}

bool Network::getSocketAddrInfo(struct addrinfo ** result, struct addrinfo &hints, ConnectionInterface &connection) {
	// Correct setup returns - 0

	bool status{};
	if (status = getaddrinfo(connection.getIP().c_str(), connection.getPort().c_str(), &hints, result)) {
		std::cerr << "getaddrinfo failed: " << status << std::endl;
		return !status;
	}
	return !status;
}

void Network::initSocket(addrinfo * addr, const int &family, const int &socketType, const int &protocol) {
	ZeroMemory(addr, sizeof(*addr));
	addr->ai_family = family;
	addr->ai_socktype = socketType;
	addr->ai_protocol = protocol;
}

void Network::initSocket(addrinfo * addr) {
	
}

void Network::initServer() {
	memset(&this->server, 0, sizeof(this->server));
	this->server.sin_family = AF_INET;
	this->server.sin_port = htons((u_short) std::stoul(this->serverAddr.getPort()));
	//this->server.sin_addr.s_addr = htonl(std::stoul(this->subscriber.getIP()));
	this->parseIP();
}

void Network::parseIP() {
	std::vector<std::string> ipVec(4);

#ifdef WIN32

	auto address{ reformatIP(ipVec, this->serverAddr.getIP(), ".") };
	this->server.sin_addr.S_un.S_un_b.s_b1 = (unsigned char)std::atoi(address[0].c_str());
	this->server.sin_addr.S_un.S_un_b.s_b2 = (unsigned char)std::atoi(address[1].c_str());
	this->server.sin_addr.S_un.S_un_b.s_b3 = (unsigned char)std::atoi(address[2].c_str());
	this->server.sin_addr.S_un.S_un_b.s_b4 = (unsigned char)std::atoi(address[3].c_str());

#else // Unix

	this->server.sin_addr.s_addr = inet_addr(this->serverAddr.getIP().c_str());

#endif	
}

void Network::socketCleanUp() {
#ifdef WIN32
    WSACleanup();
#else // UNIX
    // Linux equivalent of "WSACleanup" is to do nothing
#endif
}