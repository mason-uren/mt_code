// TelemetricClient.cpp : Defines the entry point for the console application.
//


#define WIN32_LEAN_AND_MEAN

#include <stdafx.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include "PanTiltController.h"

// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")

const int DEFAULT_BUFLEN = 512;
const std::string port = "3001";
const std::string ipAddr = "10.0.1.107";
double DEGPERCOUNT = 0.00045;

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

int __cdecl main(int argc, char **argv)
{
	bool success;
	PanTiltController panTiltUnit(ipAddr, port, success);

	char *sendbuf = "p t z f\r";

	std::string bufferString = panTiltUnit.Execute(sendbuf);

	std::vector<std::string> tokens;
	tokens = split(bufferString, ' ');

	std::cout << "Pan: " << std::atof(tokens[1].c_str()) * DEGPERCOUNT << " degrees " << tokens[1] << " (raw)" << std::endl;
	std::cout << "Tilt: " << std::atof(tokens[2].c_str()) * DEGPERCOUNT << " degrees " << tokens[2] << " (raw)" << std::endl;

	return 0;
}

