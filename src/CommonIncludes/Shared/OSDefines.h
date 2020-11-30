//
// Created by U'Ren, Mason R (VEN) on 7/7/20.
//

#ifndef ALGORITHMS_OSDEFINES_H
#define ALGORITHMS_OSDEFINES_H

#ifdef WIN32
    // NOTE: "WIN32_LEAN_AND_MEAN" declared in CMakeLists.txt
	// NOTE:  "_WINSOCK_DEPRECATED_NO_WARNINGS" declared in CMakeLists.txt 

    // UDP Connection Libs
    #include <Winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "Ws2_32.lib")

    #include <Windows.h>
    #include <direct.h>
    #include <filesystem>

    constexpr auto GetCurrentDir = _getcwd;
    constexpr char FILE_SEP[] = "\\";

#else  // Linux

    #include <unistd.h>
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <netdb.h>
    #include <unistd.h>
    #include <experimental/filesystem>
	#include <cstring>

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

    #ifndef Sleep
    #define Sleep(x)  (usleep((x)*1000)) //sleep for x milli-seconds
    #endif

    #ifndef strcpy_s
    #define strcpy_s(dest, source) strcpy(dest, source)
    #endif

    #ifndef ZeroMemory
    #define ZeroMemory(dest, len) memset(dest, 0, len)
    #endif

    typedef void * HANDLE;
    typedef unsigned int DWORD;


    constexpr auto GetCurrentDir = getcwd;
    constexpr char FILE_SEP[] = "/";

#endif // END

#endif //ALGORITHMS_OSDEFINES_H
