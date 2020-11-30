#ifndef METROLOGY2020_TOOLS_H
#define METROLOGY2020_TOOLS_H

#include <iostream>
#include <string>
#include <typeinfo>
#include <sstream>
#include <vector>

template <typename T>
inline void unused(T) {}

template<typename Base, typename T>
inline bool instanceof(const T& ptr)
{
	try
	{
		//unused(dynamic_cast<const Base&>(ptr));
		dynamic_cast<const Base&>(ptr);
		return true;
	}
	catch (std::bad_cast e)
	{
		std::cerr << e.what() << std::endl;
		return false;
	}
}

template <typename T>
inline std::vector<T> reformatIP(std::vector<T> &container, const std::string &ip, const char *delimeter = " ") {
	int containerPtr{};

	std::stringstream ss(ip);
	std::string token{};
	
	while (std::getline(ss, token, *delimeter)) {
		container[containerPtr++] = token;
	}

	containerPtr %= container.size();

	return container;
}

#endif // METROLOGY2020_TOOLS_H