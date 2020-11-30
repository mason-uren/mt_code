//
// Created by U'Ren, Mason R (VEN) on 11/9/20.
//

#ifndef LIVEMETROLOGY_CONFIGINTERFACE_H
#define LIVEMETROLOGY_CONFIGINTERFACE_H

#include <memory>

#include "ErrorHandler/ErrorHandler.h"
#include "Logger/Logger.h"

static int refCount{};

class ConfigInterface {
public:
	explicit ConfigInterface(const std::string & name) : // = std::string{"ConfigInterface_" + std::to_string(refCount++)}) :
		name(name),
        config(nullptr),
		logger(Logger::getInstance()),
		eHandler(ErrorHandler::getInstance())
    {
		logger->startListening();
	}
    virtual ~ConfigInterface() = default;

	virtual std::string getName() const {
		return name;
	}

    // Template members
    template <typename T>
    bool loadConfig(T * configuration = nullptr) {
        config = static_cast<void *>(configuration ? configuration : new T("", true));
        return true;
    }

    template <typename T>
	T * Config() {
        return static_cast<T *>(config);
    }
    // End Template members


    // Variables
    void * config;
	Logger * logger;
	ErrorHandler * eHandler;

protected:
	std::string name{};
};

#endif //LIVEMETROLOGY_CONFIGINTERFACE_H
