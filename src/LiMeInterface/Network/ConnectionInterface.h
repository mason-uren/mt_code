#ifndef METROLOGY2020_CONNECTIONINTERFACE_H
#define METROLOGY2020_CONNECTIONINTERFACE_H

class ConnectionInterface {
public:
	ConnectionInterface(const std::string &ip = "0.0.0.0", const std::string &port = "0000") :
		ip(ip),
		port(port)
	{}
	~ConnectionInterface() {}

	void setIP(const std::string &ip) {
		this->ip = ip;
	}
	void setPort(const std::string &port) {
		this->port = port;
	}
	
	std::string getIP() {
		return this->ip;
	}
	std::string getPort() {
		return this->port;
	}

private:
	std::string ip;
	std::string port;
};

#endif //  METROLOGY2020CONNECTIONINTERFACE_H