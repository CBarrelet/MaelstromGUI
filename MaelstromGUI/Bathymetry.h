#pragma once
#include "PracticalSocket.h"
#include "config.h"

class Bathymetry {
private:
	char rcv_buffer[300];
	UDPSocket socket;
	std::string ip;
	unsigned short port;

	unsigned short foreign_port = 20001;

	int size = 0;

public:
	/*------------------------------
		CONSTRUCTOR / DESTRUCTOR
	-------------------------------*/
	Bathymetry() {
		this->ip = BATHYMETRY_IP;
		this->port = BATHYMETRY_PORT;
		this->socket.init();
		this->socket.setLocalAddressAndPort(this->ip, this->port);
		this->socket.setBroadcast();
	}

	~Bathymetry() {

	}

	void set_size(int size) {
		this->size = size;

		std::string command = "*" + std::to_string(size) + "$";
		this->socket.sendTo(command.c_str(), 100, this->ip, foreign_port);

	}

};

