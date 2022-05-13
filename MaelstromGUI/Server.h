#pragma once

#include <iostream>

//#include <WS2tcpip.h>
//#pragma comment (lib, "ws2_32.lib")

#include "PracticalSocket.h" // For UDPSocket and SocketException
//#include <iostream>          // For cout and cerr
#include <cstdlib> 

#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <chrono>
#include <thread>

#include "config.h"
#define BUF_LEN 65540

using namespace std::this_thread;     // sleep_for, sleep_until
using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
using std::chrono::system_clock;

class Server {
private:
	unsigned short server_port;
	UDPSocket socket;
	char buffer[BUF_LEN]; // Buffer for echo string
	int recvMsgSize; // Size of received message
	std::string source_address; // Address of datagram source
	unsigned short source_port; // Port of datagram source

public:
	Server() {}

	~Server() {}

	void connect(unsigned short server_port) {
		this->server_port = server_port;
		this->socket.init();
		this->socket.setLocalPort(server_port);
		this->socket.setBroadcast();
	}

	int recv() {
		this->recvMsgSize = socket.recvFrom(this->buffer, BUF_LEN, this->source_address, this->source_port);
		return this->recvMsgSize;
	}

	char* getBuffer() {
		return this->buffer;
	}

	cv::Mat getFrame(char* long_buffer, int total_pack) {
		cv::Mat rawData = cv::Mat(1, PACK_SIZE * total_pack, CV_8UC1, long_buffer);
		cv::Mat frame = imdecode(rawData, cv::IMREAD_COLOR);
		if (frame.size().width == 0) {
			std::cerr << "decode failure!" << std::endl;
			//continue;
		}
		return frame;
	}
};

