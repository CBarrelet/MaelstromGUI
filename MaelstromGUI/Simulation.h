#pragma once
#include "PracticalSocket.h"
#include "config.h"

class Simulation {

private:
	char rcv_buffer[PACK_SIZE-1];
	UDPSocket socket;
	std::string server_ip;
	std::string client_ip;
	unsigned short continuous_port;

public:

	/*------------------------------
		CONSTRUCTOR / DESTRUCTOR
	-------------------------------*/
	Simulation() {
		this->server_ip = GUI_IP;
		this->client_ip = SIMULATION_IP;
		this->continuous_port = SIMULATION_PORT;
		this->socket.init();
		this->socket.setLocalAddressAndPort(this->server_ip, this->continuous_port);
		this->socket.setBroadcast();
	}

	~Simulation() {
		this->socket.disconnect();
	}

	void sendMap(std::string depth_map_frame) {
		this->socket.sendTo(depth_map_frame.c_str(), PACK_SIZE, this->client_ip, this->continuous_port);
		//log("To Simulation: " + std::to_string(depth_map_frame.size()));
		//log("To Simulation: " + depth_map_frame);

	}

	/*------------------------------
	Timestamp
	-------------------------------*/
	std::string getTimeStamp() {
		using std::chrono::system_clock;
		auto now = std::chrono::system_clock::now();
		char buffer[80];
		auto transformed = now.time_since_epoch().count() / 1000000;
		auto millis = transformed % 1000;
		std::time_t tt;
		tt = system_clock::to_time_t(now);
		auto timeinfo = localtime(&tt);
		strftime(buffer, 80, "%F", timeinfo);
		std::string timestamp = std::string(buffer) + '_' + time_in_HH_MM_SS_MMM();
		return timestamp;
	}
	void log(std::string log_str) {
		log_str.erase(std::remove(log_str.begin(), log_str.end(), '\n'), log_str.end());
		std::cout << "[" << getTimeStamp() << "]: " << log_str << std::endl;
	}
	std::string time_in_HH_MM_SS_MMM() {
		using namespace std::chrono;
		// get current time
		auto now = system_clock::now();
		// get number of milliseconds for the current second
		// (remainder after division into seconds)
		auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
		// convert to std::time_t in order to convert to std::tm (broken time)
		auto timer = system_clock::to_time_t(now);
		// convert to broken time
		std::tm bt = *std::localtime(&timer);
		std::ostringstream oss;
		oss << std::put_time(&bt, "%H:%M:%S"); // HH:MM:SS
		oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
		return oss.str();
	}


};

