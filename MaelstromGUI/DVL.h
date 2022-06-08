#pragma once
#include "PracticalSocket.h"
#include "config.h"

class DVL {
private:
	char rcv_buffer[100];
	UDPSocket socket;
	std::string server_ip;
	std::string client_ip;
	unsigned short continuous_port;
	unsigned short request_port;

	UDPSocket socket_request;

	float vx = 0, vy = 0;
	float distances[4]{ 0,0,0,0 }; // Altitude
	float imu[3] = { 0,0,0 }; // roll, pitch, yaw (degree)


public:

	/*------------------------------
		CONSTRUCTOR / DESTRUCTOR
	-------------------------------*/
	DVL() {
		this->server_ip = LOCAL_IP;
		this->client_ip = LOCAL_IP;
		this->continuous_port = DVL_CONTINUOUS_PORT;
		this->request_port = ARDUINO_REQUEST_PORT;

		this->socket.init();
		this->socket.setLocalAddressAndPort(this->server_ip, this->continuous_port);
		this->socket.setBroadcast();
	}

	~DVL() {
		this->socket.disconnect();
	}

	/*------------------------------
		Receive data continously.
	-------------------------------*/
	void rcvData() {
		this->socket.recvFrom(this->rcv_buffer, 100, (std::string)client_ip, this->continuous_port);
		//log("From DVL: " + (std::string)rcv_buffer);

		float vx=0, vy=0;
		float distances[4]{ 0,0,0,0 };
		float imu[3] = { 0,0,0 }; // roll, pitch, yaw (degree)
		int new_dvl_value = 0, new_imu_value = 0;

		int ret = sscanf(this->rcv_buffer, 
			"DVL,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d", 
			&vx, &vy, 
			&distances[0], &distances[1], &distances[2], &distances[3], 
			&imu[0], &imu[1], &imu[2], 
			&new_dvl_value, &new_imu_value);

		if (new_dvl_value) {
			this->vx = vx;
			this->vy = vy;
			memcpy(this->distances, distances, 4);
		}

		if (new_imu_value) {
			for (int i = 0; i < 3; i++)
				imu[i] = imu[i] / 10; // In degree
			memcpy(this->imu, imu, 4);
		}
			
		ZeroMemory(this->rcv_buffer, 100);
	}


	/*------------------------------
		GETTER
	-------------------------------*/
	float* getDistances() { return this->distances; }
	float* getImu() { return this->imu; }

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

