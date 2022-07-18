#pragma once
#include "PracticalSocket.h"
#include "config.h"

class Plateform {
private:
	char rcv_buffer_S[300]; // Starboard (tribord)
	char rcv_buffer_P[300]; // Port (babord)

	UDPSocket continuous_S_socket; 
	UDPSocket continuous_P_socket; 

	std::string server_ip;
	std::string client_ip;

	unsigned short continuous_S_port;
	unsigned short continuous_P_port;

public:
	float atm_pressure = 0;
	float imu_S[3] = { 0, 0, 0 }; // roll, pitch, yaw (degree)
	float v_gyro_S[3] = { 0, 0, 0 }; // Gyro x, y, z  (deg.s^-1)
	float imu_P[3] = { 0, 0, 0 };
	float v_gyro_P[3] = { 0, 0, 0 };

public:
	/*------------------------------
		CONSTRUCTOR / DESTRUCTOR
	-------------------------------*/

	Plateform() {
		this->server_ip = LOCAL_IP;
		this->client_ip = LOCAL_IP;
		this->continuous_S_port = PLATEFORM_S_CONTINUOUS_PORT;
		this->continuous_P_port = PLATEFORM_P_CONTINUOUS_PORT;

		this->continuous_S_socket.init();
		this->continuous_S_socket.setLocalAddressAndPort(this->server_ip, this->continuous_S_port);
		this->continuous_S_socket.setBroadcast();

		this->continuous_P_socket.init();
		this->continuous_P_socket.setLocalAddressAndPort(this->server_ip, this->continuous_P_port);
		this->continuous_P_socket.setBroadcast();
	}

	~Plateform() {
		/*this->continuous_S_socket.disconnect();
		this->continuous_P_socket.disconnect();*/
	}

	/*------------------------------
		Receive data continuously
	-------------------------------*/
	// Receive data from starboard side
	void rcvDataS() {
		ZeroMemory(this->rcv_buffer_S, 300);
		this->continuous_S_socket.recvFrom(this->rcv_buffer_S, 300, (std::string)client_ip, this->continuous_S_port);
		log("From Plateform S: " + (std::string)this->rcv_buffer_S);

		float roll = 0, pitch = 0, yaw = 0, depth = 0, gyrx = 0, gyry = 0, gyrz = 0;
		int atm_pressure = 0;

		int ret = sscanf(this->rcv_buffer_S, "S,%f,%f,%f,%f,%f,%f,%d", &roll, &pitch, &yaw, &gyrx, &gyry, &gyrz, &atm_pressure);

		this->atm_pressure = atm_pressure;
		this->imu_S[0] = roll;
		this->imu_S[1] = pitch;
		this->imu_S[2] = yaw;

		this->v_gyro_S[0] = gyrx;
		this->v_gyro_S[1] = gyry;
		this->v_gyro_S[2] = gyrz;
	}

	// Receive data from port side
	void rcvDataP() {
		ZeroMemory(this->rcv_buffer_P, 300);
		this->continuous_P_socket.recvFrom(this->rcv_buffer_P, 300, (std::string)client_ip, this->continuous_P_port);
		log("From Plateform P: " + (std::string)this->rcv_buffer_P);

		float roll = 0, pitch = 0, yaw = 0, depth = 0, gyrx = 0, gyry = 0, gyrz = 0;
		int atm_pressure = 0;

		int ret = sscanf(this->rcv_buffer_S, "S,%f,%f,%f,%f,%f,%f,%d", &roll, &pitch, &yaw, &gyrx, &gyry, &gyrz, &atm_pressure);

		//this->atm_pressure = atm_pressure;
		this->imu_P[0] = roll;
		this->imu_P[1] = pitch;
		this->imu_P[2] = yaw;

		this->v_gyro_P[0] = gyrx;
		this->v_gyro_P[1] = gyry;
		this->v_gyro_P[2] = gyrz;
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

