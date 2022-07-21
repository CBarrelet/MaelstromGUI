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

public:
	float vx = 0, vy = 0;
	float distances[4]{ 0,0,0,0 }; // Altitude
	cv::Point3f coordinates[4]; // Coordinates of each points
	float imu[3] = { 0,0,0 }; // roll, pitch, yaw (degree)

	int init_time = 1000; // 60000;


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
		log("From DVL: " + (std::string)rcv_buffer);

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

		float min_distance = 0.3;

		if (new_dvl_value) {
			if ((distances[0] > min_distance) && (distances[1] > min_distance) && (distances[2] > min_distance) && (distances[3] > min_distance)) {
				this->vx = vx;
				this->vy = vy;
				this->distances[0] = distances[0];
				this->distances[1] = distances[1];
				this->distances[2] = distances[2];
				this->distances[3] = distances[3];
			}
			else
				log("DVL: " + std::to_string(distances[0]) + " " + std::to_string(distances[1]) + " " + std::to_string(distances[2]) + " " + std::to_string(distances[3]));
		}

		if (new_imu_value)
			for (int i = 0; i < 3; i++)
				this->imu[i] = imu[i] / 10; // In degree
			
		ZeroMemory(this->rcv_buffer, 100);
	}

	/*------------------------------
		SETTER
	-------------------------------*/
	void set_coordinates(float depth) {

	/*	
		Calcule les coordonnées des points d'imapct du dvl dans le repère du DVL. 
		En considérant que le connecteur est orienté avec l'axe -x_dvl et on a z_dvl vers le haut.
	*/

		double sincos = 0.27059805; // sin(22.5°) * cos(45°)
		double cosalpha = 0.9238795; // cos(22.5°)

		float x1 = - this->distances[0] * sincos;
		float y1 = - this->distances[0] * sincos;
		float z1 = - this->distances[0] * cosalpha - depth;
		this->coordinates[0] = cv::Point3f(x1, y1, z1);

		float x2 = - this->distances[1] * sincos;
		float y2 =   this->distances[1] * sincos;
		float z2 = - this->distances[1] * cosalpha - depth;
		this->coordinates[1] = cv::Point3f(x2, y2, z2);

		float x3 =   this->distances[2] * sincos;
		float y3 =   this->distances[2] * sincos;
		float z3 = - this->distances[2] * cosalpha - depth;
		this->coordinates[2] = cv::Point3f(x3, y3, z3);

		float x4 =   this->distances[3] * sincos;
		float y4 = - this->distances[3] * sincos;
		float z4 = - this->distances[3] * cosalpha - depth;
		this->coordinates[3] = cv::Point3f(x4, y4, z4);

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

