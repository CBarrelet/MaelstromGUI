#pragma once
#include "PracticalSocket.h"
#include "config.h"


class Arduino {

private:
	//short int buffuer_size = 120;
	char rcv_buffer[120];
	UDPSocket request_socket;
	std::string request_server_ip;
	std::string request_client_ip;
	unsigned short request_port;

	UDPSocket continuous_socket; // To receive data from the program created by Vincent
	std::string continuous_server_ip;
	std::string continuous_client_ip;
	unsigned short continuous_port;

	// Socket to turn off the Jetson
	UDPSocket jetson_socket;
	std::string jetson_ip = JETSON_IP;
	unsigned short jetson_request_port = JETSON_REQUEST_PORT;

	public:
	char jetson_buffer[100];
	bool is_jetson_on = false;
	
	float imu[3] = {0, 0, 0}; // roll, pitch, yaw (degree)
	float v_gyro[3] = { 0,0,0 }; // Gyro x, y, z in °.s^-1

	float depth = 0; // pressure sensor (meter)
	float pressure_sm = 0; // Pressure from the smart cam in mbar

	
public:

	/*------------------------------
		CONSTRUCTOR / DESTRUCTOR
	-------------------------------*/
	Arduino() {
		this->request_server_ip = GUI_IP;
		this->request_client_ip = ARDUINO_IP;
		this->request_port = ARDUINO_CONTINUOUS_PORT;
		this->request_socket.init();
		this->request_socket.setLocalAddressAndPort(this->request_server_ip, this->request_port);
		this->request_socket.setBroadcast();

		this->continuous_server_ip = LOCAL_IP;
		this->continuous_client_ip = LOCAL_IP;
		this->continuous_port = ARDUINO_CONTINUOUS_PORT;
		this->continuous_socket.init();
		this->continuous_socket.setLocalAddressAndPort(this->continuous_server_ip, this->continuous_port);
		this->continuous_socket.setBroadcast();


		this->jetson_socket.init();
		this->jetson_socket.setLocalAddressAndPort(this->request_server_ip, this->jetson_request_port);
		this->jetson_socket.setBroadcast();
	}

	~Arduino() {
		this->request_socket.disconnect();
		this->continuous_socket.disconnect();
		this->jetson_socket.disconnect();
	}

	/*------------------------------
		Receive data
	-------------------------------*/

	void rcvAny() {
		char buffer[120];
		this->continuous_socket.recvFrom(buffer, 120, (std::string)continuous_client_ip, this->continuous_port);
		log("Arduino has started.");
		ZeroMemory(this->rcv_buffer, 100);
	}

	void rcvData(float barometer_pressure) {
		this->continuous_socket.recvFrom(this->rcv_buffer, 120, (std::string)continuous_client_ip, this->continuous_port);
		//log("From Arduino: " + (std::string)this->rcv_buffer);

		float roll = 0, pitch = 0, yaw = 0, depth = 0, gyrx = 0, gyry = 0, gyrz = 0;
		int pressure_sm = 0;
		//int ret = sscanf(this->rcv_buffer, "E,%d,%d,%d,%d", &roll, &pitch, &yaw, &depth);

		int ret = sscanf(this->rcv_buffer, "S,%f,%f,%f,%f,%f,%f,%d", &roll, &pitch, &yaw, &gyrx, &gyry, &gyrz, &pressure_sm);

		this->pressure_sm = pressure_sm;

		this->imu[0] = roll;
		this->imu[1] = pitch;
		this->imu[2] = yaw;

		this->v_gyro[0] = gyrx;
		this->v_gyro[1] = gyry;
		this->v_gyro[2] = gyrz;

		this->depth = ((float)pressure_sm - barometer_pressure) / SALINITY_GRAVITY_COEFF - PRESSURE_SENSOR_OFFSET;
		ZeroMemory(this->rcv_buffer, 100);
	}
	/*------------------------------
		Start and stop DVL.
	-------------------------------*/
	void dvlOn() {
		std::string on = "DVLON\n";
		request_socket.sendTo(on.c_str(), 120, ARDUINO_IP, ARDUINO_REQUEST_PORT);
		log("To Arduino: " + on);
	}

	void dvlOff() {
		std::string off = "DVLOFF\n";
		request_socket.sendTo(off.c_str(), 120, ARDUINO_IP, ARDUINO_REQUEST_PORT);
		log("To Arduino: " + off);
	}

	/*------------------------------
		Start and stop Jetson.
	-------------------------------*/
	void jetsonOn() {
		std::string on = "JETON\n";
		request_socket.sendTo(on.c_str(), 100, ARDUINO_IP, ARDUINO_REQUEST_PORT);
		log("To Arduino: " + on);
	}

	void jetsonOff() {
		std::string off = "JETOFF\n";
		request_socket.sendTo(off.c_str(), 100, ARDUINO_IP, ARDUINO_REQUEST_PORT);
		log("To Arduino: " + off);
	}

	/*------------------------------
		GETTER
	-------------------------------*/

	float* getImu() { return this->imu; }
	float getDepth() { return this->depth; }

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

