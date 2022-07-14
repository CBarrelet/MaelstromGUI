#pragma once
#include "PracticalSocket.h"
#include "config.h"

#include <opencv2/imgcodecs/imgcodecs.hpp>

class Jetson {

private:
	char rcv_buffer[BUF_LEN];
	UDPSocket socket;
	std::string server_ip;
	std::string client_ip;
	unsigned short continuous_port;
	unsigned short request_port;

	cv::Mat stream_frame;

public:
	/*------------------------------
		CONSTRUCTOR / DESTRUCTOR
	-------------------------------*/
	Jetson() {
		ZeroMemory(this->rcv_buffer, BUF_LEN);
		this->server_ip = GUI_IP;
		this->client_ip = JETSON_IP;
		this->continuous_port = JETSON_CONTINUOUS_PORT;
		this->request_port = JETSON_REQUEST_PORT;
		this->socket.init();
		this->socket.setLocalAddressAndPort(this->server_ip, this->continuous_port);
		this->socket.setBroadcast();

		this->stream_frame = cv::Mat((FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0)));

	}
	~Jetson() {
		this->socket.disconnect();
	}

	/*------------------------------
		Receive data continously.
	-------------------------------*/

	int recv() {
		int recvMsgSize = 0;
		recvMsgSize = socket.recvFrom(this->rcv_buffer, BUF_LEN, this->client_ip, this->continuous_port);
		return recvMsgSize;
	}

	void decodeFrame(char* long_buffer, int total_pack) {
		cv::Mat rawData = cv::Mat(1, PACK_SIZE * total_pack, CV_8UC1, long_buffer);
		this->stream_frame = imdecode(rawData, cv::IMREAD_COLOR);
		if (this->stream_frame.size().width == 0)
			log("Jetson: Decode failure");
	}

	char* getBuffer() {
		return this->rcv_buffer;
	}

	cv::Mat getFrame() {
		return this->stream_frame;
	}

	void turnOff() {
		std::string jetson_off = "JETOFF\n";
		this->socket.sendTo(jetson_off.c_str(), 100, this->client_ip, this->request_port);
		log("To Jetson: " + jetson_off);
	}

	void turnOn() {
		char buffer[100];
		ZeroMemory(buffer, 100);
		this->socket.recvFrom(buffer, 100, this->client_ip, this->request_port);
		log("From Jetson: " + (std::string)buffer);
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

