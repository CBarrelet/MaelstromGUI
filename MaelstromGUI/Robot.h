#pragma once
#include "PracticalSocket.h"
#include "config.h"

class Robot {

private:
	char rcv_buffer[300];
	UDPSocket continuous_socket;
	UDPSocket request_socket;
	std::string server_ip;
	std::string client_ip;
	unsigned short continuous_port;
	unsigned short request_port;

	// Robot positions
	float pos[3] = {0,0,0}; // x, y, z
	float angles[3] = { 0,0,0 };
	// Motor positions
	int motors[8] = { 0,0,0,0,0,0,0,0 };
	// Robot state
	int state = -1;
	/*
	States:
		0 wait
		1 home
		10 ready
		20 init
		21 advanced init
		22 check init
		23 finalize advanced init
		30 all pos
		31 all neg
		32 single pos
		33 single neg
		40 control move
		41 manual move
		42 p2p write file
		50 stop
		100 error
	*/
	// Actuators state
	int gripper_state = -1;
	int pump_state = -1;

	int request_id = 1;

	// Drawing
	float size=50;
	float radius = 0.5;

public:
	/*------------------------------
		CONSTRUCTOR / DESTRUCTOR
	-------------------------------*/
	Robot() {
		this->server_ip = GUI_IP;
		this->client_ip = ROBOT_IP;
		this->continuous_port = ROBOT_CONTINUOUS_PORT;
		this->request_port = ROBOT_REQUEST_PORT;

		this->continuous_socket.init();
		this->continuous_socket.setLocalAddressAndPort(this->server_ip, this->continuous_port);
		this->continuous_socket.setBroadcast();

		this->request_socket.init();
		this->request_socket.setLocalAddressAndPort(this->server_ip, this->request_port);
		this->request_socket.setBroadcast();
	}

	~Robot() {}

	/*------------------------------
		Receive data continuously
	-------------------------------*/
	void rcvData() {
		this->continuous_socket.recvFrom(this->rcv_buffer, 300, (std::string)client_ip, this->continuous_port);
		log("From Robot: " + (std::string)this->rcv_buffer);
		// Positions
		float pos[3] = { 0,0,0 }; // (mm)
		float angles[3] = { 0,0,0 }; // (mrad)
		// Motor positions
		int motors[8] = { 0,0,0,0,0,0,0,0 };
		// Robot state
		int state = -1;
		// Actuators state
		int gripper_state = -1;
		int pump_state = -1;

		int ret = sscanf(this->rcv_buffer,
			"*%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d",
			&pos[0], &pos[1], &pos[2],
			&angles[0], &angles[1], &angles[2],
			&motors[0], &motors[1], &motors[2], &motors[3], &motors[4], &motors[5], &motors[6], &motors[7],
			&state, &gripper_state, &pump_state);

		memcpy(this->pos, pos, 3);
		memcpy(this->angles, angles, 3);
		memcpy(this->motors, motors, 8);

		this->state = state;
		this->gripper_state = gripper_state;
		this->pump_state = pump_state;

		ZeroMemory(this->rcv_buffer, 300);
	}

	/*------------------------------
		Send a command to the Robot. 
		Should be in mm.
	-------------------------------*/
	void sendCommand(float x, float y, float z) {
		char answer[100]; // Robot answer (should repeat the received position

		std::string str2send = "*" + std::to_string(this->request_id) + ";" + std::to_string(x) + ";" + std::to_string(y) + ";" + std::to_string(z) + "#";
		this->request_id++;

		request_socket.sendTo(str2send.c_str(), 100, this->client_ip, request_port);
		log("To Robot: " + str2send);

		request_socket.recvFrom(answer, 100, this->client_ip, request_port);
		log("To Robot (answer): " + str2send);
	}

	/*------------------------------
		Send a command to the Robot.
		Should be in mm.
		Should be in a thread!
		a: current position
		d: target position
		b__c
		|  | delta z
		a  d
	-------------------------------*/
	void goTo(float x, float y, float z, float delta_z) {
		// First position (current position)
		float a_x = this->pos[0], a_y = this->pos[1], a_z = this->pos[2];
		// Second position (first command)
		float b_x = a_x, b_y = a_y, b_z = a_z + delta_z;
		// Thrid position (second command)
		float c_x = x, c_y = y, c_z = a_z + delta_z;
		// Forth position (third command)
		float d_x = x, d_y = y, d_z = z;

		//std::string str2send = "*" + std::to_string(this->request_id) + ";" + std::to_string(d_x) + ";" + std::to_string(y) + ";" + std::to_string(z) + "#";
		
		bool first_command_sent = false, second_command_sent = false, third_command_sent = false;

		std::string first_command  = "*" + std::to_string(this->request_id) + ";" + std::to_string(b_x) + ";" + std::to_string(b_y) + ";" + std::to_string(b_z) + "#";
		std::string second_command = "*" + std::to_string(this->request_id) + ";" + std::to_string(c_x) + ";" + std::to_string(c_y) + ";" + std::to_string(c_z) + "#";
		std::string third_command  = "*" + std::to_string(this->request_id) + ";" + std::to_string(d_x) + ";" + std::to_string(d_y) + ";" + std::to_string(d_z) + "#";

		// First command
		while (!first_command_sent) {
			// If robot is ready
			if (this->state == 10) {
				request_socket.sendTo(first_command.c_str(), 100, this->client_ip, request_port);
				log("To Robot (first command): " + first_command);
				// If robot is moving
				if (state == 40) {
					first_command_sent = true;
					this->request_id++;
				}
			}
		}
		Sleep(2000);

		// Second command
		while (!second_command_sent) {
			// If robot is ready
			if (this->state == 10) {
				request_socket.sendTo(second_command.c_str(), 100, this->client_ip, request_port);
				log("To Robot (first command): " + second_command);
				// If robot is moving
				if (state == 40) {
					second_command_sent = true;
					this->request_id++;
				}
			}
		}
		Sleep(2000);

		// Third command
		while (!third_command_sent) {
			// If robot is ready
			if (this->state == 10) {
				request_socket.sendTo(third_command.c_str(), 100, this->client_ip, request_port);
				log("To Robot (first command): " + third_command);
				Sleep(250);
				// If robot is moving
				if (state == 40) {
					third_command_sent = true;
					this->request_id++;
				}
			}
		}
		Sleep(2000);
	}

	/*------------------------------
		GETTER
	-------------------------------*/
	float* getPos() { return this->pos; }

	/*------------------------------
		SETTER
	-------------------------------*/
	void setPos(float x, float y, float z) {
		this->pos[0] = x;
		this->pos[1] = y;
		this->pos[2] = z;
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
