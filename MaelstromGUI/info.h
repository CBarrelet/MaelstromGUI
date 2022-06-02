#pragma once
#include <iostream>

// GUI info
std::string ip_gui = "192.168.0.11";

// Jetson info
std::string ip_jetson = "192.168.0.20";
unsigned short jetson_port = 8000;

// Robot info
std::string ip_robot = "192.168.0.10";
unsigned short request_port_robot = 10000; // Port to send data like position
unsigned short continuous_port_robot = 10001; // Port to receive data like position

// Arduino info
std::string ip_arduino = "192.168.0.254";
unsigned short continuous_port_arduino = 5817;
unsigned short request_port_arduino = 1470; // *dvlon$ *dvloff$ *jeton$ *jetoff$