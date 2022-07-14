#pragma once
#include <time.h>
#include <iostream>
#include <windows.h>
#include <stdio.h>
#include <conio.h>
#include <tchar.h>

#include <cstdio>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#include <algorithm>
#include <iterator>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#define FRAME_HEIGHT 480
#define FRAME_WIDTH 640
#define FRAME_INTERVAL (1000/30)
#define PACK_SIZE 4096 //udp pack size; note that OSX limits < 8100 bytes
#define ENCODE_QUALITY 100
#define BUF_LEN 65540

// Camera calibration
#define CAMERA_CX 3.1950000000000000e+002
#define CAMERA_CY 2.3950000000000000e+002
#define CAMERA_FX 4.2750715468970623e+002
#define CAMERA_FY 4.2750715468970623e+002

// GUI info
#define GUI_IP "192.168.0.11"
#define LOCAL_IP "127.0.0.1"
// Jetson info
#define JETSON_IP "192.168.0.20"
#define JETSON_CONTINUOUS_PORT 8000
#define JETSON_REQUEST_PORT 8001
// Robot info
#define ROBOT_IP "192.168.0.10"
#define ROBOT_REQUEST_PORT 10000 // Port to send data like position
#define ROBOT_CONTINUOUS_PORT 10001 // Port to receive data like position
// Arduino info
#define ARDUINO_IP "192.168.0.254"
#define ARDUINO_CONTINUOUS_PORT 5817
#define ARDUINO_REQUEST_PORT 1470 // *dvlon$ *dvloff$ *jeton$ *jetoff$
// DVL info
#define DVL_IP "127.0.0.1"
#define DVL_CONTINUOUS_PORT 8888

// Maths
#define PI 3.141592653589793238462
