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
#define CAMERA_CX 3.1913643429071089e+002 // In air 3.1950000000000000e+002
#define CAMERA_CY 2.3907733418623138e+002 // In air 2.3950000000000000e+002
#define CAMERA_FX 7.1820477294921875e+002 // In air 4.2750715468970623e+002
#define CAMERA_FY 7.1790618896484375e+002 // In air 4.2750715468970623e+002

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
// Barometer info
#define BAROMETER_IP "192.168.0.253"
#define BAROMETER_CONTINUOUS_PORT 5819

// IMU + Barometer plateform info
#define PLATEFORM_IP "127.0.0.1"
#define PLATEFORM_S_CONTINUOUS_PORT 5823 // STARBOARD (tribord, côté du y négatif du robot)
#define PLATEFORM_P_CONTINUOUS_PORT 5825 // PORT (babord, côté du y positif du robot)

// Maths
#define PI 3.141592653589793238462

// Arduino pressure sensor offset
#define PRESSURE_SENSOR_OFFSET 0.15
#define SALINITY_GRAVITY_COEFF 100.91

// Logs dir
#define LOGS_DIR "D:/projects/cyril/logs/"
// Videos dir
#define VIDEOS_DIR "D:/projects/cyril/videos/"
// Depth map dir
#define DEPTHMAP_DIR "D:/projects/cyril/depthmap/"

// DVL reader and closer
#define DVL_READER_START "start D:/projects/cyril/soft_lecture_DVL_envoi_UDP_Maelstrom_v0_5/executable_dvl_module.exe"
#define DVL_READER_CLOSE "taskkill /f /im executable_dvl_module.exe"

// IMU reader and closer
#define IMU_READER_START "start D:/projects/cyril/module_pimu_tampon_smartcam_v0_2/executable2/PIMU_module_Maelstrom.exe"
#define IMU_READER_CLOSE "taskkill /f /im PIMU_module_Maelstrom.exe"

// IMU starboard reader and closer
#define IMU_S_READER_START "start D:/projects/cyril/module_PIMU_barometre_A_Maelstrom_2022/executables/PIMU_A_253_barometer.exe"
#define IMU_S_READER_CLOSE "taskkill /f /im PIMU_A_253_barometer.exe"

// IMU port reader and closer
#define IMU_P_READER_START "start D:/projects/cyril/module_PIMU_barometre_B_Maelstrom_2022/executables/PIMU_B_252.exe"
#define IMU_P_READER_CLOSE "taskkill /f /im PIMU_B_252.exe"



