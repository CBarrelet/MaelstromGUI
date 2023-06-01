#pragma once

#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "Bbx.h"
#include "Bboxes.h"
#include "Video.h"
#include "Server.h"
#include "Arduino.h"
#include "DVL.h"
#include "Robot.h"
#include "Jetson.h"
#include "DepthMap.h"
#include "Plateform.h"
#include "Simulation.h"
#include "Bathymetry.h"
#include "Tiff.h"

#include "UTM.h"

#include "config.h"

#include "PracticalSocket.h"

#include <fstream>


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

// Opencv global
int x_mouse = 0, y_mouse = 0; // Mouse coordinates
bool mouse_left_click = false;
bool mouse_right_click = false;
bool click = false;
bool hold = false;

// System global
char mouse_click = 'L';

// Opencv mouse callback
void mouse_callback(int  event, int  x, int  y, int  flag, void* param) {

	click = false;
	if (event == cv::EVENT_MOUSEMOVE) {
		x_mouse = x;
		y_mouse = y;
	}
	else if (event == cv::EVENT_LBUTTONDOWN) {
		mouse_left_click = true;
		hold = !hold;;
		click = true;
	}
	else if (event == cv::EVENT_RBUTTONDOWN) {
		mouse_right_click = true;
	}
}


// UTM values ? 
float utm_x = 0, utm_y = 0;


namespace MaelstromGUI {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::Runtime::InteropServices;
	using namespace std;
	using namespace cv;

	// Logs path
	string log_dir_path(LOGS_DIR);
	string videos_dir_path(VIDEOS_DIR);
	string depthmap_dir_path(DEPTHMAP_DIR);


	// Date and time
	struct tm newtime;
	__time32_t aclock;

	// Camera calibration
	struct Camera_params {
		cv::Point center;
		float fx;
		float fy;
	};
	Camera_params camera_params;

	// Draw 2 points and a line to measure a segment
	bool draw_measure = true;
	bool first_point = true;

	struct Measure {
		cv::Point a;
		cv::Point b;
	};
	Measure measure;

	// Image in picture box
	Mat img;
	Mat original_img;
	Mat edited_img;

	// Video in picture box
	Mat frame;
	cv::Mat stream_frame;
	string play_video = "pause"; // or "play" or "replay"

	Video video;

	cv::VideoWriter output_video;
	std::string video_name;
	int video_count = 0;

	bool camera_show = true;
	bool camera_recording = false;

	Arduino arduino;

	DVL dvl;

	Robot robot;
	int request_id = 1; // Useless since it's already in the class

	cv::Point target_point2D;
	cv::Point3d target_point3D;
	bool draw_target = false;

	Jetson jetson;

	DepthMap depth_map;

	Plateform plateform;

	Simulation simulation;

	Bathymetry bathymetry;



	// Bathy
	float min_alt_bathy = -0.88;
	float max_alt_bathy = -7.22;


	// Bboxes initialization
	Mat null_img = Mat::zeros(cv::Size(1, 1), CV_8UC1);
	vector<Bbx> null_bbx_vector;
	Bboxes bboxes(null_img, null_bbx_vector);

	// Mouse position for zooming
	cv::Point mouse_pos = cv::Point(0, 0);
	bool mouse_left_down = false;

	// Related images in the images list
	vector< cv::String > realted_img_paths;

	Mat colormap_img = cv::Mat(400, 20, CV_8UC3, cv::Scalar(0, 0, 0));

	/// <summary>
	/// Summary for MyForm
	/// </summary>
	public ref class MainForm : public System::Windows::Forms::Form
	{
	public:
		MainForm(void)
		{


			// Keep track of logs
			if (false) {
				string log_path = log_dir_path + getTime() + ".txt";
				freopen(log_path.c_str(), "w", stdout);
			}
			log("Start");

			InitializeComponent();

			// Start IMU reader
			system(IMU_READER_START);
			Sleep(500);

			// Start IMU starboard reader
			system(IMU_S_READER_START);
			Sleep(500);

			// Start IMU port reader
			system(IMU_P_READER_START);
			Sleep(500);

			// Camera calibration
			camera_params.center = cv::Point(CAMERA_CX, CAMERA_CY);
			camera_params.fx = CAMERA_FX;
			camera_params.fy = CAMERA_FY;

			// Init target points
			target_point2D = cv::Point(0, 0);
			target_point3D = cv::Point3d(0, 0, 0);

			// Measurment points
			measure.a = cv::Point(-10, -10);
			measure.b = cv::Point(-10, -10);


			int value = 0;
			double prod = 0;
			for (size_t i = 0; i < colormap_img.size().height; i++) {
				for (size_t j = 0; j < colormap_img.size().width; j++) {
					prod = i * 255;
					value = (int)(prod / 400 + 0.5);
					Vec3b& color = colormap_img.at<Vec3b>(i, j);
					color[0] = 255 - value;
					color[1] = 255 - value;
					color[2] = 255 - value;
				}
			}

			cv::applyColorMap(colormap_img, colormap_img, cv::COLORMAP_JET);


			pictureBoxColorMap->Image = ConvertMat2Bitmap(colormap_img);

			// Robot
			backgroundWorkerRobotStarted->RunWorkerAsync(1); // Check if robot has started
			backgroundWorkerRobot->RunWorkerAsync(1); // To receive continuous data from the robot

			// Arduino
			backgroundWorkerArduinoStarted->RunWorkerAsync(1); // Check if arduino has started
			backgroundWorkerArduino->RunWorkerAsync(1); // To receive continuous data from the arduino

			// Jetson
			backgroundWorkerJetson->RunWorkerAsync(1); // To receive continuous data from the jetson

			// DVL
			backgroundWorkerDVL->RunWorkerAsync(1); // To receive continuous data from the dvl

			// Depth map
			backgroundWorkerDepthMap->RunWorkerAsync(1); // Display the depth map*

			// Plateform starboard
			backgroundWorkerPlateformS->RunWorkerAsync(1); // To receive continuous data from the plateform starboard

			// Plateform port
			backgroundWorkerPlateformP->RunWorkerAsync(1); // To receive continuous data from the plateform port

			// Plateform GPS starboard
			backgroundWorkerPlateformGPSS->RunWorkerAsync(1); // To receive continuous data from the plateform starboard

			// Plateform GPS port
			backgroundWorkerPlateformGPSP->RunWorkerAsync(1); // To receive continuous data from the plateform port

			// Bathy
			backgroundWorkerBathy->RunWorkerAsync(1);

			// Cables tension display
			backgroundWorkerCableTension->RunWorkerAsync(1);

		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~MainForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::Button^ button_Edition;
	private: System::Windows::Forms::PictureBox^ ptbSource;
	private: System::Windows::Forms::Button^ button_Browse;
	private: System::Windows::Forms::ListView^ listView1;
	private: System::Windows::Forms::Button^ view_button;
	private: System::Windows::Forms::ImageList^ imageList1;
	private: System::Windows::Forms::ColumnHeader^ Related;
	private: System::Windows::Forms::ColumnHeader^ columnHeader1;
	private: System::Windows::Forms::Button^ play_button;
	private: System::Windows::Forms::Button^ speed_button;
	private: System::Windows::Forms::Button^ load_button;
	private: System::Windows::Forms::TrackBar^ video_trackBar;
	private: System::Windows::Forms::Label^ video_label;
	private: System::Windows::Forms::Button^ record_button;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerJetson;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerRobot;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerArduino;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerDVL;
	private: System::Windows::Forms::Button^ buttonDVLon;
	private: System::Windows::Forms::Button^ buttonDVLyes;
	private: System::Windows::Forms::Button^ buttonDVLno;
	private: System::Windows::Forms::Label^ labelDVLon;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerRobotCommand;
	private: System::Windows::Forms::Label^ labelLine;
	private: System::Windows::Forms::Button^ buttonJetsonOn;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerJetsonOn;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerJetsonOff;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerArduinoStarted;
	private: System::Windows::Forms::Label^ label3dY;
	private: System::Windows::Forms::Label^ label3dX;
	private: System::Windows::Forms::Label^ label3dZ;
	private: System::Windows::Forms::Label^ labelDVL;
	private: System::Windows::Forms::Label^ label3dZFake;
	private: System::Windows::Forms::Label^ label3dYFake;
	private: System::Windows::Forms::Button^ buttonScan;
	private: System::Windows::Forms::Label^ label3dXFake;
	private: System::Windows::Forms::Label^ labelLineFake;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerDepthMap;
	private: System::Windows::Forms::PictureBox^ ptbDepthMap;
	private: System::Windows::Forms::Button^ buttonGoToTarget;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerRobotStarted;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerDVLOn;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerRobotScan;
	private: System::Windows::Forms::TextBox^ textBox1;
	private: System::Windows::Forms::Label^ labelDepthScan;
	private: System::Windows::Forms::Button^ buttonResolutionMap;
	private: System::Windows::Forms::Button^ buttonSaveDepthMap;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerPlateformS;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerPlateformP;
	private: System::Windows::Forms::Button^ buttonFreezeDM;
	private: System::Windows::Forms::TextBox^ textBoxTide;
	private: System::Windows::Forms::TextBox^ textBoxRobotX;
	private: System::Windows::Forms::TextBox^ textBoxRobotY;
	private: System::Windows::Forms::TextBox^ textBoxRobotZ;
	private: System::Windows::Forms::TextBox^ textBoxRobotState;
	private: System::Windows::Forms::TextBox^ textBoxRobotYaw;
	private: System::Windows::Forms::TextBox^ textBoxRobotPitch;
	private: System::Windows::Forms::TextBox^ textBoxRobotRoll;
	private: System::Windows::Forms::TextBox^ textBoxGripper;
	private: System::Windows::Forms::TextBox^ textBoxPump;
	private: System::Windows::Forms::TextBox^ textBoxArduinoYaw;
	private: System::Windows::Forms::TextBox^ textBoxArduinoPitch;
	private: System::Windows::Forms::TextBox^ textBoxArduinoRoll;
	private: System::Windows::Forms::TextBox^ textBoxArduinoDepth;
	private: System::Windows::Forms::TextBox^ textBoxAtmPressure;
	private: System::Windows::Forms::TextBox^ textBoxDVLVy;
	private: System::Windows::Forms::TextBox^ textBoxDVLVx;
	private: System::Windows::Forms::TextBox^ textBoxDVLYaw;
	private: System::Windows::Forms::TextBox^ textBoxDVLPitch;
	private: System::Windows::Forms::TextBox^ textBoxDVLRoll;
	private: System::Windows::Forms::TextBox^ textBoxDVLD2;
	private: System::Windows::Forms::TextBox^ textBoxDVLD1;
	private: System::Windows::Forms::TextBox^ textBoxDVLD0;
	private: System::Windows::Forms::TextBox^ textBoxDVLD3;
	private: System::Windows::Forms::TextBox^ textBoxPYaw;
	private: System::Windows::Forms::TextBox^ textBoxPPitch;
	private: System::Windows::Forms::TextBox^ textBoxPRoll;
	private: System::Windows::Forms::TextBox^ textBoxPGyrZ;
	private: System::Windows::Forms::TextBox^ textBoxPGyrY;
	private: System::Windows::Forms::TextBox^ textBoxPGyrX;
	private: System::Windows::Forms::TextBox^ textBoxSYaw;
	private: System::Windows::Forms::TextBox^ textBoxSPitch;
	private: System::Windows::Forms::TextBox^ textBoxSRoll;
	private: System::Windows::Forms::TextBox^ textBoxSGyrZ;
	private: System::Windows::Forms::TextBox^ textBoxSGyrY;
	private: System::Windows::Forms::TextBox^ textBoxSGyrX;
	private: System::Windows::Forms::TextBox^ textBoxSGPSY;
	private: System::Windows::Forms::TextBox^ textBoxSGPSX;
	private: System::Windows::Forms::TextBox^ textBoxPGPSY;
	private: System::Windows::Forms::TextBox^ textBoxPGPSX;
	private: System::Windows::Forms::Label^ labelGPSP;
	private: System::Windows::Forms::Label^ labelGPSS;
	private: System::Windows::Forms::GroupBox^ groupBoxRobot;
	private: System::Windows::Forms::TextBox^ textBoxSRTK;
	private: System::Windows::Forms::Label^ labelTide;
	private: System::Windows::Forms::Label^ label10;
	private: System::Windows::Forms::Label^ label9;
	private: System::Windows::Forms::Label^ label8;
	private: System::Windows::Forms::Label^ label7;
	private: System::Windows::Forms::Label^ label6;
	private: System::Windows::Forms::Label^ label4;
	private: System::Windows::Forms::Label^ label3;
	private: System::Windows::Forms::Label^ label2;
	private: System::Windows::Forms::Label^ label1;
	private: System::Windows::Forms::GroupBox^ groupBox1;
	private: System::Windows::Forms::Label^ label14;
	private: System::Windows::Forms::Label^ label11;
	private: System::Windows::Forms::Label^ label12;
	private: System::Windows::Forms::Label^ label13;
	private: System::Windows::Forms::Label^ label15;
	private: System::Windows::Forms::GroupBox^ groupBoxDVL;
	private: System::Windows::Forms::Label^ label22;
	private: System::Windows::Forms::Label^ label19;
	private: System::Windows::Forms::Label^ label20;
	private: System::Windows::Forms::Label^ label21;
	private: System::Windows::Forms::Label^ label16;
	private: System::Windows::Forms::Label^ label17;
	private: System::Windows::Forms::Label^ label18;
	private: System::Windows::Forms::Label^ label23;
	private: System::Windows::Forms::Label^ label24;
	private: System::Windows::Forms::GroupBox^ groupBoxPort;
	private: System::Windows::Forms::Label^ label25;
	private: System::Windows::Forms::Label^ label26;
	private: System::Windows::Forms::Label^ label27;
	private: System::Windows::Forms::Label^ label28;
	private: System::Windows::Forms::Label^ label29;
	private: System::Windows::Forms::Label^ label30;
	private: System::Windows::Forms::GroupBox^ groupBoxStarboard;
	private: System::Windows::Forms::Label^ label34;
	private: System::Windows::Forms::Label^ label35;
	private: System::Windows::Forms::Label^ label31;
	private: System::Windows::Forms::Label^ label36;
	private: System::Windows::Forms::Label^ label32;
	private: System::Windows::Forms::Label^ label33;
	private: System::Windows::Forms::PictureBox^ pictureBoxLogoLirmm;
	private: System::Windows::Forms::PictureBox^ pictureBoxLogoUM;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerPlateformGPSP;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerPlateformGPSS;
	private: System::Windows::Forms::TextBox^ textBoxMouseDepth;
	private: System::Windows::Forms::Label^ label5;
	private: System::Windows::Forms::PictureBox^ pictureBoxLogoMaelstrom;
	private: System::Windows::Forms::PictureBox^ pictureBoxColorMap;
	private: System::Windows::Forms::Label^ labelMaxDepth;


	private: System::Windows::Forms::Label^ labelMinDepth;
	private: System::Windows::Forms::ContextMenuStrip^ contextMenuStrip1;
	private: System::Windows::Forms::TextBox^ textBoxTargetLatitude;
	private: System::Windows::Forms::TextBox^ textBoxTargetLongitude;
	private: System::Windows::Forms::Label^ label37;
	private: System::Windows::Forms::Label^ label38;
	private: System::Windows::Forms::Button^ buttonGoTargetGPS;
	private: System::Windows::Forms::TextBox^ textBoxLatRobot;
	private: System::Windows::Forms::TextBox^ textBoxLongRobot;
	private: System::Windows::Forms::Label^ label39;
	private: System::Windows::Forms::Label^ label40;
	private: System::Windows::Forms::TextBox^ textBoxUTMXRobot;
	private: System::Windows::Forms::TextBox^ textBoxUTMYRobot;
	private: System::Windows::Forms::Label^ label41;
	private: System::Windows::Forms::Label^ label42;
	private: System::Windows::Forms::TextBox^ textBoxCap;
	private: System::Windows::Forms::Label^ label43;
	private: System::Windows::Forms::Label^ label44;
	private: System::Windows::Forms::TextBox^ textBoxCapDeg;
	private: System::Windows::Forms::Label^ label45;
	private: System::Windows::Forms::TextBox^ textBoxDistanceGPS;
	private: System::Windows::Forms::Label^ label46;
	private: System::Windows::Forms::Label^ label47;
	private: System::Windows::Forms::TextBox^ textBoxTargetUTMNorth;
	private: System::Windows::Forms::TextBox^ textBoxTargetUTMEast;
	private: System::Windows::Forms::Label^ label48;
	private: System::Windows::Forms::Label^ label49;
	private: System::Windows::Forms::TextBox^ textBoxUTMDeltaNorthing;
	private: System::Windows::Forms::TextBox^ textBoxDeltaUTMEasting;
	private: System::Windows::Forms::Label^ label50;
	private: System::Windows::Forms::Label^ label51;
	private: System::Windows::Forms::Label^ label52;
	private: System::Windows::Forms::TextBox^ textBoxTargetCap;
	private: System::Windows::Forms::TextBox^ textBoxCurrentCap;
	private: System::Windows::Forms::TextBox^ textBoxDeltaCap;
	private: System::Windows::Forms::Button^ buttonSetTargetUTM;
	private: System::Windows::Forms::Label^ label53;
	private: System::Windows::Forms::Label^ label54;
	private: System::Windows::Forms::TextBox^ textBoxDeltaXRobot;
	private: System::Windows::Forms::TextBox^ textBoxDeltaYRobot;
	private: System::Windows::Forms::PictureBox^ pictureBoxBathy;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerBathy;





	private: System::Windows::Forms::Button^ buttonBathyOnline;
	private: System::Windows::Forms::Label^ label55;
	private: System::Windows::Forms::Label^ label56;
	private: System::Windows::Forms::Label^ label57;

	private: System::Windows::Forms::PictureBox^ pictureBoxZoomedBathy;
	private: System::Windows::Forms::GroupBox^ groupBox2;
	private: System::Windows::Forms::GroupBox^ groupBox3;
	private: System::Windows::Forms::PictureBox^ pictureBoxM1;

	private: System::Windows::Forms::PictureBox^ pictureBoxM2;

	private: System::Windows::Forms::PictureBox^ pictureBoxM3;

	private: System::Windows::Forms::PictureBox^ pictureBoxM4;

	private: System::Windows::Forms::PictureBox^ pictureBoxM5;

	private: System::Windows::Forms::PictureBox^ pictureBoxM6;

	private: System::Windows::Forms::PictureBox^ pictureBoxM7;

	private: System::Windows::Forms::PictureBox^ pictureBoxM8;

	private: System::Windows::Forms::GroupBox^ groupBox4;
	private: System::Windows::Forms::Label^ label65;
	private: System::Windows::Forms::Label^ label64;
	private: System::Windows::Forms::Label^ label63;
	private: System::Windows::Forms::Label^ label62;
	private: System::Windows::Forms::Label^ label61;
	private: System::Windows::Forms::Label^ label60;
	private: System::Windows::Forms::Label^ label59;
	private: System::Windows::Forms::Label^ label58;
	private: System::Windows::Forms::ContextMenuStrip^ contextMenuStrip2;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerCableTension;
	private: System::Windows::Forms::TextBox^ textBoxM8;
	private: System::Windows::Forms::TextBox^ textBoxM7;
	private: System::Windows::Forms::TextBox^ textBoxM6;
	private: System::Windows::Forms::TextBox^ textBoxM5;
	private: System::Windows::Forms::TextBox^ textBoxM4;
	private: System::Windows::Forms::TextBox^ textBoxM3;
	private: System::Windows::Forms::TextBox^ textBoxM2;
	private: System::Windows::Forms::TextBox^ textBoxM1;
	private: System::Windows::Forms::Button^ button1;
	private: System::Windows::Forms::ErrorProvider^ errorProvider1;








	private: System::ComponentModel::IContainer^ components;

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>


#pragma region Windows Form Designer generated code
	/// <summary>
	/// Required method for Designer support - do not modify
	/// the contents of this method with the code editor.
	/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			System::ComponentModel::ComponentResourceManager^ resources = (gcnew System::ComponentModel::ComponentResourceManager(MainForm::typeid));
			this->button_Edition = (gcnew System::Windows::Forms::Button());
			this->ptbSource = (gcnew System::Windows::Forms::PictureBox());
			this->button_Browse = (gcnew System::Windows::Forms::Button());
			this->listView1 = (gcnew System::Windows::Forms::ListView());
			this->columnHeader1 = (gcnew System::Windows::Forms::ColumnHeader());
			this->Related = (gcnew System::Windows::Forms::ColumnHeader());
			this->view_button = (gcnew System::Windows::Forms::Button());
			this->imageList1 = (gcnew System::Windows::Forms::ImageList(this->components));
			this->play_button = (gcnew System::Windows::Forms::Button());
			this->speed_button = (gcnew System::Windows::Forms::Button());
			this->load_button = (gcnew System::Windows::Forms::Button());
			this->video_trackBar = (gcnew System::Windows::Forms::TrackBar());
			this->video_label = (gcnew System::Windows::Forms::Label());
			this->record_button = (gcnew System::Windows::Forms::Button());
			this->backgroundWorkerJetson = (gcnew System::ComponentModel::BackgroundWorker());
			this->backgroundWorkerRobot = (gcnew System::ComponentModel::BackgroundWorker());
			this->backgroundWorkerArduino = (gcnew System::ComponentModel::BackgroundWorker());
			this->backgroundWorkerDVL = (gcnew System::ComponentModel::BackgroundWorker());
			this->buttonDVLon = (gcnew System::Windows::Forms::Button());
			this->buttonDVLyes = (gcnew System::Windows::Forms::Button());
			this->buttonDVLno = (gcnew System::Windows::Forms::Button());
			this->labelDVLon = (gcnew System::Windows::Forms::Label());
			this->backgroundWorkerRobotCommand = (gcnew System::ComponentModel::BackgroundWorker());
			this->buttonJetsonOn = (gcnew System::Windows::Forms::Button());
			this->backgroundWorkerJetsonOn = (gcnew System::ComponentModel::BackgroundWorker());
			this->backgroundWorkerJetsonOff = (gcnew System::ComponentModel::BackgroundWorker());
			this->backgroundWorkerArduinoStarted = (gcnew System::ComponentModel::BackgroundWorker());
			this->label3dY = (gcnew System::Windows::Forms::Label());
			this->label3dX = (gcnew System::Windows::Forms::Label());
			this->label3dZ = (gcnew System::Windows::Forms::Label());
			this->labelDVL = (gcnew System::Windows::Forms::Label());
			this->label3dZFake = (gcnew System::Windows::Forms::Label());
			this->label3dYFake = (gcnew System::Windows::Forms::Label());
			this->label3dXFake = (gcnew System::Windows::Forms::Label());
			this->labelLine = (gcnew System::Windows::Forms::Label());
			this->labelLineFake = (gcnew System::Windows::Forms::Label());
			this->backgroundWorkerDepthMap = (gcnew System::ComponentModel::BackgroundWorker());
			this->ptbDepthMap = (gcnew System::Windows::Forms::PictureBox());
			this->buttonGoToTarget = (gcnew System::Windows::Forms::Button());
			this->backgroundWorkerRobotStarted = (gcnew System::ComponentModel::BackgroundWorker());
			this->backgroundWorkerDVLOn = (gcnew System::ComponentModel::BackgroundWorker());
			this->buttonScan = (gcnew System::Windows::Forms::Button());
			this->backgroundWorkerRobotScan = (gcnew System::ComponentModel::BackgroundWorker());
			this->textBox1 = (gcnew System::Windows::Forms::TextBox());
			this->labelDepthScan = (gcnew System::Windows::Forms::Label());
			this->buttonResolutionMap = (gcnew System::Windows::Forms::Button());
			this->buttonSaveDepthMap = (gcnew System::Windows::Forms::Button());
			this->backgroundWorkerPlateformS = (gcnew System::ComponentModel::BackgroundWorker());
			this->backgroundWorkerPlateformP = (gcnew System::ComponentModel::BackgroundWorker());
			this->buttonFreezeDM = (gcnew System::Windows::Forms::Button());
			this->textBoxTide = (gcnew System::Windows::Forms::TextBox());
			this->textBoxRobotX = (gcnew System::Windows::Forms::TextBox());
			this->textBoxRobotY = (gcnew System::Windows::Forms::TextBox());
			this->textBoxRobotZ = (gcnew System::Windows::Forms::TextBox());
			this->textBoxRobotState = (gcnew System::Windows::Forms::TextBox());
			this->textBoxRobotYaw = (gcnew System::Windows::Forms::TextBox());
			this->textBoxRobotPitch = (gcnew System::Windows::Forms::TextBox());
			this->textBoxRobotRoll = (gcnew System::Windows::Forms::TextBox());
			this->textBoxGripper = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPump = (gcnew System::Windows::Forms::TextBox());
			this->textBoxArduinoYaw = (gcnew System::Windows::Forms::TextBox());
			this->textBoxArduinoPitch = (gcnew System::Windows::Forms::TextBox());
			this->textBoxArduinoRoll = (gcnew System::Windows::Forms::TextBox());
			this->textBoxArduinoDepth = (gcnew System::Windows::Forms::TextBox());
			this->textBoxAtmPressure = (gcnew System::Windows::Forms::TextBox());
			this->textBoxDVLVy = (gcnew System::Windows::Forms::TextBox());
			this->textBoxDVLVx = (gcnew System::Windows::Forms::TextBox());
			this->textBoxDVLYaw = (gcnew System::Windows::Forms::TextBox());
			this->textBoxDVLPitch = (gcnew System::Windows::Forms::TextBox());
			this->textBoxDVLRoll = (gcnew System::Windows::Forms::TextBox());
			this->textBoxDVLD2 = (gcnew System::Windows::Forms::TextBox());
			this->textBoxDVLD1 = (gcnew System::Windows::Forms::TextBox());
			this->textBoxDVLD0 = (gcnew System::Windows::Forms::TextBox());
			this->textBoxDVLD3 = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPYaw = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPPitch = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPRoll = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPGyrZ = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPGyrY = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPGyrX = (gcnew System::Windows::Forms::TextBox());
			this->textBoxSYaw = (gcnew System::Windows::Forms::TextBox());
			this->textBoxSPitch = (gcnew System::Windows::Forms::TextBox());
			this->textBoxSRoll = (gcnew System::Windows::Forms::TextBox());
			this->textBoxSGyrZ = (gcnew System::Windows::Forms::TextBox());
			this->textBoxSGyrY = (gcnew System::Windows::Forms::TextBox());
			this->textBoxSGyrX = (gcnew System::Windows::Forms::TextBox());
			this->textBoxSGPSY = (gcnew System::Windows::Forms::TextBox());
			this->textBoxSGPSX = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPGPSY = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPGPSX = (gcnew System::Windows::Forms::TextBox());
			this->labelGPSP = (gcnew System::Windows::Forms::Label());
			this->labelGPSS = (gcnew System::Windows::Forms::Label());
			this->groupBoxRobot = (gcnew System::Windows::Forms::GroupBox());
			this->label10 = (gcnew System::Windows::Forms::Label());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->textBoxSRTK = (gcnew System::Windows::Forms::TextBox());
			this->labelTide = (gcnew System::Windows::Forms::Label());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->label15 = (gcnew System::Windows::Forms::Label());
			this->label14 = (gcnew System::Windows::Forms::Label());
			this->label11 = (gcnew System::Windows::Forms::Label());
			this->label12 = (gcnew System::Windows::Forms::Label());
			this->label13 = (gcnew System::Windows::Forms::Label());
			this->groupBoxDVL = (gcnew System::Windows::Forms::GroupBox());
			this->label23 = (gcnew System::Windows::Forms::Label());
			this->label24 = (gcnew System::Windows::Forms::Label());
			this->label22 = (gcnew System::Windows::Forms::Label());
			this->label19 = (gcnew System::Windows::Forms::Label());
			this->label20 = (gcnew System::Windows::Forms::Label());
			this->label21 = (gcnew System::Windows::Forms::Label());
			this->label16 = (gcnew System::Windows::Forms::Label());
			this->label17 = (gcnew System::Windows::Forms::Label());
			this->label18 = (gcnew System::Windows::Forms::Label());
			this->groupBoxPort = (gcnew System::Windows::Forms::GroupBox());
			this->label55 = (gcnew System::Windows::Forms::Label());
			this->label28 = (gcnew System::Windows::Forms::Label());
			this->label29 = (gcnew System::Windows::Forms::Label());
			this->label30 = (gcnew System::Windows::Forms::Label());
			this->label25 = (gcnew System::Windows::Forms::Label());
			this->label26 = (gcnew System::Windows::Forms::Label());
			this->label27 = (gcnew System::Windows::Forms::Label());
			this->groupBoxStarboard = (gcnew System::Windows::Forms::GroupBox());
			this->label56 = (gcnew System::Windows::Forms::Label());
			this->label34 = (gcnew System::Windows::Forms::Label());
			this->label35 = (gcnew System::Windows::Forms::Label());
			this->label31 = (gcnew System::Windows::Forms::Label());
			this->label36 = (gcnew System::Windows::Forms::Label());
			this->label32 = (gcnew System::Windows::Forms::Label());
			this->label33 = (gcnew System::Windows::Forms::Label());
			this->pictureBoxLogoLirmm = (gcnew System::Windows::Forms::PictureBox());
			this->pictureBoxLogoUM = (gcnew System::Windows::Forms::PictureBox());
			this->backgroundWorkerPlateformGPSP = (gcnew System::ComponentModel::BackgroundWorker());
			this->backgroundWorkerPlateformGPSS = (gcnew System::ComponentModel::BackgroundWorker());
			this->textBoxMouseDepth = (gcnew System::Windows::Forms::TextBox());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->pictureBoxLogoMaelstrom = (gcnew System::Windows::Forms::PictureBox());
			this->pictureBoxColorMap = (gcnew System::Windows::Forms::PictureBox());
			this->labelMaxDepth = (gcnew System::Windows::Forms::Label());
			this->labelMinDepth = (gcnew System::Windows::Forms::Label());
			this->contextMenuStrip1 = (gcnew System::Windows::Forms::ContextMenuStrip(this->components));
			this->textBoxTargetLatitude = (gcnew System::Windows::Forms::TextBox());
			this->textBoxTargetLongitude = (gcnew System::Windows::Forms::TextBox());
			this->label37 = (gcnew System::Windows::Forms::Label());
			this->label38 = (gcnew System::Windows::Forms::Label());
			this->buttonGoTargetGPS = (gcnew System::Windows::Forms::Button());
			this->textBoxLatRobot = (gcnew System::Windows::Forms::TextBox());
			this->textBoxLongRobot = (gcnew System::Windows::Forms::TextBox());
			this->label39 = (gcnew System::Windows::Forms::Label());
			this->label40 = (gcnew System::Windows::Forms::Label());
			this->textBoxUTMXRobot = (gcnew System::Windows::Forms::TextBox());
			this->textBoxUTMYRobot = (gcnew System::Windows::Forms::TextBox());
			this->label41 = (gcnew System::Windows::Forms::Label());
			this->label42 = (gcnew System::Windows::Forms::Label());
			this->textBoxCap = (gcnew System::Windows::Forms::TextBox());
			this->label43 = (gcnew System::Windows::Forms::Label());
			this->label44 = (gcnew System::Windows::Forms::Label());
			this->textBoxCapDeg = (gcnew System::Windows::Forms::TextBox());
			this->label45 = (gcnew System::Windows::Forms::Label());
			this->textBoxDistanceGPS = (gcnew System::Windows::Forms::TextBox());
			this->label46 = (gcnew System::Windows::Forms::Label());
			this->label47 = (gcnew System::Windows::Forms::Label());
			this->textBoxTargetUTMNorth = (gcnew System::Windows::Forms::TextBox());
			this->textBoxTargetUTMEast = (gcnew System::Windows::Forms::TextBox());
			this->label48 = (gcnew System::Windows::Forms::Label());
			this->label49 = (gcnew System::Windows::Forms::Label());
			this->textBoxUTMDeltaNorthing = (gcnew System::Windows::Forms::TextBox());
			this->textBoxDeltaUTMEasting = (gcnew System::Windows::Forms::TextBox());
			this->label50 = (gcnew System::Windows::Forms::Label());
			this->label51 = (gcnew System::Windows::Forms::Label());
			this->label52 = (gcnew System::Windows::Forms::Label());
			this->textBoxTargetCap = (gcnew System::Windows::Forms::TextBox());
			this->textBoxCurrentCap = (gcnew System::Windows::Forms::TextBox());
			this->textBoxDeltaCap = (gcnew System::Windows::Forms::TextBox());
			this->buttonSetTargetUTM = (gcnew System::Windows::Forms::Button());
			this->label53 = (gcnew System::Windows::Forms::Label());
			this->label54 = (gcnew System::Windows::Forms::Label());
			this->textBoxDeltaXRobot = (gcnew System::Windows::Forms::TextBox());
			this->textBoxDeltaYRobot = (gcnew System::Windows::Forms::TextBox());
			this->pictureBoxBathy = (gcnew System::Windows::Forms::PictureBox());
			this->backgroundWorkerBathy = (gcnew System::ComponentModel::BackgroundWorker());
			this->buttonBathyOnline = (gcnew System::Windows::Forms::Button());
			this->label57 = (gcnew System::Windows::Forms::Label());
			this->pictureBoxZoomedBathy = (gcnew System::Windows::Forms::PictureBox());
			this->groupBox2 = (gcnew System::Windows::Forms::GroupBox());
			this->groupBox3 = (gcnew System::Windows::Forms::GroupBox());
			this->pictureBoxM1 = (gcnew System::Windows::Forms::PictureBox());
			this->pictureBoxM2 = (gcnew System::Windows::Forms::PictureBox());
			this->pictureBoxM3 = (gcnew System::Windows::Forms::PictureBox());
			this->pictureBoxM4 = (gcnew System::Windows::Forms::PictureBox());
			this->pictureBoxM5 = (gcnew System::Windows::Forms::PictureBox());
			this->pictureBoxM6 = (gcnew System::Windows::Forms::PictureBox());
			this->pictureBoxM7 = (gcnew System::Windows::Forms::PictureBox());
			this->pictureBoxM8 = (gcnew System::Windows::Forms::PictureBox());
			this->groupBox4 = (gcnew System::Windows::Forms::GroupBox());
			this->textBoxM8 = (gcnew System::Windows::Forms::TextBox());
			this->textBoxM7 = (gcnew System::Windows::Forms::TextBox());
			this->textBoxM6 = (gcnew System::Windows::Forms::TextBox());
			this->textBoxM5 = (gcnew System::Windows::Forms::TextBox());
			this->textBoxM4 = (gcnew System::Windows::Forms::TextBox());
			this->textBoxM3 = (gcnew System::Windows::Forms::TextBox());
			this->textBoxM2 = (gcnew System::Windows::Forms::TextBox());
			this->textBoxM1 = (gcnew System::Windows::Forms::TextBox());
			this->label65 = (gcnew System::Windows::Forms::Label());
			this->label64 = (gcnew System::Windows::Forms::Label());
			this->label63 = (gcnew System::Windows::Forms::Label());
			this->label62 = (gcnew System::Windows::Forms::Label());
			this->label61 = (gcnew System::Windows::Forms::Label());
			this->label60 = (gcnew System::Windows::Forms::Label());
			this->label59 = (gcnew System::Windows::Forms::Label());
			this->label58 = (gcnew System::Windows::Forms::Label());
			this->contextMenuStrip2 = (gcnew System::Windows::Forms::ContextMenuStrip(this->components));
			this->backgroundWorkerCableTension = (gcnew System::ComponentModel::BackgroundWorker());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->errorProvider1 = (gcnew System::Windows::Forms::ErrorProvider(this->components));
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ptbSource))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->video_trackBar))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ptbDepthMap))->BeginInit();
			this->groupBoxRobot->SuspendLayout();
			this->groupBox1->SuspendLayout();
			this->groupBoxDVL->SuspendLayout();
			this->groupBoxPort->SuspendLayout();
			this->groupBoxStarboard->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxLogoLirmm))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxLogoUM))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxLogoMaelstrom))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxColorMap))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxBathy))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxZoomedBathy))->BeginInit();
			this->groupBox2->SuspendLayout();
			this->groupBox3->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM1))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM2))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM3))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM4))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM5))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM6))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM7))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM8))->BeginInit();
			this->groupBox4->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->errorProvider1))->BeginInit();
			this->SuspendLayout();
			// 
			// button_Edition
			// 
			this->button_Edition->Enabled = false;
			this->button_Edition->Location = System::Drawing::Point(981, 588);
			this->button_Edition->Name = L"button_Edition";
			this->button_Edition->Size = System::Drawing::Size(75, 23);
			this->button_Edition->TabIndex = 35;
			this->button_Edition->Text = L"Edition";
			// 
			// ptbSource
			// 
			this->ptbSource->BackColor = System::Drawing::SystemColors::ControlDark;
			this->ptbSource->Location = System::Drawing::Point(414, 58);
			this->ptbSource->Name = L"ptbSource";
			this->ptbSource->Size = System::Drawing::Size(640, 480);
			this->ptbSource->TabIndex = 1;
			this->ptbSource->TabStop = false;
			this->ptbSource->MouseClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::ptbSource_MouseClick);
			this->ptbSource->MouseDoubleClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::ptbSource_MouseDoubleClick);
			this->ptbSource->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::ptbSource_MouseDown);
			this->ptbSource->MouseMove += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::ptbSource_MouseMove);
			// 
			// button_Browse
			// 
			this->button_Browse->Enabled = false;
			this->button_Browse->Location = System::Drawing::Point(907, 587);
			this->button_Browse->Name = L"button_Browse";
			this->button_Browse->Size = System::Drawing::Size(67, 23);
			this->button_Browse->TabIndex = 2;
			this->button_Browse->Text = L"Browse";
			this->button_Browse->UseVisualStyleBackColor = true;
			this->button_Browse->Click += gcnew System::EventHandler(this, &MainForm::button_Browse_Click);
			// 
			// listView1
			// 
			this->listView1->Columns->AddRange(gcnew cli::array< System::Windows::Forms::ColumnHeader^  >(1) { this->columnHeader1 });
			this->listView1->Enabled = false;
			this->listView1->HideSelection = false;
			this->listView1->Location = System::Drawing::Point(1272, 28);
			this->listView1->Name = L"listView1";
			this->listView1->Size = System::Drawing::Size(41, 22);
			this->listView1->TabIndex = 3;
			this->listView1->UseCompatibleStateImageBehavior = false;
			this->listView1->Visible = false;
			this->listView1->MouseClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::listView1_MouseClick);
			// 
			// columnHeader1
			// 
			this->columnHeader1->Text = L"Related frames";
			// 
			// view_button
			// 
			this->view_button->Enabled = false;
			this->view_button->Location = System::Drawing::Point(1199, 29);
			this->view_button->Name = L"view_button";
			this->view_button->Size = System::Drawing::Size(67, 22);
			this->view_button->TabIndex = 4;
			this->view_button->Text = L"Click";
			this->view_button->UseVisualStyleBackColor = true;
			this->view_button->Visible = false;
			this->view_button->Click += gcnew System::EventHandler(this, &MainForm::view_button_Click);
			// 
			// imageList1
			// 
			this->imageList1->ColorDepth = System::Windows::Forms::ColorDepth::Depth8Bit;
			this->imageList1->ImageSize = System::Drawing::Size(150, 150);
			this->imageList1->TransparentColor = System::Drawing::Color::Transparent;
			// 
			// play_button
			// 
			this->play_button->Enabled = false;
			this->play_button->Location = System::Drawing::Point(498, 587);
			this->play_button->Name = L"play_button";
			this->play_button->Size = System::Drawing::Size(80, 23);
			this->play_button->TabIndex = 5;
			this->play_button->Text = L"Play";
			this->play_button->UseVisualStyleBackColor = true;
			this->play_button->Click += gcnew System::EventHandler(this, &MainForm::play_button_Click);
			// 
			// speed_button
			// 
			this->speed_button->Enabled = false;
			this->speed_button->Location = System::Drawing::Point(583, 587);
			this->speed_button->Name = L"speed_button";
			this->speed_button->Size = System::Drawing::Size(80, 23);
			this->speed_button->TabIndex = 6;
			this->speed_button->Text = L"x2";
			this->speed_button->UseVisualStyleBackColor = true;
			this->speed_button->Click += gcnew System::EventHandler(this, &MainForm::speed_button_Click);
			// 
			// load_button
			// 
			this->load_button->Enabled = false;
			this->load_button->Location = System::Drawing::Point(414, 587);
			this->load_button->Name = L"load_button";
			this->load_button->Size = System::Drawing::Size(80, 23);
			this->load_button->TabIndex = 7;
			this->load_button->Text = L"Load";
			this->load_button->UseVisualStyleBackColor = true;
			this->load_button->Click += gcnew System::EventHandler(this, &MainForm::load_button_Click);
			// 
			// video_trackBar
			// 
			this->video_trackBar->Enabled = false;
			this->video_trackBar->Location = System::Drawing::Point(504, 541);
			this->video_trackBar->Maximum = 150;
			this->video_trackBar->Name = L"video_trackBar";
			this->video_trackBar->Size = System::Drawing::Size(521, 45);
			this->video_trackBar->TabIndex = 8;
			this->video_trackBar->Scroll += gcnew System::EventHandler(this, &MainForm::video_trackBar_Scroll);
			// 
			// video_label
			// 
			this->video_label->AutoSize = true;
			this->video_label->Location = System::Drawing::Point(1020, 546);
			this->video_label->Name = L"video_label";
			this->video_label->Size = System::Drawing::Size(24, 13);
			this->video_label->TabIndex = 9;
			this->video_label->Text = L"0/0";
			// 
			// record_button
			// 
			this->record_button->BackColor = System::Drawing::SystemColors::ControlLight;
			this->record_button->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->record_button->Location = System::Drawing::Point(414, 549);
			this->record_button->Name = L"record_button";
			this->record_button->Size = System::Drawing::Size(80, 34);
			this->record_button->TabIndex = 10;
			this->record_button->Text = L"Start recording";
			this->record_button->UseVisualStyleBackColor = false;
			this->record_button->Click += gcnew System::EventHandler(this, &MainForm::record_button_Click);
			// 
			// backgroundWorkerJetson
			// 
			this->backgroundWorkerJetson->WorkerSupportsCancellation = true;
			this->backgroundWorkerJetson->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerJetson_DoWork);
			// 
			// backgroundWorkerRobot
			// 
			this->backgroundWorkerRobot->WorkerReportsProgress = true;
			this->backgroundWorkerRobot->WorkerSupportsCancellation = true;
			this->backgroundWorkerRobot->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerRobot_DoWork);
			this->backgroundWorkerRobot->ProgressChanged += gcnew System::ComponentModel::ProgressChangedEventHandler(this, &MainForm::backgroundWorkerRobot_ProgressChanged);
			// 
			// backgroundWorkerArduino
			// 
			this->backgroundWorkerArduino->WorkerReportsProgress = true;
			this->backgroundWorkerArduino->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerArduino_DoWork);
			this->backgroundWorkerArduino->ProgressChanged += gcnew System::ComponentModel::ProgressChangedEventHandler(this, &MainForm::backgroundWorkerArduino_ProgressChanged);
			// 
			// backgroundWorkerDVL
			// 
			this->backgroundWorkerDVL->WorkerReportsProgress = true;
			this->backgroundWorkerDVL->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerDVL_DoWork);
			this->backgroundWorkerDVL->ProgressChanged += gcnew System::ComponentModel::ProgressChangedEventHandler(this, &MainForm::backgroundWorkerDVL_ProgressChanged);
			// 
			// buttonDVLon
			// 
			this->buttonDVLon->Enabled = false;
			this->buttonDVLon->Location = System::Drawing::Point(106, 12);
			this->buttonDVLon->Name = L"buttonDVLon";
			this->buttonDVLon->Size = System::Drawing::Size(80, 34);
			this->buttonDVLon->TabIndex = 24;
			this->buttonDVLon->Text = L"Arduino init...";
			this->buttonDVLon->UseVisualStyleBackColor = true;
			this->buttonDVLon->Click += gcnew System::EventHandler(this, &MainForm::buttonDVLon_Click);
			// 
			// buttonDVLyes
			// 
			this->buttonDVLyes->Location = System::Drawing::Point(560, 28);
			this->buttonDVLyes->Name = L"buttonDVLyes";
			this->buttonDVLyes->Size = System::Drawing::Size(41, 21);
			this->buttonDVLyes->TabIndex = 25;
			this->buttonDVLyes->Text = L"Yes";
			this->buttonDVLyes->UseVisualStyleBackColor = true;
			this->buttonDVLyes->Visible = false;
			this->buttonDVLyes->Click += gcnew System::EventHandler(this, &MainForm::buttonDVLyes_Click);
			// 
			// buttonDVLno
			// 
			this->buttonDVLno->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->buttonDVLno->Location = System::Drawing::Point(626, 28);
			this->buttonDVLno->Name = L"buttonDVLno";
			this->buttonDVLno->Size = System::Drawing::Size(41, 21);
			this->buttonDVLno->TabIndex = 26;
			this->buttonDVLno->Text = L"No";
			this->buttonDVLno->UseVisualStyleBackColor = true;
			this->buttonDVLno->Visible = false;
			this->buttonDVLno->Click += gcnew System::EventHandler(this, &MainForm::buttonDVLno_Click);
			// 
			// labelDVLon
			// 
			this->labelDVLon->AutoSize = true;
			this->labelDVLon->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->labelDVLon->Location = System::Drawing::Point(543, 12);
			this->labelDVLon->Name = L"labelDVLon";
			this->labelDVLon->Size = System::Drawing::Size(141, 13);
			this->labelDVLon->TabIndex = 27;
			this->labelDVLon->Text = L"Is the DVL underwater\?";
			this->labelDVLon->Visible = false;
			// 
			// backgroundWorkerRobotCommand
			// 
			this->backgroundWorkerRobotCommand->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerRobotCommand_DoWork);
			this->backgroundWorkerRobotCommand->RunWorkerCompleted += gcnew System::ComponentModel::RunWorkerCompletedEventHandler(this, &MainForm::backgroundWorkerRobotCommand_RunWorkerCompleted);
			// 
			// buttonJetsonOn
			// 
			this->buttonJetsonOn->Enabled = false;
			this->buttonJetsonOn->Location = System::Drawing::Point(12, 12);
			this->buttonJetsonOn->Name = L"buttonJetsonOn";
			this->buttonJetsonOn->Size = System::Drawing::Size(80, 34);
			this->buttonJetsonOn->TabIndex = 28;
			this->buttonJetsonOn->Text = L"Arduino init...";
			this->buttonJetsonOn->UseVisualStyleBackColor = true;
			this->buttonJetsonOn->Click += gcnew System::EventHandler(this, &MainForm::buttonJetsonOn_Click);
			// 
			// backgroundWorkerJetsonOn
			// 
			this->backgroundWorkerJetsonOn->WorkerSupportsCancellation = true;
			this->backgroundWorkerJetsonOn->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerJetsonOn_DoWork);
			this->backgroundWorkerJetsonOn->RunWorkerCompleted += gcnew System::ComponentModel::RunWorkerCompletedEventHandler(this, &MainForm::backgroundWorkerJetsonOn_RunWorkerCompleted);
			// 
			// backgroundWorkerJetsonOff
			// 
			this->backgroundWorkerJetsonOff->WorkerSupportsCancellation = true;
			this->backgroundWorkerJetsonOff->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerJetsonOff_DoWork);
			this->backgroundWorkerJetsonOff->RunWorkerCompleted += gcnew System::ComponentModel::RunWorkerCompletedEventHandler(this, &MainForm::backgroundWorkerJetsonOff_RunWorkerCompleted);
			// 
			// backgroundWorkerArduinoStarted
			// 
			this->backgroundWorkerArduinoStarted->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerArduinoStarted_DoWork);
			this->backgroundWorkerArduinoStarted->RunWorkerCompleted += gcnew System::ComponentModel::RunWorkerCompletedEventHandler(this, &MainForm::backgroundWorkerArduinoStarted_RunWorkerCompleted);
			// 
			// label3dY
			// 
			this->label3dY->AutoSize = true;
			this->label3dY->Location = System::Drawing::Point(980, 26);
			this->label3dY->Name = L"label3dY";
			this->label3dY->Size = System::Drawing::Size(44, 13);
			this->label3dY->TabIndex = 32;
			this->label3dY->Text = L"y: None";
			this->label3dY->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			this->label3dY->Visible = false;
			// 
			// label3dX
			// 
			this->label3dX->AutoSize = true;
			this->label3dX->Location = System::Drawing::Point(930, 26);
			this->label3dX->Name = L"label3dX";
			this->label3dX->Size = System::Drawing::Size(44, 13);
			this->label3dX->TabIndex = 31;
			this->label3dX->Text = L"x: None";
			this->label3dX->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			this->label3dX->Visible = false;
			// 
			// label3dZ
			// 
			this->label3dZ->AutoSize = true;
			this->label3dZ->Location = System::Drawing::Point(1030, 26);
			this->label3dZ->Name = L"label3dZ";
			this->label3dZ->Size = System::Drawing::Size(44, 13);
			this->label3dZ->TabIndex = 33;
			this->label3dZ->Text = L"z: None";
			this->label3dZ->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			this->label3dZ->Visible = false;
			// 
			// labelDVL
			// 
			this->labelDVL->AutoSize = true;
			this->labelDVL->Location = System::Drawing::Point(886, 42);
			this->labelDVL->Name = L"labelDVL";
			this->labelDVL->Size = System::Drawing::Size(34, 13);
			this->labelDVL->TabIndex = 34;
			this->labelDVL->Text = L"DVL: ";
			this->labelDVL->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			this->labelDVL->Visible = false;
			// 
			// label3dZFake
			// 
			this->label3dZFake->AutoSize = true;
			this->label3dZFake->Location = System::Drawing::Point(1030, 42);
			this->label3dZFake->Name = L"label3dZFake";
			this->label3dZFake->Size = System::Drawing::Size(44, 13);
			this->label3dZFake->TabIndex = 38;
			this->label3dZFake->Text = L"z: None";
			this->label3dZFake->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			this->label3dZFake->Visible = false;
			// 
			// label3dYFake
			// 
			this->label3dYFake->AutoSize = true;
			this->label3dYFake->Location = System::Drawing::Point(980, 42);
			this->label3dYFake->Name = L"label3dYFake";
			this->label3dYFake->Size = System::Drawing::Size(44, 13);
			this->label3dYFake->TabIndex = 37;
			this->label3dYFake->Text = L"y: None";
			this->label3dYFake->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			this->label3dYFake->Visible = false;
			// 
			// label3dXFake
			// 
			this->label3dXFake->AutoSize = true;
			this->label3dXFake->Location = System::Drawing::Point(930, 42);
			this->label3dXFake->Name = L"label3dXFake";
			this->label3dXFake->Size = System::Drawing::Size(44, 13);
			this->label3dXFake->TabIndex = 36;
			this->label3dXFake->Text = L"x: None";
			this->label3dXFake->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			this->label3dXFake->Visible = false;
			// 
			// labelLine
			// 
			this->labelLine->AutoSize = true;
			this->labelLine->Location = System::Drawing::Point(1097, 29);
			this->labelLine->Name = L"labelLine";
			this->labelLine->Size = System::Drawing::Size(59, 13);
			this->labelLine->TabIndex = 39;
			this->labelLine->Text = L"Line: None";
			this->labelLine->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			this->labelLine->Visible = false;
			// 
			// labelLineFake
			// 
			this->labelLineFake->AutoSize = true;
			this->labelLineFake->Location = System::Drawing::Point(1097, 42);
			this->labelLineFake->Name = L"labelLineFake";
			this->labelLineFake->Size = System::Drawing::Size(59, 13);
			this->labelLineFake->TabIndex = 40;
			this->labelLineFake->Text = L"Line: None";
			this->labelLineFake->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			this->labelLineFake->Visible = false;
			// 
			// backgroundWorkerDepthMap
			// 
			this->backgroundWorkerDepthMap->WorkerReportsProgress = true;
			this->backgroundWorkerDepthMap->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerDepthMap_DoWork);
			this->backgroundWorkerDepthMap->ProgressChanged += gcnew System::ComponentModel::ProgressChangedEventHandler(this, &MainForm::backgroundWorkerDepthMap_ProgressChanged);
			// 
			// ptbDepthMap
			// 
			this->ptbDepthMap->BackColor = System::Drawing::SystemColors::ControlDark;
			this->ptbDepthMap->Location = System::Drawing::Point(13, 58);
			this->ptbDepthMap->Name = L"ptbDepthMap";
			this->ptbDepthMap->Size = System::Drawing::Size(320, 520);
			this->ptbDepthMap->TabIndex = 41;
			this->ptbDepthMap->TabStop = false;
			this->ptbDepthMap->MouseMove += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::ptbDepthMap_MouseMove);
			// 
			// buttonGoToTarget
			// 
			this->buttonGoToTarget->Enabled = false;
			this->buttonGoToTarget->Location = System::Drawing::Point(201, 64);
			this->buttonGoToTarget->Name = L"buttonGoToTarget";
			this->buttonGoToTarget->Size = System::Drawing::Size(72, 22);
			this->buttonGoToTarget->TabIndex = 44;
			this->buttonGoToTarget->Text = L"Go to target";
			this->buttonGoToTarget->UseVisualStyleBackColor = true;
			this->buttonGoToTarget->Click += gcnew System::EventHandler(this, &MainForm::buttonGoToTarget_Click);
			// 
			// backgroundWorkerRobotStarted
			// 
			this->backgroundWorkerRobotStarted->WorkerReportsProgress = true;
			this->backgroundWorkerRobotStarted->WorkerSupportsCancellation = true;
			this->backgroundWorkerRobotStarted->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerRobotStarted_DoWork);
			this->backgroundWorkerRobotStarted->RunWorkerCompleted += gcnew System::ComponentModel::RunWorkerCompletedEventHandler(this, &MainForm::backgroundWorkerRobotStarted_RunWorkerCompleted);
			// 
			// backgroundWorkerDVLOn
			// 
			this->backgroundWorkerDVLOn->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerDVLOn_DoWork);
			this->backgroundWorkerDVLOn->RunWorkerCompleted += gcnew System::ComponentModel::RunWorkerCompletedEventHandler(this, &MainForm::backgroundWorkerDVLOn_RunWorkerCompleted);
			// 
			// buttonScan
			// 
			this->buttonScan->Enabled = false;
			this->buttonScan->Location = System::Drawing::Point(201, 39);
			this->buttonScan->Name = L"buttonScan";
			this->buttonScan->Size = System::Drawing::Size(72, 22);
			this->buttonScan->TabIndex = 45;
			this->buttonScan->Text = L"Scan";
			this->buttonScan->UseVisualStyleBackColor = true;
			this->buttonScan->Click += gcnew System::EventHandler(this, &MainForm::buttonScan_Click);
			// 
			// backgroundWorkerRobotScan
			// 
			this->backgroundWorkerRobotScan->WorkerReportsProgress = true;
			this->backgroundWorkerRobotScan->WorkerSupportsCancellation = true;
			this->backgroundWorkerRobotScan->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerRobotScan_DoWork);
			this->backgroundWorkerRobotScan->RunWorkerCompleted += gcnew System::ComponentModel::RunWorkerCompletedEventHandler(this, &MainForm::backgroundWorkerRobotScan_RunWorkerCompleted);
			// 
			// textBox1
			// 
			this->textBox1->Location = System::Drawing::Point(280, 39);
			this->textBox1->Name = L"textBox1";
			this->textBox1->Size = System::Drawing::Size(68, 20);
			this->textBox1->TabIndex = 46;
			this->textBox1->Text = L"0";
			this->textBox1->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox1->TextChanged += gcnew System::EventHandler(this, &MainForm::textBox1_TextChanged);
			// 
			// labelDepthScan
			// 
			this->labelDepthScan->AutoSize = true;
			this->labelDepthScan->Location = System::Drawing::Point(275, 23);
			this->labelDepthScan->Name = L"labelDepthScan";
			this->labelDepthScan->Size = System::Drawing::Size(82, 13);
			this->labelDepthScan->TabIndex = 47;
			this->labelDepthScan->Text = L"Scanning depth";
			this->labelDepthScan->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// buttonResolutionMap
			// 
			this->buttonResolutionMap->Location = System::Drawing::Point(12, 584);
			this->buttonResolutionMap->Name = L"buttonResolutionMap";
			this->buttonResolutionMap->Size = System::Drawing::Size(119, 23);
			this->buttonResolutionMap->TabIndex = 57;
			this->buttonResolutionMap->Text = L"Resolution 25x25 cm";
			this->buttonResolutionMap->UseVisualStyleBackColor = true;
			this->buttonResolutionMap->Click += gcnew System::EventHandler(this, &MainForm::buttonResolutionMap_Click);
			// 
			// buttonSaveDepthMap
			// 
			this->buttonSaveDepthMap->Location = System::Drawing::Point(258, 584);
			this->buttonSaveDepthMap->Name = L"buttonSaveDepthMap";
			this->buttonSaveDepthMap->Size = System::Drawing::Size(75, 23);
			this->buttonSaveDepthMap->TabIndex = 59;
			this->buttonSaveDepthMap->Text = L"Save";
			this->buttonSaveDepthMap->UseVisualStyleBackColor = true;
			this->buttonSaveDepthMap->Click += gcnew System::EventHandler(this, &MainForm::buttonSaveDepthMap_Click);
			// 
			// backgroundWorkerPlateformS
			// 
			this->backgroundWorkerPlateformS->WorkerReportsProgress = true;
			this->backgroundWorkerPlateformS->WorkerSupportsCancellation = true;
			this->backgroundWorkerPlateformS->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerPlateformS_DoWork);
			this->backgroundWorkerPlateformS->ProgressChanged += gcnew System::ComponentModel::ProgressChangedEventHandler(this, &MainForm::backgroundWorkerPlateformS_ProgressChanged);
			// 
			// backgroundWorkerPlateformP
			// 
			this->backgroundWorkerPlateformP->WorkerReportsProgress = true;
			this->backgroundWorkerPlateformP->WorkerSupportsCancellation = true;
			this->backgroundWorkerPlateformP->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerPlateformP_DoWork);
			this->backgroundWorkerPlateformP->ProgressChanged += gcnew System::ComponentModel::ProgressChangedEventHandler(this, &MainForm::backgroundWorkerPlateformP_ProgressChanged);
			// 
			// buttonFreezeDM
			// 
			this->buttonFreezeDM->Location = System::Drawing::Point(177, 584);
			this->buttonFreezeDM->Name = L"buttonFreezeDM";
			this->buttonFreezeDM->Size = System::Drawing::Size(75, 23);
			this->buttonFreezeDM->TabIndex = 63;
			this->buttonFreezeDM->Text = L"Freeze DM";
			this->buttonFreezeDM->UseVisualStyleBackColor = true;
			this->buttonFreezeDM->Click += gcnew System::EventHandler(this, &MainForm::buttonFreezeDM_Click);
			// 
			// textBoxTide
			// 
			this->textBoxTide->Location = System::Drawing::Point(225, 41);
			this->textBoxTide->Name = L"textBoxTide";
			this->textBoxTide->ReadOnly = true;
			this->textBoxTide->Size = System::Drawing::Size(47, 20);
			this->textBoxTide->TabIndex = 65;
			this->textBoxTide->Text = L"None";
			this->textBoxTide->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxRobotX
			// 
			this->textBoxRobotX->Location = System::Drawing::Point(6, 63);
			this->textBoxRobotX->Name = L"textBoxRobotX";
			this->textBoxRobotX->ReadOnly = true;
			this->textBoxRobotX->Size = System::Drawing::Size(47, 20);
			this->textBoxRobotX->TabIndex = 68;
			this->textBoxRobotX->Text = L"None";
			this->textBoxRobotX->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxRobotX->TextChanged += gcnew System::EventHandler(this, &MainForm::textBoxRobotX_TextChanged);
			// 
			// textBoxRobotY
			// 
			this->textBoxRobotY->Location = System::Drawing::Point(59, 63);
			this->textBoxRobotY->Name = L"textBoxRobotY";
			this->textBoxRobotY->ReadOnly = true;
			this->textBoxRobotY->Size = System::Drawing::Size(47, 20);
			this->textBoxRobotY->TabIndex = 69;
			this->textBoxRobotY->Text = L"None";
			this->textBoxRobotY->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxRobotY->TextChanged += gcnew System::EventHandler(this, &MainForm::textBoxRobotY_TextChanged);
			// 
			// textBoxRobotZ
			// 
			this->textBoxRobotZ->Location = System::Drawing::Point(112, 63);
			this->textBoxRobotZ->Name = L"textBoxRobotZ";
			this->textBoxRobotZ->ReadOnly = true;
			this->textBoxRobotZ->Size = System::Drawing::Size(47, 20);
			this->textBoxRobotZ->TabIndex = 70;
			this->textBoxRobotZ->Text = L"None";
			this->textBoxRobotZ->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxRobotZ->TextChanged += gcnew System::EventHandler(this, &MainForm::textBoxRobotZ_TextChanged);
			// 
			// textBoxRobotState
			// 
			this->textBoxRobotState->Location = System::Drawing::Point(41, 24);
			this->textBoxRobotState->Name = L"textBoxRobotState";
			this->textBoxRobotState->ReadOnly = true;
			this->textBoxRobotState->Size = System::Drawing::Size(117, 20);
			this->textBoxRobotState->TabIndex = 71;
			this->textBoxRobotState->Text = L"None";
			this->textBoxRobotState->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxRobotYaw
			// 
			this->textBoxRobotYaw->Location = System::Drawing::Point(112, 104);
			this->textBoxRobotYaw->Name = L"textBoxRobotYaw";
			this->textBoxRobotYaw->ReadOnly = true;
			this->textBoxRobotYaw->Size = System::Drawing::Size(47, 20);
			this->textBoxRobotYaw->TabIndex = 74;
			this->textBoxRobotYaw->Text = L"None";
			this->textBoxRobotYaw->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxRobotYaw->TextChanged += gcnew System::EventHandler(this, &MainForm::textBoxRobotYaw_TextChanged);
			// 
			// textBoxRobotPitch
			// 
			this->textBoxRobotPitch->Location = System::Drawing::Point(59, 104);
			this->textBoxRobotPitch->Name = L"textBoxRobotPitch";
			this->textBoxRobotPitch->ReadOnly = true;
			this->textBoxRobotPitch->Size = System::Drawing::Size(47, 20);
			this->textBoxRobotPitch->TabIndex = 73;
			this->textBoxRobotPitch->Text = L"None";
			this->textBoxRobotPitch->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxRobotPitch->TextChanged += gcnew System::EventHandler(this, &MainForm::textBoxRobotPitch_TextChanged);
			// 
			// textBoxRobotRoll
			// 
			this->textBoxRobotRoll->Location = System::Drawing::Point(6, 104);
			this->textBoxRobotRoll->Name = L"textBoxRobotRoll";
			this->textBoxRobotRoll->ReadOnly = true;
			this->textBoxRobotRoll->Size = System::Drawing::Size(47, 20);
			this->textBoxRobotRoll->TabIndex = 72;
			this->textBoxRobotRoll->Text = L"None";
			this->textBoxRobotRoll->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxRobotRoll->TextChanged += gcnew System::EventHandler(this, &MainForm::textBoxRobotRoll_TextChanged);
			// 
			// textBoxGripper
			// 
			this->textBoxGripper->Location = System::Drawing::Point(202, 104);
			this->textBoxGripper->Name = L"textBoxGripper";
			this->textBoxGripper->ReadOnly = true;
			this->textBoxGripper->Size = System::Drawing::Size(47, 20);
			this->textBoxGripper->TabIndex = 75;
			this->textBoxGripper->Text = L"OFF";
			this->textBoxGripper->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPump
			// 
			this->textBoxPump->Location = System::Drawing::Point(260, 104);
			this->textBoxPump->Name = L"textBoxPump";
			this->textBoxPump->ReadOnly = true;
			this->textBoxPump->Size = System::Drawing::Size(47, 20);
			this->textBoxPump->TabIndex = 76;
			this->textBoxPump->Text = L"OFF";
			this->textBoxPump->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxArduinoYaw
			// 
			this->textBoxArduinoYaw->Location = System::Drawing::Point(112, 41);
			this->textBoxArduinoYaw->Name = L"textBoxArduinoYaw";
			this->textBoxArduinoYaw->ReadOnly = true;
			this->textBoxArduinoYaw->Size = System::Drawing::Size(47, 20);
			this->textBoxArduinoYaw->TabIndex = 79;
			this->textBoxArduinoYaw->Text = L"None";
			this->textBoxArduinoYaw->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxArduinoPitch
			// 
			this->textBoxArduinoPitch->Location = System::Drawing::Point(59, 41);
			this->textBoxArduinoPitch->Name = L"textBoxArduinoPitch";
			this->textBoxArduinoPitch->ReadOnly = true;
			this->textBoxArduinoPitch->Size = System::Drawing::Size(47, 20);
			this->textBoxArduinoPitch->TabIndex = 78;
			this->textBoxArduinoPitch->Text = L"None";
			this->textBoxArduinoPitch->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxArduinoRoll
			// 
			this->textBoxArduinoRoll->Location = System::Drawing::Point(6, 41);
			this->textBoxArduinoRoll->Name = L"textBoxArduinoRoll";
			this->textBoxArduinoRoll->ReadOnly = true;
			this->textBoxArduinoRoll->Size = System::Drawing::Size(47, 20);
			this->textBoxArduinoRoll->TabIndex = 77;
			this->textBoxArduinoRoll->Text = L"None";
			this->textBoxArduinoRoll->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxArduinoDepth
			// 
			this->textBoxArduinoDepth->Location = System::Drawing::Point(172, 41);
			this->textBoxArduinoDepth->Name = L"textBoxArduinoDepth";
			this->textBoxArduinoDepth->ReadOnly = true;
			this->textBoxArduinoDepth->Size = System::Drawing::Size(47, 20);
			this->textBoxArduinoDepth->TabIndex = 80;
			this->textBoxArduinoDepth->Text = L"None";
			this->textBoxArduinoDepth->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxAtmPressure
			// 
			this->textBoxAtmPressure->Location = System::Drawing::Point(281, 41);
			this->textBoxAtmPressure->Name = L"textBoxAtmPressure";
			this->textBoxAtmPressure->ReadOnly = true;
			this->textBoxAtmPressure->Size = System::Drawing::Size(47, 20);
			this->textBoxAtmPressure->TabIndex = 81;
			this->textBoxAtmPressure->Text = L"None";
			this->textBoxAtmPressure->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxDVLVy
			// 
			this->textBoxDVLVy->Location = System::Drawing::Point(278, 40);
			this->textBoxDVLVy->Name = L"textBoxDVLVy";
			this->textBoxDVLVy->ReadOnly = true;
			this->textBoxDVLVy->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLVy->TabIndex = 86;
			this->textBoxDVLVy->Text = L"None";
			this->textBoxDVLVy->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxDVLVy->TextChanged += gcnew System::EventHandler(this, &MainForm::textBoxDVLVy_TextChanged);
			// 
			// textBoxDVLVx
			// 
			this->textBoxDVLVx->Location = System::Drawing::Point(225, 40);
			this->textBoxDVLVx->Name = L"textBoxDVLVx";
			this->textBoxDVLVx->ReadOnly = true;
			this->textBoxDVLVx->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLVx->TabIndex = 85;
			this->textBoxDVLVx->Text = L"None";
			this->textBoxDVLVx->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxDVLVx->TextChanged += gcnew System::EventHandler(this, &MainForm::textBoxDVLVx_TextChanged);
			// 
			// textBoxDVLYaw
			// 
			this->textBoxDVLYaw->Location = System::Drawing::Point(112, 40);
			this->textBoxDVLYaw->Name = L"textBoxDVLYaw";
			this->textBoxDVLYaw->ReadOnly = true;
			this->textBoxDVLYaw->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLYaw->TabIndex = 89;
			this->textBoxDVLYaw->Text = L"None";
			this->textBoxDVLYaw->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxDVLPitch
			// 
			this->textBoxDVLPitch->Location = System::Drawing::Point(59, 40);
			this->textBoxDVLPitch->Name = L"textBoxDVLPitch";
			this->textBoxDVLPitch->ReadOnly = true;
			this->textBoxDVLPitch->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLPitch->TabIndex = 88;
			this->textBoxDVLPitch->Text = L"None";
			this->textBoxDVLPitch->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxDVLRoll
			// 
			this->textBoxDVLRoll->Location = System::Drawing::Point(6, 40);
			this->textBoxDVLRoll->Name = L"textBoxDVLRoll";
			this->textBoxDVLRoll->ReadOnly = true;
			this->textBoxDVLRoll->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLRoll->TabIndex = 87;
			this->textBoxDVLRoll->Text = L"None";
			this->textBoxDVLRoll->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxDVLD2
			// 
			this->textBoxDVLD2->Location = System::Drawing::Point(112, 87);
			this->textBoxDVLD2->Name = L"textBoxDVLD2";
			this->textBoxDVLD2->ReadOnly = true;
			this->textBoxDVLD2->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLD2->TabIndex = 92;
			this->textBoxDVLD2->Text = L"None";
			this->textBoxDVLD2->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxDVLD1
			// 
			this->textBoxDVLD1->Location = System::Drawing::Point(59, 87);
			this->textBoxDVLD1->Name = L"textBoxDVLD1";
			this->textBoxDVLD1->ReadOnly = true;
			this->textBoxDVLD1->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLD1->TabIndex = 91;
			this->textBoxDVLD1->Text = L"None";
			this->textBoxDVLD1->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxDVLD0
			// 
			this->textBoxDVLD0->Location = System::Drawing::Point(6, 87);
			this->textBoxDVLD0->Name = L"textBoxDVLD0";
			this->textBoxDVLD0->ReadOnly = true;
			this->textBoxDVLD0->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLD0->TabIndex = 90;
			this->textBoxDVLD0->Text = L"None";
			this->textBoxDVLD0->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxDVLD3
			// 
			this->textBoxDVLD3->Location = System::Drawing::Point(165, 87);
			this->textBoxDVLD3->Name = L"textBoxDVLD3";
			this->textBoxDVLD3->ReadOnly = true;
			this->textBoxDVLD3->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLD3->TabIndex = 93;
			this->textBoxDVLD3->Text = L"None";
			this->textBoxDVLD3->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPYaw
			// 
			this->textBoxPYaw->Location = System::Drawing::Point(112, 34);
			this->textBoxPYaw->Name = L"textBoxPYaw";
			this->textBoxPYaw->ReadOnly = true;
			this->textBoxPYaw->Size = System::Drawing::Size(47, 20);
			this->textBoxPYaw->TabIndex = 96;
			this->textBoxPYaw->Text = L"None";
			this->textBoxPYaw->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPPitch
			// 
			this->textBoxPPitch->Location = System::Drawing::Point(59, 34);
			this->textBoxPPitch->Name = L"textBoxPPitch";
			this->textBoxPPitch->ReadOnly = true;
			this->textBoxPPitch->Size = System::Drawing::Size(47, 20);
			this->textBoxPPitch->TabIndex = 95;
			this->textBoxPPitch->Text = L"None";
			this->textBoxPPitch->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPRoll
			// 
			this->textBoxPRoll->Location = System::Drawing::Point(6, 34);
			this->textBoxPRoll->Name = L"textBoxPRoll";
			this->textBoxPRoll->ReadOnly = true;
			this->textBoxPRoll->Size = System::Drawing::Size(47, 20);
			this->textBoxPRoll->TabIndex = 94;
			this->textBoxPRoll->Text = L"None";
			this->textBoxPRoll->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxPRoll->TextChanged += gcnew System::EventHandler(this, &MainForm::textBoxPRoll_TextChanged);
			// 
			// textBoxPGyrZ
			// 
			this->textBoxPGyrZ->Location = System::Drawing::Point(112, 77);
			this->textBoxPGyrZ->Name = L"textBoxPGyrZ";
			this->textBoxPGyrZ->ReadOnly = true;
			this->textBoxPGyrZ->Size = System::Drawing::Size(47, 20);
			this->textBoxPGyrZ->TabIndex = 99;
			this->textBoxPGyrZ->Text = L"None";
			this->textBoxPGyrZ->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPGyrY
			// 
			this->textBoxPGyrY->Location = System::Drawing::Point(59, 77);
			this->textBoxPGyrY->Name = L"textBoxPGyrY";
			this->textBoxPGyrY->ReadOnly = true;
			this->textBoxPGyrY->Size = System::Drawing::Size(47, 20);
			this->textBoxPGyrY->TabIndex = 98;
			this->textBoxPGyrY->Text = L"None";
			this->textBoxPGyrY->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPGyrX
			// 
			this->textBoxPGyrX->Location = System::Drawing::Point(6, 77);
			this->textBoxPGyrX->Name = L"textBoxPGyrX";
			this->textBoxPGyrX->ReadOnly = true;
			this->textBoxPGyrX->Size = System::Drawing::Size(47, 20);
			this->textBoxPGyrX->TabIndex = 97;
			this->textBoxPGyrX->Text = L"None";
			this->textBoxPGyrX->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSYaw
			// 
			this->textBoxSYaw->Location = System::Drawing::Point(112, 34);
			this->textBoxSYaw->Name = L"textBoxSYaw";
			this->textBoxSYaw->ReadOnly = true;
			this->textBoxSYaw->Size = System::Drawing::Size(47, 20);
			this->textBoxSYaw->TabIndex = 102;
			this->textBoxSYaw->Text = L"None";
			this->textBoxSYaw->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSPitch
			// 
			this->textBoxSPitch->Location = System::Drawing::Point(59, 34);
			this->textBoxSPitch->Name = L"textBoxSPitch";
			this->textBoxSPitch->ReadOnly = true;
			this->textBoxSPitch->Size = System::Drawing::Size(47, 20);
			this->textBoxSPitch->TabIndex = 101;
			this->textBoxSPitch->Text = L"None";
			this->textBoxSPitch->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSRoll
			// 
			this->textBoxSRoll->Location = System::Drawing::Point(6, 34);
			this->textBoxSRoll->Name = L"textBoxSRoll";
			this->textBoxSRoll->ReadOnly = true;
			this->textBoxSRoll->Size = System::Drawing::Size(47, 20);
			this->textBoxSRoll->TabIndex = 100;
			this->textBoxSRoll->Text = L"None";
			this->textBoxSRoll->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSGyrZ
			// 
			this->textBoxSGyrZ->Location = System::Drawing::Point(112, 77);
			this->textBoxSGyrZ->Name = L"textBoxSGyrZ";
			this->textBoxSGyrZ->ReadOnly = true;
			this->textBoxSGyrZ->Size = System::Drawing::Size(47, 20);
			this->textBoxSGyrZ->TabIndex = 105;
			this->textBoxSGyrZ->Text = L"None";
			this->textBoxSGyrZ->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSGyrY
			// 
			this->textBoxSGyrY->Location = System::Drawing::Point(59, 77);
			this->textBoxSGyrY->Name = L"textBoxSGyrY";
			this->textBoxSGyrY->ReadOnly = true;
			this->textBoxSGyrY->Size = System::Drawing::Size(47, 20);
			this->textBoxSGyrY->TabIndex = 104;
			this->textBoxSGyrY->Text = L"None";
			this->textBoxSGyrY->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSGyrX
			// 
			this->textBoxSGyrX->Location = System::Drawing::Point(6, 77);
			this->textBoxSGyrX->Name = L"textBoxSGyrX";
			this->textBoxSGyrX->ReadOnly = true;
			this->textBoxSGyrX->Size = System::Drawing::Size(47, 20);
			this->textBoxSGyrX->TabIndex = 103;
			this->textBoxSGyrX->Text = L"None";
			this->textBoxSGyrX->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSGPSY
			// 
			this->textBoxSGPSY->Location = System::Drawing::Point(77, 121);
			this->textBoxSGPSY->Name = L"textBoxSGPSY";
			this->textBoxSGPSY->ReadOnly = true;
			this->textBoxSGPSY->Size = System::Drawing::Size(68, 20);
			this->textBoxSGPSY->TabIndex = 107;
			this->textBoxSGPSY->Text = L"None";
			this->textBoxSGPSY->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSGPSX
			// 
			this->textBoxSGPSX->Location = System::Drawing::Point(6, 121);
			this->textBoxSGPSX->Name = L"textBoxSGPSX";
			this->textBoxSGPSX->ReadOnly = true;
			this->textBoxSGPSX->Size = System::Drawing::Size(68, 20);
			this->textBoxSGPSX->TabIndex = 106;
			this->textBoxSGPSX->Text = L"None";
			this->textBoxSGPSX->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPGPSY
			// 
			this->textBoxPGPSY->Location = System::Drawing::Point(76, 121);
			this->textBoxPGPSY->Name = L"textBoxPGPSY";
			this->textBoxPGPSY->ReadOnly = true;
			this->textBoxPGPSY->Size = System::Drawing::Size(68, 20);
			this->textBoxPGPSY->TabIndex = 110;
			this->textBoxPGPSY->Text = L"None";
			this->textBoxPGPSY->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPGPSX
			// 
			this->textBoxPGPSX->Location = System::Drawing::Point(5, 121);
			this->textBoxPGPSX->Name = L"textBoxPGPSX";
			this->textBoxPGPSX->ReadOnly = true;
			this->textBoxPGPSX->Size = System::Drawing::Size(68, 20);
			this->textBoxPGPSX->TabIndex = 109;
			this->textBoxPGPSX->Text = L"None";
			this->textBoxPGPSX->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// labelGPSP
			// 
			this->labelGPSP->AutoSize = true;
			this->labelGPSP->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->labelGPSP->Location = System::Drawing::Point(3, 105);
			this->labelGPSP->Name = L"labelGPSP";
			this->labelGPSP->Size = System::Drawing::Size(72, 13);
			this->labelGPSP->TabIndex = 111;
			this->labelGPSP->Text = L"UTM northing";
			this->labelGPSP->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// labelGPSS
			// 
			this->labelGPSS->AutoSize = true;
			this->labelGPSS->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->labelGPSS->Location = System::Drawing::Point(3, 105);
			this->labelGPSS->Name = L"labelGPSS";
			this->labelGPSS->Size = System::Drawing::Size(72, 13);
			this->labelGPSS->TabIndex = 112;
			this->labelGPSS->Text = L"UTM northing";
			this->labelGPSS->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// groupBoxRobot
			// 
			this->groupBoxRobot->BackColor = System::Drawing::Color::LightGray;
			this->groupBoxRobot->Controls->Add(this->label10);
			this->groupBoxRobot->Controls->Add(this->label9);
			this->groupBoxRobot->Controls->Add(this->label8);
			this->groupBoxRobot->Controls->Add(this->label7);
			this->groupBoxRobot->Controls->Add(this->label6);
			this->groupBoxRobot->Controls->Add(this->label4);
			this->groupBoxRobot->Controls->Add(this->label3);
			this->groupBoxRobot->Controls->Add(this->label2);
			this->groupBoxRobot->Controls->Add(this->label1);
			this->groupBoxRobot->Controls->Add(this->textBoxRobotState);
			this->groupBoxRobot->Controls->Add(this->textBoxRobotX);
			this->groupBoxRobot->Controls->Add(this->textBoxRobotY);
			this->groupBoxRobot->Controls->Add(this->textBoxRobotZ);
			this->groupBoxRobot->Controls->Add(this->textBoxRobotRoll);
			this->groupBoxRobot->Controls->Add(this->textBoxRobotPitch);
			this->groupBoxRobot->Controls->Add(this->textBoxRobotYaw);
			this->groupBoxRobot->Controls->Add(this->textBoxGripper);
			this->groupBoxRobot->Controls->Add(this->textBoxPump);
			this->groupBoxRobot->Controls->Add(this->buttonScan);
			this->groupBoxRobot->Controls->Add(this->buttonGoToTarget);
			this->groupBoxRobot->Controls->Add(this->labelDepthScan);
			this->groupBoxRobot->Controls->Add(this->textBox1);
			this->groupBoxRobot->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->groupBoxRobot->Location = System::Drawing::Point(13, 625);
			this->groupBoxRobot->Name = L"groupBoxRobot";
			this->groupBoxRobot->Size = System::Drawing::Size(370, 128);
			this->groupBoxRobot->TabIndex = 113;
			this->groupBoxRobot->TabStop = false;
			this->groupBoxRobot->Text = L"Robot";
			// 
			// label10
			// 
			this->label10->AutoSize = true;
			this->label10->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label10->Location = System::Drawing::Point(257, 88);
			this->label10->Name = L"label10";
			this->label10->Size = System::Drawing::Size(34, 13);
			this->label10->TabIndex = 117;
			this->label10->Text = L"Pump";
			this->label10->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label9->Location = System::Drawing::Point(199, 88);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(41, 13);
			this->label9->TabIndex = 117;
			this->label9->Text = L"Gripper";
			this->label9->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label8->Location = System::Drawing::Point(3, 27);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(32, 13);
			this->label8->TabIndex = 117;
			this->label8->Text = L"State";
			this->label8->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label7->Location = System::Drawing::Point(109, 88);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(41, 13);
			this->label7->TabIndex = 121;
			this->label7->Text = L"Yaw (°)";
			this->label7->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label6->Location = System::Drawing::Point(56, 88);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(44, 13);
			this->label6->TabIndex = 120;
			this->label6->Text = L"Pitch (°)";
			this->label6->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label4->Location = System::Drawing::Point(3, 88);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(38, 13);
			this->label4->TabIndex = 117;
			this->label4->Text = L"Roll (°)";
			this->label4->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label3->Location = System::Drawing::Point(110, 49);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(31, 13);
			this->label3->TabIndex = 119;
			this->label3->Text = L"Z (m)";
			this->label3->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label2->Location = System::Drawing::Point(58, 48);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(31, 13);
			this->label2->TabIndex = 118;
			this->label2->Text = L"Y (m)";
			this->label2->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label1->Location = System::Drawing::Point(5, 48);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(31, 13);
			this->label1->TabIndex = 117;
			this->label1->Text = L"X (m)";
			this->label1->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// textBoxSRTK
			// 
			this->textBoxSRTK->Location = System::Drawing::Point(148, 121);
			this->textBoxSRTK->Name = L"textBoxSRTK";
			this->textBoxSRTK->ReadOnly = true;
			this->textBoxSRTK->Size = System::Drawing::Size(28, 20);
			this->textBoxSRTK->TabIndex = 114;
			this->textBoxSRTK->Text = L"None";
			this->textBoxSRTK->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// labelTide
			// 
			this->labelTide->AutoSize = true;
			this->labelTide->Location = System::Drawing::Point(222, 25);
			this->labelTide->Name = L"labelTide";
			this->labelTide->Size = System::Drawing::Size(45, 13);
			this->labelTide->TabIndex = 116;
			this->labelTide->Text = L"Tide (m)";
			this->labelTide->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// groupBox1
			// 
			this->groupBox1->BackColor = System::Drawing::Color::LightGray;
			this->groupBox1->Controls->Add(this->label15);
			this->groupBox1->Controls->Add(this->label14);
			this->groupBox1->Controls->Add(this->label11);
			this->groupBox1->Controls->Add(this->textBoxArduinoPitch);
			this->groupBox1->Controls->Add(this->labelTide);
			this->groupBox1->Controls->Add(this->label12);
			this->groupBox1->Controls->Add(this->textBoxTide);
			this->groupBox1->Controls->Add(this->label13);
			this->groupBox1->Controls->Add(this->textBoxArduinoRoll);
			this->groupBox1->Controls->Add(this->textBoxArduinoYaw);
			this->groupBox1->Controls->Add(this->textBoxArduinoDepth);
			this->groupBox1->Controls->Add(this->textBoxAtmPressure);
			this->groupBox1->Location = System::Drawing::Point(13, 760);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(370, 67);
			this->groupBox1->TabIndex = 117;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Smartcam";
			// 
			// label15
			// 
			this->label15->AutoSize = true;
			this->label15->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label15->Location = System::Drawing::Point(270, 25);
			this->label15->Name = L"label15";
			this->label15->Size = System::Drawing::Size(100, 13);
			this->label15->TabIndex = 118;
			this->label15->Text = L"Atm pressure (mbar)";
			this->label15->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label14
			// 
			this->label14->AutoSize = true;
			this->label14->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label14->Location = System::Drawing::Point(169, 25);
			this->label14->Name = L"label14";
			this->label14->Size = System::Drawing::Size(53, 13);
			this->label14->TabIndex = 118;
			this->label14->Text = L"Depth (m)";
			this->label14->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label11
			// 
			this->label11->AutoSize = true;
			this->label11->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label11->Location = System::Drawing::Point(109, 25);
			this->label11->Name = L"label11";
			this->label11->Size = System::Drawing::Size(41, 13);
			this->label11->TabIndex = 124;
			this->label11->Text = L"Yaw (°)";
			this->label11->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label12
			// 
			this->label12->AutoSize = true;
			this->label12->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label12->Location = System::Drawing::Point(56, 25);
			this->label12->Name = L"label12";
			this->label12->Size = System::Drawing::Size(44, 13);
			this->label12->TabIndex = 123;
			this->label12->Text = L"Pitch (°)";
			this->label12->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label13
			// 
			this->label13->AutoSize = true;
			this->label13->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label13->Location = System::Drawing::Point(3, 25);
			this->label13->Name = L"label13";
			this->label13->Size = System::Drawing::Size(38, 13);
			this->label13->TabIndex = 122;
			this->label13->Text = L"Roll (°)";
			this->label13->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// groupBoxDVL
			// 
			this->groupBoxDVL->BackColor = System::Drawing::Color::LightGray;
			this->groupBoxDVL->Controls->Add(this->label23);
			this->groupBoxDVL->Controls->Add(this->label24);
			this->groupBoxDVL->Controls->Add(this->label22);
			this->groupBoxDVL->Controls->Add(this->label19);
			this->groupBoxDVL->Controls->Add(this->label20);
			this->groupBoxDVL->Controls->Add(this->label21);
			this->groupBoxDVL->Controls->Add(this->label16);
			this->groupBoxDVL->Controls->Add(this->textBoxDVLPitch);
			this->groupBoxDVL->Controls->Add(this->label17);
			this->groupBoxDVL->Controls->Add(this->textBoxDVLRoll);
			this->groupBoxDVL->Controls->Add(this->label18);
			this->groupBoxDVL->Controls->Add(this->textBoxDVLYaw);
			this->groupBoxDVL->Controls->Add(this->textBoxDVLD0);
			this->groupBoxDVL->Controls->Add(this->textBoxDVLD1);
			this->groupBoxDVL->Controls->Add(this->textBoxDVLD2);
			this->groupBoxDVL->Controls->Add(this->textBoxDVLD3);
			this->groupBoxDVL->Controls->Add(this->textBoxDVLVx);
			this->groupBoxDVL->Controls->Add(this->textBoxDVLVy);
			this->groupBoxDVL->Location = System::Drawing::Point(12, 833);
			this->groupBoxDVL->Name = L"groupBoxDVL";
			this->groupBoxDVL->Size = System::Drawing::Size(370, 113);
			this->groupBoxDVL->TabIndex = 118;
			this->groupBoxDVL->TabStop = false;
			this->groupBoxDVL->Text = L"DVL";
			// 
			// label23
			// 
			this->label23->AutoSize = true;
			this->label23->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label23->Location = System::Drawing::Point(275, 24);
			this->label23->Name = L"label23";
			this->label23->Size = System::Drawing::Size(53, 13);
			this->label23->TabIndex = 133;
			this->label23->Text = L"Vy (m.s-1)";
			this->label23->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label24
			// 
			this->label24->AutoSize = true;
			this->label24->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label24->Location = System::Drawing::Point(222, 24);
			this->label24->Name = L"label24";
			this->label24->Size = System::Drawing::Size(56, 13);
			this->label24->TabIndex = 132;
			this->label24->Text = L"Vx ( m.s-1)";
			this->label24->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label22
			// 
			this->label22->AutoSize = true;
			this->label22->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label22->Location = System::Drawing::Point(162, 71);
			this->label22->Name = L"label22";
			this->label22->Size = System::Drawing::Size(38, 13);
			this->label22->TabIndex = 131;
			this->label22->Text = L"D4 (m)";
			this->label22->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label19
			// 
			this->label19->AutoSize = true;
			this->label19->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label19->Location = System::Drawing::Point(109, 71);
			this->label19->Name = L"label19";
			this->label19->Size = System::Drawing::Size(38, 13);
			this->label19->TabIndex = 130;
			this->label19->Text = L"D3 (m)";
			this->label19->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label20
			// 
			this->label20->AutoSize = true;
			this->label20->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label20->Location = System::Drawing::Point(56, 71);
			this->label20->Name = L"label20";
			this->label20->Size = System::Drawing::Size(38, 13);
			this->label20->TabIndex = 129;
			this->label20->Text = L"D1 (m)";
			this->label20->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label21
			// 
			this->label21->AutoSize = true;
			this->label21->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label21->Location = System::Drawing::Point(3, 71);
			this->label21->Name = L"label21";
			this->label21->Size = System::Drawing::Size(38, 13);
			this->label21->TabIndex = 128;
			this->label21->Text = L"D0 (m)";
			this->label21->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label16
			// 
			this->label16->AutoSize = true;
			this->label16->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label16->Location = System::Drawing::Point(109, 24);
			this->label16->Name = L"label16";
			this->label16->Size = System::Drawing::Size(41, 13);
			this->label16->TabIndex = 127;
			this->label16->Text = L"Yaw (°)";
			this->label16->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label17
			// 
			this->label17->AutoSize = true;
			this->label17->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label17->Location = System::Drawing::Point(56, 24);
			this->label17->Name = L"label17";
			this->label17->Size = System::Drawing::Size(44, 13);
			this->label17->TabIndex = 126;
			this->label17->Text = L"Pitch (°)";
			this->label17->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label18
			// 
			this->label18->AutoSize = true;
			this->label18->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label18->Location = System::Drawing::Point(3, 24);
			this->label18->Name = L"label18";
			this->label18->Size = System::Drawing::Size(38, 13);
			this->label18->TabIndex = 125;
			this->label18->Text = L"Roll (°)";
			this->label18->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// groupBoxPort
			// 
			this->groupBoxPort->BackColor = System::Drawing::Color::LightGray;
			this->groupBoxPort->Controls->Add(this->label55);
			this->groupBoxPort->Controls->Add(this->label28);
			this->groupBoxPort->Controls->Add(this->label29);
			this->groupBoxPort->Controls->Add(this->label30);
			this->groupBoxPort->Controls->Add(this->label25);
			this->groupBoxPort->Controls->Add(this->textBoxPPitch);
			this->groupBoxPort->Controls->Add(this->label26);
			this->groupBoxPort->Controls->Add(this->label27);
			this->groupBoxPort->Controls->Add(this->textBoxPRoll);
			this->groupBoxPort->Controls->Add(this->textBoxPYaw);
			this->groupBoxPort->Controls->Add(this->labelGPSP);
			this->groupBoxPort->Controls->Add(this->textBoxPGPSY);
			this->groupBoxPort->Controls->Add(this->textBoxPGyrX);
			this->groupBoxPort->Controls->Add(this->textBoxPGPSX);
			this->groupBoxPort->Controls->Add(this->textBoxPGyrY);
			this->groupBoxPort->Controls->Add(this->textBoxPGyrZ);
			this->groupBoxPort->Location = System::Drawing::Point(389, 625);
			this->groupBoxPort->Name = L"groupBoxPort";
			this->groupBoxPort->Size = System::Drawing::Size(182, 157);
			this->groupBoxPort->TabIndex = 119;
			this->groupBoxPort->TabStop = false;
			this->groupBoxPort->Text = L"Plateform Port";
			// 
			// label55
			// 
			this->label55->AutoSize = true;
			this->label55->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label55->Location = System::Drawing::Point(74, 105);
			this->label55->Name = L"label55";
			this->label55->Size = System::Drawing::Size(68, 13);
			this->label55->TabIndex = 131;
			this->label55->Text = L"UTM easting";
			this->label55->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label28
			// 
			this->label28->AutoSize = true;
			this->label28->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label28->Location = System::Drawing::Point(111, 61);
			this->label28->Name = L"label28";
			this->label28->Size = System::Drawing::Size(49, 13);
			this->label28->TabIndex = 130;
			this->label28->Text = L"Vz (°.s-1)";
			this->label28->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label29
			// 
			this->label29->AutoSize = true;
			this->label29->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label29->Location = System::Drawing::Point(58, 61);
			this->label29->Name = L"label29";
			this->label29->Size = System::Drawing::Size(49, 13);
			this->label29->TabIndex = 129;
			this->label29->Text = L"Vy (°.s-1)";
			this->label29->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label30
			// 
			this->label30->AutoSize = true;
			this->label30->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label30->Location = System::Drawing::Point(5, 61);
			this->label30->Name = L"label30";
			this->label30->Size = System::Drawing::Size(49, 13);
			this->label30->TabIndex = 128;
			this->label30->Text = L"Vx (°.s-1)";
			this->label30->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label25
			// 
			this->label25->AutoSize = true;
			this->label25->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label25->Location = System::Drawing::Point(109, 18);
			this->label25->Name = L"label25";
			this->label25->Size = System::Drawing::Size(41, 13);
			this->label25->TabIndex = 127;
			this->label25->Text = L"Yaw (°)";
			this->label25->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label26
			// 
			this->label26->AutoSize = true;
			this->label26->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label26->Location = System::Drawing::Point(56, 18);
			this->label26->Name = L"label26";
			this->label26->Size = System::Drawing::Size(44, 13);
			this->label26->TabIndex = 126;
			this->label26->Text = L"Pitch (°)";
			this->label26->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label27
			// 
			this->label27->AutoSize = true;
			this->label27->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label27->Location = System::Drawing::Point(3, 18);
			this->label27->Name = L"label27";
			this->label27->Size = System::Drawing::Size(38, 13);
			this->label27->TabIndex = 125;
			this->label27->Text = L"Roll (°)";
			this->label27->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// groupBoxStarboard
			// 
			this->groupBoxStarboard->BackColor = System::Drawing::Color::LightGray;
			this->groupBoxStarboard->Controls->Add(this->label56);
			this->groupBoxStarboard->Controls->Add(this->label34);
			this->groupBoxStarboard->Controls->Add(this->label35);
			this->groupBoxStarboard->Controls->Add(this->label31);
			this->groupBoxStarboard->Controls->Add(this->label36);
			this->groupBoxStarboard->Controls->Add(this->label32);
			this->groupBoxStarboard->Controls->Add(this->label33);
			this->groupBoxStarboard->Controls->Add(this->labelGPSS);
			this->groupBoxStarboard->Controls->Add(this->textBoxSRTK);
			this->groupBoxStarboard->Controls->Add(this->textBoxSGPSY);
			this->groupBoxStarboard->Controls->Add(this->textBoxSRoll);
			this->groupBoxStarboard->Controls->Add(this->textBoxSGPSX);
			this->groupBoxStarboard->Controls->Add(this->textBoxSPitch);
			this->groupBoxStarboard->Controls->Add(this->textBoxSYaw);
			this->groupBoxStarboard->Controls->Add(this->textBoxSGyrZ);
			this->groupBoxStarboard->Controls->Add(this->textBoxSGyrX);
			this->groupBoxStarboard->Controls->Add(this->textBoxSGyrY);
			this->groupBoxStarboard->Location = System::Drawing::Point(389, 789);
			this->groupBoxStarboard->Name = L"groupBoxStarboard";
			this->groupBoxStarboard->Size = System::Drawing::Size(182, 157);
			this->groupBoxStarboard->TabIndex = 120;
			this->groupBoxStarboard->TabStop = false;
			this->groupBoxStarboard->Text = L"Plateform Starboard";
			// 
			// label56
			// 
			this->label56->AutoSize = true;
			this->label56->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label56->Location = System::Drawing::Point(74, 105);
			this->label56->Name = L"label56";
			this->label56->Size = System::Drawing::Size(68, 13);
			this->label56->TabIndex = 132;
			this->label56->Text = L"UTM easting";
			this->label56->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label34
			// 
			this->label34->AutoSize = true;
			this->label34->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label34->Location = System::Drawing::Point(110, 61);
			this->label34->Name = L"label34";
			this->label34->Size = System::Drawing::Size(49, 13);
			this->label34->TabIndex = 133;
			this->label34->Text = L"Vz (°.s-1)";
			this->label34->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label35
			// 
			this->label35->AutoSize = true;
			this->label35->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label35->Location = System::Drawing::Point(57, 61);
			this->label35->Name = L"label35";
			this->label35->Size = System::Drawing::Size(49, 13);
			this->label35->TabIndex = 132;
			this->label35->Text = L"Vy (°.s-1)";
			this->label35->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label31
			// 
			this->label31->AutoSize = true;
			this->label31->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label31->Location = System::Drawing::Point(110, 18);
			this->label31->Name = L"label31";
			this->label31->Size = System::Drawing::Size(41, 13);
			this->label31->TabIndex = 133;
			this->label31->Text = L"Yaw (°)";
			this->label31->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label36
			// 
			this->label36->AutoSize = true;
			this->label36->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label36->Location = System::Drawing::Point(4, 61);
			this->label36->Name = L"label36";
			this->label36->Size = System::Drawing::Size(49, 13);
			this->label36->TabIndex = 131;
			this->label36->Text = L"Vx (°.s-1)";
			this->label36->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label32
			// 
			this->label32->AutoSize = true;
			this->label32->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label32->Location = System::Drawing::Point(57, 18);
			this->label32->Name = L"label32";
			this->label32->Size = System::Drawing::Size(44, 13);
			this->label32->TabIndex = 132;
			this->label32->Text = L"Pitch (°)";
			this->label32->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label33
			// 
			this->label33->AutoSize = true;
			this->label33->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label33->Location = System::Drawing::Point(4, 18);
			this->label33->Name = L"label33";
			this->label33->Size = System::Drawing::Size(38, 13);
			this->label33->TabIndex = 131;
			this->label33->Text = L"Roll (°)";
			this->label33->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// pictureBoxLogoLirmm
			// 
			this->pictureBoxLogoLirmm->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"pictureBoxLogoLirmm.Image")));
			this->pictureBoxLogoLirmm->Location = System::Drawing::Point(876, 901);
			this->pictureBoxLogoLirmm->Name = L"pictureBoxLogoLirmm";
			this->pictureBoxLogoLirmm->Size = System::Drawing::Size(151, 47);
			this->pictureBoxLogoLirmm->SizeMode = System::Windows::Forms::PictureBoxSizeMode::StretchImage;
			this->pictureBoxLogoLirmm->TabIndex = 121;
			this->pictureBoxLogoLirmm->TabStop = false;
			// 
			// pictureBoxLogoUM
			// 
			this->pictureBoxLogoUM->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"pictureBoxLogoUM.Image")));
			this->pictureBoxLogoUM->Location = System::Drawing::Point(815, 899);
			this->pictureBoxLogoUM->Name = L"pictureBoxLogoUM";
			this->pictureBoxLogoUM->Size = System::Drawing::Size(52, 50);
			this->pictureBoxLogoUM->SizeMode = System::Windows::Forms::PictureBoxSizeMode::StretchImage;
			this->pictureBoxLogoUM->TabIndex = 122;
			this->pictureBoxLogoUM->TabStop = false;
			// 
			// backgroundWorkerPlateformGPSP
			// 
			this->backgroundWorkerPlateformGPSP->WorkerReportsProgress = true;
			this->backgroundWorkerPlateformGPSP->WorkerSupportsCancellation = true;
			this->backgroundWorkerPlateformGPSP->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerPlateformGPSP_DoWork);
			this->backgroundWorkerPlateformGPSP->ProgressChanged += gcnew System::ComponentModel::ProgressChangedEventHandler(this, &MainForm::backgroundWorkerPlateformGPSP_ProgressChanged);
			// 
			// backgroundWorkerPlateformGPSS
			// 
			this->backgroundWorkerPlateformGPSS->WorkerReportsProgress = true;
			this->backgroundWorkerPlateformGPSS->WorkerSupportsCancellation = true;
			this->backgroundWorkerPlateformGPSS->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerPlateformGPSS_DoWork);
			this->backgroundWorkerPlateformGPSS->ProgressChanged += gcnew System::ComponentModel::ProgressChangedEventHandler(this, &MainForm::backgroundWorkerPlateformGPSS_ProgressChanged);
			// 
			// textBoxMouseDepth
			// 
			this->textBoxMouseDepth->Location = System::Drawing::Point(286, 31);
			this->textBoxMouseDepth->Name = L"textBoxMouseDepth";
			this->textBoxMouseDepth->ReadOnly = true;
			this->textBoxMouseDepth->Size = System::Drawing::Size(47, 20);
			this->textBoxMouseDepth->TabIndex = 122;
			this->textBoxMouseDepth->Text = L"None";
			this->textBoxMouseDepth->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxMouseDepth->Visible = false;
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label5->Location = System::Drawing::Point(283, 15);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(36, 13);
			this->label5->TabIndex = 122;
			this->label5->Text = L"Depth";
			this->label5->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label5->Visible = false;
			// 
			// pictureBoxLogoMaelstrom
			// 
			this->pictureBoxLogoMaelstrom->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"pictureBoxLogoMaelstrom.Image")));
			this->pictureBoxLogoMaelstrom->Location = System::Drawing::Point(597, 899);
			this->pictureBoxLogoMaelstrom->Name = L"pictureBoxLogoMaelstrom";
			this->pictureBoxLogoMaelstrom->Size = System::Drawing::Size(223, 50);
			this->pictureBoxLogoMaelstrom->SizeMode = System::Windows::Forms::PictureBoxSizeMode::StretchImage;
			this->pictureBoxLogoMaelstrom->TabIndex = 123;
			this->pictureBoxLogoMaelstrom->TabStop = false;
			// 
			// pictureBoxColorMap
			// 
			this->pictureBoxColorMap->BackColor = System::Drawing::SystemColors::ControlDark;
			this->pictureBoxColorMap->Location = System::Drawing::Point(357, 122);
			this->pictureBoxColorMap->Name = L"pictureBoxColorMap";
			this->pictureBoxColorMap->Size = System::Drawing::Size(20, 400);
			this->pictureBoxColorMap->TabIndex = 124;
			this->pictureBoxColorMap->TabStop = false;
			// 
			// labelMaxDepth
			// 
			this->labelMaxDepth->AutoSize = true;
			this->labelMaxDepth->Location = System::Drawing::Point(385, 121);
			this->labelMaxDepth->Name = L"labelMaxDepth";
			this->labelMaxDepth->Size = System::Drawing::Size(24, 13);
			this->labelMaxDepth->TabIndex = 125;
			this->labelMaxDepth->Text = L"0 m";
			this->labelMaxDepth->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// labelMinDepth
			// 
			this->labelMinDepth->AutoSize = true;
			this->labelMinDepth->Location = System::Drawing::Point(385, 509);
			this->labelMinDepth->Name = L"labelMinDepth";
			this->labelMinDepth->Size = System::Drawing::Size(24, 13);
			this->labelMinDepth->TabIndex = 126;
			this->labelMinDepth->Text = L"5 m";
			this->labelMinDepth->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// contextMenuStrip1
			// 
			this->contextMenuStrip1->Name = L"contextMenuStrip1";
			this->contextMenuStrip1->Size = System::Drawing::Size(61, 4);
			// 
			// textBoxTargetLatitude
			// 
			this->textBoxTargetLatitude->Location = System::Drawing::Point(1579, 1075);
			this->textBoxTargetLatitude->Name = L"textBoxTargetLatitude";
			this->textBoxTargetLatitude->Size = System::Drawing::Size(167, 20);
			this->textBoxTargetLatitude->TabIndex = 128;
			this->textBoxTargetLatitude->Text = L"None";
			this->textBoxTargetLatitude->Visible = false;
			// 
			// textBoxTargetLongitude
			// 
			this->textBoxTargetLongitude->Location = System::Drawing::Point(1579, 1101);
			this->textBoxTargetLongitude->Name = L"textBoxTargetLongitude";
			this->textBoxTargetLongitude->Size = System::Drawing::Size(167, 20);
			this->textBoxTargetLongitude->TabIndex = 129;
			this->textBoxTargetLongitude->Text = L"None";
			this->textBoxTargetLongitude->Visible = false;
			// 
			// label37
			// 
			this->label37->AutoSize = true;
			this->label37->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label37->Location = System::Drawing::Point(1467, 1084);
			this->label37->Name = L"label37";
			this->label37->Size = System::Drawing::Size(70, 13);
			this->label37->TabIndex = 131;
			this->label37->Text = L"Latitude (DD)";
			this->label37->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label37->Visible = false;
			// 
			// label38
			// 
			this->label38->AutoSize = true;
			this->label38->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label38->Location = System::Drawing::Point(1467, 1110);
			this->label38->Name = L"label38";
			this->label38->Size = System::Drawing::Size(79, 13);
			this->label38->TabIndex = 132;
			this->label38->Text = L"Longitude (DD)";
			this->label38->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label38->Visible = false;
			// 
			// buttonGoTargetGPS
			// 
			this->buttonGoTargetGPS->Location = System::Drawing::Point(1381, 1120);
			this->buttonGoTargetGPS->Name = L"buttonGoTargetGPS";
			this->buttonGoTargetGPS->Size = System::Drawing::Size(71, 46);
			this->buttonGoTargetGPS->TabIndex = 133;
			this->buttonGoTargetGPS->Text = L"Go to target";
			this->buttonGoTargetGPS->UseVisualStyleBackColor = true;
			this->buttonGoTargetGPS->Visible = false;
			this->buttonGoTargetGPS->Click += gcnew System::EventHandler(this, &MainForm::buttonGoTargetGPS_Click);
			// 
			// textBoxLatRobot
			// 
			this->textBoxLatRobot->Location = System::Drawing::Point(1579, 1127);
			this->textBoxLatRobot->Name = L"textBoxLatRobot";
			this->textBoxLatRobot->ReadOnly = true;
			this->textBoxLatRobot->Size = System::Drawing::Size(167, 20);
			this->textBoxLatRobot->TabIndex = 134;
			this->textBoxLatRobot->Text = L"None";
			this->textBoxLatRobot->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxLatRobot->Visible = false;
			// 
			// textBoxLongRobot
			// 
			this->textBoxLongRobot->Location = System::Drawing::Point(1579, 1153);
			this->textBoxLongRobot->Name = L"textBoxLongRobot";
			this->textBoxLongRobot->ReadOnly = true;
			this->textBoxLongRobot->Size = System::Drawing::Size(167, 20);
			this->textBoxLongRobot->TabIndex = 134;
			this->textBoxLongRobot->Text = L"None";
			this->textBoxLongRobot->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxLongRobot->Visible = false;
			// 
			// label39
			// 
			this->label39->AutoSize = true;
			this->label39->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label39->Location = System::Drawing::Point(1467, 1136);
			this->label39->Name = L"label39";
			this->label39->Size = System::Drawing::Size(97, 13);
			this->label39->TabIndex = 135;
			this->label39->Text = L"Latitude robot (DD)";
			this->label39->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label39->Visible = false;
			// 
			// label40
			// 
			this->label40->AutoSize = true;
			this->label40->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label40->Location = System::Drawing::Point(1467, 1162);
			this->label40->Name = L"label40";
			this->label40->Size = System::Drawing::Size(106, 13);
			this->label40->TabIndex = 136;
			this->label40->Text = L"Longitude robot (DD)";
			this->label40->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label40->Visible = false;
			// 
			// textBoxUTMXRobot
			// 
			this->textBoxUTMXRobot->Location = System::Drawing::Point(55, 45);
			this->textBoxUTMXRobot->Name = L"textBoxUTMXRobot";
			this->textBoxUTMXRobot->ReadOnly = true;
			this->textBoxUTMXRobot->Size = System::Drawing::Size(120, 20);
			this->textBoxUTMXRobot->TabIndex = 137;
			this->textBoxUTMXRobot->Text = L"None";
			this->textBoxUTMXRobot->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxUTMYRobot
			// 
			this->textBoxUTMYRobot->Location = System::Drawing::Point(55, 19);
			this->textBoxUTMYRobot->Name = L"textBoxUTMYRobot";
			this->textBoxUTMYRobot->ReadOnly = true;
			this->textBoxUTMYRobot->Size = System::Drawing::Size(120, 20);
			this->textBoxUTMYRobot->TabIndex = 138;
			this->textBoxUTMYRobot->Text = L"None";
			this->textBoxUTMYRobot->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// label41
			// 
			this->label41->AutoSize = true;
			this->label41->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label41->Location = System::Drawing::Point(7, 48);
			this->label41->Name = L"label41";
			this->label41->Size = System::Drawing::Size(47, 13);
			this->label41->TabIndex = 139;
			this->label41->Text = L"Northing";
			this->label41->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label42
			// 
			this->label42->AutoSize = true;
			this->label42->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label42->Location = System::Drawing::Point(7, 22);
			this->label42->Name = L"label42";
			this->label42->Size = System::Drawing::Size(42, 13);
			this->label42->TabIndex = 140;
			this->label42->Text = L"Easting";
			this->label42->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// textBoxCap
			// 
			this->textBoxCap->Location = System::Drawing::Point(1386, 1183);
			this->textBoxCap->Name = L"textBoxCap";
			this->textBoxCap->ReadOnly = true;
			this->textBoxCap->Size = System::Drawing::Size(101, 20);
			this->textBoxCap->TabIndex = 141;
			this->textBoxCap->Text = L"None";
			this->textBoxCap->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxCap->Visible = false;
			// 
			// label43
			// 
			this->label43->AutoSize = true;
			this->label43->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label43->Location = System::Drawing::Point(1389, 1097);
			this->label43->Name = L"label43";
			this->label43->Size = System::Drawing::Size(50, 13);
			this->label43->TabIndex = 142;
			this->label43->Text = L"Cap (rad)";
			this->label43->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label43->Visible = false;
			// 
			// label44
			// 
			this->label44->AutoSize = true;
			this->label44->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label44->Location = System::Drawing::Point(7, 74);
			this->label44->Name = L"label44";
			this->label44->Size = System::Drawing::Size(53, 13);
			this->label44->TabIndex = 143;
			this->label44->Text = L"Course (°)";
			this->label44->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// textBoxCapDeg
			// 
			this->textBoxCapDeg->Location = System::Drawing::Point(102, 71);
			this->textBoxCapDeg->Name = L"textBoxCapDeg";
			this->textBoxCapDeg->ReadOnly = true;
			this->textBoxCapDeg->Size = System::Drawing::Size(55, 20);
			this->textBoxCapDeg->TabIndex = 144;
			this->textBoxCapDeg->Text = L"None";
			this->textBoxCapDeg->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// label45
			// 
			this->label45->AutoSize = true;
			this->label45->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label45->Location = System::Drawing::Point(6, 100);
			this->label45->Name = L"label45";
			this->label45->Size = System::Drawing::Size(89, 13);
			this->label45->TabIndex = 145;
			this->label45->Text = L"GPS distance (m)";
			this->label45->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// textBoxDistanceGPS
			// 
			this->textBoxDistanceGPS->Location = System::Drawing::Point(102, 97);
			this->textBoxDistanceGPS->Name = L"textBoxDistanceGPS";
			this->textBoxDistanceGPS->ReadOnly = true;
			this->textBoxDistanceGPS->Size = System::Drawing::Size(55, 20);
			this->textBoxDistanceGPS->TabIndex = 146;
			this->textBoxDistanceGPS->Text = L"None";
			this->textBoxDistanceGPS->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// label46
			// 
			this->label46->AutoSize = true;
			this->label46->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label46->Location = System::Drawing::Point(7, 50);
			this->label46->Name = L"label46";
			this->label46->Size = System::Drawing::Size(47, 13);
			this->label46->TabIndex = 147;
			this->label46->Text = L"Northing";
			this->label46->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label47
			// 
			this->label47->AutoSize = true;
			this->label47->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label47->Location = System::Drawing::Point(7, 24);
			this->label47->Name = L"label47";
			this->label47->Size = System::Drawing::Size(42, 13);
			this->label47->TabIndex = 148;
			this->label47->Text = L"Easting";
			this->label47->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// textBoxTargetUTMNorth
			// 
			this->textBoxTargetUTMNorth->Location = System::Drawing::Point(55, 47);
			this->textBoxTargetUTMNorth->Name = L"textBoxTargetUTMNorth";
			this->textBoxTargetUTMNorth->Size = System::Drawing::Size(120, 20);
			this->textBoxTargetUTMNorth->TabIndex = 149;
			this->textBoxTargetUTMNorth->Text = L"0";
			// 
			// textBoxTargetUTMEast
			// 
			this->textBoxTargetUTMEast->Location = System::Drawing::Point(55, 21);
			this->textBoxTargetUTMEast->Name = L"textBoxTargetUTMEast";
			this->textBoxTargetUTMEast->Size = System::Drawing::Size(120, 20);
			this->textBoxTargetUTMEast->TabIndex = 150;
			this->textBoxTargetUTMEast->Text = L"0";
			// 
			// label48
			// 
			this->label48->AutoSize = true;
			this->label48->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label48->Location = System::Drawing::Point(7, 130);
			this->label48->Name = L"label48";
			this->label48->Size = System::Drawing::Size(43, 13);
			this->label48->TabIndex = 151;
			this->label48->Text = L"Delta N";
			this->label48->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label49
			// 
			this->label49->AutoSize = true;
			this->label49->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label49->Location = System::Drawing::Point(7, 105);
			this->label49->Name = L"label49";
			this->label49->Size = System::Drawing::Size(42, 13);
			this->label49->TabIndex = 152;
			this->label49->Text = L"Delta E";
			this->label49->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// textBoxUTMDeltaNorthing
			// 
			this->textBoxUTMDeltaNorthing->Location = System::Drawing::Point(55, 127);
			this->textBoxUTMDeltaNorthing->Name = L"textBoxUTMDeltaNorthing";
			this->textBoxUTMDeltaNorthing->ReadOnly = true;
			this->textBoxUTMDeltaNorthing->Size = System::Drawing::Size(120, 20);
			this->textBoxUTMDeltaNorthing->TabIndex = 153;
			this->textBoxUTMDeltaNorthing->Text = L"None";
			this->textBoxUTMDeltaNorthing->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxDeltaUTMEasting
			// 
			this->textBoxDeltaUTMEasting->Location = System::Drawing::Point(55, 102);
			this->textBoxDeltaUTMEasting->Name = L"textBoxDeltaUTMEasting";
			this->textBoxDeltaUTMEasting->ReadOnly = true;
			this->textBoxDeltaUTMEasting->Size = System::Drawing::Size(120, 20);
			this->textBoxDeltaUTMEasting->TabIndex = 154;
			this->textBoxDeltaUTMEasting->Text = L"None";
			this->textBoxDeltaUTMEasting->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// label50
			// 
			this->label50->AutoSize = true;
			this->label50->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label50->Location = System::Drawing::Point(1392, 971);
			this->label50->Name = L"label50";
			this->label50->Size = System::Drawing::Size(60, 13);
			this->label50->TabIndex = 155;
			this->label50->Text = L"Target Cap";
			this->label50->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label50->Visible = false;
			// 
			// label51
			// 
			this->label51->AutoSize = true;
			this->label51->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label51->Location = System::Drawing::Point(1392, 1010);
			this->label51->Name = L"label51";
			this->label51->Size = System::Drawing::Size(63, 13);
			this->label51->TabIndex = 156;
			this->label51->Text = L"Current Cap";
			this->label51->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label51->Visible = false;
			// 
			// label52
			// 
			this->label52->AutoSize = true;
			this->label52->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label52->Location = System::Drawing::Point(1392, 1049);
			this->label52->Name = L"label52";
			this->label52->Size = System::Drawing::Size(54, 13);
			this->label52->TabIndex = 157;
			this->label52->Text = L"Delta Cap";
			this->label52->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label52->Visible = false;
			// 
			// textBoxTargetCap
			// 
			this->textBoxTargetCap->Location = System::Drawing::Point(1395, 987);
			this->textBoxTargetCap->Name = L"textBoxTargetCap";
			this->textBoxTargetCap->Size = System::Drawing::Size(60, 20);
			this->textBoxTargetCap->TabIndex = 158;
			this->textBoxTargetCap->Text = L"0";
			this->textBoxTargetCap->Visible = false;
			// 
			// textBoxCurrentCap
			// 
			this->textBoxCurrentCap->Location = System::Drawing::Point(1395, 1026);
			this->textBoxCurrentCap->Name = L"textBoxCurrentCap";
			this->textBoxCurrentCap->ReadOnly = true;
			this->textBoxCurrentCap->Size = System::Drawing::Size(60, 20);
			this->textBoxCurrentCap->TabIndex = 159;
			this->textBoxCurrentCap->Text = L"None";
			this->textBoxCurrentCap->Visible = false;
			// 
			// textBoxDeltaCap
			// 
			this->textBoxDeltaCap->Location = System::Drawing::Point(1395, 1065);
			this->textBoxDeltaCap->Name = L"textBoxDeltaCap";
			this->textBoxDeltaCap->ReadOnly = true;
			this->textBoxDeltaCap->Size = System::Drawing::Size(60, 20);
			this->textBoxDeltaCap->TabIndex = 160;
			this->textBoxDeltaCap->Text = L"None";
			this->textBoxDeltaCap->Visible = false;
			// 
			// buttonSetTargetUTM
			// 
			this->buttonSetTargetUTM->Location = System::Drawing::Point(100, 73);
			this->buttonSetTargetUTM->Name = L"buttonSetTargetUTM";
			this->buttonSetTargetUTM->Size = System::Drawing::Size(75, 23);
			this->buttonSetTargetUTM->TabIndex = 161;
			this->buttonSetTargetUTM->Text = L"Set target";
			this->buttonSetTargetUTM->UseVisualStyleBackColor = true;
			this->buttonSetTargetUTM->Click += gcnew System::EventHandler(this, &MainForm::buttonSetTargetUTM_Click);
			// 
			// label53
			// 
			this->label53->AutoSize = true;
			this->label53->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label53->Location = System::Drawing::Point(1461, 1027);
			this->label53->Name = L"label53";
			this->label53->Size = System::Drawing::Size(104, 13);
			this->label53->TabIndex = 162;
			this->label53->Text = L"Delta x (East = Xrob)";
			this->label53->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label53->Visible = false;
			// 
			// label54
			// 
			this->label54->AutoSize = true;
			this->label54->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label54->Location = System::Drawing::Point(1586, 1018);
			this->label54->Name = L"label54";
			this->label54->Size = System::Drawing::Size(112, 13);
			this->label54->TabIndex = 163;
			this->label54->Text = L"Delta y (North = -Yrob)";
			this->label54->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label54->Visible = false;
			// 
			// textBoxDeltaXRobot
			// 
			this->textBoxDeltaXRobot->Location = System::Drawing::Point(1462, 1043);
			this->textBoxDeltaXRobot->Name = L"textBoxDeltaXRobot";
			this->textBoxDeltaXRobot->ReadOnly = true;
			this->textBoxDeltaXRobot->Size = System::Drawing::Size(120, 20);
			this->textBoxDeltaXRobot->TabIndex = 164;
			this->textBoxDeltaXRobot->Text = L"None";
			this->textBoxDeltaXRobot->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxDeltaXRobot->Visible = false;
			// 
			// textBoxDeltaYRobot
			// 
			this->textBoxDeltaYRobot->Location = System::Drawing::Point(1583, 1034);
			this->textBoxDeltaYRobot->Name = L"textBoxDeltaYRobot";
			this->textBoxDeltaYRobot->ReadOnly = true;
			this->textBoxDeltaYRobot->Size = System::Drawing::Size(120, 20);
			this->textBoxDeltaYRobot->TabIndex = 165;
			this->textBoxDeltaYRobot->Text = L"None";
			this->textBoxDeltaYRobot->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxDeltaYRobot->Visible = false;
			// 
			// pictureBoxBathy
			// 
			this->pictureBoxBathy->BackColor = System::Drawing::SystemColors::ControlDark;
			this->pictureBoxBathy->Location = System::Drawing::Point(1091, 58);
			this->pictureBoxBathy->Name = L"pictureBoxBathy";
			this->pictureBoxBathy->Size = System::Drawing::Size(560, 560);
			this->pictureBoxBathy->TabIndex = 166;
			this->pictureBoxBathy->TabStop = false;
			this->pictureBoxBathy->MouseDoubleClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::pictureBoxBathy_MouseDoubleClick);
			// 
			// backgroundWorkerBathy
			// 
			this->backgroundWorkerBathy->WorkerReportsProgress = true;
			this->backgroundWorkerBathy->WorkerSupportsCancellation = true;
			this->backgroundWorkerBathy->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerBathy_DoWork);
			// 
			// buttonBathyOnline
			// 
			this->buttonBathyOnline->BackColor = System::Drawing::Color::Silver;
			this->buttonBathyOnline->Enabled = false;
			this->buttonBathyOnline->Location = System::Drawing::Point(1626, 25);
			this->buttonBathyOnline->Name = L"buttonBathyOnline";
			this->buttonBathyOnline->Size = System::Drawing::Size(25, 25);
			this->buttonBathyOnline->TabIndex = 172;
			this->buttonBathyOnline->UseVisualStyleBackColor = false;
			// 
			// label57
			// 
			this->label57->AutoSize = true;
			this->label57->BackColor = System::Drawing::Color::LightGray;
			this->label57->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label57->Location = System::Drawing::Point(534, 887);
			this->label57->Name = L"label57";
			this->label57->Size = System::Drawing::Size(29, 13);
			this->label57->TabIndex = 134;
			this->label57->Text = L"RTK";
			this->label57->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// pictureBoxZoomedBathy
			// 
			this->pictureBoxZoomedBathy->BackColor = System::Drawing::SystemColors::ControlDark;
			this->pictureBoxZoomedBathy->Location = System::Drawing::Point(1351, 625);
			this->pictureBoxZoomedBathy->Name = L"pictureBoxZoomedBathy";
			this->pictureBoxZoomedBathy->Size = System::Drawing::Size(300, 300);
			this->pictureBoxZoomedBathy->TabIndex = 174;
			this->pictureBoxZoomedBathy->TabStop = false;
			// 
			// groupBox2
			// 
			this->groupBox2->BackColor = System::Drawing::Color::LightGray;
			this->groupBox2->Controls->Add(this->textBoxUTMYRobot);
			this->groupBox2->Controls->Add(this->label42);
			this->groupBox2->Controls->Add(this->textBoxUTMXRobot);
			this->groupBox2->Controls->Add(this->label41);
			this->groupBox2->Controls->Add(this->textBoxCapDeg);
			this->groupBox2->Controls->Add(this->label44);
			this->groupBox2->Controls->Add(this->label45);
			this->groupBox2->Controls->Add(this->textBoxDistanceGPS);
			this->groupBox2->Location = System::Drawing::Point(1091, 625);
			this->groupBox2->Name = L"groupBox2";
			this->groupBox2->Size = System::Drawing::Size(200, 133);
			this->groupBox2->TabIndex = 175;
			this->groupBox2->TabStop = false;
			this->groupBox2->Text = L"Robot UTM coordinates";
			// 
			// groupBox3
			// 
			this->groupBox3->BackColor = System::Drawing::Color::LightGray;
			this->groupBox3->Controls->Add(this->label47);
			this->groupBox3->Controls->Add(this->textBoxTargetUTMEast);
			this->groupBox3->Controls->Add(this->label46);
			this->groupBox3->Controls->Add(this->textBoxTargetUTMNorth);
			this->groupBox3->Controls->Add(this->buttonSetTargetUTM);
			this->groupBox3->Controls->Add(this->label49);
			this->groupBox3->Controls->Add(this->textBoxDeltaUTMEasting);
			this->groupBox3->Controls->Add(this->label48);
			this->groupBox3->Controls->Add(this->textBoxUTMDeltaNorthing);
			this->groupBox3->Location = System::Drawing::Point(1091, 769);
			this->groupBox3->Name = L"groupBox3";
			this->groupBox3->Size = System::Drawing::Size(200, 156);
			this->groupBox3->TabIndex = 176;
			this->groupBox3->TabStop = false;
			this->groupBox3->Text = L"Target UTM coordinates";
			// 
			// pictureBoxM1
			// 
			this->pictureBoxM1->BackColor = System::Drawing::SystemColors::ControlDark;
			this->pictureBoxM1->Location = System::Drawing::Point(8, 34);
			this->pictureBoxM1->Name = L"pictureBoxM1";
			this->pictureBoxM1->Size = System::Drawing::Size(40, 200);
			this->pictureBoxM1->TabIndex = 177;
			this->pictureBoxM1->TabStop = false;
			// 
			// pictureBoxM2
			// 
			this->pictureBoxM2->BackColor = System::Drawing::SystemColors::ControlDark;
			this->pictureBoxM2->Location = System::Drawing::Point(69, 34);
			this->pictureBoxM2->Name = L"pictureBoxM2";
			this->pictureBoxM2->Size = System::Drawing::Size(40, 200);
			this->pictureBoxM2->TabIndex = 178;
			this->pictureBoxM2->TabStop = false;
			// 
			// pictureBoxM3
			// 
			this->pictureBoxM3->BackColor = System::Drawing::SystemColors::ControlDark;
			this->pictureBoxM3->Location = System::Drawing::Point(130, 34);
			this->pictureBoxM3->Name = L"pictureBoxM3";
			this->pictureBoxM3->Size = System::Drawing::Size(40, 200);
			this->pictureBoxM3->TabIndex = 179;
			this->pictureBoxM3->TabStop = false;
			// 
			// pictureBoxM4
			// 
			this->pictureBoxM4->BackColor = System::Drawing::SystemColors::ControlDark;
			this->pictureBoxM4->Location = System::Drawing::Point(191, 34);
			this->pictureBoxM4->Name = L"pictureBoxM4";
			this->pictureBoxM4->Size = System::Drawing::Size(40, 200);
			this->pictureBoxM4->TabIndex = 180;
			this->pictureBoxM4->TabStop = false;
			// 
			// pictureBoxM5
			// 
			this->pictureBoxM5->BackColor = System::Drawing::SystemColors::ControlDark;
			this->pictureBoxM5->Location = System::Drawing::Point(252, 34);
			this->pictureBoxM5->Name = L"pictureBoxM5";
			this->pictureBoxM5->Size = System::Drawing::Size(40, 200);
			this->pictureBoxM5->TabIndex = 181;
			this->pictureBoxM5->TabStop = false;
			// 
			// pictureBoxM6
			// 
			this->pictureBoxM6->BackColor = System::Drawing::SystemColors::ControlDark;
			this->pictureBoxM6->Location = System::Drawing::Point(314, 34);
			this->pictureBoxM6->Name = L"pictureBoxM6";
			this->pictureBoxM6->Size = System::Drawing::Size(40, 200);
			this->pictureBoxM6->TabIndex = 182;
			this->pictureBoxM6->TabStop = false;
			// 
			// pictureBoxM7
			// 
			this->pictureBoxM7->BackColor = System::Drawing::SystemColors::ControlDark;
			this->pictureBoxM7->Location = System::Drawing::Point(375, 34);
			this->pictureBoxM7->Name = L"pictureBoxM7";
			this->pictureBoxM7->Size = System::Drawing::Size(40, 200);
			this->pictureBoxM7->TabIndex = 183;
			this->pictureBoxM7->TabStop = false;
			// 
			// pictureBoxM8
			// 
			this->pictureBoxM8->BackColor = System::Drawing::SystemColors::ControlDark;
			this->pictureBoxM8->Location = System::Drawing::Point(436, 34);
			this->pictureBoxM8->Name = L"pictureBoxM8";
			this->pictureBoxM8->Size = System::Drawing::Size(40, 200);
			this->pictureBoxM8->TabIndex = 184;
			this->pictureBoxM8->TabStop = false;
			// 
			// groupBox4
			// 
			this->groupBox4->BackColor = System::Drawing::Color::LightGray;
			this->groupBox4->Controls->Add(this->textBoxM8);
			this->groupBox4->Controls->Add(this->textBoxM7);
			this->groupBox4->Controls->Add(this->textBoxM6);
			this->groupBox4->Controls->Add(this->textBoxM5);
			this->groupBox4->Controls->Add(this->textBoxM4);
			this->groupBox4->Controls->Add(this->textBoxM3);
			this->groupBox4->Controls->Add(this->textBoxM2);
			this->groupBox4->Controls->Add(this->textBoxM1);
			this->groupBox4->Controls->Add(this->label65);
			this->groupBox4->Controls->Add(this->label64);
			this->groupBox4->Controls->Add(this->label63);
			this->groupBox4->Controls->Add(this->label62);
			this->groupBox4->Controls->Add(this->label61);
			this->groupBox4->Controls->Add(this->label60);
			this->groupBox4->Controls->Add(this->label59);
			this->groupBox4->Controls->Add(this->label58);
			this->groupBox4->Controls->Add(this->pictureBoxM8);
			this->groupBox4->Controls->Add(this->pictureBoxM7);
			this->groupBox4->Controls->Add(this->pictureBoxM6);
			this->groupBox4->Controls->Add(this->pictureBoxM5);
			this->groupBox4->Controls->Add(this->pictureBoxM4);
			this->groupBox4->Controls->Add(this->pictureBoxM3);
			this->groupBox4->Controls->Add(this->pictureBoxM2);
			this->groupBox4->Controls->Add(this->pictureBoxM1);
			this->groupBox4->Location = System::Drawing::Point(588, 625);
			this->groupBox4->Name = L"groupBox4";
			this->groupBox4->Size = System::Drawing::Size(486, 267);
			this->groupBox4->TabIndex = 185;
			this->groupBox4->TabStop = false;
			this->groupBox4->Text = L"Cables tension (kg)";
			// 
			// textBoxM8
			// 
			this->textBoxM8->Location = System::Drawing::Point(436, 240);
			this->textBoxM8->Name = L"textBoxM8";
			this->textBoxM8->ReadOnly = true;
			this->textBoxM8->Size = System::Drawing::Size(40, 20);
			this->textBoxM8->TabIndex = 198;
			this->textBoxM8->Text = L"None";
			this->textBoxM8->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxM7
			// 
			this->textBoxM7->Location = System::Drawing::Point(375, 240);
			this->textBoxM7->Name = L"textBoxM7";
			this->textBoxM7->ReadOnly = true;
			this->textBoxM7->Size = System::Drawing::Size(40, 20);
			this->textBoxM7->TabIndex = 197;
			this->textBoxM7->Text = L"None";
			this->textBoxM7->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxM6
			// 
			this->textBoxM6->Location = System::Drawing::Point(314, 240);
			this->textBoxM6->Name = L"textBoxM6";
			this->textBoxM6->ReadOnly = true;
			this->textBoxM6->Size = System::Drawing::Size(40, 20);
			this->textBoxM6->TabIndex = 196;
			this->textBoxM6->Text = L"None";
			this->textBoxM6->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxM5
			// 
			this->textBoxM5->Location = System::Drawing::Point(252, 240);
			this->textBoxM5->Name = L"textBoxM5";
			this->textBoxM5->ReadOnly = true;
			this->textBoxM5->Size = System::Drawing::Size(40, 20);
			this->textBoxM5->TabIndex = 195;
			this->textBoxM5->Text = L"None";
			this->textBoxM5->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxM4
			// 
			this->textBoxM4->Location = System::Drawing::Point(191, 240);
			this->textBoxM4->Name = L"textBoxM4";
			this->textBoxM4->ReadOnly = true;
			this->textBoxM4->Size = System::Drawing::Size(40, 20);
			this->textBoxM4->TabIndex = 194;
			this->textBoxM4->Text = L"None";
			this->textBoxM4->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxM3
			// 
			this->textBoxM3->Location = System::Drawing::Point(130, 240);
			this->textBoxM3->Name = L"textBoxM3";
			this->textBoxM3->ReadOnly = true;
			this->textBoxM3->Size = System::Drawing::Size(40, 20);
			this->textBoxM3->TabIndex = 193;
			this->textBoxM3->Text = L"None";
			this->textBoxM3->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxM2
			// 
			this->textBoxM2->Location = System::Drawing::Point(69, 240);
			this->textBoxM2->Name = L"textBoxM2";
			this->textBoxM2->ReadOnly = true;
			this->textBoxM2->Size = System::Drawing::Size(40, 20);
			this->textBoxM2->TabIndex = 192;
			this->textBoxM2->Text = L"None";
			this->textBoxM2->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxM1
			// 
			this->textBoxM1->Location = System::Drawing::Point(8, 240);
			this->textBoxM1->Name = L"textBoxM1";
			this->textBoxM1->ReadOnly = true;
			this->textBoxM1->Size = System::Drawing::Size(40, 20);
			this->textBoxM1->TabIndex = 134;
			this->textBoxM1->Text = L"None";
			this->textBoxM1->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// label65
			// 
			this->label65->AutoSize = true;
			this->label65->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label65->Location = System::Drawing::Point(445, 18);
			this->label65->Name = L"label65";
			this->label65->Size = System::Drawing::Size(22, 13);
			this->label65->TabIndex = 191;
			this->label65->Text = L"M8";
			this->label65->TextAlign = System::Drawing::ContentAlignment::TopCenter;
			// 
			// label64
			// 
			this->label64->AutoSize = true;
			this->label64->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label64->Location = System::Drawing::Point(383, 18);
			this->label64->Name = L"label64";
			this->label64->Size = System::Drawing::Size(22, 13);
			this->label64->TabIndex = 190;
			this->label64->Text = L"M7";
			this->label64->TextAlign = System::Drawing::ContentAlignment::TopCenter;
			// 
			// label63
			// 
			this->label63->AutoSize = true;
			this->label63->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label63->Location = System::Drawing::Point(322, 18);
			this->label63->Name = L"label63";
			this->label63->Size = System::Drawing::Size(22, 13);
			this->label63->TabIndex = 189;
			this->label63->Text = L"M6";
			this->label63->TextAlign = System::Drawing::ContentAlignment::TopCenter;
			// 
			// label62
			// 
			this->label62->AutoSize = true;
			this->label62->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label62->Location = System::Drawing::Point(260, 18);
			this->label62->Name = L"label62";
			this->label62->Size = System::Drawing::Size(22, 13);
			this->label62->TabIndex = 188;
			this->label62->Text = L"M5";
			this->label62->TextAlign = System::Drawing::ContentAlignment::TopCenter;
			// 
			// label61
			// 
			this->label61->AutoSize = true;
			this->label61->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label61->Location = System::Drawing::Point(200, 18);
			this->label61->Name = L"label61";
			this->label61->Size = System::Drawing::Size(22, 13);
			this->label61->TabIndex = 187;
			this->label61->Text = L"M4";
			this->label61->TextAlign = System::Drawing::ContentAlignment::TopCenter;
			// 
			// label60
			// 
			this->label60->AutoSize = true;
			this->label60->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label60->Location = System::Drawing::Point(139, 18);
			this->label60->Name = L"label60";
			this->label60->Size = System::Drawing::Size(22, 13);
			this->label60->TabIndex = 186;
			this->label60->Text = L"M3";
			this->label60->TextAlign = System::Drawing::ContentAlignment::TopCenter;
			// 
			// label59
			// 
			this->label59->AutoSize = true;
			this->label59->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label59->Location = System::Drawing::Point(76, 18);
			this->label59->Name = L"label59";
			this->label59->Size = System::Drawing::Size(22, 13);
			this->label59->TabIndex = 185;
			this->label59->Text = L"M2";
			this->label59->TextAlign = System::Drawing::ContentAlignment::TopCenter;
			// 
			// label58
			// 
			this->label58->AutoSize = true;
			this->label58->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label58->Location = System::Drawing::Point(16, 18);
			this->label58->Name = L"label58";
			this->label58->Size = System::Drawing::Size(22, 13);
			this->label58->TabIndex = 132;
			this->label58->Text = L"M1";
			this->label58->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// contextMenuStrip2
			// 
			this->contextMenuStrip2->Name = L"contextMenuStrip2";
			this->contextMenuStrip2->Size = System::Drawing::Size(61, 4);
			// 
			// backgroundWorkerCableTension
			// 
			this->backgroundWorkerCableTension->WorkerReportsProgress = true;
			this->backgroundWorkerCableTension->WorkerSupportsCancellation = true;
			this->backgroundWorkerCableTension->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerCableTension_DoWork);
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(1091, 960);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(75, 23);
			this->button1->TabIndex = 186;
			this->button1->Text = L"button1";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Visible = false;
			this->button1->Click += gcnew System::EventHandler(this, &MainForm::button1_Click);
			// 
			// errorProvider1
			// 
			this->errorProvider1->ContainerControl = this;
			// 
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1781, 1061);
			this->Controls->Add(this->button1);
			this->Controls->Add(this->groupBox4);
			this->Controls->Add(this->groupBox3);
			this->Controls->Add(this->groupBox2);
			this->Controls->Add(this->pictureBoxZoomedBathy);
			this->Controls->Add(this->label57);
			this->Controls->Add(this->buttonBathyOnline);
			this->Controls->Add(this->pictureBoxBathy);
			this->Controls->Add(this->textBoxDeltaYRobot);
			this->Controls->Add(this->textBoxDeltaXRobot);
			this->Controls->Add(this->label54);
			this->Controls->Add(this->label53);
			this->Controls->Add(this->textBoxDeltaCap);
			this->Controls->Add(this->textBoxCurrentCap);
			this->Controls->Add(this->textBoxTargetCap);
			this->Controls->Add(this->label52);
			this->Controls->Add(this->label51);
			this->Controls->Add(this->label50);
			this->Controls->Add(this->label43);
			this->Controls->Add(this->textBoxCap);
			this->Controls->Add(this->label40);
			this->Controls->Add(this->label39);
			this->Controls->Add(this->textBoxLongRobot);
			this->Controls->Add(this->textBoxLatRobot);
			this->Controls->Add(this->buttonGoTargetGPS);
			this->Controls->Add(this->label38);
			this->Controls->Add(this->label37);
			this->Controls->Add(this->textBoxTargetLongitude);
			this->Controls->Add(this->textBoxTargetLatitude);
			this->Controls->Add(this->labelMinDepth);
			this->Controls->Add(this->labelMaxDepth);
			this->Controls->Add(this->pictureBoxColorMap);
			this->Controls->Add(this->pictureBoxLogoUM);
			this->Controls->Add(this->pictureBoxLogoMaelstrom);
			this->Controls->Add(this->label5);
			this->Controls->Add(this->textBoxMouseDepth);
			this->Controls->Add(this->pictureBoxLogoLirmm);
			this->Controls->Add(this->groupBoxStarboard);
			this->Controls->Add(this->groupBoxPort);
			this->Controls->Add(this->groupBoxDVL);
			this->Controls->Add(this->groupBox1);
			this->Controls->Add(this->buttonFreezeDM);
			this->Controls->Add(this->buttonSaveDepthMap);
			this->Controls->Add(this->buttonResolutionMap);
			this->Controls->Add(this->ptbDepthMap);
			this->Controls->Add(this->labelLineFake);
			this->Controls->Add(this->labelLine);
			this->Controls->Add(this->label3dZFake);
			this->Controls->Add(this->label3dYFake);
			this->Controls->Add(this->label3dXFake);
			this->Controls->Add(this->labelDVL);
			this->Controls->Add(this->label3dZ);
			this->Controls->Add(this->label3dY);
			this->Controls->Add(this->label3dX);
			this->Controls->Add(this->buttonJetsonOn);
			this->Controls->Add(this->labelDVLon);
			this->Controls->Add(this->buttonDVLno);
			this->Controls->Add(this->buttonDVLyes);
			this->Controls->Add(this->buttonDVLon);
			this->Controls->Add(this->record_button);
			this->Controls->Add(this->video_label);
			this->Controls->Add(this->video_trackBar);
			this->Controls->Add(this->load_button);
			this->Controls->Add(this->speed_button);
			this->Controls->Add(this->play_button);
			this->Controls->Add(this->view_button);
			this->Controls->Add(this->listView1);
			this->Controls->Add(this->button_Browse);
			this->Controls->Add(this->ptbSource);
			this->Controls->Add(this->button_Edition);
			this->Controls->Add(this->groupBoxRobot);
			this->Icon = (cli::safe_cast<System::Drawing::Icon^>(resources->GetObject(L"$this.Icon")));
			this->Name = L"MainForm";
			this->Text = L"Maelstrom GUI";
			this->FormClosing += gcnew System::Windows::Forms::FormClosingEventHandler(this, &MainForm::MainForm_FormClosing);
			this->Load += gcnew System::EventHandler(this, &MainForm::MainForm_Load);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ptbSource))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->video_trackBar))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ptbDepthMap))->EndInit();
			this->groupBoxRobot->ResumeLayout(false);
			this->groupBoxRobot->PerformLayout();
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			this->groupBoxDVL->ResumeLayout(false);
			this->groupBoxDVL->PerformLayout();
			this->groupBoxPort->ResumeLayout(false);
			this->groupBoxPort->PerformLayout();
			this->groupBoxStarboard->ResumeLayout(false);
			this->groupBoxStarboard->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxLogoLirmm))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxLogoUM))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxLogoMaelstrom))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxColorMap))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxBathy))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxZoomedBathy))->EndInit();
			this->groupBox2->ResumeLayout(false);
			this->groupBox2->PerformLayout();
			this->groupBox3->ResumeLayout(false);
			this->groupBox3->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM1))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM2))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM3))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM4))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM5))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM6))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM7))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBoxM8))->EndInit();
			this->groupBox4->ResumeLayout(false);
			this->groupBox4->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->errorProvider1))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}

#pragma endregion

		/* --------------------------------------------------
		*
		*
		*		Some cool functions
		*
		*
		----------------------------------------------------- */
	public:
		// Video path creation with timestamp
		string getTime() {
			char time[32];
			errno_t errNum;
			_time32(&aclock);
			_localtime32_s(&newtime, &aclock);
			errNum = asctime_s(time, 32, &newtime);
			std::replace(std::begin(time), std::end(time), ' ', '_');
			std::replace(std::begin(time), std::end(time), ':', '-');
			char* new_time = new char[32];
			std::strcpy(new_time, time);
			string str_time(new_time);
			str_time.erase(str_time.length() - 1);
			return str_time;
		}
		string getNewVideoPath() {
			string new_path = videos_dir_path + "video_" + getTime() + ".mp4";
			return new_path;
		}
		// Get rounded precision
		float getPrecision(float number, int precision) {
			std::stringstream ss;
			ss << std::fixed << std::setprecision(precision) << number;
			std::string number_str = ss.str();
			float new_number = std::stof(number_str);
			return new_number;
		}
		// Convert String to Char
		char* ConvertString2Char(System::String^ str) { // Marshal method
			char* str2 = (char*)(void*)Marshal::StringToHGlobalAnsi(str);
			return str2;
		}
		// Convert Mat to Bitmap
		System::Drawing::Bitmap^ ConvertMat2Bitmap(Mat img) {
			Bitmap^ newBitmap = gcnew System::Drawing::Bitmap(img.size().width,
				img.size().height,
				img.step,
				System::Drawing::Imaging::PixelFormat::Format24bppRgb,
				(IntPtr)img.data);
			return newBitmap;
		}

		/* --------------------------------------------------
		*
		*
		*		Video related functions
		*
		*
		* ---------------------------------------------------- */

		// Bbx edition
	private: System::Void Edition_Click(System::Object^ sender, System::EventArgs^ e) {
		namedWindow("Edition");
		setMouseCallback("Edition", mouse_callback);
		video.video_bboxes[video.getFrameId()].set_img_size(video.frame.size()); // TODO: DO better ^^
		bool close_window = false; // Bool to close the windows when hitting the close button
		video.setEditedFrame();
		while (true) {
			// Close the window on close button
			close_window = getWindowProperty("Edition", WND_PROP_VISIBLE) < 1;
			if (close_window) {
				display(video.getEditedFrame(), 0);
				destroyAllWindows();
				break;
			}
			video.updateBboxes(x_mouse, y_mouse, &click, &hold);
			imshow("Edition", video.getEditedFrame());
			if (mouse_right_click) {
				cv::Point center(x_mouse, y_mouse);
				video.addBbx(center);
				mouse_right_click = false;
			}
			waitKey(1);
		}
	}
		   // Load and show image from PC into picture box
	private: System::Void button_Browse_Click(System::Object^ sender, System::EventArgs^ e) {
		OpenFileDialog^ dgOpen = gcnew OpenFileDialog();
		dgOpen->Filter = "Image(*.bmp; *.jpg)|*.bmp;*.jpg|All files (*.*)|*.*||";
		if (dgOpen->ShowDialog() == System::Windows::Forms::DialogResult::Cancel)
			return;
		// Read the image with opencv
		img = imread(ConvertString2Char(dgOpen->FileName));
		// Resize the image to its placeholder dimensions
		int ptb_width = ptbSource->Size.Width;
		int ptb_height = ptbSource->Size.Height;
		resize(img, img, cv::Size(ptb_width, ptb_height), INTER_CUBIC);
		original_img = img.clone();
		// Convert the image into bitmap
		ptbSource->Image = ConvertMat2Bitmap(img);
		ptbSource->Refresh();
		// Set image size within the Bboxes class
		bboxes.set_img_size(img.size());
	}
		   // Display a cv::Mat in the picture box
	private: void display(Mat img, int ptb) {
		// Display the camera
		if (ptb == 0)
			if (!img.empty()) {
				ptbSource->Image = ConvertMat2Bitmap(img); // Refresh the image on the Windows application
				//ptbSource->Refresh(); // If streaming, freeze
			}
		// Display the depth map
		if (ptb == 1)
			if (!img.empty()) {
				ptbDepthMap->Image = ConvertMat2Bitmap(img); // Refresh the image on the Windows application
				//ptbSource->Refresh(); // If streaming, freeze
				//ptbDepthMap->Refresh();
			}
	}
		   // Double click event on the Image
	private: System::Void ptbSource_MouseDoubleClick(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e) {
		int mod = 0;
		if (mod == 0) {
			target_point2D = cv::Point(e->X, e->Y);

			// Need to take into account the Depth map... 

			float* distances = dvl.getDistances();
			float d = distances[0];

			d = 0.715;


			int u = target_point2D.x - camera_params.center.x;
			int v = target_point2D.y - camera_params.center.y;
			//target_point3D = cv::Point3d(u * d / camera_params.fx, v * d / camera_params.fy, d);
			target_point3D = cv::Point3d(-v * d / camera_params.fy, -u * d / camera_params.fx, d);

			robot.setTarget(target_point3D);

			/*this->labelTarget->Text = "Target (x, y, z): " + getPrecision(target_point3D.x, 3) + " m " +
				getPrecision(target_point3D.y, 3) + " m " +
				getPrecision(target_point3D.z, 3) + " m";*/

			draw_target = true;

		}

		//// Zoom
		//if (mouse_click == 'L') {
		//	int zoom = 500;
		//	int ptb_width  = ptbSource->Size.Width;
		//	int ptb_height = ptbSource->Size.Height;
		//	int x1 = int(mouse_pos.x - zoom / 2);
		//	int y1 = int(mouse_pos.y - zoom / 2);
		//	if (x1 + zoom > ptb_width) {
		//		int delta = x1 + zoom - ptb_width;
		//		x1 -= delta;
		//	}
		//	else if (x1 < 0)
		//		x1 = 0;
		//	if (y1 + zoom > ptb_height) {
		//		int delta = y1 + zoom - ptb_height;
		//		y1 -= delta;
		//	}
		//	else if (y1 < 0)
		//		y1 = 0;
		//	cv::Rect myROI(x1, y1, zoom, zoom);
		//	img = img(myROI);
		//	resize(img, img, cv::Size(ptb_width, ptb_height), INTER_CUBIC);
		//	display(img, 0);
		//}
		//// Dezoom
		//else if (mouse_click == 'R') {
		//	if(edited_img.empty())
		//		img = original_img.clone();
		//	else
		//		img = edited_img.clone();
		//	display(img, 0);
		//}
	}
		   // Get mouse state when clicked
	private: System::Void ptbSource_MouseDown(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e) {
		/*mouse_pos = cv::Point(e->X, e->Y);
		cv::Point mouseDownLocation = cv::Point(e->X, e->Y);
		System::String^ eventString = nullptr;
		switch (e->Button) {
		case System::Windows::Forms::MouseButtons::Left:
			mouse_click = 'L';
			break;
		case System::Windows::Forms::MouseButtons::Right:
			mouse_click = 'R';
			break;
		case System::Windows::Forms::MouseButtons::Middle:
			mouse_click = 'M';
			break;
		case System::Windows::Forms::MouseButtons::None:
		default:
			mouse_click = ' ';
			break;
		}*/
	}

	private: System::Void pictureBoxBathy_MouseDoubleClick(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e) {
		std::cout << "ici meme" << std::endl;
		int mod = 0;
		float scale_factor_resized_map = 28./560. ;
		if (mod == 0) {
			target_point2D = cv::Point(e->X, e->Y); // endroit du click
			std::cout << "scale factor : "<< scale_factor_resized_map << std::endl;
			double target_point_metres_x_robot = (double)(e->X - 280) * scale_factor_resized_map; // 280 la moitie de la taille de l'image
			double target_point_metres_y_robot = (double)(-e->Y + 280) * scale_factor_resized_map;// dans le repere du robot inversee par rapport a la carte resized
			
			target_point3D = cv::Point3d(target_point_metres_x_robot, target_point_metres_y_robot, 0);
																				  // Need to take into account the Depth map... 
			std::cout << "valeur x : " << target_point3D.x << "valeur y : " << target_point3D.y << "valeur z : " << target_point3D.z << std::endl;
			//float* distances = dvl.getDistances();
			//float d = distances[0];

			//d = 0.715;


			//int u = target_point2D.x - camera_params.center.x;
			//int v = target_point2D.y - camera_params.center.y;
			////target_point3D = cv::Point3d(u * d / camera_params.fx, v * d / camera_params.fy, d);
			//target_point3D = cv::Point3d(-v * d / camera_params.fy, -u * d / camera_params.fx, d);

			robot.setTarget(target_point3D);
			robot.goToTarget();

			/*this->labelTarget->Text = "Target (x, y, z): " + getPrecision(target_point3D.x, 3) + " m " +
				getPrecision(target_point3D.y, 3) + " m " +
				getPrecision(target_point3D.z, 3) + " m";*/

			//draw_target = true;

		}
	}






		   // Temp button
	private: System::Void view_button_Click(System::Object^ sender, System::EventArgs^ e) {
		//listView1->Items->Clear();
		listView1->View = System::Windows::Forms::View::Details;
		//listView1->Columns->Add("Related frames", 200);
		listView1->AutoResizeColumn(0, ColumnHeaderAutoResizeStyle::HeaderSize);
		populate();
	}
		   // Fill the image list
	private: void populate() {
		cv::String dirname = "C:/Users/Utilisateur/Pictures/*.jpg";
		glob(dirname, realted_img_paths);
		for (size_t i = 0; i < realted_img_paths.size(); ++i) {
			System::String^ path = gcnew System::String(realted_img_paths[i].c_str()); // Convert string to System::String
			imageList1->Images->Add(Image::FromFile(path));
		}
		listView1->SmallImageList = imageList1;
		for (size_t i = 0; i < realted_img_paths.size(); ++i) {
			std::string str = " - 00:00::0" + std::to_string(i) + "::00"; // Create a string
			System::String^ name = gcnew System::String(str.c_str()); // Convert back to String^
			listView1->Items->Add(name, i);
		}
	}
		   // Display the selected img in the picturebox
	private: System::Void listView1_MouseClick(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e) {
		System::String^ selected = listView1->SelectedItems[0]->SubItems[0]->Text;
		int img_index = listView1->SelectedItems[0]->Index;
		img = imread(realted_img_paths[img_index]);
		int ptb_width = ptbSource->Size.Width;
		int ptb_height = ptbSource->Size.Height;
		resize(img, img, cv::Size(ptb_width, ptb_height), INTER_CUBIC);
		original_img = img.clone();
		display(img, 0);
	}
		   // Load the video
	private: System::Void load_button_Click(System::Object^ sender, System::EventArgs^ e) {
		std::string video_path = "C:/Users/Utilisateur/Videos/test.mp4";
		cv::Size window_size(ptbSource->Size.Width, ptbSource->Size.Height);
		video.init(video_path, window_size);
		if (!video.isOpened()) // TODO: Manage this error
			cout << "Error opening video stream or file" << endl;
		this->video_trackBar->Maximum = video.getFramesNr() - 1;
		this->video_trackBar->Value = 0;
		set_video_label();
		video.nextFrame();
		display(video.getEditedFrame(), 0);
	}
		   // Play or pause the video
	private: System::Void play_button_Click(System::Object^ sender, System::EventArgs^ e) {
		if (video.getMode() == "play") {
			this->play_button->Text = L"Play";
			video.setMode("pause");
		}
		else if (video.getMode() == "pause") {
			this->play_button->Text = L"Pause";
			video.setMode("play");
		}
		else if (video.getMode() == "replay") {
			video.setFrame(0);
			this->video_trackBar->Value = video.getFrameId() + 1;
			set_video_label();
			this->play_button->Text = L"Pause";
			video.setMode("play");
		}
		while (video.getMode() == "play") {
			video.nextFrame();
			if (video.frame.empty()) {
				this->play_button->Text = L"Replay";
				video.setMode("replay");
				video.setFrame(video.getFrameId() - 1);
				video.nextFrame();
				set_video_label();
				video.nextFrame(); // Only god know why I have to add this line
				break;
			}
			else {
				this->video_trackBar->Value = video.getFrameId();
				set_video_label();
			}
			display(video.getEditedFrame(), 0);
			waitKey(video.getWaitTimer());
		}
	}
		   // Speed up the frame rate
	private: System::Void speed_button_Click(System::Object^ sender, System::EventArgs^ e) {
		if (video.getFpsFactor() == 1) {
			string s = "x3";
			System::String^ name = gcnew System::String(s.c_str());
			this->speed_button->Text = name;
			video.setFpsFactor(2);
		}
		else if (video.getFpsFactor() == 2) {
			string s = "x5";
			System::String^ name = gcnew System::String(s.c_str());
			this->speed_button->Text = name;
			video.setFpsFactor(3);
		}
		else if (video.getFpsFactor() == 3) {
			string s = "x10";
			System::String^ name = gcnew System::String(s.c_str());
			this->speed_button->Text = name;
			video.setFpsFactor(5);
		}
		else if (video.getFpsFactor() == 5) {
			string s = "x1";
			System::String^ name = gcnew System::String(s.c_str());
			this->speed_button->Text = name;
			video.setFpsFactor(10);
		}
		else if (video.getFpsFactor() == 10) {
			string s = "x2";
			System::String^ name = gcnew System::String(s.c_str());
			this->speed_button->Text = name;
			video.setFpsFactor(1);
		}
	}
		   // Set frame to track bar
	private: System::Void video_trackBar_Scroll(System::Object^ sender, System::EventArgs^ e) {
		if (video.getMode() == "replay")
			if (video.getFrameId() < video.getFramesNr()) {
				play_video = "pause";
				video.setMode("pause");
				this->play_button->Text = L"Play";
			}
		video.setFrame(video_trackBar->Value);
		set_video_label();
		video.nextFrame();
		display(video.getEditedFrame(), 0);
	}
		   // Set the video label
	private: void set_video_label() {
		int displayed_label_frame = video.getFrameId() + 1;
		string s = std::to_string(displayed_label_frame + 1) + "/" + std::to_string(this->video_trackBar->Maximum + 1);
		System::String^ name = gcnew System::String(s.c_str());
		this->video_label->Text = name;
	}
		   // Record the video
	private: System::Void record_button_Click(System::Object^ sender, System::EventArgs^ e) {
		camera_recording = !camera_recording;

		if (camera_recording) {
			camera_recording = true;

			this->record_button->Text = L"Recording";
			this->record_button->BackColor = System::Drawing::Color::Red;
			this->record_button->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));

			// Start video writer
			std::string vid_path = getNewVideoPath();
			//output_video.open(vid_path, VideoWriter::fourcc('a', 'v', 'c', '1'), 15, cv::Size(640, 480), true);
			//output_video.open(vid_path, VideoWriter::fourcc('m', 'p', '4', 'v'), 15, cv::Size(640, 480), true);
#define OPENCV_FFMPEG_WRITER_OPTIONS = "vcodec;x264|vprofile;high|vlevel;9.0";
			output_video.open(vid_path, VideoWriter::fourcc('H', '2', '6', '4'), 15, cv::Size(640, 480), true);
			//output_video.open(vid_path, VideoWriter::fourcc('M', 'J', 'P', 'G'), 15, cv::Size(640, 480), true);

			log("From video recording: " + vid_path + " created");
		}
		else {
			//camera_show = false;
			camera_recording = false;
			this->record_button->Text = L"Start recording";
			this->record_button->BackColor = System::Drawing::SystemColors::Control;
			this->record_button->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
		}
	}


		   /*--------------------------------------------------------------
		   *
		   *
		   *		ARDUINO
		   *
		   *
		   * --------------------------------------------------------------*/

		   // Check if Arduino has started
	private: System::Void backgroundWorkerArduinoStarted_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		arduino.rcvAny();
		backgroundWorkerArduinoStarted->CancelAsync();
		e->Cancel = true;
	}
		   // Enable Jetson and DVL starting
	private: System::Void backgroundWorkerArduinoStarted_RunWorkerCompleted(System::Object^ sender, System::ComponentModel::RunWorkerCompletedEventArgs^ e) {
		this->buttonJetsonOn->Text = L"Jetson on";
		this->buttonJetsonOn->Enabled = true;
		this->buttonDVLon->Text = L"DVL on";
		this->buttonDVLon->Enabled = true;
	}
		   // Receive data from Arduino continuously
	private: System::Void backgroundWorkerArduino_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		while (true) {
			arduino.rcvData(plateform.atm_pressure);
			backgroundWorkerArduino->ReportProgress(arduino.depth); // To change
			backgroundWorkerArduino->ReportProgress(arduino.imu[0]);
			backgroundWorkerArduino->ReportProgress(arduino.imu[1]);
			backgroundWorkerArduino->ReportProgress(arduino.imu[2]);
		}
	}
		   // Display arduino state in real-time
	private: System::Void backgroundWorkerArduino_ProgressChanged(System::Object^ sender, System::ComponentModel::ProgressChangedEventArgs^ e) {

		this->textBoxArduinoRoll->Text = "" + getPrecision(arduino.imu[0], 3);
		this->textBoxArduinoPitch->Text = "" + getPrecision(arduino.imu[1], 3);
		this->textBoxArduinoYaw->Text = "" + getPrecision(arduino.imu[2], 3);

		this->textBoxArduinoDepth->Text = "" + getPrecision(arduino.depth, 3);

		/*if (arduino.depth >= -0.5) {
			if (this->buttonDVLon->Text == "DVL off") {
				arduino.dvlOff();
				system(DVL_READER_CLOSE);
				MessageBox::Show("DVL turned off. Minimal depth (-0.5m) reached.");
				this->buttonDVLon->Text = L"DVL on";
				this->labelDVLon->Visible = true;
				this->buttonDVLyes->Visible = true;
				this->buttonDVLno->Visible = true;
				log("DVL turned off automatically. Minimal depth (-0.5m) reached.");
			}
		}*/
	}

		   /*--------------------------------------------------------------
		   *
		   *
		   *		ROBOT
		   *
		   *
		   * --------------------------------------------------------------*/

		   // Has the robot started to communicate ?
	private: System::Void backgroundWorkerRobotStarted_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		robot.rcvData();
		backgroundWorkerRobotStarted->CancelAsync();
		e->Cancel = true;
	}
		   // Receive data from robot continuously
	private: System::Void backgroundWorkerRobot_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		while (true) {
			robot.rcvData();
			backgroundWorkerRobot->ReportProgress(robot.pos[0]);
			backgroundWorkerRobot->ReportProgress(robot.angles[0]);
			backgroundWorkerRobot->ReportProgress(robot.state);
			backgroundWorkerRobot->ReportProgress(robot.gripper_state);
			backgroundWorkerRobot->ReportProgress(robot.pump_state);
			//Sleep(150);
		}
	}
		   // Send a command to the robot
	private: System::Void backgroundWorkerRobotCommand_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		//robot.goTo(0, 0, 0, 0);
		//robot.goToTarget(); // mai 2023
		// Stop background worker when finished
		backgroundWorkerRobotCommand->CancelAsync();
		e->Cancel = true;
	}
		   // Display robot state in real-time
	private: System::Void backgroundWorkerRobot_ProgressChanged(System::Object^ sender, System::ComponentModel::ProgressChangedEventArgs^ e) {
		this->textBoxRobotState->Text = gcnew System::String(robot.current_state.c_str());
		//this->labelRobotState->Text = "Robot state: " + gcnew System::String(robot.current_state.c_str());
		//this->labelRobotX->Text = "Robot (x, y, z): " + getPrecision(robot.pos[0], 3) + " m " + getPrecision(robot.pos[1], 3) + " m " + getPrecision(robot.pos[2], 3) + " m";
		this->textBoxRobotX->Text = "" + getPrecision(robot.pos[0], 3);
		this->textBoxRobotY->Text = "" + getPrecision(robot.pos[1], 3);
		this->textBoxRobotZ->Text = "" + getPrecision(robot.pos[2], 3);

		//this->labelRobotAngles->Text = "Robot (roll, pitch, yaw): " + getPrecision(robot.angles[0], 3) + "° " + getPrecision(robot.angles[1], 3) + "° " + getPrecision(robot.angles[2], 3) + "°";
		this->textBoxRobotRoll->Text = "" + getPrecision(robot.angles[0], 3);
		this->textBoxRobotPitch->Text = "" + getPrecision(robot.angles[1], 3);
		this->textBoxRobotYaw->Text = "" + getPrecision(robot.angles[2], 3);

		if (robot.gripper_state)
			this->textBoxGripper->Text = "ON";
		//this->labelGripper->Text = "Gripper: ON";
		else
			this->textBoxGripper->Text = "OFF";
		//this->labelGripper->Text = "Gripper: OFF";
		if (robot.pump_state)
			this->textBoxPump->Text = "ON";
		//this->labelPump->Text = "Pump: ON";
		else
			this->textBoxPump->Text = "OFF";
		//this->labelPump->Text = "Pump: OFF";


		// Display geo coordinates of the robot in DD
		double lat_P = 0, long_P = 0;
		plateform.getGeoFromXYPos(robot.pos[0], robot.pos[1], &lat_P, &long_P);
		this->textBoxLatRobot->Text = gcnew System::String(std::to_string(lat_P).c_str());
		this->textBoxLongRobot->Text = gcnew System::String(std::to_string(long_P).c_str());

		// LatLonToUTMXY(FLOAT lat, FLOAT lon, int zone, FLOAT& x, FLOAT& y)

		float* utm_x = 0;
		float* utm_y = 0;
		int zone = 33;
		//LatLonToUTMXY(float(lat_P), float(long_P), zone, *utm_x, *utm_y);

		//cout << "Test :" << utm_x << endl;

		//this->textBoxUTMXRobot->Text = "";
		//this->textBoxUTMYRobot->Text = "";


	}
		   // Check if command is done
	private: System::Void backgroundWorkerRobotCommand_RunWorkerCompleted(System::Object^ sender, System::ComponentModel::RunWorkerCompletedEventArgs^ e) {
		log("Robot command done");
		this->buttonGoToTarget->Enabled = true;
	}
		   // Enable go to target button
	private: System::Void backgroundWorkerRobotStarted_RunWorkerCompleted(System::Object^ sender, System::ComponentModel::RunWorkerCompletedEventArgs^ e) {
		log("Robot started");
		this->buttonGoToTarget->Enabled = true;
		this->buttonScan->Enabled = true;
	}
		   // Start scanning
	private: System::Void backgroundWorkerRobotScan_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		log("Start scanning");
		robot.scan();
		backgroundWorkerRobotScan->CancelAsync();
		e->Cancel = true;
	}
	private: System::Void backgroundWorkerRobotScan_RunWorkerCompleted(System::Object^ sender, System::ComponentModel::RunWorkerCompletedEventArgs^ e) {
		this->buttonScan->Enabled = true;
		this->textBox1->Enabled = true;
		log("Scanning completed");
	}
		   // Go to target
	private: System::Void buttonGoToTarget_Click(System::Object^ sender, System::EventArgs^ e) {
		log("Go to target");
		this->buttonGoToTarget->Enabled = false;
		backgroundWorkerRobotCommand->RunWorkerAsync(1);
	}
		   // Scanning the zone button
	private: System::Void buttonScan_Click(System::Object^ sender, System::EventArgs^ e) {
		this->buttonScan->Enabled = false;
		this->textBox1->Enabled = false;
		backgroundWorkerRobotScan->RunWorkerAsync(1);

	}
		   // Chose the scanning depth
	private: System::Void textBox1_TextChanged(System::Object^ sender, System::EventArgs^ e) {
		std::string depth_str = ConvertString2Char(textBox1->Text);
		robot.scanning_depth = std::stof(depth_str);
	}

		   /*--------------------------------------------------------------
		   *
		   *
		   *		DVL
		   *
		   *		TODO: Verify if DVL is unerwater function System::Void backgroundWorkerDVLOn_RunWorkerCompleted
		   *		TODO: STOP DVL if DVL is near to sea level function to be done
		   *
		   *
		   * --------------------------------------------------------------*/

		   // Receive data from DVL continuously
	private: System::Void backgroundWorkerDVL_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		while (true) {
			dvl.rcvData();
			dvl.set_coordinates(arduino.depth);
			backgroundWorkerDVL->ReportProgress(dvl.vx);
			backgroundWorkerDVL->ReportProgress(dvl.vy);
			backgroundWorkerDVL->ReportProgress(dvl.imu[0]);
			backgroundWorkerDVL->ReportProgress(dvl.imu[1]);
			backgroundWorkerDVL->ReportProgress(dvl.imu[2]);
			backgroundWorkerDVL->ReportProgress(dvl.distances[0]);
			backgroundWorkerDVL->ReportProgress(dvl.distances[1]);
			backgroundWorkerDVL->ReportProgress(dvl.distances[2]);
			backgroundWorkerDVL->ReportProgress(dvl.distances[3]);
		}
	}
		   // Display DVL state in real-time
	private: System::Void backgroundWorkerDVL_ProgressChanged(System::Object^ sender, System::ComponentModel::ProgressChangedEventArgs^ e) {
		this->textBoxDVLVx->Text = "" + getPrecision(dvl.vx, 3);
		this->textBoxDVLVy->Text = "" + getPrecision(dvl.vy, 3);

		this->textBoxDVLRoll->Text = "" + getPrecision(dvl.imu[0], 3);
		this->textBoxDVLPitch->Text = "" + getPrecision(dvl.imu[1], 3);
		this->textBoxDVLYaw->Text = "" + getPrecision(dvl.imu[2], 3);

		this->textBoxDVLD0->Text = "" + getPrecision(dvl.distances[0], 3);
		this->textBoxDVLD1->Text = "" + getPrecision(dvl.distances[1], 3);
		this->textBoxDVLD2->Text = "" + getPrecision(dvl.distances[2], 3);
		this->textBoxDVLD3->Text = "" + getPrecision(dvl.distances[3], 3);
	}
		   // Start DVL and UDP reader
	private: System::Void backgroundWorkerDVLOn_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		log("DVL initialization");

		arduino.dvlOn();
		Sleep(dvl.init_time);
		// Start DVL reader
		system(DVL_READER_START);
		Sleep(500);
		log("DVL turned on successfully.");
		arduino.is_dvl_on = true;



		//else
		//	log("DVL cannot be turned on. Minimal depth (-0.5m) not reached." + std::to_string(arduino.depth));
		backgroundWorkerDVLOn->CancelAsync();
		e->Cancel = true;
	}



		   // Set the DVL button to "On" (which means dvl currently off) if minimal depth hasn't be reached
	private: System::Void backgroundWorkerDVLOn_RunWorkerCompleted(System::Object^ sender, System::ComponentModel::RunWorkerCompletedEventArgs^ e) {
		/*if (arduino.depth < 0.5) {
			this->buttonDVLon->Text = L"DVL on";

			MessageBox::Show("The DVL cannot be turned on. Minimal depth (0.5m) not reached.");
			this->labelDVLon->Visible = true;
			this->buttonDVLyes->Visible = true;
			this->buttonDVLno->Visible = true;
		}*/

	}
		   // Start DVL button
	private: System::Void buttonDVLon_Click(System::Object^ sender, System::EventArgs^ e) {
		if (this->buttonDVLon->Text == L"DVL on") {
			MessageBox::Show("Please make sure the DVL is underwater. It will break if not turned on underwtaer.");
			this->labelDVLon->Visible = true;
			this->buttonDVLyes->Visible = true;
			this->buttonDVLno->Visible = true;
		}
		else {
			arduino.dvlOff();
			system(DVL_READER_CLOSE);
			this->buttonDVLon->Text = L"DVL on";
		}

	}
	private: System::Void buttonDVLyes_Click(System::Object^ sender, System::EventArgs^ e) {
		this->buttonDVLon->Text = L"DVL off";
		this->labelDVLon->Visible = false;
		this->buttonDVLyes->Visible = false;
		this->buttonDVLno->Visible = false;
		backgroundWorkerDVLOn->RunWorkerAsync(1); // 30s and exceute DVL reader

	}
	private: System::Void buttonDVLno_Click(System::Object^ sender, System::EventArgs^ e) {
		this->labelDVLon->Visible = false;
		this->buttonDVLyes->Visible = false;
		this->buttonDVLno->Visible = false;
	}


		   /*--------------------------------------------------------------
		   *
		   *
		   *		JETSON
		   *
		   *
		   * --------------------------------------------------------------*/

		   // Receive video stream from Jetson
	private: System::Void backgroundWorkerJetson_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		int recvMsgSize;

		while (true) {

			do {
				recvMsgSize = jetson.recv();
			} while (recvMsgSize > sizeof(int));

			int total_pack = ((int*)jetson.getBuffer())[0];

			int pack_count = 0;

			char* long_buffer = new char[PACK_SIZE * total_pack];
			for (int i = 0; i < total_pack; i++) {
				recvMsgSize = jetson.recv();

				if (recvMsgSize != PACK_SIZE)
					log("From video recording: received unexpected size pack:" + std::to_string(recvMsgSize));
				else {
					pack_count++;
					memcpy(&long_buffer[i * PACK_SIZE], jetson.getBuffer(), PACK_SIZE);
				}
			}

			if (pack_count == total_pack) {
				jetson.decodeFrame(long_buffer, total_pack);
				stream_frame = jetson.getFrame();

				// Record the stream
				if (camera_recording)
					output_video.write(stream_frame);
				else
					output_video.release();

				// Draw center
				cv::line(stream_frame, cv::Point(0, FRAME_HEIGHT / 2), cv::Point(FRAME_WIDTH, FRAME_HEIGHT / 2), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
				cv::line(stream_frame, cv::Point(FRAME_WIDTH / 2, 0), cv::Point(FRAME_WIDTH / 2, FRAME_HEIGHT), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

				// Draw target
				if (draw_target) {
					cv::circle(stream_frame, target_point2D, 5, cv::Scalar(0, 255, 0), 1);
					cv::line(stream_frame, cv::Point(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), target_point2D, cv::Scalar(0, 150, 0), 1, cv::LINE_AA);
				}

				// Draw measurement line
				if (draw_measure) {
					if (measure.a.x > 0 && measure.a.y > 0 && measure.b.x > 0 && measure.b.y)
						cv::line(stream_frame, measure.a, measure.b, cv::Scalar(0, 0, 255), 1);
					cv::circle(stream_frame, measure.a, 3, cv::Scalar(0, 0, 255), -1);
					cv::circle(stream_frame, measure.b, 3, cv::Scalar(0, 0, 255), -1);
				}



				// Display the stream
				if (camera_show)
					if (!(stream_frame.size().width == 0))
						display(stream_frame, 0);

			}
			stream_frame.release();
			free(long_buffer);
		}
	}
		   // Start Jetson
	private: System::Void buttonJetsonOn_Click(System::Object^ sender, System::EventArgs^ e) {

		if (this->buttonJetsonOn->Text == L"Jetson on") {
			this->buttonJetsonOn->Enabled = false;
			backgroundWorkerJetsonOn->RunWorkerAsync(1);
		}
		else {
			buttonJetsonOn->Enabled = false;
			try {
				output_video.release();
				camera_recording = false;
				this->record_button->Text = L"Start recording";
				this->record_button->BackColor = System::Drawing::SystemColors::Control;
				this->record_button->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					static_cast<System::Byte>(0)));
				this->record_button->Enabled = false;
			}
			catch (...) {
				log("Error while stop recording");
			}
			backgroundWorkerJetsonOff->RunWorkerAsync(1);
		}
	}
		   // Check if Jetsin is on
	private: System::Void backgroundWorkerJetsonOn_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		// Order is important
		arduino.jetsonOn();
		jetson.turnOn();
		backgroundWorkerJetsonOn->CancelAsync();
		e->Cancel = true;
	}
	private: System::Void backgroundWorkerJetsonOn_RunWorkerCompleted(System::Object^ sender, System::ComponentModel::RunWorkerCompletedEventArgs^ e) {
		this->buttonJetsonOn->Text = L"Jetson off";
		this->buttonJetsonOn->Enabled = true;
		this->record_button->Enabled = true;
	}
		   // Turn off the Jetson
	private: System::Void backgroundWorkerJetsonOff_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		// Order is important
		jetson.turnOff();
		Sleep(25000); // Sleep 25s to let the jetson shutdown
		arduino.jetsonOff();
		backgroundWorkerJetsonOff->CancelAsync();
		e->Cancel = true;
	}
	private: System::Void backgroundWorkerJetsonOff_RunWorkerCompleted(System::Object^ sender, System::ComponentModel::RunWorkerCompletedEventArgs^ e) {
		this->buttonJetsonOn->Text = L"Jetson on";
		buttonJetsonOn->Enabled = true;
	}

		   /*--------------------------------------------------------------
		   *
		   *
		   *		Depth map
		   *
		   *
		   * --------------------------------------------------------------*/

		   // Compute and display the depth map
	private: System::Void backgroundWorkerDepthMap_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {

		while (true) {
			if (robot.has_started || true) {

				depth_map.init(); // Reinit displayed map


				// Should update only if DVL is on ? TODO
				if (arduino.is_dvl_on)
					depth_map.update(dvl.coordinates,
						robot.coordinates,
						dvl.distances,
						arduino.depth,
						plateform.lat_orig_carte, plateform.long_orig_carte,
						plateform.latitude_GPS_Master_babord, plateform.longitude_GPS_Master_babord,
						plateform.cap_GPS_babord_vers_tribord); // Update mapping

				depth_map.setDepthMap(); // Update drawing
				depth_map.setTide(dvl.coordinates); // Compute tide if depth map ha been freezed
				depth_map.drawGrid(); // Draw a grid
				depth_map.setPos(robot.pos[0], robot.pos[1]); // Set the robot position
				depth_map.drawRobot(); // Draw the robot
				if (draw_target) {
					depth_map.drawTarget(cv::Point2f(robot.target[0], robot.target[1])); // Draw target if clicked on the camera
					// TODO, click on the map
				}
				display(depth_map.getMap(), 1);

				// Send map to the simulation PC
				simulation.sendMap(depth_map.getUDPFrame());

				backgroundWorkerDepthMap->ReportProgress(depth_map.tide);
				backgroundWorkerDepthMap->ReportProgress(depth_map.altitude_max);
				backgroundWorkerDepthMap->ReportProgress(depth_map.altitude_min);
			}
			Sleep(50);
		}
	}

		   // Display tide in real time when possible
	private: System::Void backgroundWorkerDepthMap_ProgressChanged(System::Object^ sender, System::ComponentModel::ProgressChangedEventArgs^ e) {
		if (depth_map.is_freezed)
			this->textBoxTide->Text = "" + getPrecision(depth_map.tide, 4);
		// Set min and max text for the color map // TODO erreur 
		this->labelMinDepth->Text = "" + getPrecision(depth_map.altitude_min, 3) + " m";
		this->labelMaxDepth->Text = "" + getPrecision(depth_map.altitude_max, 3) + " m";
	}

		   // Show depth values with mouse
	private: System::Void ptbDepthMap_MouseMove(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e) {
		cv::Point mouse = cv::Point(e->X, e->Y);
		float depth = depth_map.getDepth(mouse);
		this->textBoxMouseDepth->Text = "" + getPrecision(depth, 2);

	}
		   // Change resolution of the depth map
	private: System::Void buttonResolutionMap_Click(System::Object^ sender, System::EventArgs^ e) {

		depth_map.which_res += 1;
		if (depth_map.which_res > 2)
			depth_map.which_res = 0;

		if (depth_map.which_res == 0)
			buttonResolutionMap->Text = "Resolution 05x05 cm";
		else if (depth_map.which_res == 1)
			buttonResolutionMap->Text = "Resolution 25x25 cm";
		else if (depth_map.which_res == 2)
			buttonResolutionMap->Text = "Resolution 50x50 cm";

	}
		   // Button to save both the image and the data.altitude of the depth map. Should add timestamp and more data in the future
	private: System::Void buttonSaveDepthMap_Click(System::Object^ sender, System::EventArgs^ e) {
		string img_path = depthmap_dir_path + "map_" + getTime() + ".png";
		string data_path = depthmap_dir_path + "map_" + getTime() + ".yml";

		cv::imwrite(img_path, depth_map.map);

		Mat_<double> data(depth_map.high_depth_map.size(), depth_map.high_depth_map[0].size());

		for (size_t i = 0; i < depth_map.high_depth_map.size(); i++)
			for (size_t j = 0; j < depth_map.high_depth_map[i].size(); j++)
				data[i][j] = depth_map.high_depth_map[i][j].altitude;

		cv::FileStorage fs(data_path, cv::FileStorage::WRITE); // create FileStorage object
		fs << "depthMap" << data; // command to save the data
		fs.release(); // releasing the file.
		log("Depthmap saved! " + img_path);
		log("Depthmap saved! " + data_path);
	}

	private: System::Void buttonFreezeDM_Click(System::Object^ sender, System::EventArgs^ e) {
		depth_map.freezeDepthMap();

		string img_path = depthmap_dir_path + "freezed_map_" + getTime() + ".png";
		string data_path = depthmap_dir_path + "freezed_map_" + getTime() + ".yml";

		cv::imwrite(img_path, depth_map.map);

		Mat_<double> data(depth_map.freezed_high_depth_map.size(), depth_map.freezed_high_depth_map[0].size());

		for (size_t i = 0; i < depth_map.freezed_high_depth_map.size(); i++)
			for (size_t j = 0; j < depth_map.freezed_high_depth_map[i].size(); j++)
				data[i][j] = depth_map.freezed_high_depth_map[i][j].altitude;

		cv::FileStorage fs(data_path, cv::FileStorage::WRITE); // create FileStorage object
		fs << "freezedDepthMap" << data; // command to save the data
		fs.release(); // releasing the file.
		log("Freezed depthmap saved! " + img_path);
		log("Freezed depthmap saved! " + data_path);
	}

		   /*--------------------------------------------------------------
		   *
		   *
		   *		Plateform (Starboard IMU, Port IMU and atmospherical pressure)
		   *
		   *
		   * --------------------------------------------------------------*/

		   // Receive data from starboard plateform continuously (imu + atm pressure)
	private: System::Void backgroundWorkerPlateformS_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		while (true) {
			plateform.rcvDataS();
			backgroundWorkerPlateformS->ReportProgress(plateform.imu_S[0]);
			backgroundWorkerPlateformS->ReportProgress(plateform.imu_S[1]);
			backgroundWorkerPlateformS->ReportProgress(plateform.imu_S[2]);
			backgroundWorkerPlateformS->ReportProgress(plateform.v_gyro_S[0]);
			backgroundWorkerPlateformS->ReportProgress(plateform.v_gyro_S[1]);
			backgroundWorkerPlateformS->ReportProgress(plateform.v_gyro_S[2]);
			backgroundWorkerPlateformS->ReportProgress(plateform.atm_pressure);
		}

	}
		   // Receive data from port plateform continuously
	private: System::Void backgroundWorkerPlateformP_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		while (true) {
			plateform.rcvDataP();
			backgroundWorkerPlateformP->ReportProgress(plateform.imu_P[0]);
			backgroundWorkerPlateformP->ReportProgress(plateform.imu_P[1]);
			backgroundWorkerPlateformP->ReportProgress(plateform.imu_P[2]);
			backgroundWorkerPlateformP->ReportProgress(plateform.v_gyro_P[0]);
			backgroundWorkerPlateformP->ReportProgress(plateform.v_gyro_P[1]);
			backgroundWorkerPlateformP->ReportProgress(plateform.v_gyro_P[2]);
		}
	}
		   // Display data from starboard continuously
	private: System::Void backgroundWorkerPlateformS_ProgressChanged(System::Object^ sender, System::ComponentModel::ProgressChangedEventArgs^ e) {
		this->textBoxSRoll->Text = "" + getPrecision(plateform.imu_S[0], 3);
		this->textBoxSPitch->Text = "" + getPrecision(plateform.imu_S[1], 3);
		this->textBoxSYaw->Text = "" + getPrecision(plateform.imu_S[2], 3);

		this->textBoxSGyrX->Text = "" + getPrecision(plateform.v_gyro_S[0], 3);
		this->textBoxSGyrY->Text = "" + getPrecision(plateform.v_gyro_S[1], 3);
		this->textBoxSGyrZ->Text = "" + getPrecision(plateform.v_gyro_S[2], 3);

		this->textBoxAtmPressure->Text = "" + getPrecision(plateform.atm_pressure, 3);


	}
		   // Display data from port continuously
	private: System::Void backgroundWorkerPlateformP_ProgressChanged(System::Object^ sender, System::ComponentModel::ProgressChangedEventArgs^ e) {
		this->textBoxPRoll->Text = "" + getPrecision(plateform.imu_P[0], 3);
		this->textBoxPPitch->Text = "" + getPrecision(plateform.imu_P[1], 3);
		this->textBoxPYaw->Text = "" + getPrecision(plateform.imu_P[2], 3);

		this->textBoxPGyrX->Text = "" + getPrecision(plateform.v_gyro_P[0], 3);
		this->textBoxPGyrY->Text = "" + getPrecision(plateform.v_gyro_P[1], 3);
		this->textBoxPGyrZ->Text = "" + getPrecision(plateform.v_gyro_P[2], 3);

	}

		   // Receive GPS from port side continuously
	private: System::Void backgroundWorkerPlateformGPSP_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		while (true) {
			plateform.rcvDataGps();

			backgroundWorkerPlateformGPSP->ReportProgress(plateform.latitude_GPS_Master_babord);
			backgroundWorkerPlateformGPSP->ReportProgress(plateform.longitude_GPS_Master_babord);
			backgroundWorkerPlateformGPSP->ReportProgress(plateform.latitude_GPS_Slave_tribord);
			backgroundWorkerPlateformGPSP->ReportProgress(plateform.longitude_GPS_Slave_tribord);
			backgroundWorkerPlateformGPSP->ReportProgress(plateform.cap_GPS_babord_vers_tribord);
		}
	}
		   // Receive GPS from starboard side continuously
	private: System::Void backgroundWorkerPlateformGPSS_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		while (true) {
			plateform.rcvDataGpsS();
			backgroundWorkerPlateformGPSS->ReportProgress(plateform.gps_S[0]);
			backgroundWorkerPlateformGPSS->ReportProgress(plateform.gps_S[1]);
			backgroundWorkerPlateformGPSS->ReportProgress(plateform.gps_rtk_S);
		}
	}
		   // Display GPS from port side continuously
	private: System::Void backgroundWorkerPlateformGPSP_ProgressChanged(System::Object^ sender, System::ComponentModel::ProgressChangedEventArgs^ e) {

		//this->textBoxPGPSX->Text = "" + getPrecision(plateform.latitude_GPS_Master_babord, 4);
		//this->textBoxPGPSY->Text = "" + getPrecision(plateform.longitude_GPS_Master_babord, 4);

		this->textBoxPGPSX->Text = gcnew System::String(std::to_string(plateform.latitude_GPS_Master_babord).c_str());
		this->textBoxPGPSY->Text = gcnew System::String(std::to_string(plateform.longitude_GPS_Master_babord).c_str());

		double utm_northing = 0, utm_easting = 0;
		char* utm_zone = "33";

		LLtoUTM((const double)plateform.latitude_GPS_Master_babord, (const double)plateform.longitude_GPS_Master_babord,
			utm_northing, utm_easting,
			utm_zone);

		this->textBoxPGPSX->Text = gcnew System::String(std::to_string(utm_northing).c_str());
		this->textBoxPGPSY->Text = gcnew System::String(std::to_string(utm_easting).c_str());


		this->textBoxSGPSX->Text = gcnew System::String(std::to_string(plateform.latitude_GPS_Slave_tribord).c_str());
		this->textBoxSGPSY->Text = gcnew System::String(std::to_string(plateform.longitude_GPS_Slave_tribord).c_str());

		LLtoUTM((const double)plateform.latitude_GPS_Slave_tribord, (const double)plateform.longitude_GPS_Slave_tribord,
			utm_northing, utm_easting,
			utm_zone);

		this->textBoxSGPSX->Text = gcnew System::String(std::to_string(utm_northing).c_str());
		this->textBoxSGPSY->Text = gcnew System::String(std::to_string(utm_easting).c_str());

		/*this->textBoxSGPSX->Text = "" + getPrecision(plateform.latitude_GPS_Slave_tribord, 4);
		this->textBoxSGPSY->Text = "" + getPrecision(plateform.longitude_GPS_Slave_tribord, 4);*/
		this->textBoxSRTK->Text = "" + getPrecision(plateform.RTK_S, 1);

		this->textBoxCap->Text = gcnew System::String(std::to_string(plateform.cap_GPS_babord_vers_tribord).c_str());
		this->textBoxCapDeg->Text = gcnew System::String(std::to_string(plateform.cap_GPS_babord_vers_tribord * 180 / PI).c_str());


		// Display geo coordinates of the robot in DD
		double lat_P = 0, long_P = 0;
		plateform.getGeoFromXYPos(robot.pos[0], robot.pos[1], &lat_P, &long_P);

		this->textBoxLatRobot->Text = gcnew System::String(std::to_string(lat_P).c_str());
		this->textBoxLongRobot->Text = gcnew System::String(std::to_string(long_P).c_str());
		LLtoUTM((const double)lat_P, (const double)long_P,
			utm_northing, utm_easting,
			utm_zone);
		this->textBoxUTMXRobot->Text = gcnew System::String(std::to_string(utm_northing).c_str());
		this->textBoxUTMYRobot->Text = gcnew System::String(std::to_string(utm_easting).c_str());

		// Distance between 2 gps
		this->textBoxDistanceGPS->Text = gcnew System::String(std::to_string(plateform.distanceGPS_P_S()).c_str());
		// Current cap in deg
		this->textBoxCurrentCap->Text = gcnew System::String(std::to_string(plateform.cap_GPS_babord_vers_tribord * 180 / 3.1415).c_str());

		// Compute delta to target UTM
		double delta_north = plateform.utm_target_north - utm_northing;
		double delta_east = plateform.utm_target_east - utm_easting;

		double robot_cap = plateform.cap_GPS_babord_vers_tribord;
		double delta_cap = plateform.target_cap - robot_cap * 180 / PI;

		this->textBoxUTMDeltaNorthing->Text = gcnew System::String(std::to_string(delta_north).c_str());
		this->textBoxDeltaUTMEasting->Text = gcnew System::String(std::to_string(delta_east).c_str());
		this->textBoxDeltaCap->Text = gcnew System::String(std::to_string(delta_cap).c_str());

		double targetx = delta_north * cos(PI - robot_cap) - delta_east * sin(PI - robot_cap);
		double targety = delta_north * sin(PI - robot_cap) + delta_east * cos(PI - robot_cap);

		this->textBoxDeltaXRobot->Text = gcnew System::String(std::to_string(targetx).c_str());
		this->textBoxDeltaYRobot->Text = gcnew System::String(std::to_string(targety).c_str());

		depth_map.setTarget(targetx, targety);

		// To display target in the map
		/*double zero_robot_lat = 0, zero_robot_long = 0;
		plateform.getGeoFromXYPos(0, 0, &zero_robot_lat, &zero_robot_long);
		double zero_robot_utm_n = 0, zero_robot_utm_e = 0;
		LLtoUTM((const double)zero_robot_lat, (const double)zero_robot_long,
			zero_robot_utm_n, zero_robot_utm_e,
			utm_zone);
		delta_north = plateform.utm_target_north - zero_robot_utm_n;
		delta_east = plateform.utm_target_east - zero_robot_utm_e;
		double target_zero_x = delta_north * cos(robot_cap - PI) - delta_east * sin(robot_cap - PI);
		double target_zero_y = delta_north * sin(robot_cap - PI) + delta_east * cos(robot_cap - PI);
		depth_map.setTarget(target_zero_x, target_zero_y);*/

	}
		   // Display GPS from starboard side continuously
	private: System::Void backgroundWorkerPlateformGPSS_ProgressChanged(System::Object^ sender, System::ComponentModel::ProgressChangedEventArgs^ e) {
	}

		   /*--------------------------------------------------------------
		   *
		   *
		   *		Mouse on the picture box, measurement etc.
		   *
		   *
		   * --------------------------------------------------------------*/

		   // Display mouse coordinate when moved on the picture box
	private: System::Void ptbSource_MouseMove(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e) {
		int u = e->X - camera_params.center.x;
		int v = e->Y - camera_params.center.y;

		float d = 1.0;

		float pos_3d_x = -v * d / camera_params.fy;
		float pos_3d_y = -u * d / camera_params.fx;
		float pos_3d_z = d;


		this->label3dX->Text = "x: " + getPrecision(pos_3d_x, 3) + " m";
		this->label3dY->Text = "y: " + getPrecision(pos_3d_y, 3) + " m";
		this->label3dZ->Text = "z: " + getPrecision(pos_3d_z, 3) + " m";

		// Fake
		float* distances = dvl.getDistances();
		d = distances[0];

		pos_3d_x = -v * d / camera_params.fy;
		pos_3d_y = -u * d / camera_params.fx;
		pos_3d_z = d;

		this->label3dXFake->Text = "x: " + getPrecision(pos_3d_x, 3) + " m";
		this->label3dYFake->Text = "y: " + getPrecision(pos_3d_y, 3) + " m";
		this->label3dZFake->Text = "z: " + getPrecision(pos_3d_z, 3) + " m";
	}
		   // Only for measurment purpose
	private: System::Void ptbSource_MouseClick(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e) {

		int x = e->X;
		int y = e->Y;
		if (first_point) {
			measure.a = cv::Point(x, y);
			first_point = !first_point;
		}
		else {
			measure.b = cv::Point(x, y);
			first_point = !first_point;
		}
		if (measure.a.x > 0 && measure.a.y > 0 && measure.b.x > 0 && measure.b.y) {
			float d = 1.0;
			d = 0.715;
			int u = measure.a.x - camera_params.center.x;
			int v = measure.a.y - camera_params.center.y;
			cv::Point3d p1 = cv::Point3d(u * d / camera_params.fx, v * d / camera_params.fy, d);
			u = measure.b.x - camera_params.center.x;
			v = measure.b.y - camera_params.center.y;
			cv::Point3d p2 = cv::Point3d(u * d / camera_params.fx, v * d / camera_params.fy, d);
			double result = cv::norm(p1 - p2);
			//this->labelLine->Text = "Line: " + getPrecision(result, 3) + " m";

			this->labelLine->Text = "Line: u1:" + (measure.a.x - camera_params.center.x) + " - u2 : " + (measure.b.x - camera_params.center.x);

			// Fake
			float* distances = dvl.getDistances();
			d = distances[0];
			d = 0.715;
			u = measure.a.x - camera_params.center.x;
			v = measure.a.y - camera_params.center.y;
			p1 = cv::Point3d(u * d / camera_params.fx, v * d / camera_params.fy, d);
			u = measure.b.x - camera_params.center.x;
			v = measure.b.y - camera_params.center.y;
			p2 = cv::Point3d(u * d / camera_params.fx, v * d / camera_params.fy, d);
			result = cv::norm(p1 - p2);
			this->labelLineFake->Text = "Line: " + getPrecision(result, 3) + " m";
		}
	}

		   /*--------------------------------------------------------------
		   *
		   *
		   *		Close secondary programs
		   *
		   *
		   * --------------------------------------------------------------*/

		   // Close imu and dvl readers when the form is closed
	private: System::Void MainForm_FormClosing(System::Object^ sender, System::Windows::Forms::FormClosingEventArgs^ e) {
		// Close dvl reader
		system(DVL_READER_CLOSE);
		// Close imu reader
		system(IMU_READER_CLOSE);
		// Close imu starboard  reader
		system(IMU_S_READER_CLOSE);
		// Close imu port  reader
		system(IMU_P_READER_CLOSE);
	}


	private: System::Void textBoxRobotYaw_TextChanged(System::Object^ sender, System::EventArgs^ e) {
	}
	private: System::Void textBoxRobotZ_TextChanged(System::Object^ sender, System::EventArgs^ e) {
	}
	private: System::Void textBoxRobotY_TextChanged(System::Object^ sender, System::EventArgs^ e) {
	}
	private: System::Void textBoxRobotX_TextChanged(System::Object^ sender, System::EventArgs^ e) {
	}
	private: System::Void textBoxRobotRoll_TextChanged(System::Object^ sender, System::EventArgs^ e) {
	}
	private: System::Void textBoxRobotPitch_TextChanged(System::Object^ sender, System::EventArgs^ e) {
	}
	private: System::Void textBoxDVLVy_TextChanged(System::Object^ sender, System::EventArgs^ e) {
	}
	private: System::Void textBoxDVLVx_TextChanged(System::Object^ sender, System::EventArgs^ e) {
	}
	private: System::Void MainForm_Load(System::Object^ sender, System::EventArgs^ e) {
	}
	private: System::Void textBoxSRTK_TextChanged(System::Object^ sender, System::EventArgs^ e) {
	}


		   /*--------------------------------------------------------------
		   *
		   *
		   *		UTM target
		   *
		   *
		   * --------------------------------------------------------------*/
		   // Send command to the robot. The robot will go at the GPS point written in the text box.
	private: System::Void buttonGoTargetGPS_Click(System::Object^ sender, System::EventArgs^ e) {
		std::string latitude_str = ConvertString2Char(textBoxTargetLatitude->Text);
		double latitude = std::stof(latitude_str);
		std::string longitude_str = ConvertString2Char(textBoxTargetLongitude->Text);
		double longitude = std::stof(longitude_str);
	}

	private: System::Void buttonSetTargetUTM_Click(System::Object^ sender, System::EventArgs^ e) {
		std::string northing_str = ConvertString2Char(this->textBoxTargetUTMNorth->Text);
		double northing = std::stof(northing_str);
		std::string easting_str = ConvertString2Char(this->textBoxTargetUTMEast->Text);
		double easting = std::stof(easting_str);
		std::string cap_str = ConvertString2Char(this->textBoxTargetCap->Text);
		double cap = std::stof(cap_str);
		plateform.set_utm_target(northing, easting, cap);
	}

		   /*--------------------------------------------------------------
		   *
		   *
		   *		Bathymetry display
		   *
		   *
		   * --------------------------------------------------------------*/


		   // Read "SHM", update the robot postition, and display the bathymetry
	private: System::Void backgroundWorkerBathy_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {


		//the new way of bathy
		Geotiff tiff((const char*)"C:/Users/admin/Desktop/sacca_clip.tif"); //saccaFisola_5cm.tif    //arsenale_2022_06_5cm.tiff");

			//plot the window around the GPS position
		float** band = tiff.GetRasterBand(1);
		double* transform = tiff.GetGeoTransform();
		double origin_global_geotiff_x = transform[0];
		double origin_global_geotiff_y = transform[3];
		double pixelWidth = transform[1];
		double pixelHeight = transform[5];





		cv::Mat bathy = cv::Mat::ones(cv::Size(560, 560), CV_8UC3);
		cv::Mat resized_bathy = cv::Mat::ones(cv::Size(400, 400), CV_8UC3);
		unsigned char altitude_color = 0;

		vector<double> bathy_map;


		int total = 0;

		double temp = 0;
		int count = 0;

		double lat_P = 0, long_P = 0;
		double utm_northing_robot = 0, utm_easting_robot = 0;
		double utm_northing_babord = 0, utm_easting_babord = 0;
		double utm_northing_tribord = 0, utm_easting_tribord = 0;
		cv::Point pixel_tribord;
		cv::Point secondPointRectangle;
		cv::Point UTM_northing_robot;


		char* utm_zone = "33";

		bool update_check = true;

		double cap_angle = 0; //en degres
		Mat for_Rotation;

		Mat zoomed_bathy;
		Mat resized_zoomed_bathy;

		double UTM_window_origin_x = 0;
		double UTM_window_origin_y = 0;

		const double width_pool = 7.75; //distance entre les GPS
		const double length_pool = 12.25;

		cv::Scalar gris = cv::Scalar(50, 50, 50); // BGR
		cv::Scalar vert = cv::Scalar(0, 160, 0); // BGR
		cv::Scalar rouge = cv::Scalar(0, 0, 255); // BGR

		double largeur_panier = 2.50;
		double longueur_panier = 2.50;

		double lateral_margin = 1.5; //colision margin
		double avant_margin = 2.;
		double arriere_margin = 1.;
		double largeur_workspace = width_pool - 2 * lateral_margin;
		double longueur_workspace = length_pool - avant_margin - arriere_margin - largeur_panier;



		while (true) {

			try
			{

				plateform.getGeoFromXYPos(robot.pos[0], robot.pos[1], &lat_P, &long_P);
				LLtoUTM((const double)lat_P, (const double)long_P,
					utm_northing_robot, utm_easting_robot,
					utm_zone);
				/*std::cout << "northing . " << utm_northing_robot << " easting . " << utm_easting_robot;*/



				LLtoUTM((const double)plateform.latitude_GPS_Master_babord, (const double)plateform.longitude_GPS_Master_babord,
					utm_northing_babord, utm_easting_babord,
					utm_zone);


				LLtoUTM((const double)plateform.latitude_GPS_Slave_tribord, (const double)plateform.longitude_GPS_Slave_tribord,
					utm_northing_tribord, utm_easting_tribord,
					utm_zone);



				Sleep(250);


				cap_angle = plateform.cap_GPS_babord_vers_tribord * 180 / PI;
				cap_angle = cap_angle - 180;
				double cap_angle_rad = plateform.cap_GPS_babord_vers_tribord - PI;

				//cap_angle_rad = 50*DEG2RAD - PI; //debug
				//cap_angle = cap_angle_rad * RAD2DEG; //debug
				//Triche pour visualiser
				utm_easting_tribord = 293083;
				utm_northing_tribord = 5034918;
				utm_easting_robot = 293083;
				utm_northing_robot = 5034918;




				//New way of bathy 
				std::vector<std::vector<float>> rasterBandData = tiff.readWindowRaster(band, utm_easting_robot, utm_northing_robot, pixelWidth, pixelHeight, origin_global_geotiff_x, origin_global_geotiff_y, UTM_window_origin_x, UTM_window_origin_y);

				Mat img_color = tiff.plotBathy(rasterBandData);

				/*std::cout << img_color.size().width << std::endl;
				std::cout << img_color.size().height << std::endl;*/

				//Plot piscine
				//cap angle dans le sens anti trigo
				pixel_tribord.x = (int)((utm_easting_tribord - UTM_window_origin_x) / pixelWidth);
				pixel_tribord.y = (int)((utm_northing_tribord - UTM_window_origin_y) / pixelHeight); //took the same length and width
				cv::Point pixel_babord = cv::Point(pixel_tribord.x + (int)(sin(cap_angle_rad) * width_pool / pixelWidth), pixel_tribord.y - (int)(cos(cap_angle_rad) * width_pool / pixelWidth));//secondPointRectangle
				cv::Point avant_pixel_tribord = cv::Point(pixel_tribord.x + (int)(cos(cap_angle_rad) * length_pool / pixelWidth), (int)(pixel_tribord.y + sin(cap_angle_rad) * length_pool / pixelWidth));//secondPointRectangle
				cv::Point avant_pixel_babord = cv::Point((int)(pixel_tribord.x + sin(cap_angle_rad) * width_pool / pixelWidth + cos(cap_angle_rad) * length_pool / pixelWidth), (int)(pixel_tribord.y + sin(cap_angle_rad) * length_pool / pixelWidth - cos(cap_angle_rad) * width_pool / pixelWidth));


				line(img_color, pixel_tribord, pixel_babord, gris, (int)(0.2 / pixelWidth), LINE_AA);
				line(img_color, pixel_tribord, avant_pixel_tribord, gris, (int)(0.2 / pixelWidth), LINE_AA);
				line(img_color, avant_pixel_tribord, avant_pixel_babord, gris, (int)(0.2 / pixelWidth), LINE_AA);
				line(img_color, pixel_babord, avant_pixel_babord, gris, (int)(0.2 / pixelWidth), LINE_AA);

				// panier
				cv::Point tribord_panier = cv::Point((int)(pixel_tribord.x + sin(cap_angle_rad) * 3.2 / pixelWidth + cos(cap_angle_rad) * (length_pool - largeur_panier) / pixelWidth), (int)(pixel_tribord.y + sin(cap_angle_rad) * (length_pool - largeur_panier) / pixelWidth - cos(cap_angle_rad) * 3.2 / pixelWidth));
				cv::Point pixel_babord_panier = cv::Point(tribord_panier.x + (int)(sin(cap_angle_rad) * longueur_panier / pixelWidth), tribord_panier.y - (int)(cos(cap_angle_rad) * longueur_panier / pixelWidth));//secondPointRectangle
				cv::Point avant_pixel_tribord_panier = cv::Point(tribord_panier.x + (int)(cos(cap_angle_rad) * largeur_panier / pixelWidth), (int)(tribord_panier.y + sin(cap_angle_rad) * largeur_panier / pixelWidth));//secondPointRectangle
				cv::Point avant_pixel_babord_panier = cv::Point((int)(tribord_panier.x + sin(cap_angle_rad) * longueur_panier / pixelWidth + cos(cap_angle_rad) * largeur_panier / pixelWidth), (int)(tribord_panier.y + sin(cap_angle_rad) * largeur_panier / pixelWidth - cos(cap_angle_rad) * longueur_panier / pixelWidth));

				line(img_color, tribord_panier, pixel_babord_panier, gris, (int)(0.1 / pixelWidth), LINE_AA);
				line(img_color, tribord_panier, avant_pixel_tribord_panier, gris, (int)(0.1 / pixelWidth), LINE_AA);
				line(img_color, avant_pixel_tribord_panier, avant_pixel_babord_panier, gris, (int)(0.1 / pixelWidth), LINE_AA);
				line(img_color, pixel_babord_panier, avant_pixel_babord_panier, gris, (int)(0.1 / pixelWidth), LINE_AA);


				cv::Point tribord_workspace = cv::Point((int)(pixel_tribord.x + sin(cap_angle_rad) * lateral_margin / pixelWidth + cos(cap_angle_rad) * arriere_margin / pixelWidth), (int)(pixel_tribord.y + sin(cap_angle_rad) * arriere_margin / pixelWidth - cos(cap_angle_rad) * lateral_margin / pixelWidth));
				cv::Point pixel_babord_workspace = cv::Point(tribord_workspace.x + (int)(sin(cap_angle_rad) * largeur_workspace / pixelWidth), tribord_workspace.y - (int)(cos(cap_angle_rad) * largeur_workspace / pixelWidth));//secondPointRectangle
				cv::Point avant_pixel_tribord_workspace = cv::Point(tribord_workspace.x + (int)(cos(cap_angle_rad) * longueur_workspace / pixelWidth), (int)(tribord_workspace.y + sin(cap_angle_rad) * longueur_workspace / pixelWidth));//secondPointRectangle
				cv::Point avant_pixel_babord_workspace = cv::Point((int)(tribord_workspace.x + sin(cap_angle_rad) * largeur_workspace / pixelWidth + cos(cap_angle_rad) * longueur_workspace / pixelWidth), (int)(tribord_workspace.y + sin(cap_angle_rad) * longueur_workspace / pixelWidth - cos(cap_angle_rad) * largeur_workspace / pixelWidth));

				line(img_color, tribord_workspace, pixel_babord_workspace, vert, (int)(0.1 / pixelWidth), LINE_AA);
				line(img_color, tribord_workspace, avant_pixel_tribord_workspace, vert, (int)(0.1 / pixelWidth), LINE_AA);
				line(img_color, avant_pixel_tribord_workspace, avant_pixel_babord_workspace, vert, (int)(0.1 / pixelWidth), LINE_AA);
				line(img_color, pixel_babord_workspace, avant_pixel_babord_workspace, vert, (int)(0.1 / pixelWidth), LINE_AA);
				circle(img_color, pixel_tribord, 5, vert, (int)(0.1 / pixelWidth), LINE_AA);
				circle(img_color, pixel_babord, 5, rouge, (int)(0.1 / pixelWidth), LINE_AA);
				/*cv::imshow("Test", img_color);
				cv::waitKey(0);*/

				cv::Point centre_image_bathy = cv::Point(img_color.size().width / 2, img_color.size().height / 2);

				for_Rotation = getRotationMatrix2D(centre_image_bathy, (cap_angle), 1); //cap angle dans le sens trigo
				Mat rotated_image_color;


				//max(abs(cos(x)+sin(x)),abs( -cos(x)+sin(x)))
			/*	double scale_factor= max(abs(cos(cap_angle_rad) + sin(cap_angle_rad)), abs(-cos(cap_angle_rad) + sin(cap_angle_rad)));
				cv::Size rotated_image_size = cv::Size((int)(img_color.size().width * scale_factor), (int)(img_color.size().height * scale_factor));*/
				warpAffine(img_color, rotated_image_color, for_Rotation, img_color.size());//applying affine transformation//
				//cv::imshow("Test2", rotated_image_color);

				Mat cropped_image_bathy = rotated_image_color(Range(centre_image_bathy.x - (int)(14 / pixelWidth), centre_image_bathy.x + (int)(14 / pixelWidth)), Range(centre_image_bathy.y - (int)(14 / pixelWidth), centre_image_bathy.y + (int)(14 / pixelWidth)));
				Mat zoomed_bathy = rotated_image_color(Range(centre_image_bathy.x - (int)(3 / pixelWidth), centre_image_bathy.x + (int)(3 / pixelWidth)), Range(centre_image_bathy.y - (int)(3 / pixelWidth), centre_image_bathy.y + (int)(3 / pixelWidth)));
				/*cv::imshow("Test3", cropped_image_bathy);
				cv::waitKey(0);*/

				//scale_factor_resized_map = 28/560; //meter size/pixel size

				cv::resize(cropped_image_bathy, resized_bathy, cv::Size(560, 560));


				//zoomed_bathy = resized_bathy(cv::Rect(220, 220, 120, 120));

				cv::resize(zoomed_bathy, resized_zoomed_bathy, cv::Size(300, 300));

				cv::line(resized_zoomed_bathy, cv::Point(0, 150), cv::Point(300, 150), cv::Scalar(0, 0, 255), 1);
				cv::line(resized_zoomed_bathy, cv::Point(150, 0), cv::Point(150, 300), cv::Scalar(0, 0, 255), 1);

				for (size_t i = 0; i < 300; i += 50)
				{
					cv::line(resized_zoomed_bathy, cv::Point(i, 150 - 5), cv::Point(i, 150 + 5), cv::Scalar(0, 0, 255), 1);
					cv::line(resized_zoomed_bathy, cv::Point(150 - 5, i), cv::Point(150 + 5, i), cv::Scalar(0, 0, 255), 1);
				}

				cv::line(resized_bathy, cv::Point(0, 280), cv::Point(560, 280), cv::Scalar(0, 0, 255), 1);
				cv::line(resized_bathy, cv::Point(280, 0), cv::Point(280, 560), cv::Scalar(0, 0, 255), 1);

				/*cv::line(resized_bathy, cv::Point(0, 280), cv::Point(560, 280), cv::Scalar(0, 0, 255), 1);
				cv::line(resized_bathy, cv::Point(280, 0), cv::Point(280, 560), cv::Scalar(0, 0, 255), 1);*/

				for (size_t i = 0; i < 560; i += 20)
				{
					cv::line(resized_bathy, cv::Point(i, 280 - 2), cv::Point(i, 280 + 2), cv::Scalar(0, 0, 255), 1);
					cv::line(resized_bathy, cv::Point(280 - 2, i), cv::Point(280 + 2, i), cv::Scalar(0, 0, 255), 1);
				}




				pictureBoxBathy->Image = ConvertMat2Bitmap(resized_bathy);

				pictureBoxZoomedBathy->Image = ConvertMat2Bitmap(resized_zoomed_bathy);

				if (update_check) {
					this->buttonBathyOnline->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)),
						static_cast<System::Int32>(static_cast<System::Byte>(0)));
					update_check = !update_check;
				}
				else {
					this->buttonBathyOnline->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(192)), static_cast<System::Int32>(static_cast<System::Byte>(192)),
						static_cast<System::Int32>(static_cast<System::Byte>(192)));
					update_check = !update_check;
				}


				Sleep(250);
			}
			catch (const std::exception&)
			{
				cout << "Error bathy" << endl;
				Sleep(150);
			}
		}
	}

		   float scaleTension(float tension) {
			   tension = 1000 - tension;
			   return (tension - 0) / (1000 - 0) * (200 - 0) + 0;
		   }

		   /*--------------------------------------------------------------
		   *
		   *
		   *		TODO: Cabble tension display (need to receive appropiate frame from P-E)
		   *
		   *
		   * --------------------------------------------------------------*/

	private: System::Void backgroundWorkerCableTension_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {

		Mat bar_m1 = Mat::zeros(cv::Size(40, 200), CV_8UC3);
		Mat bar_m2 = Mat::zeros(cv::Size(40, 200), CV_8UC3);
		Mat bar_m3 = Mat::zeros(cv::Size(40, 200), CV_8UC3);
		Mat bar_m4 = Mat::zeros(cv::Size(40, 200), CV_8UC3);
		Mat bar_m5 = Mat::zeros(cv::Size(40, 200), CV_8UC3);
		Mat bar_m6 = Mat::zeros(cv::Size(40, 200), CV_8UC3);
		Mat bar_m7 = Mat::zeros(cv::Size(40, 200), CV_8UC3);
		Mat bar_m8 = Mat::zeros(cv::Size(40, 200), CV_8UC3);

		float m1 = 231, m2 = 754, m3 = 790, m4 = 120, m5 = 322, m6 = 72, m7 = 167, m8 = 356;

		while (true) {

			bar_m1.setTo(cv::Scalar(180, 180, 180));
			bar_m2.setTo(cv::Scalar(180, 180, 180));
			bar_m3.setTo(cv::Scalar(180, 180, 180));
			bar_m4.setTo(cv::Scalar(180, 180, 180));
			bar_m5.setTo(cv::Scalar(180, 180, 180));
			bar_m6.setTo(cv::Scalar(180, 180, 180));
			bar_m7.setTo(cv::Scalar(180, 180, 180));
			bar_m8.setTo(cv::Scalar(180, 180, 180));

			textBoxM1->Text = "" + m1;
			textBoxM2->Text = "" + m2;
			textBoxM3->Text = "" + m3;
			textBoxM4->Text = "" + m4;
			textBoxM5->Text = "" + m5;
			textBoxM6->Text = "" + m6;
			textBoxM7->Text = "" + m7;
			textBoxM8->Text = "" + m8;

			cv::rectangle(bar_m1, cv::Point(0, scaleTension(m1)), cv::Point(40, 220), cv::Scalar(255, 50, 0), -1);
			cv::rectangle(bar_m2, cv::Point(0, scaleTension(m2)), cv::Point(40, 220), cv::Scalar(255, 50, 0), -1);
			cv::rectangle(bar_m3, cv::Point(0, scaleTension(m3)), cv::Point(40, 220), cv::Scalar(255, 50, 0), -1);
			cv::rectangle(bar_m4, cv::Point(0, scaleTension(m4)), cv::Point(40, 220), cv::Scalar(255, 50, 0), -1);
			cv::rectangle(bar_m5, cv::Point(0, scaleTension(m5)), cv::Point(40, 220), cv::Scalar(255, 50, 0), -1);
			cv::rectangle(bar_m6, cv::Point(0, scaleTension(m6)), cv::Point(40, 220), cv::Scalar(255, 50, 0), -1);
			cv::rectangle(bar_m7, cv::Point(0, scaleTension(m7)), cv::Point(40, 220), cv::Scalar(255, 50, 0), -1);
			cv::rectangle(bar_m8, cv::Point(0, scaleTension(m8)), cv::Point(40, 220), cv::Scalar(255, 50, 0), -1);

			pictureBoxM1->Image = ConvertMat2Bitmap(bar_m1);
			pictureBoxM2->Image = ConvertMat2Bitmap(bar_m2);
			pictureBoxM3->Image = ConvertMat2Bitmap(bar_m3);
			pictureBoxM4->Image = ConvertMat2Bitmap(bar_m4);
			pictureBoxM5->Image = ConvertMat2Bitmap(bar_m5);
			pictureBoxM6->Image = ConvertMat2Bitmap(bar_m6);
			pictureBoxM7->Image = ConvertMat2Bitmap(bar_m7);
			pictureBoxM8->Image = ConvertMat2Bitmap(bar_m8);

			Sleep(250);
		}
	}

	private: System::Void button1_Click(System::Object^ sender, System::EventArgs^ e) {

		bathymetry.set_size(50);

	}

	private: System::Void textBoxPRoll_TextChanged(System::Object^ sender, System::EventArgs^ e) {
	}
	};
}
