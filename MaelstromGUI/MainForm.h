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

#include "config.h"

#include "PracticalSocket.h"


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

	// Bboxes initialization
	Mat null_img = Mat::zeros(cv::Size(1, 1), CV_8UC1);
	vector<Bbx> null_bbx_vector;
	Bboxes bboxes(null_img, null_bbx_vector);

	// Mouse position for zooming
	cv::Point mouse_pos = cv::Point(0, 0);
	bool mouse_left_down = false;

	// Related images in the images list
	vector< cv::String > realted_img_paths;

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

			// Start IMU starboard reader
			system(IMU_S_READER_START);

			// Start IMU port reader
			system(IMU_P_READER_START);

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
private: System::Windows::Forms::TextBox^ textBoxPRTK;
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
			this->textBoxPRTK = (gcnew System::Windows::Forms::TextBox());
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
			this->label28 = (gcnew System::Windows::Forms::Label());
			this->label29 = (gcnew System::Windows::Forms::Label());
			this->label30 = (gcnew System::Windows::Forms::Label());
			this->label25 = (gcnew System::Windows::Forms::Label());
			this->label26 = (gcnew System::Windows::Forms::Label());
			this->label27 = (gcnew System::Windows::Forms::Label());
			this->groupBoxStarboard = (gcnew System::Windows::Forms::GroupBox());
			this->label34 = (gcnew System::Windows::Forms::Label());
			this->label35 = (gcnew System::Windows::Forms::Label());
			this->label31 = (gcnew System::Windows::Forms::Label());
			this->label36 = (gcnew System::Windows::Forms::Label());
			this->label32 = (gcnew System::Windows::Forms::Label());
			this->label33 = (gcnew System::Windows::Forms::Label());
			this->pictureBoxLogoLirmm = (gcnew System::Windows::Forms::PictureBox());
			this->pictureBoxLogoUM = (gcnew System::Windows::Forms::PictureBox());
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
			this->SuspendLayout();
			// 
			// button_Edition
			// 
			this->button_Edition->Enabled = false;
			this->button_Edition->Location = System::Drawing::Point(1393, 29);
			this->button_Edition->Name = L"button_Edition";
			this->button_Edition->Size = System::Drawing::Size(75, 23);
			this->button_Edition->TabIndex = 35;
			this->button_Edition->Text = L"Edition";
			// 
			// ptbSource
			// 
			this->ptbSource->BackColor = System::Drawing::SystemColors::ControlDark;
			this->ptbSource->Location = System::Drawing::Point(827, 58);
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
			this->button_Browse->Location = System::Drawing::Point(1319, 28);
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
			this->play_button->Location = System::Drawing::Point(913, 588);
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
			this->speed_button->Location = System::Drawing::Point(999, 588);
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
			this->load_button->Location = System::Drawing::Point(827, 588);
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
			this->video_trackBar->Location = System::Drawing::Point(917, 544);
			this->video_trackBar->Maximum = 150;
			this->video_trackBar->Name = L"video_trackBar";
			this->video_trackBar->Size = System::Drawing::Size(521, 45);
			this->video_trackBar->TabIndex = 8;
			this->video_trackBar->Scroll += gcnew System::EventHandler(this, &MainForm::video_trackBar_Scroll);
			// 
			// video_label
			// 
			this->video_label->AutoSize = true;
			this->video_label->Location = System::Drawing::Point(1444, 549);
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
			this->record_button->Location = System::Drawing::Point(827, 548);
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
			this->buttonDVLyes->Location = System::Drawing::Point(209, 25);
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
			this->buttonDVLno->Location = System::Drawing::Point(275, 25);
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
			this->labelDVLon->Location = System::Drawing::Point(192, 9);
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
			this->label3dY->Location = System::Drawing::Point(879, 26);
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
			this->label3dX->Location = System::Drawing::Point(829, 26);
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
			this->label3dZ->Location = System::Drawing::Point(929, 26);
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
			this->labelDVL->Location = System::Drawing::Point(789, 42);
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
			this->label3dZFake->Location = System::Drawing::Point(929, 42);
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
			this->label3dYFake->Location = System::Drawing::Point(879, 42);
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
			this->label3dXFake->Location = System::Drawing::Point(829, 42);
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
			this->labelLine->Location = System::Drawing::Point(996, 29);
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
			this->labelLineFake->Location = System::Drawing::Point(996, 42);
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
			this->ptbDepthMap->MouseHover += gcnew System::EventHandler(this, &MainForm::ptbDepthMap_MouseHover);
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
			this->textBoxTide->Text = L"0";
			this->textBoxTide->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxRobotX
			// 
			this->textBoxRobotX->Location = System::Drawing::Point(6, 63);
			this->textBoxRobotX->Name = L"textBoxRobotX";
			this->textBoxRobotX->ReadOnly = true;
			this->textBoxRobotX->Size = System::Drawing::Size(47, 20);
			this->textBoxRobotX->TabIndex = 68;
			this->textBoxRobotX->Text = L"0";
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
			this->textBoxRobotY->Text = L"0";
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
			this->textBoxRobotZ->Text = L"0";
			this->textBoxRobotZ->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBoxRobotZ->TextChanged += gcnew System::EventHandler(this, &MainForm::textBoxRobotZ_TextChanged);
			// 
			// textBoxRobotState
			// 
			this->textBoxRobotState->Location = System::Drawing::Point(41, 24);
			this->textBoxRobotState->Name = L"textBoxRobotState";
			this->textBoxRobotState->ReadOnly = true;
			this->textBoxRobotState->Size = System::Drawing::Size(100, 20);
			this->textBoxRobotState->TabIndex = 71;
			this->textBoxRobotState->Text = L"0";
			this->textBoxRobotState->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxRobotYaw
			// 
			this->textBoxRobotYaw->Location = System::Drawing::Point(112, 104);
			this->textBoxRobotYaw->Name = L"textBoxRobotYaw";
			this->textBoxRobotYaw->ReadOnly = true;
			this->textBoxRobotYaw->Size = System::Drawing::Size(47, 20);
			this->textBoxRobotYaw->TabIndex = 74;
			this->textBoxRobotYaw->Text = L"0";
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
			this->textBoxRobotPitch->Text = L"0";
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
			this->textBoxRobotRoll->Text = L"0";
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
			this->textBoxGripper->Text = L"0";
			this->textBoxGripper->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPump
			// 
			this->textBoxPump->Location = System::Drawing::Point(260, 104);
			this->textBoxPump->Name = L"textBoxPump";
			this->textBoxPump->ReadOnly = true;
			this->textBoxPump->Size = System::Drawing::Size(47, 20);
			this->textBoxPump->TabIndex = 76;
			this->textBoxPump->Text = L"0";
			this->textBoxPump->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxArduinoYaw
			// 
			this->textBoxArduinoYaw->Location = System::Drawing::Point(112, 41);
			this->textBoxArduinoYaw->Name = L"textBoxArduinoYaw";
			this->textBoxArduinoYaw->ReadOnly = true;
			this->textBoxArduinoYaw->Size = System::Drawing::Size(47, 20);
			this->textBoxArduinoYaw->TabIndex = 79;
			this->textBoxArduinoYaw->Text = L"0";
			this->textBoxArduinoYaw->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxArduinoPitch
			// 
			this->textBoxArduinoPitch->Location = System::Drawing::Point(59, 41);
			this->textBoxArduinoPitch->Name = L"textBoxArduinoPitch";
			this->textBoxArduinoPitch->ReadOnly = true;
			this->textBoxArduinoPitch->Size = System::Drawing::Size(47, 20);
			this->textBoxArduinoPitch->TabIndex = 78;
			this->textBoxArduinoPitch->Text = L"0";
			this->textBoxArduinoPitch->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxArduinoRoll
			// 
			this->textBoxArduinoRoll->Location = System::Drawing::Point(6, 41);
			this->textBoxArduinoRoll->Name = L"textBoxArduinoRoll";
			this->textBoxArduinoRoll->ReadOnly = true;
			this->textBoxArduinoRoll->Size = System::Drawing::Size(47, 20);
			this->textBoxArduinoRoll->TabIndex = 77;
			this->textBoxArduinoRoll->Text = L"0";
			this->textBoxArduinoRoll->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxArduinoDepth
			// 
			this->textBoxArduinoDepth->Location = System::Drawing::Point(172, 41);
			this->textBoxArduinoDepth->Name = L"textBoxArduinoDepth";
			this->textBoxArduinoDepth->ReadOnly = true;
			this->textBoxArduinoDepth->Size = System::Drawing::Size(47, 20);
			this->textBoxArduinoDepth->TabIndex = 80;
			this->textBoxArduinoDepth->Text = L"0";
			this->textBoxArduinoDepth->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxAtmPressure
			// 
			this->textBoxAtmPressure->Location = System::Drawing::Point(281, 41);
			this->textBoxAtmPressure->Name = L"textBoxAtmPressure";
			this->textBoxAtmPressure->ReadOnly = true;
			this->textBoxAtmPressure->Size = System::Drawing::Size(47, 20);
			this->textBoxAtmPressure->TabIndex = 81;
			this->textBoxAtmPressure->Text = L"0";
			this->textBoxAtmPressure->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxDVLVy
			// 
			this->textBoxDVLVy->Location = System::Drawing::Point(278, 40);
			this->textBoxDVLVy->Name = L"textBoxDVLVy";
			this->textBoxDVLVy->ReadOnly = true;
			this->textBoxDVLVy->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLVy->TabIndex = 86;
			this->textBoxDVLVy->Text = L"0";
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
			this->textBoxDVLVx->Text = L"0";
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
			this->textBoxDVLYaw->Text = L"0";
			this->textBoxDVLYaw->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxDVLPitch
			// 
			this->textBoxDVLPitch->Location = System::Drawing::Point(59, 40);
			this->textBoxDVLPitch->Name = L"textBoxDVLPitch";
			this->textBoxDVLPitch->ReadOnly = true;
			this->textBoxDVLPitch->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLPitch->TabIndex = 88;
			this->textBoxDVLPitch->Text = L"0";
			this->textBoxDVLPitch->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxDVLRoll
			// 
			this->textBoxDVLRoll->Location = System::Drawing::Point(6, 40);
			this->textBoxDVLRoll->Name = L"textBoxDVLRoll";
			this->textBoxDVLRoll->ReadOnly = true;
			this->textBoxDVLRoll->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLRoll->TabIndex = 87;
			this->textBoxDVLRoll->Text = L"0";
			this->textBoxDVLRoll->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxDVLD2
			// 
			this->textBoxDVLD2->Location = System::Drawing::Point(112, 87);
			this->textBoxDVLD2->Name = L"textBoxDVLD2";
			this->textBoxDVLD2->ReadOnly = true;
			this->textBoxDVLD2->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLD2->TabIndex = 92;
			this->textBoxDVLD2->Text = L"0";
			this->textBoxDVLD2->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxDVLD1
			// 
			this->textBoxDVLD1->Location = System::Drawing::Point(59, 87);
			this->textBoxDVLD1->Name = L"textBoxDVLD1";
			this->textBoxDVLD1->ReadOnly = true;
			this->textBoxDVLD1->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLD1->TabIndex = 91;
			this->textBoxDVLD1->Text = L"0";
			this->textBoxDVLD1->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxDVLD0
			// 
			this->textBoxDVLD0->Location = System::Drawing::Point(6, 87);
			this->textBoxDVLD0->Name = L"textBoxDVLD0";
			this->textBoxDVLD0->ReadOnly = true;
			this->textBoxDVLD0->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLD0->TabIndex = 90;
			this->textBoxDVLD0->Text = L"0";
			this->textBoxDVLD0->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxDVLD3
			// 
			this->textBoxDVLD3->Location = System::Drawing::Point(165, 87);
			this->textBoxDVLD3->Name = L"textBoxDVLD3";
			this->textBoxDVLD3->ReadOnly = true;
			this->textBoxDVLD3->Size = System::Drawing::Size(47, 20);
			this->textBoxDVLD3->TabIndex = 93;
			this->textBoxDVLD3->Text = L"0";
			this->textBoxDVLD3->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPYaw
			// 
			this->textBoxPYaw->Location = System::Drawing::Point(112, 39);
			this->textBoxPYaw->Name = L"textBoxPYaw";
			this->textBoxPYaw->ReadOnly = true;
			this->textBoxPYaw->Size = System::Drawing::Size(47, 20);
			this->textBoxPYaw->TabIndex = 96;
			this->textBoxPYaw->Text = L"0";
			this->textBoxPYaw->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPPitch
			// 
			this->textBoxPPitch->Location = System::Drawing::Point(59, 39);
			this->textBoxPPitch->Name = L"textBoxPPitch";
			this->textBoxPPitch->ReadOnly = true;
			this->textBoxPPitch->Size = System::Drawing::Size(47, 20);
			this->textBoxPPitch->TabIndex = 95;
			this->textBoxPPitch->Text = L"0";
			this->textBoxPPitch->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPRoll
			// 
			this->textBoxPRoll->Location = System::Drawing::Point(6, 39);
			this->textBoxPRoll->Name = L"textBoxPRoll";
			this->textBoxPRoll->ReadOnly = true;
			this->textBoxPRoll->Size = System::Drawing::Size(47, 20);
			this->textBoxPRoll->TabIndex = 94;
			this->textBoxPRoll->Text = L"0";
			this->textBoxPRoll->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPGyrZ
			// 
			this->textBoxPGyrZ->Location = System::Drawing::Point(112, 87);
			this->textBoxPGyrZ->Name = L"textBoxPGyrZ";
			this->textBoxPGyrZ->ReadOnly = true;
			this->textBoxPGyrZ->Size = System::Drawing::Size(47, 20);
			this->textBoxPGyrZ->TabIndex = 99;
			this->textBoxPGyrZ->Text = L"0";
			this->textBoxPGyrZ->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPGyrY
			// 
			this->textBoxPGyrY->Location = System::Drawing::Point(59, 87);
			this->textBoxPGyrY->Name = L"textBoxPGyrY";
			this->textBoxPGyrY->ReadOnly = true;
			this->textBoxPGyrY->Size = System::Drawing::Size(47, 20);
			this->textBoxPGyrY->TabIndex = 98;
			this->textBoxPGyrY->Text = L"0";
			this->textBoxPGyrY->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPGyrX
			// 
			this->textBoxPGyrX->Location = System::Drawing::Point(6, 87);
			this->textBoxPGyrX->Name = L"textBoxPGyrX";
			this->textBoxPGyrX->ReadOnly = true;
			this->textBoxPGyrX->Size = System::Drawing::Size(47, 20);
			this->textBoxPGyrX->TabIndex = 97;
			this->textBoxPGyrX->Text = L"0";
			this->textBoxPGyrX->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSYaw
			// 
			this->textBoxSYaw->Location = System::Drawing::Point(112, 39);
			this->textBoxSYaw->Name = L"textBoxSYaw";
			this->textBoxSYaw->ReadOnly = true;
			this->textBoxSYaw->Size = System::Drawing::Size(47, 20);
			this->textBoxSYaw->TabIndex = 102;
			this->textBoxSYaw->Text = L"0";
			this->textBoxSYaw->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSPitch
			// 
			this->textBoxSPitch->Location = System::Drawing::Point(59, 39);
			this->textBoxSPitch->Name = L"textBoxSPitch";
			this->textBoxSPitch->ReadOnly = true;
			this->textBoxSPitch->Size = System::Drawing::Size(47, 20);
			this->textBoxSPitch->TabIndex = 101;
			this->textBoxSPitch->Text = L"0";
			this->textBoxSPitch->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSRoll
			// 
			this->textBoxSRoll->Location = System::Drawing::Point(6, 39);
			this->textBoxSRoll->Name = L"textBoxSRoll";
			this->textBoxSRoll->ReadOnly = true;
			this->textBoxSRoll->Size = System::Drawing::Size(47, 20);
			this->textBoxSRoll->TabIndex = 100;
			this->textBoxSRoll->Text = L"0";
			this->textBoxSRoll->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSGyrZ
			// 
			this->textBoxSGyrZ->Location = System::Drawing::Point(112, 87);
			this->textBoxSGyrZ->Name = L"textBoxSGyrZ";
			this->textBoxSGyrZ->ReadOnly = true;
			this->textBoxSGyrZ->Size = System::Drawing::Size(47, 20);
			this->textBoxSGyrZ->TabIndex = 105;
			this->textBoxSGyrZ->Text = L"0";
			this->textBoxSGyrZ->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSGyrY
			// 
			this->textBoxSGyrY->Location = System::Drawing::Point(59, 87);
			this->textBoxSGyrY->Name = L"textBoxSGyrY";
			this->textBoxSGyrY->ReadOnly = true;
			this->textBoxSGyrY->Size = System::Drawing::Size(47, 20);
			this->textBoxSGyrY->TabIndex = 104;
			this->textBoxSGyrY->Text = L"0";
			this->textBoxSGyrY->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSGyrX
			// 
			this->textBoxSGyrX->Location = System::Drawing::Point(6, 87);
			this->textBoxSGyrX->Name = L"textBoxSGyrX";
			this->textBoxSGyrX->ReadOnly = true;
			this->textBoxSGyrX->Size = System::Drawing::Size(47, 20);
			this->textBoxSGyrX->TabIndex = 103;
			this->textBoxSGyrX->Text = L"0";
			this->textBoxSGyrX->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSGPSY
			// 
			this->textBoxSGPSY->Location = System::Drawing::Point(59, 134);
			this->textBoxSGPSY->Name = L"textBoxSGPSY";
			this->textBoxSGPSY->ReadOnly = true;
			this->textBoxSGPSY->Size = System::Drawing::Size(47, 20);
			this->textBoxSGPSY->TabIndex = 107;
			this->textBoxSGPSY->Text = L"0";
			this->textBoxSGPSY->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSGPSX
			// 
			this->textBoxSGPSX->Location = System::Drawing::Point(6, 134);
			this->textBoxSGPSX->Name = L"textBoxSGPSX";
			this->textBoxSGPSX->ReadOnly = true;
			this->textBoxSGPSX->Size = System::Drawing::Size(47, 20);
			this->textBoxSGPSX->TabIndex = 106;
			this->textBoxSGPSX->Text = L"0";
			this->textBoxSGPSX->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPGPSY
			// 
			this->textBoxPGPSY->Location = System::Drawing::Point(58, 134);
			this->textBoxPGPSY->Name = L"textBoxPGPSY";
			this->textBoxPGPSY->ReadOnly = true;
			this->textBoxPGPSY->Size = System::Drawing::Size(47, 20);
			this->textBoxPGPSY->TabIndex = 110;
			this->textBoxPGPSY->Text = L"0";
			this->textBoxPGPSY->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxPGPSX
			// 
			this->textBoxPGPSX->Location = System::Drawing::Point(5, 134);
			this->textBoxPGPSX->Name = L"textBoxPGPSX";
			this->textBoxPGPSX->ReadOnly = true;
			this->textBoxPGPSX->Size = System::Drawing::Size(47, 20);
			this->textBoxPGPSX->TabIndex = 109;
			this->textBoxPGPSX->Text = L"0";
			this->textBoxPGPSX->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// labelGPSP
			// 
			this->labelGPSP->AutoSize = true;
			this->labelGPSP->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->labelGPSP->Location = System::Drawing::Point(3, 118);
			this->labelGPSP->Name = L"labelGPSP";
			this->labelGPSP->Size = System::Drawing::Size(29, 13);
			this->labelGPSP->TabIndex = 111;
			this->labelGPSP->Text = L"GPS";
			this->labelGPSP->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// labelGPSS
			// 
			this->labelGPSS->AutoSize = true;
			this->labelGPSS->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->labelGPSS->Location = System::Drawing::Point(3, 118);
			this->labelGPSS->Name = L"labelGPSS";
			this->labelGPSS->Size = System::Drawing::Size(29, 13);
			this->labelGPSS->TabIndex = 112;
			this->labelGPSS->Text = L"GPS";
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
			this->groupBoxRobot->Location = System::Drawing::Point(397, 58);
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
			this->label7->Size = System::Drawing::Size(28, 13);
			this->label7->TabIndex = 121;
			this->label7->Text = L"Yaw";
			this->label7->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label6->Location = System::Drawing::Point(56, 88);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(31, 13);
			this->label6->TabIndex = 120;
			this->label6->Text = L"Pitch";
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
			// textBoxPRTK
			// 
			this->textBoxPRTK->Location = System::Drawing::Point(111, 134);
			this->textBoxPRTK->Name = L"textBoxPRTK";
			this->textBoxPRTK->ReadOnly = true;
			this->textBoxPRTK->Size = System::Drawing::Size(47, 20);
			this->textBoxPRTK->TabIndex = 114;
			this->textBoxPRTK->Text = L"0";
			this->textBoxPRTK->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// textBoxSRTK
			// 
			this->textBoxSRTK->Location = System::Drawing::Point(112, 134);
			this->textBoxSRTK->Name = L"textBoxSRTK";
			this->textBoxSRTK->ReadOnly = true;
			this->textBoxSRTK->Size = System::Drawing::Size(47, 20);
			this->textBoxSRTK->TabIndex = 115;
			this->textBoxSRTK->Text = L"0";
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
			this->groupBox1->Location = System::Drawing::Point(397, 192);
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
			this->label11->Size = System::Drawing::Size(55, 13);
			this->label11->TabIndex = 124;
			this->label11->Text = L"Yaw (deg)";
			this->label11->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label12
			// 
			this->label12->AutoSize = true;
			this->label12->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label12->Location = System::Drawing::Point(56, 25);
			this->label12->Name = L"label12";
			this->label12->Size = System::Drawing::Size(58, 13);
			this->label12->TabIndex = 123;
			this->label12->Text = L"Pitch (deg)";
			this->label12->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label13
			// 
			this->label13->AutoSize = true;
			this->label13->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label13->Location = System::Drawing::Point(3, 25);
			this->label13->Name = L"label13";
			this->label13->Size = System::Drawing::Size(52, 13);
			this->label13->TabIndex = 122;
			this->label13->Text = L"Roll (deg)";
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
			this->groupBoxDVL->Location = System::Drawing::Point(397, 265);
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
			this->label24->Size = System::Drawing::Size(53, 13);
			this->label24->TabIndex = 132;
			this->label24->Text = L"Vx (m.s-1)";
			this->label24->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label22
			// 
			this->label22->AutoSize = true;
			this->label22->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label22->Location = System::Drawing::Point(162, 71);
			this->label22->Name = L"label22";
			this->label22->Size = System::Drawing::Size(35, 13);
			this->label22->TabIndex = 131;
			this->label22->Text = L"D4(m)";
			this->label22->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label19
			// 
			this->label19->AutoSize = true;
			this->label19->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label19->Location = System::Drawing::Point(109, 71);
			this->label19->Name = L"label19";
			this->label19->Size = System::Drawing::Size(35, 13);
			this->label19->TabIndex = 130;
			this->label19->Text = L"D3(m)";
			this->label19->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label20
			// 
			this->label20->AutoSize = true;
			this->label20->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label20->Location = System::Drawing::Point(56, 71);
			this->label20->Name = L"label20";
			this->label20->Size = System::Drawing::Size(35, 13);
			this->label20->TabIndex = 129;
			this->label20->Text = L"D1(m)";
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
			this->label16->Size = System::Drawing::Size(55, 13);
			this->label16->TabIndex = 127;
			this->label16->Text = L"Yaw (deg)";
			this->label16->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label17
			// 
			this->label17->AutoSize = true;
			this->label17->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label17->Location = System::Drawing::Point(56, 24);
			this->label17->Name = L"label17";
			this->label17->Size = System::Drawing::Size(58, 13);
			this->label17->TabIndex = 126;
			this->label17->Text = L"Pitch (deg)";
			this->label17->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label18
			// 
			this->label18->AutoSize = true;
			this->label18->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label18->Location = System::Drawing::Point(3, 24);
			this->label18->Name = L"label18";
			this->label18->Size = System::Drawing::Size(52, 13);
			this->label18->TabIndex = 125;
			this->label18->Text = L"Roll (deg)";
			this->label18->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// groupBoxPort
			// 
			this->groupBoxPort->BackColor = System::Drawing::Color::LightGray;
			this->groupBoxPort->Controls->Add(this->label28);
			this->groupBoxPort->Controls->Add(this->label29);
			this->groupBoxPort->Controls->Add(this->label30);
			this->groupBoxPort->Controls->Add(this->label25);
			this->groupBoxPort->Controls->Add(this->textBoxPPitch);
			this->groupBoxPort->Controls->Add(this->label26);
			this->groupBoxPort->Controls->Add(this->textBoxPRTK);
			this->groupBoxPort->Controls->Add(this->label27);
			this->groupBoxPort->Controls->Add(this->textBoxPRoll);
			this->groupBoxPort->Controls->Add(this->textBoxPYaw);
			this->groupBoxPort->Controls->Add(this->labelGPSP);
			this->groupBoxPort->Controls->Add(this->textBoxPGPSY);
			this->groupBoxPort->Controls->Add(this->textBoxPGyrX);
			this->groupBoxPort->Controls->Add(this->textBoxPGPSX);
			this->groupBoxPort->Controls->Add(this->textBoxPGyrY);
			this->groupBoxPort->Controls->Add(this->textBoxPGyrZ);
			this->groupBoxPort->Location = System::Drawing::Point(397, 384);
			this->groupBoxPort->Name = L"groupBoxPort";
			this->groupBoxPort->Size = System::Drawing::Size(182, 170);
			this->groupBoxPort->TabIndex = 119;
			this->groupBoxPort->TabStop = false;
			this->groupBoxPort->Text = L"Plateform Port";
			// 
			// label28
			// 
			this->label28->AutoSize = true;
			this->label28->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label28->Location = System::Drawing::Point(111, 71);
			this->label28->Name = L"label28";
			this->label28->Size = System::Drawing::Size(19, 13);
			this->label28->TabIndex = 130;
			this->label28->Text = L"Vz";
			this->label28->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label29
			// 
			this->label29->AutoSize = true;
			this->label29->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label29->Location = System::Drawing::Point(58, 71);
			this->label29->Name = L"label29";
			this->label29->Size = System::Drawing::Size(19, 13);
			this->label29->TabIndex = 129;
			this->label29->Text = L"Vy";
			this->label29->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label30
			// 
			this->label30->AutoSize = true;
			this->label30->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label30->Location = System::Drawing::Point(5, 71);
			this->label30->Name = L"label30";
			this->label30->Size = System::Drawing::Size(19, 13);
			this->label30->TabIndex = 128;
			this->label30->Text = L"Vx";
			this->label30->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label25
			// 
			this->label25->AutoSize = true;
			this->label25->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label25->Location = System::Drawing::Point(109, 23);
			this->label25->Name = L"label25";
			this->label25->Size = System::Drawing::Size(28, 13);
			this->label25->TabIndex = 127;
			this->label25->Text = L"Yaw";
			this->label25->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label26
			// 
			this->label26->AutoSize = true;
			this->label26->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label26->Location = System::Drawing::Point(56, 23);
			this->label26->Name = L"label26";
			this->label26->Size = System::Drawing::Size(31, 13);
			this->label26->TabIndex = 126;
			this->label26->Text = L"Pitch";
			this->label26->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label27
			// 
			this->label27->AutoSize = true;
			this->label27->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label27->Location = System::Drawing::Point(3, 23);
			this->label27->Name = L"label27";
			this->label27->Size = System::Drawing::Size(25, 13);
			this->label27->TabIndex = 125;
			this->label27->Text = L"Roll";
			this->label27->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// groupBoxStarboard
			// 
			this->groupBoxStarboard->BackColor = System::Drawing::Color::LightGray;
			this->groupBoxStarboard->Controls->Add(this->label34);
			this->groupBoxStarboard->Controls->Add(this->label35);
			this->groupBoxStarboard->Controls->Add(this->label31);
			this->groupBoxStarboard->Controls->Add(this->label36);
			this->groupBoxStarboard->Controls->Add(this->label32);
			this->groupBoxStarboard->Controls->Add(this->textBoxSRTK);
			this->groupBoxStarboard->Controls->Add(this->label33);
			this->groupBoxStarboard->Controls->Add(this->labelGPSS);
			this->groupBoxStarboard->Controls->Add(this->textBoxSGPSY);
			this->groupBoxStarboard->Controls->Add(this->textBoxSRoll);
			this->groupBoxStarboard->Controls->Add(this->textBoxSGPSX);
			this->groupBoxStarboard->Controls->Add(this->textBoxSPitch);
			this->groupBoxStarboard->Controls->Add(this->textBoxSYaw);
			this->groupBoxStarboard->Controls->Add(this->textBoxSGyrZ);
			this->groupBoxStarboard->Controls->Add(this->textBoxSGyrX);
			this->groupBoxStarboard->Controls->Add(this->textBoxSGyrY);
			this->groupBoxStarboard->Location = System::Drawing::Point(585, 384);
			this->groupBoxStarboard->Name = L"groupBoxStarboard";
			this->groupBoxStarboard->Size = System::Drawing::Size(182, 170);
			this->groupBoxStarboard->TabIndex = 120;
			this->groupBoxStarboard->TabStop = false;
			this->groupBoxStarboard->Text = L"Plateform Starboard";
			// 
			// label34
			// 
			this->label34->AutoSize = true;
			this->label34->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label34->Location = System::Drawing::Point(110, 71);
			this->label34->Name = L"label34";
			this->label34->Size = System::Drawing::Size(19, 13);
			this->label34->TabIndex = 133;
			this->label34->Text = L"Vz";
			this->label34->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label35
			// 
			this->label35->AutoSize = true;
			this->label35->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label35->Location = System::Drawing::Point(57, 71);
			this->label35->Name = L"label35";
			this->label35->Size = System::Drawing::Size(19, 13);
			this->label35->TabIndex = 132;
			this->label35->Text = L"Vy";
			this->label35->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label31
			// 
			this->label31->AutoSize = true;
			this->label31->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label31->Location = System::Drawing::Point(110, 23);
			this->label31->Name = L"label31";
			this->label31->Size = System::Drawing::Size(28, 13);
			this->label31->TabIndex = 133;
			this->label31->Text = L"Yaw";
			this->label31->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label36
			// 
			this->label36->AutoSize = true;
			this->label36->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label36->Location = System::Drawing::Point(4, 71);
			this->label36->Name = L"label36";
			this->label36->Size = System::Drawing::Size(19, 13);
			this->label36->TabIndex = 131;
			this->label36->Text = L"Vx";
			this->label36->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label32
			// 
			this->label32->AutoSize = true;
			this->label32->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label32->Location = System::Drawing::Point(57, 23);
			this->label32->Name = L"label32";
			this->label32->Size = System::Drawing::Size(31, 13);
			this->label32->TabIndex = 132;
			this->label32->Text = L"Pitch";
			this->label32->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// label33
			// 
			this->label33->AutoSize = true;
			this->label33->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->label33->Location = System::Drawing::Point(4, 23);
			this->label33->Name = L"label33";
			this->label33->Size = System::Drawing::Size(25, 13);
			this->label33->TabIndex = 131;
			this->label33->Text = L"Roll";
			this->label33->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// pictureBoxLogoLirmm
			// 
			this->pictureBoxLogoLirmm->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"pictureBoxLogoLirmm.Image")));
			this->pictureBoxLogoLirmm->Location = System::Drawing::Point(536, 561);
			this->pictureBoxLogoLirmm->Name = L"pictureBoxLogoLirmm";
			this->pictureBoxLogoLirmm->Size = System::Drawing::Size(177, 50);
			this->pictureBoxLogoLirmm->SizeMode = System::Windows::Forms::PictureBoxSizeMode::StretchImage;
			this->pictureBoxLogoLirmm->TabIndex = 121;
			this->pictureBoxLogoLirmm->TabStop = false;
			// 
			// pictureBoxLogoUM
			// 
			this->pictureBoxLogoUM->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"pictureBoxLogoUM.Image")));
			this->pictureBoxLogoUM->Location = System::Drawing::Point(438, 561);
			this->pictureBoxLogoUM->Name = L"pictureBoxLogoUM";
			this->pictureBoxLogoUM->Size = System::Drawing::Size(62, 50);
			this->pictureBoxLogoUM->SizeMode = System::Windows::Forms::PictureBoxSizeMode::StretchImage;
			this->pictureBoxLogoUM->TabIndex = 122;
			this->pictureBoxLogoUM->TabStop = false;
			// 
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1479, 622);
			this->Controls->Add(this->pictureBoxLogoUM);
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
			ss << std::fixed << std::setprecision(2) << number;
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
			output_video.open(vid_path, VideoWriter::fourcc('H', '2', '6', '4'), 15, cv::Size(640, 480), true);

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
		//this->labelArduinoIMU->Text = "Arduino IMU (roll, pitch, yaw): " + getPrecision(arduino.imu[0], 3) + "° " + getPrecision(arduino.imu[1], 3) + "° " + getPrecision(arduino.imu[2], 3) + "° ";
		
		this->textBoxArduinoRoll->Text = "" + getPrecision(arduino.imu[0], 3);
		this->textBoxArduinoPitch->Text = "" + getPrecision(arduino.imu[1], 3);
		this->textBoxArduinoYaw->Text = "" + getPrecision(arduino.imu[2], 3);
		
		//this->labelArduinoDepth->Text = "Arduino depth: " + getPrecision(arduino.depth, 3) + " m";
		this->textBoxArduinoDepth->Text = "" + getPrecision(arduino.depth, 3);
		//this->labelArduinoDepth->Text = "Arduino pressure sm: " + getPrecision(arduino.pressure_sm, 3) + " mbar";
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
		robot.goToTarget();
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
		this->textBoxRobotRoll->Text  = "" + getPrecision(robot.angles[0], 3);
		this->textBoxRobotPitch->Text = "" + getPrecision(robot.angles[1], 3);
		this->textBoxRobotYaw->Text   = "" + getPrecision(robot.angles[2], 3);
		
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

		this->textBoxDVLRoll->Text  = "" + getPrecision(dvl.imu[0], 3);
		this->textBoxDVLPitch->Text = "" + getPrecision(dvl.imu[1], 3);
		this->textBoxDVLYaw->Text   = "" + getPrecision(dvl.imu[2], 3);

		this->textBoxDVLD0->Text = "" + getPrecision(dvl.distances[0], 3);
		this->textBoxDVLD1->Text = "" + getPrecision(dvl.distances[1], 3);
		this->textBoxDVLD2->Text = "" + getPrecision(dvl.distances[2], 3);
		this->textBoxDVLD3->Text = "" + getPrecision(dvl.distances[3], 3);

		// this->labelDvlV->Text = "DVL (vx, vy): " + getPrecision(dvl.vx, 3) + "m.s^-1 " + getPrecision(dvl.vy, 3) + "m.s^-1 ";
		// this->labelDvlImu->Text = "DVL (roll, pitch, yaw): " + getPrecision(dvl.imu[0], 3) + "° " + getPrecision(dvl.imu[1], 3) + "° " + getPrecision(dvl.imu[2], 3) + "° ";
		// this->labelDvlDistances->Text = "DVL (d0, d1, d2, d3): " + getPrecision(dvl.distances[0], 3) + "m " + getPrecision(dvl.distances[1], 3) + "m " + getPrecision(dvl.distances[2], 3) + "m " + getPrecision(dvl.distances[3], 3) + "m ";
	}
		   // Start DVL and UDP reader
	private: System::Void backgroundWorkerDVLOn_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		log("DVL initialization");
		Sleep(30000);
		// Start DVL reader
		system(DVL_READER_START);
		backgroundWorkerDVLOn->CancelAsync();
		e->Cancel = true;
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
		arduino.dvlOn();
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
			if (robot.has_started) {
				depth_map.init(); // Reinit displayed map
				depth_map.update(dvl.coordinates, robot.coordinates, dvl.distances, arduino.depth); // Update mapping
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
				backgroundWorkerDepthMap->ReportProgress(depth_map.tide);
			}
			Sleep(50);
		}
	}


		   // Change resolution of the depth map
	private: System::Void buttonResolutionMap_Click(System::Object^ sender, System::EventArgs^ e) {
		depth_map.which_res = !depth_map.which_res;
		if (depth_map.which_res)
			buttonResolutionMap->Text = "Resolution 25x25 cm";
		else
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
		this->textBoxSRoll->Text  = "" + getPrecision(plateform.imu_S[0], 3);
		this->textBoxSPitch->Text = "" + getPrecision(plateform.imu_S[1], 3);
		this->textBoxSYaw->Text   = "" + getPrecision(plateform.imu_S[2], 3);

		this->textBoxSGyrX->Text = "" + getPrecision(plateform.v_gyro_S[0], 3);
		this->textBoxSGyrY->Text = "" + getPrecision(plateform.v_gyro_S[1], 3);
		this->textBoxSGyrZ->Text = "" + getPrecision(plateform.v_gyro_S[2], 3);

		this->textBoxAtmPressure->Text = "" + getPrecision(plateform.atm_pressure, 3);


	}
		   // Display data from port continuously
	private: System::Void backgroundWorkerPlateformP_ProgressChanged(System::Object^ sender, System::ComponentModel::ProgressChangedEventArgs^ e) {
		this->textBoxPRoll->Text  = "" + getPrecision(plateform.imu_P[0], 3);
		this->textBoxPPitch->Text = "" + getPrecision(plateform.imu_P[1], 3);
		this->textBoxPYaw->Text   = "" + getPrecision(plateform.imu_P[2], 3);

		this->textBoxPGyrX->Text = "" + getPrecision(plateform.v_gyro_P[0], 3);
		this->textBoxPGyrY->Text = "" + getPrecision(plateform.v_gyro_P[1], 3);
		this->textBoxPGyrZ->Text = "" + getPrecision(plateform.v_gyro_P[2], 3);

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
		   // Only to measurment purpose
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

		   // Show values
	private: System::Void ptbDepthMap_MouseHover(System::Object^ sender, System::EventArgs^ e) {
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

		   // Display tide in real time when possible
	private: System::Void backgroundWorkerDepthMap_ProgressChanged(System::Object^ sender, System::ComponentModel::ProgressChangedEventArgs^ e) {
		this->textBoxTide->Text = ""+ getPrecision(depth_map.tide, 3);
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
};
}
