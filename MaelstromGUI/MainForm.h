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
	string log_dir_path("D:/projects/cyril/logs/");
	string videos_dir_path("D:/projects/cyril/videos/");

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
			InitializeComponent();

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

			cout << getTimeStamp() << endl;

			// Keep track of logs
			if (false) {
				string log_path = log_dir_path + getTime() + ".txt";
				freopen(log_path.c_str(), "w", stdout);
			}
			log("Start");

			// Robot
			// // Check if robot has started
			backgroundWorkerRobotStarted->RunWorkerAsync(1);
			// To receive continuous data from the robot
			backgroundWorkerRobot->RunWorkerAsync(1);
			// Arduino
			// Check if arduino has started
			backgroundWorkerArduinoStarted->RunWorkerAsync(1);
			// To receive continuous data from the arduino
			backgroundWorkerArduino->RunWorkerAsync(1);
			// Jetson
			// To receive continuous data from the jetson
			backgroundWorkerJetson->RunWorkerAsync(1);
			// DVL
			// To receive continuous data from the dvl
			backgroundWorkerDVL->RunWorkerAsync(1);
			// Depth map
			// Display the depth map
			backgroundWorkerDepthMap->RunWorkerAsync(1);
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
	private: System::Windows::Forms::Label^ label1;
	private: System::Windows::Forms::Label^ label2;
	private: System::Windows::Forms::Label^ label3;
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
	private: System::Windows::Forms::Label^ labelRobotX;

	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerDepthMap;
	private: System::Windows::Forms::PictureBox^ ptbDepthMap;
	private: System::Windows::Forms::Label^ labelTarget;
	private: System::Windows::Forms::Button^ buttonGoToTarget;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerRobotStarted;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerDVLOn;
	private: System::ComponentModel::BackgroundWorker^ backgroundWorkerRobotScan;
	private: System::Windows::Forms::TextBox^ textBox1;
	private: System::Windows::Forms::Label^ labelDepthScan;
	private: System::Windows::Forms::Label^ labelRobotAngles;
	private: System::Windows::Forms::Label^ labelRobotState;
	private: System::Windows::Forms::Label^ labelArduinoIMU;
	private: System::Windows::Forms::Label^ labelArduinoDepth;
private: System::Windows::Forms::Label^ labelGripper;
private: System::Windows::Forms::Label^ labelPump;
private: System::Windows::Forms::Label^ labelDvlV;
private: System::Windows::Forms::Label^ labelDvlImu;
private: System::Windows::Forms::Label^ labelDvlDistances;
private: System::Windows::Forms::Button^ buttonResolutionMap;




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
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->label3 = (gcnew System::Windows::Forms::Label());
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
			this->labelRobotX = (gcnew System::Windows::Forms::Label());
			this->labelTarget = (gcnew System::Windows::Forms::Label());
			this->buttonGoToTarget = (gcnew System::Windows::Forms::Button());
			this->backgroundWorkerRobotStarted = (gcnew System::ComponentModel::BackgroundWorker());
			this->backgroundWorkerDVLOn = (gcnew System::ComponentModel::BackgroundWorker());
			this->buttonScan = (gcnew System::Windows::Forms::Button());
			this->backgroundWorkerRobotScan = (gcnew System::ComponentModel::BackgroundWorker());
			this->textBox1 = (gcnew System::Windows::Forms::TextBox());
			this->labelDepthScan = (gcnew System::Windows::Forms::Label());
			this->labelRobotAngles = (gcnew System::Windows::Forms::Label());
			this->labelRobotState = (gcnew System::Windows::Forms::Label());
			this->labelArduinoIMU = (gcnew System::Windows::Forms::Label());
			this->labelArduinoDepth = (gcnew System::Windows::Forms::Label());
			this->labelGripper = (gcnew System::Windows::Forms::Label());
			this->labelPump = (gcnew System::Windows::Forms::Label());
			this->labelDvlV = (gcnew System::Windows::Forms::Label());
			this->labelDvlImu = (gcnew System::Windows::Forms::Label());
			this->labelDvlDistances = (gcnew System::Windows::Forms::Label());
			this->buttonResolutionMap = (gcnew System::Windows::Forms::Button());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ptbSource))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->video_trackBar))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ptbDepthMap))->BeginInit();
			this->SuspendLayout();
			// 
			// button_Edition
			// 
			this->button_Edition->Enabled = false;
			this->button_Edition->Location = System::Drawing::Point(1309, 29);
			this->button_Edition->Name = L"button_Edition";
			this->button_Edition->Size = System::Drawing::Size(75, 23);
			this->button_Edition->TabIndex = 35;
			this->button_Edition->Text = L"Edition";
			// 
			// ptbSource
			// 
			this->ptbSource->BackColor = System::Drawing::SystemColors::ControlDark;
			this->ptbSource->Location = System::Drawing::Point(743, 58);
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
			this->button_Browse->Location = System::Drawing::Point(1235, 28);
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
			this->listView1->Location = System::Drawing::Point(1188, 28);
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
			this->view_button->Location = System::Drawing::Point(1115, 29);
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
			this->play_button->Location = System::Drawing::Point(829, 588);
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
			this->speed_button->Location = System::Drawing::Point(915, 588);
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
			this->load_button->Location = System::Drawing::Point(743, 588);
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
			this->video_trackBar->Location = System::Drawing::Point(833, 544);
			this->video_trackBar->Maximum = 150;
			this->video_trackBar->Name = L"video_trackBar";
			this->video_trackBar->Size = System::Drawing::Size(521, 45);
			this->video_trackBar->TabIndex = 8;
			this->video_trackBar->Scroll += gcnew System::EventHandler(this, &MainForm::video_trackBar_Scroll);
			// 
			// video_label
			// 
			this->video_label->AutoSize = true;
			this->video_label->Location = System::Drawing::Point(1360, 549);
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
			this->record_button->Location = System::Drawing::Point(743, 548);
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
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(1170, 614);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(37, 13);
			this->label1->TabIndex = 19;
			this->label1->Text = L"x (mm)";
			this->label1->Visible = false;
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(1244, 614);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(37, 13);
			this->label2->TabIndex = 20;
			this->label2->Text = L"y (mm)";
			this->label2->Visible = false;
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(1318, 614);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(37, 13);
			this->label3->TabIndex = 21;
			this->label3->Text = L"z (mm)";
			this->label3->Visible = false;
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
			this->label3dY->Location = System::Drawing::Point(795, 26);
			this->label3dY->Name = L"label3dY";
			this->label3dY->Size = System::Drawing::Size(44, 13);
			this->label3dY->TabIndex = 32;
			this->label3dY->Text = L"y: None";
			this->label3dY->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label3dX
			// 
			this->label3dX->AutoSize = true;
			this->label3dX->Location = System::Drawing::Point(745, 26);
			this->label3dX->Name = L"label3dX";
			this->label3dX->Size = System::Drawing::Size(44, 13);
			this->label3dX->TabIndex = 31;
			this->label3dX->Text = L"x: None";
			this->label3dX->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label3dZ
			// 
			this->label3dZ->AutoSize = true;
			this->label3dZ->Location = System::Drawing::Point(845, 26);
			this->label3dZ->Name = L"label3dZ";
			this->label3dZ->Size = System::Drawing::Size(44, 13);
			this->label3dZ->TabIndex = 33;
			this->label3dZ->Text = L"z: None";
			this->label3dZ->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// labelDVL
			// 
			this->labelDVL->AutoSize = true;
			this->labelDVL->Location = System::Drawing::Point(705, 42);
			this->labelDVL->Name = L"labelDVL";
			this->labelDVL->Size = System::Drawing::Size(34, 13);
			this->labelDVL->TabIndex = 34;
			this->labelDVL->Text = L"DVL: ";
			this->labelDVL->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label3dZFake
			// 
			this->label3dZFake->AutoSize = true;
			this->label3dZFake->Location = System::Drawing::Point(845, 42);
			this->label3dZFake->Name = L"label3dZFake";
			this->label3dZFake->Size = System::Drawing::Size(44, 13);
			this->label3dZFake->TabIndex = 38;
			this->label3dZFake->Text = L"z: None";
			this->label3dZFake->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label3dYFake
			// 
			this->label3dYFake->AutoSize = true;
			this->label3dYFake->Location = System::Drawing::Point(795, 42);
			this->label3dYFake->Name = L"label3dYFake";
			this->label3dYFake->Size = System::Drawing::Size(44, 13);
			this->label3dYFake->TabIndex = 37;
			this->label3dYFake->Text = L"y: None";
			this->label3dYFake->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label3dXFake
			// 
			this->label3dXFake->AutoSize = true;
			this->label3dXFake->Location = System::Drawing::Point(745, 42);
			this->label3dXFake->Name = L"label3dXFake";
			this->label3dXFake->Size = System::Drawing::Size(44, 13);
			this->label3dXFake->TabIndex = 36;
			this->label3dXFake->Text = L"x: None";
			this->label3dXFake->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// labelLine
			// 
			this->labelLine->AutoSize = true;
			this->labelLine->Location = System::Drawing::Point(912, 29);
			this->labelLine->Name = L"labelLine";
			this->labelLine->Size = System::Drawing::Size(59, 13);
			this->labelLine->TabIndex = 39;
			this->labelLine->Text = L"Line: None";
			this->labelLine->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// labelLineFake
			// 
			this->labelLineFake->AutoSize = true;
			this->labelLineFake->Location = System::Drawing::Point(912, 42);
			this->labelLineFake->Name = L"labelLineFake";
			this->labelLineFake->Size = System::Drawing::Size(59, 13);
			this->labelLineFake->TabIndex = 40;
			this->labelLineFake->Text = L"Line: None";
			this->labelLineFake->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// backgroundWorkerDepthMap
			// 
			this->backgroundWorkerDepthMap->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerDepthMap_DoWork);
			// 
			// ptbDepthMap
			// 
			this->ptbDepthMap->BackColor = System::Drawing::SystemColors::ControlDark;
			this->ptbDepthMap->Location = System::Drawing::Point(13, 58);
			this->ptbDepthMap->Name = L"ptbDepthMap";
			this->ptbDepthMap->Size = System::Drawing::Size(320, 520);
			this->ptbDepthMap->TabIndex = 41;
			this->ptbDepthMap->TabStop = false;
			// 
			// labelRobotX
			// 
			this->labelRobotX->AutoSize = true;
			this->labelRobotX->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->labelRobotX->Location = System::Drawing::Point(341, 175);
			this->labelRobotX->Name = L"labelRobotX";
			this->labelRobotX->Size = System::Drawing::Size(104, 13);
			this->labelRobotX->TabIndex = 42;
			this->labelRobotX->Text = L"Robot (x, y, z): None";
			this->labelRobotX->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// labelTarget
			// 
			this->labelTarget->AutoSize = true;
			this->labelTarget->Location = System::Drawing::Point(417, 69);
			this->labelTarget->Name = L"labelTarget";
			this->labelTarget->Size = System::Drawing::Size(106, 13);
			this->labelTarget->TabIndex = 43;
			this->labelTarget->Text = L"Target (x, y, z): None";
			this->labelTarget->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// buttonGoToTarget
			// 
			this->buttonGoToTarget->Enabled = false;
			this->buttonGoToTarget->Location = System::Drawing::Point(339, 58);
			this->buttonGoToTarget->Name = L"buttonGoToTarget";
			this->buttonGoToTarget->Size = System::Drawing::Size(72, 35);
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
			this->buttonScan->Location = System::Drawing::Point(339, 99);
			this->buttonScan->Name = L"buttonScan";
			this->buttonScan->Size = System::Drawing::Size(72, 35);
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
			this->textBox1->Location = System::Drawing::Point(417, 114);
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
			this->labelDepthScan->Location = System::Drawing::Point(434, 99);
			this->labelDepthScan->Name = L"labelDepthScan";
			this->labelDepthScan->Size = System::Drawing::Size(36, 13);
			this->labelDepthScan->TabIndex = 47;
			this->labelDepthScan->Text = L"Depth";
			this->labelDepthScan->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// labelRobotAngles
			// 
			this->labelRobotAngles->AutoSize = true;
			this->labelRobotAngles->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->labelRobotAngles->Location = System::Drawing::Point(341, 197);
			this->labelRobotAngles->Name = L"labelRobotAngles";
			this->labelRobotAngles->Size = System::Drawing::Size(144, 13);
			this->labelRobotAngles->TabIndex = 48;
			this->labelRobotAngles->Text = L"Robot (roll, pitch, yaw): None";
			this->labelRobotAngles->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// labelRobotState
			// 
			this->labelRobotState->AutoSize = true;
			this->labelRobotState->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->labelRobotState->Location = System::Drawing::Point(341, 152);
			this->labelRobotState->Name = L"labelRobotState";
			this->labelRobotState->Size = System::Drawing::Size(94, 13);
			this->labelRobotState->TabIndex = 49;
			this->labelRobotState->Text = L"Robot state: None";
			this->labelRobotState->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// labelArduinoIMU
			// 
			this->labelArduinoIMU->AutoSize = true;
			this->labelArduinoIMU->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->labelArduinoIMU->Location = System::Drawing::Point(341, 263);
			this->labelArduinoIMU->Name = L"labelArduinoIMU";
			this->labelArduinoIMU->Size = System::Drawing::Size(177, 13);
			this->labelArduinoIMU->TabIndex = 50;
			this->labelArduinoIMU->Text = L"Arduino IMU: (roll, pitch, yaw): None";
			this->labelArduinoIMU->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// labelArduinoDepth
			// 
			this->labelArduinoDepth->AutoSize = true;
			this->labelArduinoDepth->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->labelArduinoDepth->Location = System::Drawing::Point(341, 286);
			this->labelArduinoDepth->Name = L"labelArduinoDepth";
			this->labelArduinoDepth->Size = System::Drawing::Size(105, 13);
			this->labelArduinoDepth->TabIndex = 51;
			this->labelArduinoDepth->Text = L"Arduino depth: None";
			this->labelArduinoDepth->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// labelGripper
			// 
			this->labelGripper->AutoSize = true;
			this->labelGripper->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->labelGripper->Location = System::Drawing::Point(341, 219);
			this->labelGripper->Name = L"labelGripper";
			this->labelGripper->Size = System::Drawing::Size(67, 13);
			this->labelGripper->TabIndex = 52;
			this->labelGripper->Text = L"Gripper: OFF";
			this->labelGripper->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// labelPump
			// 
			this->labelPump->AutoSize = true;
			this->labelPump->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->labelPump->Location = System::Drawing::Point(341, 241);
			this->labelPump->Name = L"labelPump";
			this->labelPump->Size = System::Drawing::Size(60, 13);
			this->labelPump->TabIndex = 53;
			this->labelPump->Text = L"Pump: OFF";
			this->labelPump->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// labelDvlV
			// 
			this->labelDvlV->AutoSize = true;
			this->labelDvlV->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->labelDvlV->Location = System::Drawing::Point(341, 308);
			this->labelDvlV->Name = L"labelDvlV";
			this->labelDvlV->Size = System::Drawing::Size(97, 13);
			this->labelDvlV->TabIndex = 54;
			this->labelDvlV->Text = L"DVL (vx, vy): None";
			this->labelDvlV->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// labelDvlImu
			// 
			this->labelDvlImu->AutoSize = true;
			this->labelDvlImu->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->labelDvlImu->Location = System::Drawing::Point(341, 330);
			this->labelDvlImu->Name = L"labelDvlImu";
			this->labelDvlImu->Size = System::Drawing::Size(136, 13);
			this->labelDvlImu->TabIndex = 55;
			this->labelDvlImu->Text = L"DVL (roll, pitch, yaw): None";
			this->labelDvlImu->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// labelDvlDistances
			// 
			this->labelDvlDistances->AutoSize = true;
			this->labelDvlDistances->ImageAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->labelDvlDistances->Location = System::Drawing::Point(341, 352);
			this->labelDvlDistances->Name = L"labelDvlDistances";
			this->labelDvlDistances->Size = System::Drawing::Size(135, 13);
			this->labelDvlDistances->TabIndex = 56;
			this->labelDvlDistances->Text = L"DVL (d0, d1, d2, d3): None";
			this->labelDvlDistances->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// buttonResolutionMap
			// 
			this->buttonResolutionMap->Location = System::Drawing::Point(12, 584);
			this->buttonResolutionMap->Name = L"buttonResolutionMap";
			this->buttonResolutionMap->Size = System::Drawing::Size(75, 23);
			this->buttonResolutionMap->TabIndex = 57;
			this->buttonResolutionMap->Text = L"Resolution";
			this->buttonResolutionMap->UseVisualStyleBackColor = true;
			// 
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1404, 624);
			this->Controls->Add(this->buttonResolutionMap);
			this->Controls->Add(this->labelDvlDistances);
			this->Controls->Add(this->labelDvlImu);
			this->Controls->Add(this->labelDvlV);
			this->Controls->Add(this->labelPump);
			this->Controls->Add(this->labelGripper);
			this->Controls->Add(this->labelArduinoDepth);
			this->Controls->Add(this->labelArduinoIMU);
			this->Controls->Add(this->labelRobotState);
			this->Controls->Add(this->labelRobotAngles);
			this->Controls->Add(this->labelDepthScan);
			this->Controls->Add(this->textBox1);
			this->Controls->Add(this->buttonScan);
			this->Controls->Add(this->buttonGoToTarget);
			this->Controls->Add(this->labelTarget);
			this->Controls->Add(this->labelRobotX);
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
			this->Controls->Add(this->label3);
			this->Controls->Add(this->label2);
			this->Controls->Add(this->label1);
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
			this->Name = L"MainForm";
			this->Text = L"Maelstrom GUI";
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ptbSource))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->video_trackBar))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ptbDepthMap))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}

#pragma endregion

	/* --------------------------------------------------
	*
	*	Some cool functions
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
	*	Video related functions
	*
	----------------------------------------------------- */

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

			this->labelTarget->Text = "Target (x, y, z): " + getPrecision(target_point3D.x, 3) + " m " +
				getPrecision(target_point3D.y, 3) + " m " +
				getPrecision(target_point3D.z, 3) + " m";

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

	/*----------------------------------------------------------------------------------------------------
			Background workers:
				- Arduino
				- DVL
				- Robot
				- Robot command
				- Jetson
	------------------------------------------------------------------------------------------------------*/

	/*-------------------------------
	* 
	*  ARDUINO workers
	* 
	* -------------------------------*/

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
			arduino.rcvData();
			backgroundWorkerArduino->ReportProgress(arduino.depth);
		}
	}

	// Display arduino state in real-time
	private: System::Void backgroundWorkerArduino_ProgressChanged(System::Object^ sender, System::ComponentModel::ProgressChangedEventArgs^ e) {
		this->labelArduinoIMU->Text = "Arduino IMU (roll, pitch, yaw): " + getPrecision(arduino.imu[0], 3) + "° " + getPrecision(arduino.imu[1], 3) + "° " + getPrecision(arduino.imu[2], 3) + "° ";
		this->labelArduinoDepth->Text = "Arduino depth: " + getPrecision(arduino.depth, 3) + " m";
	}
	
	/*-------------------------------
	*
	*  ROBOT workers
	*
	* -------------------------------*/

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
		this->labelRobotState->Text = "Robot state: " + gcnew System::String(robot.current_state.c_str());
		this->labelRobotX->Text = "Robot (x, y, z): " + getPrecision(robot.pos[0], 3) + " m " + getPrecision(robot.pos[1], 3) + " m " + getPrecision(robot.pos[2], 3) + " m";
		this->labelRobotAngles->Text = "Robot (roll, pitch, yaw): " + getPrecision(robot.angles[0], 3) + "° " + getPrecision(robot.angles[1], 3) + "° " + getPrecision(robot.angles[2], 3) + "°";
		if (robot.gripper_state)
			this->labelGripper->Text = "Gripper: ON";
		else
			this->labelGripper->Text = "Gripper: OFF";
		if (robot.pump_state)
			this->labelPump->Text = "Pump: ON";
		else
			this->labelPump->Text = "Pump: OFF";
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

	/*-------------------------------
	*
	*  DVL workers
	*
	* -------------------------------*/

	// Receive data from DVL continuously
	private: System::Void backgroundWorkerDVL_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		while (true) {
			dvl.rcvData();
			backgroundWorkerDVL->ReportProgress(dvl.vx);
			backgroundWorkerDVL->ReportProgress(dvl.vy);
			backgroundWorkerDVL->ReportProgress(dvl.imu[0]);
			backgroundWorkerDVL->ReportProgress(dvl.imu[1]);
			backgroundWorkerDVL->ReportProgress(dvl.imu[2]);
			backgroundWorkerDVL->ReportProgress(dvl.distances[0]);
			backgroundWorkerDVL->ReportProgress(dvl.distances[1]);
			backgroundWorkerDVL->ReportProgress(dvl.distances[2]);
		}
	}

	// Display DVL state in real-time
	private: System::Void backgroundWorkerDVL_ProgressChanged(System::Object^ sender, System::ComponentModel::ProgressChangedEventArgs^ e) {
		this->labelDvlV->Text = "DVL (vx, vy): " + getPrecision(dvl.vx, 3) + "m.s^-1 " + getPrecision(dvl.vy, 3) + "m.s^-1 ";
		this->labelDvlImu->Text = "DVL (roll, pitch, yaw): " + getPrecision(dvl.imu[0], 3) + "° " + getPrecision(dvl.imu[1], 3) + "° " + getPrecision(dvl.imu[2], 3) + "° ";
		this->labelDvlDistances->Text = "DVL (d0, d1, d2, d3): " + getPrecision(dvl.distances[0], 3) + "m " + getPrecision(dvl.distances[1], 3) + "m " + getPrecision(dvl.distances[2], 3) + "m " + getPrecision(dvl.distances[3], 3) + "m ";
	}

	// Start DVL
	private: System::Void buttonDVLon_Click(System::Object^ sender, System::EventArgs^ e) {
		if (this->buttonDVLon->Text == L"DVL on") {
			MessageBox::Show("Please make sure the DVL is underwater. It will break if not turned on underwtaer.");
			this->labelDVLon->Visible = true;
			this->buttonDVLyes->Visible = true;
			this->buttonDVLno->Visible = true;
		}
		else {
			arduino.dvlOff();
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

	// Start DVL UDP reader
	private: System::Void backgroundWorkerDVLOn_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		log("DVL initialization");
		Sleep(30000);
		char* path = "start D:/projects/cyril/soft_lecture_DVL_envoi_UDP_Maelstrom_v0_5/executable_dvl_module.exe";
		int result = system(path);
		backgroundWorkerDVLOn->CancelAsync();
		e->Cancel = true;
	}


	/*-------------------------------
	*
	*  JETSON workers
	*
	* -------------------------------*/

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

	/*-------------------------------
	*
	*  Depth map workers
	*
	* -------------------------------*/

	// Compute and display the depth map
	private: System::Void backgroundWorkerDepthMap_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {

		while (true) {
			depth_map.init();
			depth_map.update(dvl.coordinates, robot.coordinates);
			//depth_map.setDepthMap();
			depth_map.drawGrid();
			depth_map.setPos(robot.pos[0], robot.pos[1]);
			depth_map.drawRobot();
			if (draw_target) {
				depth_map.drawTarget(cv::Point2f(robot.target[0], robot.target[1]));
			}
			
			display(depth_map.getMap(), 1);
			Sleep(100);
		}
	}

	/*-------------------------------
	*
	*  Mouse on the picture box, measurement etc.
	*
	* -------------------------------*/

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

	/*-------------------------------
	*
	*  Robot command:
	*	- Scanning
	*	- Go to target
	*
	* -------------------------------*/

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

};
}
