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
	int request_id = 1;

	Jetson jetson;

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
			//
			//TODO: Add the constructor code here
			//

			// Camera calibration
			camera_params.center = cv::Point(3.1950000000000000e+002, 2.3950000000000000e+002);
			camera_params.fx = 4.2750715468970623e+002;
			camera_params.fy = 4.2750715468970623e+002;

			// Measurment points
			measure.a = cv::Point(-10, -10);
			measure.b = cv::Point(-10, -10);

			cout << time_in_HH_MM_SS_MMM() << endl;
			cout << getTimeStamp() << endl;

			// Keep track of logs
			if (false) {
				string log_path = log_dir_path + getTime() + ".txt";
				freopen(log_path.c_str(), "w", stdout);
			}
			log("Start");

			// Init robot/arduino/jetson communication

			// To receive continuous data from the robot
			backgroundWorkerRobot->RunWorkerAsync(1);

			// Check if arduino has started
			backgroundWorkerArduinoStarted->RunWorkerAsync(1);

			// To receive continuous data from the arduino
			backgroundWorkerArduino->RunWorkerAsync(1);

			// To receive continuous data from the jetson
			backgroundWorkerJetson->RunWorkerAsync(1);
			// 
			// To receive continuous data from the dvl
			backgroundWorkerDVL->RunWorkerAsync(1);

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
	private: System::Windows::Forms::Button^ goToButton;
	private: System::Windows::Forms::TextBox^ xCooText;
	private: System::Windows::Forms::TextBox^ yCooText;
	private: System::Windows::Forms::TextBox^ zCooText;
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
	private: System::Windows::Forms::Label^ labelMouseX;
	private: System::Windows::Forms::Label^ labelMouseY;
	private: System::Windows::Forms::Label^ label3dZFake;
	private: System::Windows::Forms::Label^ label3dYFake;
	private: System::Windows::Forms::Label^ label3dXFake;
	private: System::Windows::Forms::Label^ labelLineFake;

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
		this->goToButton = (gcnew System::Windows::Forms::Button());
		this->xCooText = (gcnew System::Windows::Forms::TextBox());
		this->yCooText = (gcnew System::Windows::Forms::TextBox());
		this->zCooText = (gcnew System::Windows::Forms::TextBox());
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
		this->labelMouseX = (gcnew System::Windows::Forms::Label());
		this->labelMouseY = (gcnew System::Windows::Forms::Label());
		this->label3dY = (gcnew System::Windows::Forms::Label());
		this->label3dX = (gcnew System::Windows::Forms::Label());
		this->label3dZ = (gcnew System::Windows::Forms::Label());
		this->labelDVL = (gcnew System::Windows::Forms::Label());
		this->label3dZFake = (gcnew System::Windows::Forms::Label());
		this->label3dYFake = (gcnew System::Windows::Forms::Label());
		this->label3dXFake = (gcnew System::Windows::Forms::Label());
		this->labelLine = (gcnew System::Windows::Forms::Label());
		this->labelLineFake = (gcnew System::Windows::Forms::Label());
		(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ptbSource))->BeginInit();
		(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->video_trackBar))->BeginInit();
		this->SuspendLayout();
		// 
		// button_Edition
		// 
		this->button_Edition->Enabled = false;
		this->button_Edition->Location = System::Drawing::Point(1234, 29);
		this->button_Edition->Name = L"button_Edition";
		this->button_Edition->Size = System::Drawing::Size(75, 23);
		this->button_Edition->TabIndex = 35;
		this->button_Edition->Text = L"Edition";
		// 
		// ptbSource
		// 
		this->ptbSource->BackColor = System::Drawing::SystemColors::ControlDark;
		this->ptbSource->Location = System::Drawing::Point(668, 58);
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
		this->button_Browse->Location = System::Drawing::Point(1160, 28);
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
		this->listView1->Location = System::Drawing::Point(1113, 28);
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
		this->view_button->Location = System::Drawing::Point(1040, 29);
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
		this->play_button->Location = System::Drawing::Point(754, 588);
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
		this->speed_button->Location = System::Drawing::Point(840, 588);
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
		this->load_button->Location = System::Drawing::Point(668, 588);
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
		this->video_trackBar->Location = System::Drawing::Point(758, 544);
		this->video_trackBar->Maximum = 150;
		this->video_trackBar->Name = L"video_trackBar";
		this->video_trackBar->Size = System::Drawing::Size(521, 45);
		this->video_trackBar->TabIndex = 8;
		this->video_trackBar->Scroll += gcnew System::EventHandler(this, &MainForm::video_trackBar_Scroll);
		// 
		// video_label
		// 
		this->video_label->AutoSize = true;
		this->video_label->Location = System::Drawing::Point(1285, 549);
		this->video_label->Name = L"video_label";
		this->video_label->Size = System::Drawing::Size(24, 13);
		this->video_label->TabIndex = 9;
		this->video_label->Text = L"0/0";
		// 
		// record_button
		// 
		this->record_button->BackColor = System::Drawing::SystemColors::ControlLight;
		this->record_button->Enabled = false;
		this->record_button->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
			static_cast<System::Byte>(0)));
		this->record_button->Location = System::Drawing::Point(668, 548);
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
		this->backgroundWorkerRobot->WorkerSupportsCancellation = true;
		this->backgroundWorkerRobot->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerRobot_DoWork);
		// 
		// goToButton
		// 
		this->goToButton->Enabled = false;
		this->goToButton->Location = System::Drawing::Point(973, 588);
		this->goToButton->Name = L"goToButton";
		this->goToButton->Size = System::Drawing::Size(75, 23);
		this->goToButton->TabIndex = 15;
		this->goToButton->Text = L"Go to";
		this->goToButton->UseVisualStyleBackColor = true;
		this->goToButton->Visible = false;
		this->goToButton->Click += gcnew System::EventHandler(this, &MainForm::goToButton_Click);
		// 
		// xCooText
		// 
		this->xCooText->Enabled = false;
		this->xCooText->Location = System::Drawing::Point(1064, 591);
		this->xCooText->Name = L"xCooText";
		this->xCooText->Size = System::Drawing::Size(68, 20);
		this->xCooText->TabIndex = 16;
		this->xCooText->Text = L"- - -";
		this->xCooText->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
		this->xCooText->Visible = false;
		this->xCooText->Click += gcnew System::EventHandler(this, &MainForm::xCooText_Click);
		// 
		// yCooText
		// 
		this->yCooText->Enabled = false;
		this->yCooText->Location = System::Drawing::Point(1138, 591);
		this->yCooText->Name = L"yCooText";
		this->yCooText->Size = System::Drawing::Size(68, 20);
		this->yCooText->TabIndex = 17;
		this->yCooText->Text = L"- - -";
		this->yCooText->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
		this->yCooText->Visible = false;
		this->yCooText->Click += gcnew System::EventHandler(this, &MainForm::yCooText_Click);
		// 
		// zCooText
		// 
		this->zCooText->Enabled = false;
		this->zCooText->Location = System::Drawing::Point(1212, 591);
		this->zCooText->Name = L"zCooText";
		this->zCooText->Size = System::Drawing::Size(68, 20);
		this->zCooText->TabIndex = 18;
		this->zCooText->Text = L"- - -";
		this->zCooText->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
		this->zCooText->Visible = false;
		this->zCooText->Click += gcnew System::EventHandler(this, &MainForm::zCooText_Click);
		// 
		// label1
		// 
		this->label1->AutoSize = true;
		this->label1->Location = System::Drawing::Point(1095, 614);
		this->label1->Name = L"label1";
		this->label1->Size = System::Drawing::Size(37, 13);
		this->label1->TabIndex = 19;
		this->label1->Text = L"x (mm)";
		this->label1->Visible = false;
		// 
		// label2
		// 
		this->label2->AutoSize = true;
		this->label2->Location = System::Drawing::Point(1169, 614);
		this->label2->Name = L"label2";
		this->label2->Size = System::Drawing::Size(37, 13);
		this->label2->TabIndex = 20;
		this->label2->Text = L"y (mm)";
		this->label2->Visible = false;
		// 
		// label3
		// 
		this->label3->AutoSize = true;
		this->label3->Location = System::Drawing::Point(1243, 614);
		this->label3->Name = L"label3";
		this->label3->Size = System::Drawing::Size(37, 13);
		this->label3->TabIndex = 21;
		this->label3->Text = L"z (mm)";
		this->label3->Visible = false;
		// 
		// backgroundWorkerArduino
		// 
		this->backgroundWorkerArduino->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerArduino_DoWork);
		// 
		// backgroundWorkerDVL
		// 
		this->backgroundWorkerDVL->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MainForm::backgroundWorkerDVL_DoWork);
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
		// labelMouseX
		// 
		this->labelMouseX->AutoSize = true;
		this->labelMouseX->Location = System::Drawing::Point(670, 10);
		this->labelMouseX->Name = L"labelMouseX";
		this->labelMouseX->Size = System::Drawing::Size(45, 13);
		this->labelMouseX->TabIndex = 29;
		this->labelMouseX->Text = L"u: None";
		this->labelMouseX->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
		// 
		// labelMouseY
		// 
		this->labelMouseY->AutoSize = true;
		this->labelMouseY->Location = System::Drawing::Point(719, 10);
		this->labelMouseY->Name = L"labelMouseY";
		this->labelMouseY->Size = System::Drawing::Size(45, 13);
		this->labelMouseY->TabIndex = 30;
		this->labelMouseY->Text = L"v: None";
		this->labelMouseY->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
		// 
		// label3dY
		// 
		this->label3dY->AutoSize = true;
		this->label3dY->Location = System::Drawing::Point(720, 26);
		this->label3dY->Name = L"label3dY";
		this->label3dY->Size = System::Drawing::Size(44, 13);
		this->label3dY->TabIndex = 32;
		this->label3dY->Text = L"y: None";
		this->label3dY->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
		// 
		// label3dX
		// 
		this->label3dX->AutoSize = true;
		this->label3dX->Location = System::Drawing::Point(670, 26);
		this->label3dX->Name = L"label3dX";
		this->label3dX->Size = System::Drawing::Size(44, 13);
		this->label3dX->TabIndex = 31;
		this->label3dX->Text = L"x: None";
		this->label3dX->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
		// 
		// label3dZ
		// 
		this->label3dZ->AutoSize = true;
		this->label3dZ->Location = System::Drawing::Point(770, 26);
		this->label3dZ->Name = L"label3dZ";
		this->label3dZ->Size = System::Drawing::Size(44, 13);
		this->label3dZ->TabIndex = 33;
		this->label3dZ->Text = L"z: None";
		this->label3dZ->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
		// 
		// labelDVL
		// 
		this->labelDVL->AutoSize = true;
		this->labelDVL->Location = System::Drawing::Point(630, 42);
		this->labelDVL->Name = L"labelDVL";
		this->labelDVL->Size = System::Drawing::Size(34, 13);
		this->labelDVL->TabIndex = 34;
		this->labelDVL->Text = L"DVL: ";
		this->labelDVL->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
		// 
		// label3dZFake
		// 
		this->label3dZFake->AutoSize = true;
		this->label3dZFake->Location = System::Drawing::Point(770, 42);
		this->label3dZFake->Name = L"label3dZFake";
		this->label3dZFake->Size = System::Drawing::Size(44, 13);
		this->label3dZFake->TabIndex = 38;
		this->label3dZFake->Text = L"z: None";
		this->label3dZFake->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
		// 
		// label3dYFake
		// 
		this->label3dYFake->AutoSize = true;
		this->label3dYFake->Location = System::Drawing::Point(720, 42);
		this->label3dYFake->Name = L"label3dYFake";
		this->label3dYFake->Size = System::Drawing::Size(44, 13);
		this->label3dYFake->TabIndex = 37;
		this->label3dYFake->Text = L"y: None";
		this->label3dYFake->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
		// 
		// label3dXFake
		// 
		this->label3dXFake->AutoSize = true;
		this->label3dXFake->Location = System::Drawing::Point(670, 42);
		this->label3dXFake->Name = L"label3dXFake";
		this->label3dXFake->Size = System::Drawing::Size(44, 13);
		this->label3dXFake->TabIndex = 36;
		this->label3dXFake->Text = L"x: None";
		this->label3dXFake->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
		// 
		// labelLine
		// 
		this->labelLine->AutoSize = true;
		this->labelLine->Location = System::Drawing::Point(837, 29);
		this->labelLine->Name = L"labelLine";
		this->labelLine->Size = System::Drawing::Size(59, 13);
		this->labelLine->TabIndex = 39;
		this->labelLine->Text = L"Line: None";
		this->labelLine->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
		// 
		// labelLineFake
		// 
		this->labelLineFake->AutoSize = true;
		this->labelLineFake->Location = System::Drawing::Point(837, 42);
		this->labelLineFake->Name = L"labelLineFake";
		this->labelLineFake->Size = System::Drawing::Size(59, 13);
		this->labelLineFake->TabIndex = 40;
		this->labelLineFake->Text = L"Line: None";
		this->labelLineFake->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
		// 
		// MainForm
		// 
		this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
		this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
		this->ClientSize = System::Drawing::Size(1320, 634);
		this->Controls->Add(this->labelLineFake);
		this->Controls->Add(this->labelLine);
		this->Controls->Add(this->label3dZFake);
		this->Controls->Add(this->label3dYFake);
		this->Controls->Add(this->label3dXFake);
		this->Controls->Add(this->labelDVL);
		this->Controls->Add(this->label3dZ);
		this->Controls->Add(this->label3dY);
		this->Controls->Add(this->label3dX);
		this->Controls->Add(this->labelMouseY);
		this->Controls->Add(this->labelMouseX);
		this->Controls->Add(this->buttonJetsonOn);
		this->Controls->Add(this->labelDVLon);
		this->Controls->Add(this->buttonDVLno);
		this->Controls->Add(this->buttonDVLyes);
		this->Controls->Add(this->buttonDVLon);
		this->Controls->Add(this->label3);
		this->Controls->Add(this->label2);
		this->Controls->Add(this->label1);
		this->Controls->Add(this->zCooText);
		this->Controls->Add(this->yCooText);
		this->Controls->Add(this->xCooText);
		this->Controls->Add(this->goToButton);
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
		this->ResumeLayout(false);
		this->PerformLayout();

	}

#pragma endregion
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
				display(video.getEditedFrame());
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
	// Convert String to Char
	private: char* ConvertString2Char(System::String^ str) { // Marshal method
		char* str2 = (char*)(void*)Marshal::StringToHGlobalAnsi(str);
		return str2;
	}
	// Convert Mat to Bitmap
	private: System::Drawing::Bitmap^ ConvertMat2Bitmap(Mat img) {
		Bitmap^ newBitmap = gcnew System::Drawing::Bitmap(img.size().width,
			img.size().height,
			img.step,
			System::Drawing::Imaging::PixelFormat::Format24bppRgb,
			(IntPtr)img.data);
		return newBitmap;
	}
	// Display a cv::Mat in the picture box
	private: void display(Mat img) {
		if (!img.empty()) {
			ptbSource->Image = ConvertMat2Bitmap(img); // Refresh the image on the Windows application
			//ptbSource->Refresh(); // If streaming, freeze
		}
	}
	// Double click event on the Image
	private: System::Void ptbSource_MouseDoubleClick(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e) {
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
		//	display(img);
		//}
		//// Dezoom
		//else if (mouse_click == 'R') {
		//	if(edited_img.empty())
		//		img = original_img.clone();
		//	else
		//		img = edited_img.clone();
		//	display(img);
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
		display(img);
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
		display(video.getEditedFrame());
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
			display(video.getEditedFrame());
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
		display(video.getEditedFrame());
	}
	// Set the video label
	private: void set_video_label() {
		int displayed_label_frame = video.getFrameId() + 1;
		string s = std::to_string(displayed_label_frame + 1) + "/" + std::to_string(this->video_trackBar->Maximum + 1);
		System::String^ name = gcnew System::String(s.c_str());
		this->video_label->Text = name;
	}

		/* --------------------------------------------------
			SERVEUR

			Server button
			Background worker

		----------------------------------------------------- */
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
			output_video.open(vid_path, VideoWriter::fourcc('a', 'v', 'c', '1'), 15, cv::Size(640, 480), true);
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

	/*----------------------------------------------------------------------------------------------------
			Send command to robot:
				Request: "*xxxxxx;xxxxxx;xxxxxx;xxxxxx#
								id  ;   x  ;   y  ;   z
	------------------------------------------------------------------------------------------------------*/

	// Should be replace with the new robot class
	private: System::Void goToButton_Click(System::Object^ sender, System::EventArgs^ e) {
		char answer[100]; // Robot answer (should repeat the received position
		string x_str, y_str, z_str;
		x_str = ConvertString2Char(xCooText->Text);
		y_str = ConvertString2Char(yCooText->Text);
		z_str = ConvertString2Char(zCooText->Text);
		string str2send = "*" + to_string(request_id) + ";" + x_str + ";" + y_str + ";" + z_str + "#";
		request_id++;
		log("Request from GUI: " + str2send);
		//robot_request_socket.sendTo(str2send.c_str(), 100, ip_robot, request_port);
		//robot_request_socket.recvFrom(answer, 100, ip_robot, request_port);
		log("Answer from robot: " + str2send);
	}
	// Reinit text box on click
	private: System::Void xCooText_Click(System::Object^ sender, System::EventArgs^ e) {
		xCooText->Text = "";
	}
	private: System::Void yCooText_Click(System::Object^ sender, System::EventArgs^ e) {
		yCooText->Text = "";
	}
	private: System::Void zCooText_Click(System::Object^ sender, System::EventArgs^ e) {
		zCooText->Text = "";
	}

	/*----------------------------------------------------------------------------------------------------
			Background workers:
				- Arduino
				- DVL
				- Robot
				- Robot command
				- Jetson
	------------------------------------------------------------------------------------------------------*/

	// Check if Arduino has started
	private: System::Void backgroundWorkerArduinoStarted_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		arduino.rcvAny();
		backgroundWorkerArduinoStarted->CancelAsync();
		e->Cancel = true;
	}

	// Receive data from robot continuously
	private: System::Void backgroundWorkerRobot_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		while (true) {
			robot.rcvData();
		}
	}
	// Receive data from Arduino continuously
	private: System::Void backgroundWorkerArduino_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		while (true) {
			arduino.rcvData();
		}
	}
	// Receive data from DVL continuously
	private: System::Void backgroundWorkerDVL_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		while (true) {
			dvl.rcvData();
		}
	}
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

				if (draw_measure) {
					if (measure.a.x > 0 && measure.a.y > 0 && measure.b.x > 0 && measure.b.y)
						cv::line(stream_frame, measure.a, measure.b, cv::Scalar(0, 0, 255), 1);
					cv::circle(stream_frame, measure.a, 3, cv::Scalar(0, 0, 255), -1);
					cv::circle(stream_frame, measure.b, 3, cv::Scalar(0, 0, 255), -1);
				}

				// Display the stream
				if (camera_show)
					if (!(stream_frame.size().width == 0))
						display(stream_frame);

			}
			stream_frame.release();
			free(long_buffer);
		}
	}

	// Send a command to the robot
	private: System::Void backgroundWorkerRobotCommand_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		robot.goTo(0, 0, 0, 0);
		// Stop background worker when finished
		backgroundWorkerRobotCommand->CancelAsync();
		e->Cancel = true;
	}

	// Start DVL
	private: System::Void buttonDVLon_Click(System::Object^ sender, System::EventArgs^ e) {
		if (this->buttonDVLon->Text == L"DVL on") {
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
	}
	private: System::Void buttonDVLno_Click(System::Object^ sender, System::EventArgs^ e) {
		this->labelDVLon->Visible = false;
		this->buttonDVLyes->Visible = false;
		this->buttonDVLno->Visible = false;
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
			catch(...) {
				log("Error while stop recording");
			}
			backgroundWorkerJetsonOff->RunWorkerAsync(1);
		}
	}

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

	// Enable Jetson and DVL starting
	private: System::Void backgroundWorkerArduinoStarted_RunWorkerCompleted(System::Object^ sender, System::ComponentModel::RunWorkerCompletedEventArgs^ e) {
		this->buttonJetsonOn->Text = L"Jetson on";
		this->buttonJetsonOn->Enabled = true;
		this->buttonDVLon->Text = L"DVL on";
		this->buttonDVLon->Enabled = true;
	}

	public: float getPrecision(float number, int precision) {
		std::stringstream ss;
		ss << std::fixed << std::setprecision(2) << number;
		std::string number_str = ss.str();
		float new_number = std::stof(number_str);
		return new_number;
	}

	// Display mouse coordinate when moved on the picture box
	private: System::Void ptbSource_MouseMove(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e) {
		int u = e->X - camera_params.center.x;
		int v = e->Y - camera_params.center.y;

		float d = 1.0;

		float pos_3d_x = u * d / camera_params.fx;
		float pos_3d_y = v * d / camera_params.fy;
		float pos_3d_z = d;

		this->labelMouseX->Text = "u: " + u.ToString() + " p";
		this->labelMouseY->Text = "v: " + v.ToString() + " p";

		this->label3dX->Text = "x: " + getPrecision(pos_3d_x, 3) + " m";
		this->label3dY->Text = "y: " + getPrecision(pos_3d_y, 3) + " m";
		this->label3dZ->Text = "z: " + getPrecision(pos_3d_z, 3) + " m";

		// Fake
		float* distances = dvl.getDistances();
		d = distances[0];

		pos_3d_x = u * d / camera_params.fx;
		pos_3d_y = v * d / camera_params.fy;
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
			int u = measure.a.x - camera_params.center.x;
			int v = measure.a.y - camera_params.center.y;
			cv::Point3d p1 = cv::Point3d(u * d / camera_params.fx, v * d / camera_params.fy, d);
			u = measure.b.x - camera_params.center.x;
			v = measure.b.y - camera_params.center.y;
			cv::Point3d p2 = cv::Point3d(u * d / camera_params.fx, v * d / camera_params.fy, d);
			double result = cv::norm(p1 - p2);
			this->labelLine->Text = "Line: " + getPrecision(result, 3) + " m";

			// Fake
			float* distances = dvl.getDistances();
			d = distances[0];
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

};
}