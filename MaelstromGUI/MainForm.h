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

	// GUI info
	string ip_gui = "192.168.0.11";

	// Jetson info
	string ip_jetson = "192.168.0.20";
	unsigned short jetson_port = 8000;

	// Robot info
	string ip_robot = "192.168.0.10";
	unsigned short request_port = 10000; // Port to send data like position
	int request_id = 1;
	unsigned short continuous_port = 10001; // Port to receive data like position
	
	// Arduino info
	string ip_arduino = "192.168.0.254";
	unsigned short arduino_port = 5817;

	// Server Jetson
	Server server_jetson;
	bool server_jetson_initialized = false;
	bool server_jetson_is_running = false; // Set to true if running, false if error or shuted down
	bool camera_show = false; // To start or shutdown with the same button

	//Test server Jetson
	//UDPSocket jetson_socket(ip_gui, jetson_port);
	
	// Server Robot
	// To send request to the robot controler
	//UDPSocket robot_request_socket(ip_gui, request_port);
	// To receive robot information continuously 
	//UDPSocket robot_info_socket(ip_gui, continuous_port);

	// Server Arduino
	// To receive arduino information continuously 
	//UDPSocket arduino_info_socket(ip_gui, arduino_port); // Is it needed?

	Arduino arduino;

	DVL dvl;

	Robot robot;

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
			// To receive continuous data from the arduino
			backgroundWorkerArduino->RunWorkerAsync(1);
			// To receive continuous data from the jetson
			//backgroundWorkerJetson->RunWorkerAsync(1);
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
	private: System::Windows::Forms::Button^ server_button;
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
			this->server_button = (gcnew System::Windows::Forms::Button());
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
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ptbSource))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->video_trackBar))->BeginInit();
			this->SuspendLayout();
			// 
			// button_Edition
			// 
			this->button_Edition->Location = System::Drawing::Point(1106, 25);
			this->button_Edition->Name = L"button_Edition";
			this->button_Edition->Size = System::Drawing::Size(75, 23);
			this->button_Edition->TabIndex = 0;
			this->button_Edition->Text = L"Edition";
			this->button_Edition->UseVisualStyleBackColor = true;
			this->button_Edition->Click += gcnew System::EventHandler(this, &MainForm::Edition_Click);
			// 
			// ptbSource
			// 
			this->ptbSource->BackColor = System::Drawing::SystemColors::ControlDark;
			this->ptbSource->Location = System::Drawing::Point(541, 55);
			this->ptbSource->Name = L"ptbSource";
			this->ptbSource->Size = System::Drawing::Size(640, 640);
			this->ptbSource->TabIndex = 1;
			this->ptbSource->TabStop = false;
			this->ptbSource->MouseDoubleClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::ptbSource_MouseDoubleClick);
			this->ptbSource->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::ptbSource_MouseDown);
			// 
			// button_Browse
			// 
			this->button_Browse->Location = System::Drawing::Point(54, 25);
			this->button_Browse->Name = L"button_Browse";
			this->button_Browse->Size = System::Drawing::Size(80, 34);
			this->button_Browse->TabIndex = 2;
			this->button_Browse->Text = L"Browse";
			this->button_Browse->UseVisualStyleBackColor = true;
			this->button_Browse->Click += gcnew System::EventHandler(this, &MainForm::button_Browse_Click);
			// 
			// listView1
			// 
			this->listView1->Columns->AddRange(gcnew cli::array< System::Windows::Forms::ColumnHeader^  >(1) { this->columnHeader1 });
			this->listView1->HideSelection = false;
			this->listView1->Location = System::Drawing::Point(302, 55);
			this->listView1->Name = L"listView1";
			this->listView1->Size = System::Drawing::Size(201, 370);
			this->listView1->TabIndex = 3;
			this->listView1->UseCompatibleStateImageBehavior = false;
			this->listView1->MouseClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::listView1_MouseClick);
			// 
			// columnHeader1
			// 
			this->columnHeader1->Text = L"Related frames";
			// 
			// view_button
			// 
			this->view_button->Location = System::Drawing::Point(54, 107);
			this->view_button->Name = L"view_button";
			this->view_button->Size = System::Drawing::Size(80, 34);
			this->view_button->TabIndex = 4;
			this->view_button->Text = L"Click";
			this->view_button->UseVisualStyleBackColor = true;
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
			this->play_button->Location = System::Drawing::Point(621, 707);
			this->play_button->Name = L"play_button";
			this->play_button->Size = System::Drawing::Size(75, 23);
			this->play_button->TabIndex = 5;
			this->play_button->Text = L"Play";
			this->play_button->UseVisualStyleBackColor = true;
			this->play_button->Click += gcnew System::EventHandler(this, &MainForm::play_button_Click);
			// 
			// speed_button
			// 
			this->speed_button->Location = System::Drawing::Point(702, 707);
			this->speed_button->Name = L"speed_button";
			this->speed_button->Size = System::Drawing::Size(75, 23);
			this->speed_button->TabIndex = 6;
			this->speed_button->Text = L"x2";
			this->speed_button->UseVisualStyleBackColor = true;
			this->speed_button->Click += gcnew System::EventHandler(this, &MainForm::speed_button_Click);
			// 
			// load_button
			// 
			this->load_button->Location = System::Drawing::Point(540, 707);
			this->load_button->Name = L"load_button";
			this->load_button->Size = System::Drawing::Size(75, 23);
			this->load_button->TabIndex = 7;
			this->load_button->Text = L"Load";
			this->load_button->UseVisualStyleBackColor = true;
			this->load_button->Click += gcnew System::EventHandler(this, &MainForm::load_button_Click);
			// 
			// video_trackBar
			// 
			this->video_trackBar->Location = System::Drawing::Point(783, 707);
			this->video_trackBar->Maximum = 150;
			this->video_trackBar->Name = L"video_trackBar";
			this->video_trackBar->Size = System::Drawing::Size(365, 45);
			this->video_trackBar->TabIndex = 8;
			this->video_trackBar->Scroll += gcnew System::EventHandler(this, &MainForm::video_trackBar_Scroll);
			// 
			// video_label
			// 
			this->video_label->AutoSize = true;
			this->video_label->Location = System::Drawing::Point(1154, 712);
			this->video_label->Name = L"video_label";
			this->video_label->Size = System::Drawing::Size(24, 13);
			this->video_label->TabIndex = 9;
			this->video_label->Text = L"0/0";
			// 
			// server_button
			// 
			this->server_button->BackColor = System::Drawing::SystemColors::ControlLight;
			this->server_button->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->server_button->Location = System::Drawing::Point(423, 701);
			this->server_button->Name = L"server_button";
			this->server_button->Size = System::Drawing::Size(80, 34);
			this->server_button->TabIndex = 10;
			this->server_button->Text = L"Start recording";
			this->server_button->UseVisualStyleBackColor = false;
			this->server_button->Click += gcnew System::EventHandler(this, &MainForm::server_button_Click);
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
			this->goToButton->Location = System::Drawing::Point(54, 701);
			this->goToButton->Name = L"goToButton";
			this->goToButton->Size = System::Drawing::Size(75, 23);
			this->goToButton->TabIndex = 15;
			this->goToButton->Text = L"Go to";
			this->goToButton->UseVisualStyleBackColor = true;
			this->goToButton->Click += gcnew System::EventHandler(this, &MainForm::goToButton_Click);
			// 
			// xCooText
			// 
			this->xCooText->Location = System::Drawing::Point(145, 704);
			this->xCooText->Name = L"xCooText";
			this->xCooText->Size = System::Drawing::Size(68, 20);
			this->xCooText->TabIndex = 16;
			this->xCooText->Text = L"- - -";
			this->xCooText->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->xCooText->Click += gcnew System::EventHandler(this, &MainForm::xCooText_Click);
			// 
			// yCooText
			// 
			this->yCooText->Location = System::Drawing::Point(219, 704);
			this->yCooText->Name = L"yCooText";
			this->yCooText->Size = System::Drawing::Size(68, 20);
			this->yCooText->TabIndex = 17;
			this->yCooText->Text = L"- - -";
			this->yCooText->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->yCooText->Click += gcnew System::EventHandler(this, &MainForm::yCooText_Click);
			// 
			// zCooText
			// 
			this->zCooText->Location = System::Drawing::Point(293, 704);
			this->zCooText->Name = L"zCooText";
			this->zCooText->Size = System::Drawing::Size(68, 20);
			this->zCooText->TabIndex = 18;
			this->zCooText->Text = L"- - -";
			this->zCooText->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->zCooText->Click += gcnew System::EventHandler(this, &MainForm::zCooText_Click);
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(176, 727);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(37, 13);
			this->label1->TabIndex = 19;
			this->label1->Text = L"x (mm)";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(250, 727);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(37, 13);
			this->label2->TabIndex = 20;
			this->label2->Text = L"y (mm)";
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(324, 727);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(37, 13);
			this->label3->TabIndex = 21;
			this->label3->Text = L"z (mm)";
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
			this->buttonDVLon->Location = System::Drawing::Point(41, 236);
			this->buttonDVLon->Name = L"buttonDVLon";
			this->buttonDVLon->Size = System::Drawing::Size(80, 34);
			this->buttonDVLon->TabIndex = 24;
			this->buttonDVLon->Text = L"DVL on";
			this->buttonDVLon->UseVisualStyleBackColor = true;
			this->buttonDVLon->Click += gcnew System::EventHandler(this, &MainForm::buttonDVLon_Click);
			// 
			// buttonDVLyes
			// 
			this->buttonDVLyes->Location = System::Drawing::Point(144, 249);
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
			this->buttonDVLno->Location = System::Drawing::Point(210, 249);
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
			this->labelDVLon->Location = System::Drawing::Point(127, 233);
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
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1208, 1041);
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
			this->Controls->Add(this->server_button);
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
		// Zoom
		if (mouse_click == 'L') {
			int zoom = 500;
			int ptb_width  = ptbSource->Size.Width;
			int ptb_height = ptbSource->Size.Height;
			int x1 = int(mouse_pos.x - zoom / 2);
			int y1 = int(mouse_pos.y - zoom / 2);
			if (x1 + zoom > ptb_width) {
				int delta = x1 + zoom - ptb_width;
				x1 -= delta;
			}
			else if (x1 < 0)
				x1 = 0;
			if (y1 + zoom > ptb_height) {
				int delta = y1 + zoom - ptb_height;
				y1 -= delta;
			}
			else if (y1 < 0)
				y1 = 0;
			cv::Rect myROI(x1, y1, zoom, zoom);
			img = img(myROI);
			resize(img, img, cv::Size(ptb_width, ptb_height), INTER_CUBIC);
			display(img);
		}
		// Dezoom
		else if (mouse_click == 'R') {
			if(edited_img.empty())
				img = original_img.clone();
			else
				img = edited_img.clone();
			display(img);
		}
	}
	// Get mouse state when clicked
	private: System::Void ptbSource_MouseDown(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e) {
		mouse_pos = cv::Point(e->X, e->Y);
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
		display(img);
	}
	// Load the video
	private: System::Void load_button_Click(System::Object^ sender, System::EventArgs^ e) {
		std::string video_path = "C:/Users/Utilisateur/Videos/test.mp4";
		cv::Size window_size(ptbSource->Size.Width, ptbSource->Size.Height);
		video.init(video_path, window_size);
		if (!video.isOpened()) // TODO: Manage this error
			cout << "Error opening video stream or file" << endl;
		this->video_trackBar->Maximum = video.getFramesNr()-1;
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
			this->video_trackBar->Value = video.getFrameId()+1;
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
	private: System::Void server_button_Click(System::Object^ sender, System::EventArgs^ e) {
		camera_show = !camera_show; // Default is false, fist click launch the server

		if(!server_jetson_initialized) {
			server_jetson.connect("192.168.0.11", 8000);
			server_jetson_initialized = true;
		}
		
		if (camera_show) {

			backgroundWorkerJetson->RunWorkerAsync(1);
			camera_show = true;

			this->server_button->Text = L"Recording";
			this->server_button->BackColor = System::Drawing::Color::Red;
			this->server_button->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));

			// Start video writer
			std::string vid_path = getNewVideoPath();
			output_video.open(vid_path, VideoWriter::fourcc('a', 'v', 'c', '1'), 10, cv::Size(640, 640), true);
			log("From video recording: " + vid_path + " created");
		}
		else {
			camera_show = false;
			this->server_button->Text = L"Start recording";
			this->server_button->BackColor = System::Drawing::SystemColors::Control;
			this->server_button->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
		}		
	}

	public:
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
			
			//string new_path = videos_dir_path + "video_" + getTime() + ".mp4";
			string new_path = videos_dir_path + "video.mp4";
			//string path("D:\\projects\\cyril\\videos/video_");
			//string new_path = path + getTime() + ".mp4";

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
	// Server Jetson
	private: System::Void backgroundWorkerJetson_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		int recvMsgSize;
		int frame_id = 0;

		//int codec = VideoWriter::fourcc('a', 'v', 'c', '1');
		//cv::VideoWriter output_video(getNewVideoPath(), codec, 30, cv::Size(640, 640), true);

		while (true) {

			do {
				recvMsgSize = server_jetson.recv();
			} while (recvMsgSize > sizeof(int));

			int total_pack = ((int*)server_jetson.getBuffer())[0];

			int pack_count = 0;

			char* long_buffer = new char[PACK_SIZE * total_pack];
			for (int i = 0; i < total_pack; i++) {
				recvMsgSize = server_jetson.recv();

				if (recvMsgSize != PACK_SIZE) {
					std::cerr << "Received unexpected size pack:" << recvMsgSize << std::endl;
					log("From video recording: received unexpected size pack:" + std::to_string(recvMsgSize));
					continue;
				}
				else {
					pack_count++;
					memcpy(&long_buffer[i * PACK_SIZE], server_jetson.getBuffer(), PACK_SIZE);
				}
			}

			if (pack_count == total_pack) {
				stream_frame = server_jetson.getFrame(long_buffer, total_pack);
				if (stream_frame.size().width == 0) {
					cerr << "decode failure!" << endl;
					log("From video recording: decode failure");
					continue;
				}

				if (camera_show) {

					if (!(stream_frame.size().width == 0)) {
						display(stream_frame);
						output_video.write(stream_frame);
					}
				}
				else {
					//cv::imwrite(getNewVideoPath(), stream_frame);
					output_video.release();
					stream_frame.release();
					cv::waitKey(1);
					cv:destroyAllWindows();
					backgroundWorkerJetson->CancelAsync();
					e->Cancel = true;
					free(long_buffer);
					break;
				}
			}
			free(long_buffer);
		}
	}
	// Send a command to the robot
	private: System::Void backgroundWorkerRobotCommand_DoWork(System::Object^ sender, System::ComponentModel::DoWorkEventArgs^ e) {
		robot.goTo(0, 0, 0, 0);
		// Stop background worker when finished
		backgroundWorkerJetson->CancelAsync();
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
};
}