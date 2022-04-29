#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "Bbx.h"
#include "Bboxes.h"
#include "Video.h"

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

	// Image in picture box
	Mat img;
	Mat original_img;
	Mat edited_img;

	// Video in picture box
	Mat frame;
	string play_video = "pause"; // or "play" or "replay"

	Video video;


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


	private: System::ComponentModel::IContainer^ components;

	protected:

	protected:

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
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ptbSource))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->video_trackBar))->BeginInit();
			this->SuspendLayout();
			// 
			// button_Edition
			// 
			this->button_Edition->Location = System::Drawing::Point(1617, 31);
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
			this->ptbSource->Location = System::Drawing::Point(792, 60);
			this->ptbSource->Name = L"ptbSource";
			this->ptbSource->Size = System::Drawing::Size(900, 900);
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
			this->listView1->Location = System::Drawing::Point(525, 60);
			this->listView1->Name = L"listView1";
			this->listView1->Size = System::Drawing::Size(250, 400);
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
			this->view_button->Location = System::Drawing::Point(611, 466);
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
			this->play_button->Location = System::Drawing::Point(872, 966);
			this->play_button->Name = L"play_button";
			this->play_button->Size = System::Drawing::Size(75, 23);
			this->play_button->TabIndex = 5;
			this->play_button->Text = L"Play";
			this->play_button->UseVisualStyleBackColor = true;
			this->play_button->Click += gcnew System::EventHandler(this, &MainForm::play_button_Click);
			// 
			// speed_button
			// 
			this->speed_button->Location = System::Drawing::Point(953, 966);
			this->speed_button->Name = L"speed_button";
			this->speed_button->Size = System::Drawing::Size(75, 23);
			this->speed_button->TabIndex = 6;
			this->speed_button->Text = L"x2";
			this->speed_button->UseVisualStyleBackColor = true;
			this->speed_button->Click += gcnew System::EventHandler(this, &MainForm::speed_button_Click);
			// 
			// load_button
			// 
			this->load_button->Location = System::Drawing::Point(791, 966);
			this->load_button->Name = L"load_button";
			this->load_button->Size = System::Drawing::Size(75, 23);
			this->load_button->TabIndex = 7;
			this->load_button->Text = L"Load";
			this->load_button->UseVisualStyleBackColor = true;
			this->load_button->Click += gcnew System::EventHandler(this, &MainForm::load_button_Click);
			// 
			// video_trackBar
			// 
			this->video_trackBar->Location = System::Drawing::Point(1034, 966);
			this->video_trackBar->Maximum = 150;
			this->video_trackBar->Name = L"video_trackBar";
			this->video_trackBar->Size = System::Drawing::Size(565, 45);
			this->video_trackBar->TabIndex = 8;
			this->video_trackBar->Scroll += gcnew System::EventHandler(this, &MainForm::video_trackBar_Scroll);
			// 
			// video_label
			// 
			this->video_label->AutoSize = true;
			this->video_label->Location = System::Drawing::Point(1605, 971);
			this->video_label->Name = L"video_label";
			this->video_label->Size = System::Drawing::Size(24, 13);
			this->video_label->TabIndex = 9;
			this->video_label->Text = L"0/0";
			// 
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1904, 1041);
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
		bool close_window = false; // Bool to close the windows when hitting the close button
		edited_img = original_img.clone(); // Dezoom the image before editing
		while (true) {
			// Close the window on close button
			close_window = getWindowProperty("Edition", WND_PROP_VISIBLE) < 1;
			if (close_window) {
				img = edited_img.clone();
				ptbSource->Image = ConvertMat2Bitmap(img); // Refresh the image on the Windows application
				ptbSource->Refresh();
				destroyAllWindows();
				break;
			}
			edited_img = original_img.clone(); // Refresh the img
			bboxes.update(x_mouse, y_mouse, &click, &hold);
			bboxes.draw(edited_img);
			imshow("Edition", edited_img);
			if (mouse_right_click) {
				cv::Point center(x_mouse, y_mouse);
				bboxes.add(center);
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

	// Double click event on the Image
	private: System::Void ptbSource_MouseDoubleClick(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e) {
		if (mouse_click == 'L') {
			int ptb_width  = ptbSource->Size.Width;
			int ptb_height = ptbSource->Size.Height;
			// Zomm
			int zoom = 500;
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
			ptbSource->Image = ConvertMat2Bitmap(img); // Refresh the image on the Windows application
			ptbSource->Refresh();
		}
		else if (mouse_click == 'R') {
			// Dezoom
			if(edited_img.empty())
				img = original_img.clone();
			else
				img = edited_img.clone();
			ptbSource->Image = ConvertMat2Bitmap(img); // Refresh the image on the Windows application
			ptbSource->Refresh();
		}
	}

	// Get mouse state when clicked
	private: System::Void ptbSource_MouseDown(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e) {
		//cout << e->X << endl;
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
		ptbSource->Image = ConvertMat2Bitmap(img); // Refresh the image on the Windows application
		ptbSource->Refresh();
	}

	// Load the video
	private: System::Void load_button_Click(System::Object^ sender, System::EventArgs^ e) {
		// Path to change latter
		std::string video_path = "C:/Users/Utilisateur/Videos/test.mp4";
		video.init(video_path);
		// Open the video file
		cap.open(video_path);
		if (!cap.isOpened()) // TODO: Manage this error
			cout << "Error opening video stream or file" << endl;
		// Set the track bar step number
		this->video_trackBar->Maximum = frames_nr;
		// Set fvideo label
		set_video_label();
		// Read the first frame
		frame = video.nextFrame();
		// Display on the picture box
		ptbSource->Image = ConvertMat2Bitmap(frame); // Refresh the image on the Windows application
		ptbSource->Refresh();

		
	}

	// Play or pause the video
	private: System::Void play_button_Click(System::Object^ sender, System::EventArgs^ e) {
		if (play_video == "play") {
			this->play_button->Text = L"Play";
			play_video = "pause";
		}	
		else if (play_video == "pause") {
			this->play_button->Text = L"Pause";
			play_video = "play";
		}
		else if (play_video == "replay") {
			video.setFrame(0);
			this->video_trackBar->Value = video.getFrameId();
			set_video_label();
			this->play_button->Text = L"Pause";
			play_video = "play";
		}
		while (play_video == "play") {
			// Set the track bar to the current frame
			this->video_trackBar->Value = video.getFrameId();
			// Read next frame
			frame = video.nextFrame();
			// If the frame is empty, break immediately
			if (frame.empty()) {
				this->play_button->Text = L"Replay";
				play_video = "replay";
				break;
			}
			else
				set_video_label();
			// Display on the picture box
			ptbSource->Image = ConvertMat2Bitmap(frame); // Refresh the image on the Windows application
			ptbSource->Refresh();
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
		if (play_video == "replay")
			if (video.getFrameId() < video.getFramesNr()) {
				play_video = "pause";
				this->play_button->Text = L"Play";
			}
		video.setFrame(video_trackBar->Value - 1);
		// Set current frame label
		set_video_label();
		frame = video.nextFrame();
		ptbSource->Image = ConvertMat2Bitmap(frame); // Refresh the image on the Windows application
		ptbSource->Refresh();
	}

	// Set the video label
	private: void set_video_label() {
		int displayed_label_frame = video.getFrameId();
		if (displayed_label_frame + 1 > video.getFramesNr())
			displayed_label_frame = displayed_label_frame - 1;
		string s = std::to_string(displayed_label_frame + 1) + "/" + std::to_string(video.getFramesNr());
		System::String^ name = gcnew System::String(s.c_str());
		this->video_label->Text = name;
	}
};
}
