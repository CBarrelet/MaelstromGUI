#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "Bbx.h"
#include "Bboxes.h"

int x_mouse = 0, y_mouse = 0; // Mouse coordinates
bool mouse_left_click = false;
char mouse_click = 'L';

void mouse_callback(int  event, int  x, int  y, int  flag, void* param)
{
	if (event == cv::EVENT_MOUSEMOVE) {
		x_mouse = x;
		y_mouse = y;
	}
	else if (event == cv::EVENT_LBUTTONDOWN) {
		mouse_left_click = true;
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

	Mat img;
	Mat original_img;

	Mat null_img = Mat::zeros(cv::Size(1, 1), CV_8UC1);
	vector<Bbx> null_bbx_vector;
	Bboxes bboxes(null_img, null_bbx_vector);

	cv::Point mouse_pos = cv::Point(0, 0);
	bool mouse_left_down = false;

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

	protected:

	protected:

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->button_Edition = (gcnew System::Windows::Forms::Button());
			this->ptbSource = (gcnew System::Windows::Forms::PictureBox());
			this->button_Browse = (gcnew System::Windows::Forms::Button());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ptbSource))->BeginInit();
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
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1904, 1041);
			this->Controls->Add(this->button_Browse);
			this->Controls->Add(this->ptbSource);
			this->Controls->Add(this->button_Edition);
			this->Name = L"MainForm";
			this->Text = L"MainForm";
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ptbSource))->EndInit();
			this->ResumeLayout(false);

		}
#pragma endregion
	// Bbx edition
	private: System::Void Edition_Click(System::Object^ sender, System::EventArgs^ e) {
		namedWindow("Edition");
		setMouseCallback("Edition", mouse_callback);
		bool close_window = false; // Bool to close the windows when hitting the close button
		Mat edited_img = original_img.clone(); // Dezoom the image before editing
		while (true) {
			waitKey(10);
			// Close the window on close button
			close_window = getWindowProperty("Edition", WND_PROP_VISIBLE) < 1;
			if (close_window) {
				original_img = edited_img.clone();
				img = edited_img;
				destroyAllWindows();
				break;
			}
			bboxes.draw(edited_img);
			imshow("Edition", edited_img);
			if (mouse_left_click) {
				cv::Point center(x_mouse, y_mouse);
				bboxes.add(center);
				// string to system string
				//System::String^ name = gcnew System::String(s.c_str());
				//label1->Text = name;
				mouse_left_click = false;
			}
			ptbSource->Image = ConvertMat2Bitmap(edited_img); // Refresh the image on the Windows application
			ptbSource->Refresh();
		}
		ptbSource->Image = ConvertMat2Bitmap(edited_img); // Refresh the image on the Windows application
		ptbSource->Refresh();
	}

	// Load and show image from PC into picture box
	private: System::Void button_Browse_Click(System::Object^ sender, System::EventArgs^ e) {

		OpenFileDialog^ dgOpen = gcnew OpenFileDialog();
		dgOpen->Filter = "Image(*.bmp; *.jpg)|*.bmp;*.jpg|All files (*.*)|*.*||";
		if (dgOpen->ShowDialog() == System::Windows::Forms::DialogResult::Cancel)
		{
			return;
		}
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
			else if (x1 < 0) {
				x1 = 0;
			}
			if (y1 + zoom > ptb_height) {
				int delta = y1 + zoom - ptb_height;
				y1 -= delta;
			}
			else if (y1 < 0) {
				y1 = 0;
			}
			cv::Rect myROI(x1, y1, zoom, zoom);
			img = img(myROI);
			resize(img, img, cv::Size(ptb_width, ptb_height), INTER_CUBIC);
			ptbSource->Image = ConvertMat2Bitmap(img); // Refresh the image on the Windows application
			ptbSource->Refresh();
		}
		else if (mouse_click == 'R') {
			// Dezoom
			img = original_img;
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

};
}
