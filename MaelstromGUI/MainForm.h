#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "Bbx.h"
#include "Bboxes.h"

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
			this->Related = (gcnew System::Windows::Forms::ColumnHeader());
			this->view_button = (gcnew System::Windows::Forms::Button());
			this->imageList1 = (gcnew System::Windows::Forms::ImageList(this->components));
			this->columnHeader1 = (gcnew System::Windows::Forms::ColumnHeader());
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
			// columnHeader1
			// 
			this->columnHeader1->Text = L"Related frames";
			// 
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1904, 1041);
			this->Controls->Add(this->view_button);
			this->Controls->Add(this->listView1);
			this->Controls->Add(this->button_Browse);
			this->Controls->Add(this->ptbSource);
			this->Controls->Add(this->button_Edition);
			this->Name = L"MainForm";
			this->Text = L"Maelstrom GUI";
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ptbSource))->EndInit();
			this->ResumeLayout(false);

		}
#pragma endregion
	public: void load() {
		listView1->Columns->Add("Related frames", 200);
	}

	// Bbx edition
	private: System::Void Edition_Click(System::Object^ sender, System::EventArgs^ e) {
		System::String^ message = "Edition mode:\n\tRight clik to create a bounding box\n\tLeft click to modify";
		MessageBox::Show(message);
		namedWindow("Edition");
		setMouseCallback("Edition", mouse_callback);
		bool close_window = false; // Bool to close the windows when hitting the close button
		Mat edited_img = original_img.clone(); // Dezoom the image before editing
		while (true) {
			// Close the window on close button
			close_window = getWindowProperty("Edition", WND_PROP_VISIBLE) < 1;
			if (close_window) {
				//original_img = edited_img.clone();
				//img = edited_img.clone();
				ptbSource->Image = ConvertMat2Bitmap(edited_img); // Refresh the image on the Windows application
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
				// string to system string
				//System::String^ name = gcnew System::String(s.c_str());
				//label1->Text = name;
				mouse_right_click = false;
			}
			//ptbSource->Image = ConvertMat2Bitmap(edited_img); // Refresh the image on the Windows application
			//ptbSource->Refresh();
			waitKey(10);
		}
		//ptbSource->Image = ConvertMat2Bitmap(edited_img); // Refresh the image on the Windows application
		//ptbSource->Refresh();
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
		//MessageBox::Show(selected);
	}
};
}
