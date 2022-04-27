#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"

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

	Mat src;

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
			this->button_Edition->Location = System::Drawing::Point(1229, 36);
			this->button_Edition->Name = L"button_Edition";
			this->button_Edition->Size = System::Drawing::Size(75, 23);
			this->button_Edition->TabIndex = 0;
			this->button_Edition->Text = L"Edition";
			this->button_Edition->UseVisualStyleBackColor = true;
			this->button_Edition->Click += gcnew System::EventHandler(this, &MainForm::Edition_Click);
			// 
			// ptbSource
			// 
			this->ptbSource->Location = System::Drawing::Point(704, 65);
			this->ptbSource->Name = L"ptbSource";
			this->ptbSource->Size = System::Drawing::Size(600, 600);
			this->ptbSource->TabIndex = 1;
			this->ptbSource->TabStop = false;
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
			this->ClientSize = System::Drawing::Size(1316, 677);
			this->Controls->Add(this->button_Browse);
			this->Controls->Add(this->ptbSource);
			this->Controls->Add(this->button_Edition);
			this->Name = L"MainForm";
			this->Text = L"MainForm";
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ptbSource))->EndInit();
			this->ResumeLayout(false);

		}
#pragma endregion
	private: System::Void Edition_Click(System::Object^ sender, System::EventArgs^ e) {
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
		src = imread(ConvertString2Char(dgOpen->FileName));

		// Resize the image to its placeholder dimensions
		resize(src, src, cv::Size(600, 600), INTER_CUBIC);
		//original_src = src;

		// Convert the image into bitmap
		ptbSource->Image = ConvertMat2Bitmap(src);
		ptbSource->Refresh();
	}

	// Convert String to Char
	private: char* ConvertString2Char(System::String^ str) { // Marshal method
		char* str2 = (char*)(void*)Marshal::StringToHGlobalAnsi(str);
		return str2;
	}

	// Convert Mat to Bitmap
	private: System::Drawing::Bitmap^ ConvertMat2Bitmap(Mat img) {
		Bitmap^ newBitmap = gcnew System::Drawing::Bitmap(src.size().width,
			src.size().height,
			src.step,
			System::Drawing::Imaging::PixelFormat::Format24bppRgb,
			(IntPtr)src.data);
		return newBitmap;
	}
	};
}
