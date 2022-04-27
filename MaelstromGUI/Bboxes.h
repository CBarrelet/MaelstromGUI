#pragma once

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "Bbx.h"
#include <vector>
#include <random>

class Bboxes {

public:

	cv::Size img_size;
	std::vector<Bbx> bboxes;
	int count = 0;
	bool one_is_selected = false;

	/*--------------------------------------------------
		Bbx constructor
	----------------------------------------------------*/

	Bboxes(cv::Mat img, std::vector<Bbx> bboxes) {

		// Fill Bbx vector if no Bbx vector is initialized in constructor
		if (bboxes.empty()) {
			for (int i = 0; i < bboxes.size(); i++) {
				this->bboxes.push_back(bboxes[i]);
			}
		}
		this->img_size = img.size();
	}

	void set_img_size(cv::Size img_size) {
		this->img_size = img_size;
	}

	int size() {
		return this->bboxes.size();
	}

	// Return a vector with every bbx pos (x1, y1, x2, y2)
	std::vector<std::vector<cv::Point>> get_bboxes() {
		std::vector<std::vector<cv::Point>> bboxes;
		for (int i = 0; i < size(); i++) {
			if (!this->bboxes[i].removed) {
				std::vector<cv::Point> pos = { this->bboxes[i].p1, this->bboxes[i].p3 };
				bboxes.push_back(pos);
			}
		}
		return bboxes;
	}

	// Add a new bbx from mouse
	void add(cv::Point center) {

		cv::Scalar color = get_random_color(50, 205);
		int id = this->count;
		this->count++;


		std::cout << "NOT from bbx" << std::endl;
		std::cout << center << std::endl;

		Bbx bbx(id, center, this->img_size, color);
		std::cout << "Bbx : " << std::endl;
		std::cout << bbx.p1 << bbx.p3 << bbx.color << std::endl;



		this->bboxes.push_back(bbx);

		for (int i = 0; i < size(); i++) {
			if (!this->bboxes[i].removed) {
				std::cout << "vector bboxes : " << i << std::endl;
				//this->bboxes[i].p3 += cv::Point(20, 20);
				std::cout << this->bboxes[i].p1 << this->bboxes[i].p3 << std::endl;
			}
		}
	}

	void draw(cv::Mat img) {
		for (int i = 0; i < size(); i++) {
			if (!this->bboxes[i].removed) {
				cv::rectangle(img, this->bboxes[i].p1, this->bboxes[i].p3, this->bboxes[i].color, this->bboxes[i].thickness);
			}
		}
	}

private:

	cv::Scalar get_random_color(int low_range = 0, int high_range = 255) {
		std::random_device rd;	// Obtain a random number from hardware
		std::mt19937 gen(rd());	// Seed the generator
		std::uniform_int_distribution<> distr(low_range, high_range); // Define the range
		return cv::Scalar(distr(gen), distr(gen), distr(gen));
	}

};
