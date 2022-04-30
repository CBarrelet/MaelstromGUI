#pragma once

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "Bbx.h"
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>

class Bboxes {

public:

	cv::Size img_size;
	std::vector<Bbx> bboxes;
	int count = 0;
	bool one_is_selected = false;

	bool move_from_up = false;
	bool move_from_left = false;
	bool move_from_right = false;

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
		Bbx bbx(id, center, this->img_size, color);
		this->bboxes.push_back(bbx);
	}

	cv::Mat draw(cv::Mat img) {
		for (int i = 0; i < size(); i++) {
			
			if (!this->bboxes[i].removed) {
				// Actual bbx
				cv::rectangle(img, this->bboxes[i].p1, this->bboxes[i].p3, this->bboxes[i].color, this->bboxes[i].thickness);

				if (this->bboxes[i].selected && !this->one_is_selected) {
					cv::circle(img, this->bboxes[i].p1, this->bboxes[i].corner_radius, this->bboxes[i].color, 2);
					cv::circle(img, this->bboxes[i].p3, this->bboxes[i].corner_radius, this->bboxes[i].color, 2);
					cv::circle(img, this->bboxes[i].p4, this->bboxes[i].corner_radius, this->bboxes[i].color, 2);

					// Cancel circle
					cv::circle(img, this->bboxes[i].p2, this->bboxes[i].corner_radius + 1, cv::Scalar(0, 0, 255), -1);
					cv::circle(img, this->bboxes[i].p2, this->bboxes[i].corner_radius + 1, cv::Scalar(0, 0, 0), 1);

					// Cancel cross
					int line_size = 4;
					cv::Point p1_cancel = cv::Point(this->bboxes[i].p2) + cv::Point(-line_size, -line_size);
					cv::Point p2_cancel = cv::Point(this->bboxes[i].p2) + cv::Point(line_size, line_size);
					cv::Point p3_cancel = cv::Point(this->bboxes[i].p2) + cv::Point(-line_size, line_size);
					cv::Point p4_cancel = cv::Point(this->bboxes[i].p2) + cv::Point(line_size, -line_size);
					cv::line(img, p1_cancel, p2_cancel, cv::Scalar(0, 0, 0), 2);
					cv::line(img, p3_cancel, p4_cancel, cv::Scalar(0, 0, 0), 2);
				}
			}
		}
		return img;
	}

	void update(int x_mouse, int y_mouse, bool* click, bool* hold) {
		for (int i = 0; i < size(); i++) {
			
			if (!this->bboxes[i].removed) {

				bool selected = this->bboxes[i].is_selected(x_mouse, y_mouse);

				if (!this->one_is_selected) {
					if (this->bboxes[i].selected || this->bboxes[i].held) {
						this->bboxes[i].color = this->bboxes[i].selection_color;
					}
					else {
						this->bboxes[i].color = this->bboxes[i].original_color;
					}
				}

				if ((this->bboxes[i].is_selected(x_mouse, y_mouse) || this->bboxes[i].held) && !this->bboxes[i].removed) {

					if (*click &&(this->bboxes[i].is_corner_selected(x_mouse, y_mouse, this->bboxes[i].p1) ||
									this->bboxes[i].is_corner_selected(x_mouse, y_mouse, this->bboxes[i].p3) ||
									this->bboxes[i].is_corner_selected(x_mouse, y_mouse, this->bboxes[i].p4)) && 
									!this->bboxes[i].removed) {
						this->bboxes[i].corner_clicked = true;
					}
					if (*click && !this->bboxes[i].is_corner_selected(x_mouse, y_mouse, this->bboxes[i].p1) &&
									!this->bboxes[i].is_corner_selected(x_mouse, y_mouse, this->bboxes[i].p2) &&
									!this->bboxes[i].is_corner_selected(x_mouse, y_mouse, this->bboxes[i].p3) &&
									!this->bboxes[i].is_corner_selected(x_mouse, y_mouse, this->bboxes[i].p4) &&
									!this->bboxes[i].removed) {
						this->bboxes[i].clicked = true;
						if (this->bboxes[i].p1_mouse_offset == cv::Point(0, 0)) {
							this->bboxes[i].p1_mouse_offset = cv::Point(x_mouse - this->bboxes[i].p1.x, y_mouse - this->bboxes[i].p1.y);
						}
						
					}
					// Remove bbx ?
					if (*click && this->bboxes[i].is_corner_selected(x_mouse, y_mouse, this->bboxes[i].p2)) {
						this->bboxes[i].removed_clicked = true;
						this->bboxes[i].removed = true;
					}

					if (*hold) {

						// Modify from p3
						if ((this->bboxes[i].p3_selected || this->bboxes[i].held) && 
								!this->move_from_left && !this->move_from_up && 
								this->bboxes[i].corner_clicked && !this->bboxes[i].removed) {

							this->move_from_right = true;
							this->one_is_selected = true;
							if (this->bboxes[i].selected || this->bboxes[i].held) {
								this->bboxes[i].color = this->bboxes[i].selection_color;
							}
							else {
								this->bboxes[i].color = this->bboxes[i].original_color;
							}
							this->bboxes[i].held = true;
							this->bboxes[i].p1 = this->bboxes[i].p1;
							this->bboxes[i].p2 = cv::Point(x_mouse, this->bboxes[i].p2.y);
							this->bboxes[i].p3 = cv::Point(x_mouse, y_mouse);
							this->bboxes[i].p4 = cv::Point(this->bboxes[i].p4.x, y_mouse);
						}

						// Modify from p4
						if ((this->bboxes[i].p4_selected || this->bboxes[i].held) &&
								!this->move_from_right && !this->move_from_up &&
								this->bboxes[i].corner_clicked && !this->bboxes[i].removed) {

							this->move_from_left = true;
							this->one_is_selected = true;
							if (this->bboxes[i].selected || this->bboxes[i].held) {
								this->bboxes[i].color = this->bboxes[i].selection_color;
							}
							else {
								this->bboxes[i].color = this->bboxes[i].original_color;
							}
							this->bboxes[i].held = true;
							this->bboxes[i].p1 = cv::Point(x_mouse, this->bboxes[i].p1.y);
							this->bboxes[i].p2 = this->bboxes[i].p2;
							this->bboxes[i].p3 = cv::Point(this->bboxes[i].p3.x, y_mouse);
							this->bboxes[i].p4 = cv::Point(x_mouse, y_mouse);
						}

						// Modify from p1
						if ((this->bboxes[i].p1_selected || this->bboxes[i].held) &&
							!this->move_from_right && !this->move_from_left &&
							this->bboxes[i].corner_clicked && !this->bboxes[i].removed) {

							this->move_from_up = true;
							this->one_is_selected = true;
							if (this->bboxes[i].selected || this->bboxes[i].held) {
								this->bboxes[i].color = this->bboxes[i].selection_color;
							}
							else {
								this->bboxes[i].color = this->bboxes[i].original_color;
							}
							this->bboxes[i].held = true;
							this->bboxes[i].p1 = cv::Point(x_mouse, y_mouse);

							this->bboxes[i].p2 = cv::Point(this->bboxes[i].p2.x, y_mouse);
							this->bboxes[i].p3 = bboxes[i].p3;
							this->bboxes[i].p4 = cv::Point(x_mouse, this->bboxes[i].p4.y);
						}


						// Move bbx
						if (this->bboxes[i].clicked && !this->bboxes[i].removed) {
							this->one_is_selected = true;
							this->bboxes[i].held = true;

							if (this->bboxes[i].selected || this->bboxes[i].held) {
								this->bboxes[i].color = this->bboxes[i].selection_color;
							}
							else {
								this->bboxes[i].color = this->bboxes[i].original_color;
							}

							int size_x = std::max(this->bboxes[i].x1, this->bboxes[i].x2) - std::min(this->bboxes[i].x1, this->bboxes[i].x2);
							int size_y = std::max(this->bboxes[i].y1, this->bboxes[i].y2) - std::min(this->bboxes[i].y1, this->bboxes[i].y2);

							this->bboxes[i].p1 = cv::Point(x_mouse - this->bboxes[i].p1_mouse_offset.x, y_mouse - this->bboxes[i].p1_mouse_offset.y);
							this->bboxes[i].p2 = this->bboxes[i].p1 + cv::Point(size_x, 0);
							this->bboxes[i].p3 = this->bboxes[i].p1 + cv::Point(size_x, size_y);
							this->bboxes[i].p4 = this->bboxes[i].p1 + cv::Point(0, size_y);

						}
					}
					else {
						this->bboxes[i].update_pos();
						this->bboxes[i].held = false;
						this->bboxes[i].clicked = false;
						this->bboxes[i].corner_clicked = false;
						this->bboxes[i].removed_clicked = false;
						this->one_is_selected = false;
						this->bboxes[i].p1_mouse_offset = cv::Point(0, 0);
						*hold = false;
						this->move_from_left = false;
						this->move_from_right = false;
						this->move_from_up = false;

						// TODO move from left and right add to global

					}
						


				}
				
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
