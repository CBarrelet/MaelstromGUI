#pragma once

#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <opencv2/core/core.hpp>


class Bbx {

public:
	std::string name = "name";
	int id = -1;
	int size = 50;

	int x1 = 0;
	int y1 = 0;
	int x2 = 0;
	int y2 = 0;

	cv::Point p1 = cv::Point(0, 0); // Upper left corner
	cv::Point p2 = cv::Point(0, 0); // Upper right corner
	cv::Point p3 = cv::Point(0, 0); // Bottom right corner
	cv::Point p4 = cv::Point(0, 0); // Bottom left corner

	int width = 0;
	int height = 0;

	cv::Size img_size = cv::Size(0, 0);
	int max_height = 0;	// Image height
	int max_width = 0;  // Image width

	cv::Scalar color = cv::Scalar(0, 0, 0); // Bbx color
	cv::Scalar selection_color = cv::Scalar(0, 0, 0); // Bbx color when selected
	int thickness = 2;
	int corner_radius = int(size / 6);

	bool p1_selected = false;
	bool p2_selected = false;
	bool p3_selected = false;
	bool p4_selected = false;

	bool selected = false;
	bool held = false;
	bool clicked = false;
	bool corner_clicked = false;
	bool removed_clicked = false;

	bool removed = false; // Has been removed ?

	// p1_mouse_offset ?

	/*--------------------------------------------------
		Bbx constructor from mouse
	----------------------------------------------------*/
	Bbx(int id, cv::Point center, cv::Size img_size, cv::Scalar color) {

		this->id = id;

		this->x1 = center.x - int(size / 2);
		this->y1 = center.y - int(size / 2);
		this->x2 = center.x + int(size / 2);
		this->y2 = center.y + int(size / 2);

		this->width = std::abs(this->x1 - this->x2);
		this->height = std::abs(this->y1 - this->y2);

		this->p1.x = this->x1;
		this->p1.y = this->y1;
		this->p2.x = this->x2;
		this->p2.y = this->y1;
		this->p3.x = this->x2;
		this->p3.y = this->y2;
		this->p4.x = this->x1;
		this->p4.y = this->y2;

		this->img_size = img_size;
		this->max_height = img_size.height;
		this->max_width = img_size.width;

		this->color = color;
		this->selection_color = color + cv::Scalar(50, 50, 50);

		update_pos();
	}

	void update_pos() {
		this->x1 = this->p1.x;
		this->y1 = this->p1.y;
		this->x2 = this->p3.x;
		this->y2 = this->p3.y;

		// Prevent under and over coordinates
		if (this->x1 > this->max_width) { this->x1 = this->max_width; }
		if (this->x1 < 0) { this->x1 = 0; }
		if (this->y1 > this->max_height) { this->y1 = this->max_height; }
		if (this->y1 < 0) { this->y1 = 0; }
		if (this->x2 > this->max_width) { this->x2 = this->max_width; }
		if (this->x2 < 0) { this->x2 = 0; }
		if (this->y2 > this->max_height) { this->y2 = this->max_height; }
		if (this->y2 < 0) { this->y2 = 0; }

		this->width = std::abs(this->x1 - this->x2);
		this->height = std::abs(this->y1 - this->y2);

		int min_x = std::min(this->x1, this->x2);
		int max_x = std::max(this->x1, this->x2);
		int min_y = std::min(this->y1, this->y2);
		int max_y = std::max(this->y1, this->y2);

		this->p1.x = min_x;
		this->p1.y = min_y;
		this->p2.x = min_x + this->width;
		this->p2.y = min_y;
		this->p3.x = min_x + this->width;
		this->p3.y = min_y + this->height;
		this->p4.x = min_x;
		this->p4.y = min_y + this->height;
	}

	// Check if a round corner is selected
	bool is_corner_selected(int x_mouse, int y_mouse, cv::Point p) {
		return std::pow(x_mouse - p.x, 2) + std::pow(y_mouse - p.y, 2) <= std::pow(this->corner_radius, 2);
	}

	// Check if x, y coordinates are within a rectangle (x1,y1,x2,y2)
	bool contains(int x, int y, int x1, int y1, int x2, int y2) {
		x1 = std::min(x1, x2);
		x2 = std::max(x1, x2);
		y1 = std::min(y1, y2);
		y2 = std::max(y1, y2);
		return (x1 <= x) && (x <= x2) && (y1 <= y) && (y <= y2);
	}

};

