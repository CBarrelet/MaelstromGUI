#pragma once
#include "config.h"


class DepthMap {
private:
	struct map_elmt {
		int time;
		int depth;
		int altitude;
	};
	
	int resolution = 50;

	int margin = 100;
	int width = 1200 + 100;
	int height = 700 + 100;

	cv::Point center;

	// Robot pos
	cv::Point p1;
	cv::Point p2;
	cv::Point p3;
	cv::Point p4;
	cv::Point robot_center;

	int robot_size = 150;
	int robot_thickness = 2;
	float radius = 0.5;

	cv::Mat map;

public:
	DepthMap() {
		/*this->width += this->margin;
		this->height += this->margin;*/

		this->center = cv::Point(this->width, this->height) / 2;
		
		this->map = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);
	}

	~DepthMap() {
	}

	void init() {
		this->map = cv::Mat::zeros(cv::Size(this->width, this->height), CV_8UC3);
	}

	void drawGrid() {
		for (size_t i = 0; i < this->width; i+=this->resolution) {
			cv::line(this->map, cv::Point(i, 0), cv::Point(i, this->height), cv::Scalar(255, 255, 255), 1);
		}
		for (size_t i = 0; i < this->height; i += this->resolution) {
			cv::line(this->map, cv::Point(0, i), cv::Point(this->width, i), cv::Scalar(255, 255, 255), 1);
		}
	}

	cv::Mat getMap() {
		cv::resize(this->map, this->map, cv::Size(), 0.4, 0.4);
		return this->map;
	}

	void setPos(int x, int y) {
		this->p1 = cv::Point(x - int(this->robot_size / 2), y - int(this->robot_size / 2));
		this->p2 = cv::Point(x + int(this->robot_size / 2), y - int(this->robot_size / 2));
		this->p3 = cv::Point(x + int(this->robot_size / 2), y + int(this->robot_size / 2));
		this->p4 = cv::Point(x - int(this->robot_size / 2), y + int(this->robot_size / 2));

		// Make it in the robot coordinate system which start at the center --> robot(0,0) == map(center)
		/*this->p1 = this->p1 + this->center - cv::Point(0, 50);
		this->p2 = this->p2 + this->center - cv::Point(0, 50);
		this->p3 = this->p3 + this->center - cv::Point(0, 50);
		this->p4 = this->p4 + this->center - cv::Point(0, 50);*/

		this->p1 = this->p1 + this->center;
		this->p2 = this->p2 + this->center;
		this->p3 = this->p3 + this->center;
		this->p4 = this->p4 + this->center;

		this->robot_center = cv::Point(this->p1.x + robot_size/2, this->p1.y + robot_size / 2);
	}

	void drawRobot() {

		float corner_radius = int(this->radius * this->robot_size / 2);
		cv::Scalar color(0, 0, 255);

		// Draw robot
		cv::line(this->map, cv::Point(this->p1.x + corner_radius, this->p1.y), cv::Point(this->p2.x - corner_radius, this->p2.y), color, robot_thickness, cv::LINE_AA);
		cv::line(this->map, cv::Point(this->p2.x, this->p2.y + corner_radius), cv::Point(this->p3.x, this->p3.y - corner_radius), color, robot_thickness, cv::LINE_AA);
		cv::line(this->map, cv::Point(this->p3.x - corner_radius, this->p4.y), cv::Point(this->p4.x + corner_radius, this->p3.y), color, robot_thickness, cv::LINE_AA);
		cv::line(this->map, cv::Point(this->p4.x, this->p4.y - corner_radius), cv::Point(this->p1.x, this->p1.y + corner_radius), color, robot_thickness, cv::LINE_AA);

		cv::ellipse(this->map, cv::Point(this->p1.x + corner_radius, this->p1.y + corner_radius), cv::Point(corner_radius, corner_radius), 180.0, 0, 90, color, robot_thickness, cv::LINE_AA);
		cv::ellipse(this->map, cv::Point(this->p2.x - corner_radius, this->p2.y + corner_radius), cv::Point(corner_radius, corner_radius), 270.0, 0, 90, color, robot_thickness, cv::LINE_AA);
		cv::ellipse(this->map, cv::Point(this->p3.x - corner_radius, this->p3.y - corner_radius), cv::Point(corner_radius, corner_radius), 0.0, 0, 90, color, robot_thickness, cv::LINE_AA);
		cv::ellipse(this->map, cv::Point(this->p4.x + corner_radius, this->p4.y - corner_radius), cv::Point(corner_radius, corner_radius), 90.0, 0, 90, color, robot_thickness, cv::LINE_AA);
	}

	void drawTarget(cv::Point2f target) {

		target = this->robot_center + cv::Point(target * this->resolution*2);

		cv::circle(this->map, target, 6, cv::Scalar(0, 255, 0), -1);
		cv::line(this->map, this->robot_center, target, cv::Scalar(0, 180, 0), 2, cv::LINE_AA);

		//cv::circle(this->map, this->robot_center, 10, cv::Scalar(255, 255, 255), 1);
	}




};

