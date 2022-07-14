#pragma once
#include "config.h"
#include <algorithm>
#include <iterator>
#include <random>


class DepthMap {
private:
	struct map_elmt {
		int time;
		float depth;
		float altitude;
		float presure;
	};

	struct low_map_elmt {
		std::vector<float> depth;
		std::vector<float> altitude;
	};

	std::vector<std::vector<map_elmt>> high_depth_map;
	std::vector<std::vector<low_map_elmt>> low_depth_map;
	
	int high_resolution = 10; // 10 cm
	int low_resolution = 50;  // 50 cm
	int resolution_ratio = low_resolution / high_resolution;

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
	int robot_thickness = 3;
	float radius = 0.5;

	cv::Mat map;
	cv::Mat displayed_map;

public:
	DepthMap() {

		std::random_device rd; // obtain a random number from hardware
		std::mt19937 gen(rd()); // seed the generator
		std::uniform_int_distribution<> distr(50, 200); // define the range

		/*this->width += this->margin;
		this->height += this->margin;*/

		// Init high depth map
		for (size_t i = 0; i < this->width; i += this->high_resolution) {
			std::vector<map_elmt> v1;
			for (size_t j = 0; j < this->height; j += this->high_resolution) {
				map_elmt elmt;
				elmt.time = 0;
				elmt.depth = 0; // distr(gen);
				elmt.altitude = 0;
				elmt.presure = 0;
				v1.push_back(elmt);
			}
			this->high_depth_map.push_back(v1);
		}

		// Init low depth map
		for (size_t i = 0; i < this->width; i += this->low_resolution) {
			std::vector<low_map_elmt> v1;
			for (size_t j = 0; j < this->height; j += this->low_resolution) {
				low_map_elmt elmt;
				v1.push_back(elmt);
			}
			this->low_depth_map.push_back(v1);
		}

		// Center of the map
		this->center = cv::Point(this->width, this->height) / 2;
	
		this->map = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);
	}

	~DepthMap() {
	}

	void init() {
		this->map *= 0;
	}

	void update(cv::Point3f dvl_coo[4], cv::Point3f robot_coo) {

		// Test

		double sincos = 0.27059805; // sin(22.5°) * cos(45°)
		double cosalpha = 0.9238795; // cos(22.5°)

		float d = 1;

		float x1 = -d * sincos;
		float y1 = -d * sincos;
		float z1 = -d * cosalpha;
		dvl_coo[0] = cv::Point3f(x1, y1, z1);

		float x2 = -d * sincos;
		float y2 = d * sincos;
		float z2 = -d * cosalpha;
		dvl_coo[1] = cv::Point3f(x2, y2, z2);

		float x3 = d * sincos;
		float y3 = d * sincos;
		float z3 = -d * cosalpha;
		dvl_coo[2] = cv::Point3f(x3, y3, z3);

		float x4 = d * sincos;
		float y4 = -d * sincos;
		float z4 = -d * cosalpha;
		dvl_coo[3] = cv::Point3f(x4, y4, z4);

		dvl_coo[0].x = dvl_coo[0].x * this->low_resolution * 2;
		dvl_coo[0].y = dvl_coo[0].y * this->low_resolution * 2 * -1;

		dvl_coo[1].x = dvl_coo[1].x * this->low_resolution * 2;
		dvl_coo[1].y = dvl_coo[1].y * this->low_resolution * 2 * -1;

		dvl_coo[2].x = dvl_coo[2].x * this->low_resolution * 2;
		dvl_coo[2].y = dvl_coo[2].y * this->low_resolution * 2 * -1;

		dvl_coo[3].x = dvl_coo[3].x * this->low_resolution * 2;
		dvl_coo[3].y = dvl_coo[3].y * this->low_resolution * 2 * -1;

		robot_coo.x = robot_coo.x * this->low_resolution * 2;
		robot_coo.y = robot_coo.y * this->low_resolution * 2 * -1;

		cv::Point beam_1 = cv::Point(dvl_coo[0].x + robot_coo.x, dvl_coo[0].y + robot_coo.y) + this->center;
		cv::Point beam_2 = cv::Point(dvl_coo[1].x + robot_coo.x, dvl_coo[1].y + robot_coo.y) + this->center;
		cv::Point beam_3 = cv::Point(dvl_coo[2].x + robot_coo.x, dvl_coo[2].y + robot_coo.y) + this->center;
		cv::Point beam_4 = cv::Point(dvl_coo[3].x + robot_coo.x, dvl_coo[3].y + robot_coo.y) + this->center;

		cv::circle(this->map, beam_1, 6, cv::Scalar(255, 0, 0), -1);
		cv::circle(this->map, beam_2, 6, cv::Scalar(0, 255, 0), -1);
		cv::circle(this->map, beam_3, 6, cv::Scalar(0, 0, 255), -1);
		cv::circle(this->map, beam_4, 6, cv::Scalar(255, 0, 255), -1);

	}

	void setDepthMap(/*cv::Point2f robot_center_pos, float dvl_altitude*/) {

		for (size_t j = 0; j < this->high_depth_map.size(); j++) {
			for (size_t i = 0; i < this->high_depth_map[j].size(); i++) {

				cv::Point p1(this->high_resolution * j, this->high_resolution * i);
				cv::Point p2(this->high_resolution * (j + 1), this->high_resolution * (i + 1));

				low_depth_map[int(j / this->resolution_ratio)][int(i / this->resolution_ratio)].depth.push_back(high_depth_map[j][i].depth);
				low_depth_map[int(j / this->resolution_ratio)][int(i / this->resolution_ratio)].altitude.push_back(high_depth_map[j][i].altitude);
				
			}
		}

		for (size_t j = 0; j < this->low_depth_map.size(); j++) {
			for (size_t i = 0; i < this->low_depth_map[j].size(); i++) {

				cv::Point p1(this->low_resolution * j, this->low_resolution * i);
				cv::Point p2(this->low_resolution * (j + 1), this->low_resolution * (i + 1));

				//int color = low_depth_map[j][i].altitude

				auto min_depth = std::min_element(std::begin(low_depth_map[j][i].depth), std::end(low_depth_map[j][i].depth));
				float depth = *min_depth;

				cv::rectangle(this->map, cv::Rect(p1, p2), cv::Scalar(int(depth), int(depth), int(depth)), -1, cv::LINE_8);


				/*if ((i % 2 == 0) && (j % 2 == 0))
					cv::rectangle(this->map, cv::Rect(p1, p2), cv::Scalar(250, 120, 150), -1, cv::LINE_8);
				if ((i % 2 != 0) && (j % 2 == 0))
					cv::rectangle(this->map, cv::Rect(p1, p2), cv::Scalar(120, 250, 150), -1, cv::LINE_8);
				if ((i % 2 == 0) && (j % 2 != 0))
					cv::rectangle(this->map, cv::Rect(p1, p2), cv::Scalar(120, 250, 250), -1, cv::LINE_8);
				if ((i % 2 != 0) && (j % 2 != 0))
					cv::rectangle(this->map, cv::Rect(p1, p2), cv::Scalar(250, 120, 250), -1, cv::LINE_8);*/

			}
		}

	}

	void drawGrid() {
		for (size_t i = 0; i < this->width; i+=this->low_resolution) {
			cv::line(this->map, cv::Point(i, 0), cv::Point(i, this->height), cv::Scalar(255, 255, 255), 1);
		}
		for (size_t i = 0; i < this->height; i += this->low_resolution) {
			cv::line(this->map, cv::Point(0, i), cv::Point(this->width, i), cv::Scalar(255, 255, 255), 1);
		}
		// Draw center pos
		cv::circle(this->map, cv::Point(this->width / 2, this->height / 2), 4, cv::Scalar(255, 255, 255), -1);
	}

	cv::Mat getMap() {
		cv::resize(this->map, this->displayed_map, cv::Size(), 0.4, 0.4);
		cv::rotate(this->displayed_map, this->displayed_map, cv::ROTATE_90_COUNTERCLOCKWISE);
		return this->displayed_map;
	}
	void setPos(float x, float y) {
		x = x * this->low_resolution * 2;
		y = -y * this->low_resolution * 2;
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

		std::cout << "test: " << this->robot_center.x << std::endl;
	}
	void drawRobot() {

		float corner_radius = int(this->radius * this->robot_size / 2);
		cv::Scalar color(0, 0, 255);

		// Draw robot
		cv::line(this->map, cv::Point(this->p1.x + corner_radius, this->p1.y), cv::Point(this->p2.x - corner_radius, this->p2.y), color, this->robot_thickness, cv::LINE_AA);
		cv::line(this->map, cv::Point(this->p2.x, this->p2.y + corner_radius), cv::Point(this->p3.x, this->p3.y - corner_radius), color, this->robot_thickness, cv::LINE_AA);
		cv::line(this->map, cv::Point(this->p3.x - corner_radius, this->p4.y), cv::Point(this->p4.x + corner_radius, this->p3.y), color, this->robot_thickness, cv::LINE_AA);
		cv::line(this->map, cv::Point(this->p4.x, this->p4.y - corner_radius), cv::Point(this->p1.x, this->p1.y + corner_radius), color, this->robot_thickness, cv::LINE_AA);

		cv::ellipse(this->map, cv::Point(this->p1.x + corner_radius, this->p1.y + corner_radius), cv::Point(corner_radius, corner_radius), 180.0, 0, 90, color, this->robot_thickness, cv::LINE_AA);
		cv::ellipse(this->map, cv::Point(this->p2.x - corner_radius, this->p2.y + corner_radius), cv::Point(corner_radius, corner_radius), 270.0, 0, 90, color, this->robot_thickness, cv::LINE_AA);
		cv::ellipse(this->map, cv::Point(this->p3.x - corner_radius, this->p3.y - corner_radius), cv::Point(corner_radius, corner_radius), 0.0, 0, 90, color, this->robot_thickness, cv::LINE_AA);
		cv::ellipse(this->map, cv::Point(this->p4.x + corner_radius, this->p4.y - corner_radius), cv::Point(corner_radius, corner_radius), 90.0, 0, 90, color, this->robot_thickness, cv::LINE_AA);
	}
	void drawTarget(cv::Point2f target) {
		// Target is in the 3d world from the robot

		target.x = target.x * this->low_resolution * 2;
		target.y = -target.y * this->low_resolution * 2; // -1 * y because image y axis
		target = cv::Point(target) + this->center;//; + this->center - this->robot_center;// +this->robot_center;

		cv::circle(this->map, target, 6, cv::Scalar(0, 255, 0), -1);
		cv::line(this->map, this->robot_center, target, cv::Scalar(0, 180, 0), 2, cv::LINE_AA);
		//cv::circle(this->map, this->robot_center, 10, cv::Scalar(255, 255, 255), 1);
	}


	/*------------------------------
		Timestamp
	-------------------------------*/
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

};

