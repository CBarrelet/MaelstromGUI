#pragma once
#include "config.h"
#include <algorithm>
#include <iterator>
#include <random>
#include <minmax.h>


class DepthMap {
private:
	struct map_elmt {
		int time;
		float depth;
		float altitude;
		float presure;
	};

public:
	std::vector<std::vector<map_elmt>> high_depth_map;
	std::vector<std::vector<float>> low_depth_map;

private:
	int high_resolution = 25; // 10 cm
	int low_resolution = 50;  // 50 cm
	float resolution_ratio = low_resolution / high_resolution;

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

public: 
	cv::Mat map;
private: 
	cv::Mat displayed_map;

	int min_altitude = 0;
	int max_altitude = 5; // 5 meters for the color map

	// Last dvl values
	float last_distances[4][4]; // Altitude buffer with the 4 last values

public:
	bool which_res = true; // Means high resolution else low resolution

public:
	DepthMap() {

		std::random_device rd; // obtain a random number from hardware
		std::mt19937 gen(rd()); // seed the generator
		std::uniform_int_distribution<> distr(50, 200); // define the range

		// Init last altitudes values
		for (size_t i = 0; i < 4; i++)
			for (size_t j = 0; j < 4; i = j++)
				last_distances[i][j] = 0;

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
			std::vector<float> v1;
			for (size_t j = 0; j < this->height; j += this->low_resolution) {
				v1.push_back(0);
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

	void update(cv::Point3f dvl_coo[4], cv::Point3f robot_coo, float dvl_distances[4], float depth) {

		initFirstDvlValues(dvl_distances);
		
		// Set the coorinates in the depth map coordinates space
		for (size_t i = 0; i < 4; i++) {
			dvl_coo[i].x = dvl_coo[i].x * this->low_resolution * 2;
			dvl_coo[i].y = dvl_coo[i].y * this->low_resolution * 2 * -1;
		}
		robot_coo.x = robot_coo.x * this->low_resolution * 2;
		robot_coo.y = robot_coo.y * this->low_resolution * 2 * -1;

		cv::Point beams[4];
		for (size_t i = 0; i < 4; i++) {
			beams[i] = cv::Point(dvl_coo[i].x + robot_coo.x, dvl_coo[i].y + robot_coo.y) + this->center;
			beams[i].x = int(beams[i].x / this->high_resolution);
			beams[i].y = int(beams[i].y / this->high_resolution);
		}
			
		// Filter artefacts and set the depth map
		valuesFiltering(dvl_distances, dvl_coo, beams);

		// Update last_value and shift the buffer
		updateLastDvlValues(dvl_distances);
	}

private:

	void initFirstDvlValues(float dvl_distances[4]) {
		// Init first altitude values
		if ((this->last_distances[0][0] == 0) && (this->last_distances[0][1] == 0) && (this->last_distances[0][2] == 0) && (this->last_distances[0][3] == 0)) {
			this->last_distances[0][0] = dvl_distances[0];
			this->last_distances[0][1] = dvl_distances[1];
			this->last_distances[0][2] = dvl_distances[2];
			this->last_distances[0][3] = dvl_distances[3];
		}
		else if ((this->last_distances[1][0] == 0) && (this->last_distances[1][1] == 0) && (this->last_distances[1][2] == 0) && (this->last_distances[1][3] == 0)) {
			this->last_distances[1][0] = dvl_distances[0];
			this->last_distances[1][1] = dvl_distances[1];
			this->last_distances[1][2] = dvl_distances[2];
			this->last_distances[1][3] = dvl_distances[3];
		}
		else if ((this->last_distances[2][0] == 0) && (this->last_distances[2][1] == 0) && (this->last_distances[2][2] == 0) && (this->last_distances[2][3] == 0)) {
			this->last_distances[2][0] = dvl_distances[0];
			this->last_distances[2][1] = dvl_distances[1];
			this->last_distances[2][2] = dvl_distances[2];
			this->last_distances[2][3] = dvl_distances[3];
		}
		else if ((this->last_distances[3][0] == 0) && (this->last_distances[3][1] == 0) && (this->last_distances[3][2] == 0) && (this->last_distances[3][3] == 0)) {
			this->last_distances[3][0] = dvl_distances[0];
			this->last_distances[3][1] = dvl_distances[1];
			this->last_distances[3][2] = dvl_distances[2];
			this->last_distances[3][3] = dvl_distances[3];
		}
	}

	void valuesFiltering(float dvl_distances[4], cv::Point3f dvl_coo[4], cv::Point beams[4]) {
		float coeff_filter = 1.8;
		bool is_good_value = true;

		// Check value for each DVL beam
		for (size_t i = 0; i < 4; i++) {
			is_good_value = true;

			if ((beams[i].x >= 0) && (beams[i].y >= 0) && (beams[i].x <= this->high_depth_map.size()) && (beams[i].y <= this->high_depth_map[0].size())) {
				// Check value for the 4 last values
				for (size_t j = 0; j < 4; j++) {
					// Filter some artefacts, if at least one value is wrong, is_good_value == false
					if ((coeff_filter * this->last_distances[j][i] > dvl_distances[i]) && (this->last_distances[j][i] / coeff_filter < dvl_distances[i])) 	
						is_good_value *= true;
					else
						is_good_value *= false;
				}

				if (is_good_value) {
					if (this->high_depth_map[beams[i].x][beams[i].y].altitude < 0)
						this->high_depth_map[beams[i].x][beams[i].y].altitude = max(high_depth_map[beams[i].x][beams[i].y].altitude, dvl_coo[0].z);
					else
						this->high_depth_map[beams[i].x][beams[i].y].altitude = dvl_coo[0].z;
				}
			}
		}
	}

	void updateLastDvlValues(float dvl_distances[4]) {
		// Shift all the values by one, and add the new one at the first position
		for (size_t i = 0; i < 4; i++) {
			// Shift values
			for (size_t j = 1; j < 4; j++)
				this->last_distances[j][i] = this->last_distances[j - 1][i];
			
			// Update first values
			this->last_distances[0][i] = dvl_distances[i];
		}
	}

public:

	void setDepthMap() {
		unsigned char altitude_color = 0;
		cv::Point p1(0, 0);
		cv::Point p2(0, 0);
		int low_grid_x = 0, low_grid_y = 0;

		// High resolution
		for (size_t j = 0; j < this->high_depth_map.size(); j++) {
			for (size_t i = 0; i < this->high_depth_map[j].size(); i++) {

				p1.x = this->high_resolution * j;
				p1.y = this->high_resolution * i;

				p2.x = this->high_resolution * (j + 1);
				p2.y = this->high_resolution * (i + 1);

				//altitude_color = -(((high_depth_map[j][i].altitude - 0) * (255 - 0)) / (5 - 0)) + 0;	
				altitude_color = max(min(255, (5 + high_depth_map[j][i].altitude) * 255 / 5), 0);
				//altitude_color = max(min(255, (- high_depth_map[j][i].altitude) * 255 / 5), 0); // To invert the color

				if(this->which_res)
					cv::rectangle(this->map, cv::Rect(p1, p2), cv::Scalar(altitude_color, altitude_color, altitude_color), -1, cv::LINE_8);
				
				low_grid_x = int(j / this->resolution_ratio);
				low_grid_y = int(i / this->resolution_ratio);

				if (low_depth_map[low_grid_x][low_grid_y] < 0)
					low_depth_map[low_grid_x][low_grid_y] = min(low_depth_map[low_grid_x][low_grid_y], high_depth_map[j][i].altitude);

				else
					low_depth_map[low_grid_x][low_grid_y] = high_depth_map[j][i].altitude;	
			}
		}

		// Low resolution
		for (size_t j = 0; j < this->low_depth_map.size(); j++) {
			for (size_t i = 0; i < this->low_depth_map[j].size(); i++) {

				p1.x = this->low_resolution * j;
				p1.y = this->low_resolution * i;

				p2.x = this->low_resolution * (j + 1);
				p2.y = this->low_resolution * (i + 1);

				// altitude_color = -(((low_depth_map[j][i] - 0) * (255 - 0)) / (5 - 0)) + 0; // Invert color
				altitude_color = max(min(255, (5 + low_depth_map[j][i]) * 255 / 5), 0);

				if (!this->which_res)
					cv::rectangle(this->map, cv::Rect(p1, p2), cv::Scalar(int(altitude_color), int(altitude_color), int(altitude_color)), -1, cv::LINE_8);
			}
		}
		cv::applyColorMap(this->map, this->map, cv::COLORMAP_JET);
		this->displayed_map = this->map.clone();
	}

	void drawGrid() {
		int resolution = 0;
		if (this->which_res)
			resolution = this->high_resolution;
		else
			resolution = low_resolution;

		for (size_t i = 0; i < this->width; i += resolution)
			cv::line(this->map, cv::Point(i, 0), cv::Point(i, this->height), cv::Scalar(255, 255, 255), 1);
		for (size_t i = 0; i < this->height; i += resolution)
			cv::line(this->map, cv::Point(0, i), cv::Point(this->width, i), cv::Scalar(255, 255, 255), 1);
		// Draw center pos
		cv::circle(this->displayed_map, cv::Point(this->width / 2, this->height / 2), 4, cv::Scalar(255, 255, 255), -1);

		this->displayed_map = this->map.clone();
	}

	cv::Mat getMap() {
		cv::resize(this->displayed_map, this->displayed_map, cv::Size(), 0.4, 0.4);
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

		this->p1 = this->p1 + this->center;
		this->p2 = this->p2 + this->center;
		this->p3 = this->p3 + this->center;
		this->p4 = this->p4 + this->center;

		this->robot_center = cv::Point(this->p1.x + robot_size / 2, this->p1.y + robot_size / 2);
	}
	void drawRobot() {

		float corner_radius = int(this->radius * this->robot_size / 2);
		cv::Scalar color(0, 0, 255);

		// Draw robot
		cv::line(this->displayed_map, cv::Point(this->p1.x + corner_radius, this->p1.y), cv::Point(this->p2.x - corner_radius, this->p2.y), color, this->robot_thickness, cv::LINE_AA);
		cv::line(this->displayed_map, cv::Point(this->p2.x, this->p2.y + corner_radius), cv::Point(this->p3.x, this->p3.y - corner_radius), color, this->robot_thickness, cv::LINE_AA);
		cv::line(this->displayed_map, cv::Point(this->p3.x - corner_radius, this->p4.y), cv::Point(this->p4.x + corner_radius, this->p3.y), color, this->robot_thickness, cv::LINE_AA);
		cv::line(this->displayed_map, cv::Point(this->p4.x, this->p4.y - corner_radius), cv::Point(this->p1.x, this->p1.y + corner_radius), color, this->robot_thickness, cv::LINE_AA);

		cv::ellipse(this->displayed_map, cv::Point(this->p1.x + corner_radius, this->p1.y + corner_radius), cv::Point(corner_radius, corner_radius), 180.0, 0, 90, color, this->robot_thickness, cv::LINE_AA);
		cv::ellipse(this->displayed_map, cv::Point(this->p2.x - corner_radius, this->p2.y + corner_radius), cv::Point(corner_radius, corner_radius), 270.0, 0, 90, color, this->robot_thickness, cv::LINE_AA);
		cv::ellipse(this->displayed_map, cv::Point(this->p3.x - corner_radius, this->p3.y - corner_radius), cv::Point(corner_radius, corner_radius), 0.0, 0, 90, color, this->robot_thickness, cv::LINE_AA);
		cv::ellipse(this->displayed_map, cv::Point(this->p4.x + corner_radius, this->p4.y - corner_radius), cv::Point(corner_radius, corner_radius), 90.0, 0, 90, color, this->robot_thickness, cv::LINE_AA);
	}
	void drawTarget(cv::Point2f target) {
		// Target is in the 3d world from the robot

		target.x = target.x * this->low_resolution * 2;
		target.y = -target.y * this->low_resolution * 2; // -1 * y because image y axis
		target = cv::Point(target) + this->center;//; + this->center - this->robot_center;// +this->robot_center;

		cv::circle(this->displayed_map, target, 6, cv::Scalar(0, 255, 0), -1);
		cv::line(this->displayed_map, this->robot_center, target, cv::Scalar(0, 180, 0), 2, cv::LINE_AA);
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

