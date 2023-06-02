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
		float pressure;
	};

public:

	// Maximum and minimum altitude to scale the depth map color map
	float altitude_min = 0;
	float altitude_max = -5;


	// Global 5x5 cm map
	std::vector<std::vector<map_elmt>> global_very_high_depth_map;
	// Global 25x25 cm map
	std::vector<std::vector<map_elmt>> global_high_depth_map;

	cv::Mat displayed_global_map;
	cv::Mat displayed_very_high_global_map;

	// 5x5 cm map
	std::vector<std::vector<map_elmt>> very_high_depth_map;
	std::vector<std::vector<map_elmt>> freezed_very_high_depth_map;
	// 25x25 cm map
	std::vector<std::vector<map_elmt>> high_depth_map;
	std::vector<std::vector<map_elmt>> freezed_high_depth_map;

	// 25x25 cm map
	std::vector<std::vector<float>> low_depth_map;

	
	// Dvl beams in high map space
	cv::Point beams[4];

	bool is_freezed = false;

private:
	int very_high_resolution = 5;	// 5 cm
	int high_resolution = 25;		// 25 cm
	int low_resolution = 50;		// 50 cm
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
	cv::Size displayed_size = cv::Size(320, 520);

	float min_altitude = 0;
	float max_altitude = 5; // 5 meters for the color map

	// Last dvl values
	float last_distances[4][4]; // Altitude buffer with the 4 last values
	float coeff_filter = 1.8;

public:
	int which_res = 1; // 0, 1, 2 Means very high, high or low resolution
	float tide = 0;
	float tide_coeff = 0.001;


	// Simulation counter 
	int simulation_counter = 0;


	// Target utm to x to display 
	double x_target = 0, y_target = 0;

public:
	DepthMap() {

		std::random_device rd; // obtain a random number from hardware
		std::mt19937 gen(rd()); // seed the generator
		std::uniform_int_distribution<> distr1(0, 13); // define the range
		std::uniform_int_distribution<> distr2(0, 8); // define the range

		std::uniform_int_distribution<> distr3(-3000, -2000); // define the range

		this->displayed_global_map = cv::Mat(NB_CASES_COTE_CARTE_GLOBALE, NB_CASES_COTE_CARTE_GLOBALE, CV_8UC1, cv::Scalar(0));
		this->displayed_very_high_global_map = cv::Mat(NB_CASES_COTE_CARTE_GLOBALE_VERY_HIGH, NB_CASES_COTE_CARTE_GLOBALE_VERY_HIGH, CV_8UC1, cv::Scalar(0));
		// this->displayed_global_map = cv::Mat(400, 400, CV_8UC1, cv::Scalar(0));


		// Init last altitudes values
		for (size_t i = 0; i < 4; i++)
			for (size_t j = 0; j < 4; i = j++)
				last_distances[i][j] = 0;

		// Init global very high depth map
		for (size_t i = 0; i < NB_CASES_COTE_CARTE_GLOBALE_VERY_HIGH; i++) {
			std::vector<map_elmt> v1;
			for (size_t j = 0; j < NB_CASES_COTE_CARTE_GLOBALE_VERY_HIGH; j++) {
				map_elmt elmt;
				elmt.time = 0;
				elmt.depth = 0; // distr(gen);
				elmt.altitude = 0; // distr3(gen);;
				elmt.pressure = 0;
				v1.push_back(elmt);
			}
			this->global_very_high_depth_map.push_back(v1);
		}

		// Init global high depth map
		for (size_t i = 0; i < NB_CASES_COTE_CARTE_GLOBALE; i++) {
			std::vector<map_elmt> v1;
			for (size_t j = 0; j < NB_CASES_COTE_CARTE_GLOBALE; j++) {
				map_elmt elmt;
				elmt.time = 0;
				elmt.depth = 0; // distr(gen);
				elmt.altitude = 0;
				elmt.pressure = 0;
				v1.push_back(elmt);
			}
			this->global_high_depth_map.push_back(v1);
		}

		// Init very high depth map
		for (size_t i = 0; i < this->width; i += this->very_high_resolution) {
			std::vector<map_elmt> v1;
			for (size_t j = 0; j < this->height; j += this->very_high_resolution) {
				map_elmt elmt;
				elmt.time = 0;
				elmt.depth = 0; // distr(gen);
				elmt.altitude = 0;
				elmt.pressure = 0;
				v1.push_back(elmt);
			}
			this->very_high_depth_map.push_back(v1);
			this->freezed_very_high_depth_map.push_back(v1);
		}

		// Init high depth map
		for (size_t i = 0; i < this->width; i += this->high_resolution) {
			std::vector<map_elmt> v1;
			for (size_t j = 0; j < this->height; j += this->high_resolution) {
				map_elmt elmt;
				elmt.time = 0;
				elmt.depth = 0; // distr(gen);
				elmt.altitude = 0;
				elmt.pressure = 0;
				v1.push_back(elmt);
			}
			this->high_depth_map.push_back(v1);
			this->freezed_high_depth_map.push_back(v1);
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
		this->map = cv::Mat::ones(cv::Size(width, height), CV_8UC3);// *204;
		//this->map = cv::Mat(cv::Size(width, height), CV_8UC3, cv::Scalar(255, 255, 255));
	}

	~DepthMap() {
	}

	void setTarget(double x_target, double y_target) {
		this->x_target = x_target;
		this->y_target = y_target;
	}

	float getPrecision(float number, int precision) {
		std::stringstream ss;
		ss << std::fixed << std::setprecision(precision) << number;
		std::string number_str = ss.str();
		float new_number = std::stof(number_str);
		return new_number;
	}

	std::string getUDPFrame() {
		float altitude = 0;

		std::string frame = "*";
		frame += std::to_string(this->simulation_counter);
		this->simulation_counter++;
		frame += ";";

		int random_number = 0;

		// *counter;val1;val2.......valxxx#

		for (size_t j = 0; j < this->low_depth_map.size(); j++) {
			for (size_t i = 0; i < this->low_depth_map[j].size(); i++) {
				altitude = low_depth_map[j][i];

				// Just for test
				random_number = (rand() % 10) + 5;
				//std::cout << random_number << std::endl;
				//altitude = double(random_number) / 10000;
				//altitude = double(random_number) / 10000;
				altitude = -(double)random_number;

				//altitude = 0;

				if(altitude != 0)
					altitude *= -1;	 // Make it positive
				altitude *= 100; // Make it in centimeter

				std::stringstream stream;
				stream << std::fixed << std::setprecision(0) << altitude;
				std::string s = stream.str();
				frame += s;
				frame += ";";
			}
		}
		frame += "#";

		std::replace(frame.begin(), frame.end(), '.', ',');
		return frame;


		/*std::cout << "Test :" << std::endl;
		std::cout << frame << std::endl;

		std::cout << this->low_depth_map.size() << std::endl;
		std::cout << this->low_depth_map[0].size() << std::endl;*/

	}

	void init() {
		this->map *= 0;

		//// Test only
		//this->high_depth_map[10][10].altitude = -3;
		//this->high_depth_map[10][20].altitude = -3.2;
		//this->high_depth_map[10][30].altitude = -3.4;
	}

	void scale_color_map() {
		for (size_t i = 0; i < this->high_depth_map.size(); i++)
		{
			for (size_t j = 0; j < this->high_depth_map[0].size(); j++)
			{
				if (high_depth_map[i][j].altitude != 0) {

					this->altitude_min = min(high_depth_map[i][j].altitude, this->altitude_min);
					this->altitude_max = max(high_depth_map[i][j].altitude, this->altitude_max);
					this->altitude_min = -5;
					this->altitude_max = -3;
				}
			}
		}

	}

	void update(cv::Point3f dvl_coo[4], cv::Point3f robot_coo, float dvl_distances[4], float depth, double origine_global_map_latitude, double origine_global_map_longitude, double gps_P_latitude, double gps_P_longitude, double gps_S_cap) {

		scale_color_map();

		//std::random_device rd; // obtain a random number from hardware
		//std::mt19937 gen(rd()); // seed the generator
		//std::uniform_int_distribution<> distr1(0, 13); // define the range
		//std::uniform_int_distribution<> distr2(0, 8); // define the range

		//std::uniform_int_distribution<> distr3(-5000, -1000); // define the range

		initFirstDvlValues(dvl_distances);


		int i_very_high_global = 0, j_very_high_global = 0;
		int i_global = 0, j_global = 0;
		bool is_good_value = true;

		unsigned char altitude_color = 0;

		for (size_t i = 0; i < 4; i++) {

			is_good_value = true;

			// Very high map i, j coordinates
			convertit_coordonnees_x_y_locales_en_i_j_tres_grande_carte(
				origine_global_map_latitude, origine_global_map_longitude,
				gps_P_latitude, gps_P_longitude,
				gps_S_cap,
				dvl_coo[i].x + robot_coo.x,
				dvl_coo[i].y + robot_coo.y,
				&i_very_high_global, &j_very_high_global);

			// High map i, j coordinates
			convertit_coordonnees_x_y_locales_en_i_j_grande_carte(
				origine_global_map_latitude, origine_global_map_longitude,
				gps_P_latitude, gps_P_longitude,
				gps_S_cap,
				dvl_coo[i].x + robot_coo.x,
				dvl_coo[i].y + robot_coo.y,
				&i_global, &j_global);

			//std::cout << i_very_high_global << " " << i_global << std::endl;

			

			if ((i_global >= 0) && (i_global < NB_CASES_COTE_CARTE_GLOBALE) && (j_global >= 0) && (j_global < NB_CASES_COTE_CARTE_GLOBALE)) {
				if ((i_very_high_global >= 0) && (i_very_high_global < NB_CASES_COTE_CARTE_GLOBALE_VERY_HIGH) && (j_very_high_global >= 0) && (j_very_high_global < NB_CASES_COTE_CARTE_GLOBALE_VERY_HIGH)) {

					// Need to filter this
					is_good_value = true;

					for (size_t j = 0; j < 4; j++) {
						// Filter some artefacts, if at least one value is wrong, is_good_value == false
						if ((1.5 * this->last_distances[j][i] > dvl_distances[i]) && (this->last_distances[j][i] * 0.7 < dvl_distances[i]))
							is_good_value *= true;
						else
							is_good_value *= false;
					}
					//filtrage des coordonnees du fond marin
					double profondeur_min = -3; 
					double profondeur_max  = -5;
					if ((dvl_coo[0].z < profondeur_min) && (dvl_coo[0].z > profondeur_max) 
						&& (dvl_coo[1].z < profondeur_min) && (dvl_coo[1].z > profondeur_max)
						&& (dvl_coo[2].z < profondeur_min) && (dvl_coo[2].z > profondeur_max)
						&& (dvl_coo[3].z < profondeur_min) && (dvl_coo[3].z > profondeur_max)
						) is_good_value = true;
						

					// Very high map
					if (is_good_value) {
						if ((this->global_very_high_depth_map[j_very_high_global][i_very_high_global].altitude < 0))
							this->global_very_high_depth_map[j_very_high_global][i_very_high_global].altitude = max(this->global_very_high_depth_map[j_very_high_global][i_very_high_global].altitude, dvl_coo[i].z);
						else
							this->global_very_high_depth_map[j_very_high_global][i_very_high_global].altitude = dvl_coo[i].z;
						//altitude_color = max(min(255, (5 + global_very_high_depth_map[j_very_high_global][i_very_high_global].altitude) * 255 / 5), 0);
						altitude_color = (-this->global_very_high_depth_map[j_very_high_global][i_very_high_global].altitude + altitude_min) / (-altitude_max + altitude_min) * (255 - 0) + 0;
						altitude_color = 127; //debug
						// Draw
						cv::rectangle(this->displayed_very_high_global_map, cv::Point(i_very_high_global, j_very_high_global), cv::Point(i_very_high_global, j_very_high_global), altitude_color, 1);
					}

					// High map
					if (is_good_value) {
						if ((this->global_high_depth_map[j_global][i_global].altitude < 0))
							this->global_high_depth_map[j_global][i_global].altitude = max(this->global_high_depth_map[j_global][i_global].altitude, dvl_coo[i].z);
						else
							this->global_high_depth_map[j_global][i_global].altitude = dvl_coo[i].z;
						//altitude_color = max(min(255, (5 + global_high_depth_map[j_global][i_global].altitude) * 255 / 5), 0);
						altitude_color = (-this->global_high_depth_map[j_global][i_global].altitude + altitude_min) / (-altitude_max + altitude_min) * (255 - 0) + 0;
						altitude_color = 127; //debug

						// Draw
						cv::rectangle(this->displayed_global_map, cv::Point(i_global, j_global), cv::Point(i_global, j_global), altitude_color, 1);

					}
				}
			}	
		}


		// Fill the very high resolution local space depth map
		for (size_t i = 0; i < this->very_high_depth_map.size(); i++)
		{
			for (size_t j = 0; j < this->very_high_depth_map[0].size(); j++)
			{
				double x = i * (double)this->very_high_resolution - (double)this->very_high_depth_map.size() / 2 * (double)this->very_high_resolution;
				double y = j * (double)this->very_high_resolution - (double)this->very_high_depth_map[0].size() / 2 * (double)this->very_high_resolution;
				x /= 100; // Make it in meter
				y /= 100;
				y = -y;   // Make it in the robot coordinate system

				convertit_coordonnees_x_y_locales_en_i_j_tres_grande_carte(
					origine_global_map_latitude, origine_global_map_longitude,
					gps_P_latitude, gps_P_longitude,
					gps_S_cap,
					x,
					y,
					&i_global, &j_global);

				if(i_global != -1 && j_global != -1)
					this->very_high_depth_map[i][j].altitude = this->global_very_high_depth_map[j_global][i_global].altitude;
			}
		}

		// Fill the high resolution local space depth map
		for (size_t i = 0; i < this->high_depth_map.size(); i++)
		{
			for (size_t j = 0; j < this->high_depth_map[0].size(); j++)
			{
				double x = i * (double)this->high_resolution - (double)this->high_depth_map.size() / 2 * (double)this->high_resolution;
				double y = j * (double)this->high_resolution - (double)this->high_depth_map[0].size() / 2 * (double)this->high_resolution;
				x /= 100; // Make it in meter
				y /= 100;
				y = -y;   // Make it in the robot coordinate system

				convertit_coordonnees_x_y_locales_en_i_j_grande_carte(
					origine_global_map_latitude, origine_global_map_longitude,
					gps_P_latitude, gps_P_longitude,
					gps_S_cap,
					x,
					y,
					&i_global, &j_global);

				if (i_global != -1 && j_global != -1)
					this->high_depth_map[i][j].altitude = this->global_high_depth_map[j_global][i_global].altitude;
			}
		}

		//cv::applyColorMap(this->displayed_global_map, this->displayed_global_map, cv::COLORMAP_JET);
		//cv::imshow("Global map", this->displayed_global_map);
		//cv::waitKey(30);


		//// Set the coorinates in the depth map coordinates space
		//for (size_t i = 0; i < 4; i++) {
		//	dvl_coo[i].x = dvl_coo[i].x * this->low_resolution * 2;
		//	dvl_coo[i].y = dvl_coo[i].y * this->low_resolution * 2 * -1;
		//}
		//robot_coo.x = robot_coo.x * this->low_resolution * 2;
		//robot_coo.y = robot_coo.y * this->low_resolution * 2 * -1;

		//for (size_t i = 0; i < 4; i++) {
		//	this->beams[i] = cv::Point(dvl_coo[i].x + robot_coo.x, dvl_coo[i].y + robot_coo.y) + this->center;
		//	this->beams[i].x = int(this->beams[i].x / this->high_resolution);
		//	this->beams[i].y = int(this->beams[i].y / this->high_resolution);
		//}
			
		// Filter artefacts and set the depth map
		//valuesFiltering(dvl_distances, dvl_coo);

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

	void valuesFiltering(float dvl_distances[4], cv::Point3f dvl_coo[4]) {
		
		bool is_good_value = true;

		// Check value for each DVL beam
		for (size_t i = 0; i < 4; i++) {
			is_good_value = true;

			if ((this->beams[i].x >= 0) && (this->beams[i].y >= 0) && (this->beams[i].x <= this->high_depth_map.size()) && (this->beams[i].y <= this->high_depth_map[0].size())) {
				// Check value for the 4 last values
				for (size_t j = 0; j < 4; j++) {
					// Filter some artefacts, if at least one value is wrong, is_good_value == false
					if ((this->coeff_filter * this->last_distances[j][i] > dvl_distances[i]) && (this->last_distances[j][i] / this->coeff_filter < dvl_distances[i]))
						is_good_value *= true;
					else
						is_good_value *= false;
				}

				if (is_good_value) {
					if (this->high_depth_map[this->beams[i].x][this->beams[i].y].altitude < 0) 
						this->high_depth_map[this->beams[i].x][this->beams[i].y].altitude = max(high_depth_map[this->beams[i].x][this->beams[i].y].altitude, dvl_coo[i].z);
					else
						this->high_depth_map[this->beams[i].x][this->beams[i].y].altitude = dvl_coo[i].z;
				}
			}
		}

		//this->high_depth_map[5][10].altitude = -3;
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

	float getDepth(cv::Point mouse) {

		// 52 - 32 high_depth_map[51][31]  h = 32 w = 52

		float depth = 0;
		if (this->which_res == 1) {

			mouse.y = this->displayed_size.height - mouse.y;
			int row = int(mouse.y / 10);
			int col = int(mouse.x / 10);

			// Not stable
			//depth = this->high_depth_map[row][col].altitude;
		}
			
		else if (this->which_res == 2) {

			mouse.y = this->displayed_size.height - mouse.y;
			int row = int(mouse.y / 20);
			int col = int(mouse.x / 20);

			// Not stable
			//depth = this->low_depth_map[row][col];
		}
			

		return depth;
	}

	void setTide(cv::Point3f dvl_coo[4]) {
		float new_tide = 0;
		float delta[4] = { 0, 0, 0, 0 };
		if (this->is_freezed) {

			for (size_t i = 0; i < 4; i++)
				if ((this->beams[i].x >= 0) && (this->beams[i].y >= 0) && (this->beams[i].x <= this->high_depth_map.size()) && (this->beams[i].y <= this->high_depth_map[0].size()))
					if (this->freezed_high_depth_map[this->beams[i].x][this->beams[i].y].altitude < 0)
						delta[i] = this->freezed_high_depth_map[this->beams[i].x][this->beams[i].y].altitude - dvl_coo[i].z;
	
			for (size_t i = 0; i < 4; i++)
				new_tide += delta[i];
			new_tide /= 4;

			this->tide = this->tide_coeff * new_tide + (1 - this->tide_coeff) * tide;
		}
	}

	void freezeDepthMap() {
		// Init high depth map
		for (size_t i = 0; i < this->high_depth_map.size(); i++)
			for (size_t j = 0; j < this->high_depth_map[0].size(); j++)
				this->freezed_high_depth_map[i][j].altitude = this->high_depth_map[i][j].altitude;
		this->is_freezed = true;
	}

	void setDepthMap() {
		unsigned char altitude_color = 0;
		cv::Point p1(0, 0);
		cv::Point p2(0, 0);
		int low_grid_x = 0, low_grid_y = 0;

		// Very high resolution
		for (size_t j = 0; j < this->very_high_depth_map.size(); j++) {
			for (size_t i = 0; i < this->very_high_depth_map[j].size(); i++) {
				p1.x = this->very_high_resolution * j;
				p1.y = this->very_high_resolution * i;
				p2.x = this->very_high_resolution * (j + 1);
				p2.y = this->very_high_resolution * (i + 1);
				/*altitude_color = max(min(255, (6 + high_depth_map[j][i].altitude) * 255 / 6), 3);
				if(altitude_color < 250)
					log(std::to_string(altitude_color));*/
				//altitude_color = max(min(255, (5 + very_high_depth_map[j][i].altitude) * 255 / 5), 0);
				// altitude_color = (very_high_depth_map[j][i].altitude - altitude_min) / (altitude_max - altitude_min) * (255 - 0) + 0; but we want to reverse the color
				altitude_color = (-very_high_depth_map[j][i].altitude + altitude_min) / (-altitude_max + altitude_min) * (255 - 0) + 0;
				// 
				//altitude_color = max(min(255, (- high_depth_map[j][i].altitude) * 255 / 5), 0); // To invert the color
				if (this->which_res == 0) {
					cv::rectangle(this->map, cv::Rect(p1, p2), cv::Scalar(altitude_color, altitude_color, altitude_color), -1, cv::LINE_8);
				}
					
			}
		}

		// High resolution
		for (size_t j = 0; j < this->high_depth_map.size(); j++) {
			for (size_t i = 0; i < this->high_depth_map[j].size(); i++) {
				p1.x = this->high_resolution * j;
				p1.y = this->high_resolution * i;
				p2.x = this->high_resolution * (j + 1);
				p2.y = this->high_resolution * (i + 1);
				/*altitude_color = max(min(255, (6 + high_depth_map[j][i].altitude) * 255 / 6), 3);
				if(altitude_color < 250)
					log(std::to_string(altitude_color));*/

				//altitude_color = max(min(255, (5 + high_depth_map[j][i].altitude) * 255 / 5), 0);
				altitude_color = (-high_depth_map[j][i].altitude + altitude_min) / (-altitude_max + altitude_min) * (255 - 0) + 0;
				// 
				//altitude_color = max(min(255, (- high_depth_map[j][i].altitude) * 255 / 5), 0); // To invert the color

				//altitude_color = (-low_depth_map[j][i] + altitude_min) / (-altitude_max + altitude_min) * (255 - 0) + 0;

				if(this->which_res == 1)
					cv::rectangle(this->map, cv::Rect(p1, p2), cv::Scalar(altitude_color, altitude_color, altitude_color), -1, cv::LINE_8);
				low_grid_x = int(j / this->resolution_ratio);
				low_grid_y = int(i / this->resolution_ratio);
				if (low_depth_map[low_grid_x][low_grid_y] < 0) {
					if (high_depth_map[j][i].altitude < 0)
						low_depth_map[low_grid_x][low_grid_y] = max(low_depth_map[low_grid_x][low_grid_y], high_depth_map[j][i].altitude);
				}
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
				// altitude_color = max(min(255, (5 + low_depth_map[j][i]) * 255 / 5), 0);
				altitude_color = (-low_depth_map[j][i] + altitude_min) / (-altitude_max + altitude_min) * (255 - 0) + 0;
				if (this->which_res == 2)
					cv::rectangle(this->map, cv::Rect(p1, p2), cv::Scalar(int(altitude_color), int(altitude_color), int(altitude_color)), -1, cv::LINE_8);
			}
		}
		cv::applyColorMap(this->map, this->map, cv::COLORMAP_JET);



		// Make black squares when altitude is unknown
		if (this->which_res == 0)
			for (size_t j = 0; j < this->very_high_depth_map.size(); j++) {
				for (size_t i = 0; i < this->very_high_depth_map[j].size(); i++) {
					if (very_high_depth_map[j][i].altitude == 0) {
						p1.x = this->very_high_resolution * j;
						p1.y = this->very_high_resolution * i;
						p2.x = this->very_high_resolution * (j + 1);
						p2.y = this->very_high_resolution * (i + 1);
						cv::rectangle(this->map, cv::Rect(p1, p2), cv::Scalar(int(0), int(0), int(0)), -1, cv::LINE_8);
					}
					
				}
			}
		if (this->which_res == 1)
			for (size_t j = 0; j < this->high_depth_map.size(); j++) {
				for (size_t i = 0; i < this->high_depth_map[j].size(); i++) {
					if (high_depth_map[j][i].altitude == 0) {
						p1.x = this->high_resolution * j;
						p1.y = this->high_resolution * i;
						p2.x = this->high_resolution * (j + 1);
						p2.y = this->high_resolution * (i + 1);
						cv::rectangle(this->map, cv::Rect(p1, p2), cv::Scalar(int(0), int(0), int(0)), -1, cv::LINE_8);
					}
				}
			}
		if (this->which_res == 2)
			for (size_t j = 0; j < this->low_depth_map.size(); j++) {
				for (size_t i = 0; i < this->low_depth_map[j].size(); i++) {
					if (low_depth_map[j][i] == 0) {
						p1.x = this->low_resolution * j;
						p1.y = this->low_resolution * i;
						p2.x = this->low_resolution * (j + 1);
						p2.y = this->low_resolution * (i + 1);
						cv::rectangle(this->map, cv::Rect(p1, p2), cv::Scalar(int(0), int(0), int(0)), -1, cv::LINE_8);
					}
				}
			}

		double target_x_temp = this->x_target;
		double target_y_temp = this->y_target;
		target_x_temp = target_x_temp * this->low_resolution * 2;
		target_y_temp = - target_y_temp * this->low_resolution * 2;
		cv::Point p_target_temp = cv::Point(target_x_temp, target_y_temp);
		cv::Point p_target = p_target_temp - this->robot_center;
		cv::circle(this->map, p_target, 10, cv::Scalar(255, 0, 255), 2);

		//target.x = target.x * this->low_resolution * 2;
		//target.y = -target.y * this->low_resolution * 2; // -1 * y because image y axis
		//target = cv::Point(target) + this->center;//; + this->center - this->robot_center;// +this->robot_center;

		this->displayed_map = this->map.clone();
	}

	void drawGrid() {
		// Draw the grid for all resolution but le highest (5x5)
		if (this->which_res != 0) {
			int resolution = 0;
			if (this->which_res == 0)
				resolution = this->very_high_resolution;
			else if (this->which_res == 1)
				resolution = this->high_resolution;
			else if (this->which_res == 2)
				resolution = low_resolution;

			for (size_t i = 0; i < this->width; i += resolution)
				cv::line(this->map, cv::Point(i, 0), cv::Point(i, this->height), cv::Scalar(255, 255, 255), 1);
			for (size_t i = 0; i < this->height; i += resolution)
				cv::line(this->map, cv::Point(0, i), cv::Point(this->width, i), cv::Scalar(255, 255, 255), 1);
		}
		
		this->displayed_map = this->map.clone();

		// Draw center pos
		cv::circle(this->displayed_map, cv::Point(this->width / 2, this->height / 2), 4, cv::Scalar(255, 255, 255), -1);
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

	/*------------------------------
		GPS RELATED
	-------------------------------*/

	double distance_entre_deux_points_geographiques(double lat1, double long1, double lat2, double long2)
	{
		//calcule la distance en mètres entre deux points géographiques exprimés en DD.dddddd.
		double delta = DEG2RAD * (long2 - long1);
		double cdlong = cos(delta);
		lat1 = DEG2RAD * lat1;
		lat2 = DEG2RAD * lat2;
		double slat1 = sin(lat1);
		double clat1 = cos(lat1);
		double slat2 = sin(lat2);
		double clat2 = cos(lat2);
		delta = acos(slat1 * slat2 + clat1 * clat2 * cdlong);
		return delta * RAYON_TERRE;
	}

	double cap_pour_aller_du_point_1_au_point_2(double lat1, double long1, double lat2, double long2)
	{
		// calcule l'angle d'orientation en radians (entre 0 pour le Nord et 2PI) du vecteur allant de la position 1 à la position 2,
		// exprimés en DD.dddddd (signés)
		double dlon = DEG2RAD * (long2 - long1);
		lat1 = DEG2RAD * lat1;
		lat2 = DEG2RAD * lat2;
		double a1 = sin(dlon) * cos(lat2);
		double a2 = sin(lat1) * cos(lat2) * cos(dlon);
		a2 = cos(lat1) * sin(lat2) - a2;
		a2 = atan2(a1, a2);
		if (a2 < 0.0)
		{
			a2 += 2 * PI;
		}
		return a2;  //radians
	}

	void calcul_coordonnees_geo_point(double lat_GPS_babord, double long_GPS_babord, double distance, double azimut, double* lat_P, double* long_P)
	{
		//calcule les coordonnées géographiques d'un point défini par sa distance en mètres et son azimut en radians par rapport à un point geographique (dans notre cas, le GPS bâbord)
		lat_GPS_babord = DEG2RAD * lat_GPS_babord;
		long_GPS_babord = DEG2RAD * long_GPS_babord;
		*lat_P = asin(sin(lat_GPS_babord) * cos(distance / RAYON_TERRE) + cos(lat_GPS_babord) * sin(distance / RAYON_TERRE) * cos(azimut));
		*long_P = long_GPS_babord + atan2(sin(azimut) * sin(distance / RAYON_TERRE) * cos(lat_GPS_babord), cos(distance / RAYON_TERRE) - sin(lat_GPS_babord) * sin(*lat_P));
		*lat_P *= RAD2DEG;
		*long_P *= RAD2DEG;
		return;
	}

	void calcul_coordonnees_i_j_du_point_P_dans_carte_globale(double lat_orig_carte, double long_orig_carte, double lat_P, double long_P, int nb_cases_cote_carte, int longueur_cote_carte, int* i, int* j)
	{
		//origine de la carte en haut à gauche (nord ouest). Axe i orienté vers la droite (est) et Axe j orienté vers le bas (sud)
		//La fonction renvoie les coordonnées i,j dans la matrice (nb_cases_cote_carte x nb_cases_cote_carte), ou bien renvoie -1 si le point est hors de la carte
		double distance_par_rapport_origine_carte = distance_entre_deux_points_geographiques(lat_orig_carte, long_orig_carte, lat_P, long_P);
		double azimut_point_P_par_rapport_origine_carte = cap_pour_aller_du_point_1_au_point_2(lat_orig_carte, long_orig_carte, lat_P, long_P);
		//printf("distance,azimut du point P par rapport a origine carte globale = %lf m , %lf deg", distance_par_rapport_origine_carte, RAD2DEG*azimut_point_P_par_rapport_origine_carte);
		double x_carte_globale = distance_par_rapport_origine_carte * sin(azimut_point_P_par_rapport_origine_carte);
		double y_carte_globale = -distance_par_rapport_origine_carte * cos(azimut_point_P_par_rapport_origine_carte);
		*i = (int)(x_carte_globale * nb_cases_cote_carte / longueur_cote_carte);
		*j = (int)(y_carte_globale * nb_cases_cote_carte / longueur_cote_carte);
		if ((*i < 0) || (*j < 0) || (*i >= nb_cases_cote_carte) || (*j >= nb_cases_cote_carte)) //si le point est hors de la carte
		{
			*i = -1;
			*j = -1;
		}
		return;
	}

	void coordonnees_geo_d_un_point_P_x_y_du_repere_robot(double lat_GPS_babord, double long_GPS_babord, double x, double y, double* lat_P, double* long_P, double cap_GPS_Babord_vers_Tribord)
	{
		//Le GPS bâbord est le Master et est le centre du repère GPS
		double x_dans_repere_gps_babord = x - X_GPS_BABORD_DANS_REPERE_ROBOT;                //Les axes de ce reprèe sont orientés comme ceux du robot. Le centre est sur le poteau du GPS arrière bâbord.
		double y_dans_repere_gps_babord = y - Y_GPS_BABORD_DANS_REPERE_ROBOT;
		double azimut_de_P_dans_repere_GPS_babord = -atan2(x_dans_repere_gps_babord, -y_dans_repere_gps_babord); //Angle compté négatif dans le sens horaire à partir de l'axe -y (i.e. l'axe calculé par la fonction GPS RTK qui mesure l'axe Master -> Slave)
		//printf("azimut local = %lf\n", RAD2DEG*azimut_de_P_dans_repere_GPS_babord);
		double azimut_de_P_dans_repere_geographique = cap_GPS_Babord_vers_Tribord + azimut_de_P_dans_repere_GPS_babord;
		//printf("azimut geographique = %lf\n", RAD2DEG*azimut_de_P_dans_repere_geographique);
		double distance_de_P_par_rapport_au_GPS_babord = sqrt(x_dans_repere_gps_babord * x_dans_repere_gps_babord + y_dans_repere_gps_babord * y_dans_repere_gps_babord);
		//printf("distance du point P par rapport au GPS babord = %lf\n", distance_de_P_par_rapport_au_GPS_babord);
		double latP, longP;
		calcul_coordonnees_geo_point(lat_GPS_babord, long_GPS_babord, distance_de_P_par_rapport_au_GPS_babord, azimut_de_P_dans_repere_geographique, &latP, &longP);
		//printf("Coordonnees geographiques de P = %lf %lf\n", latP, longP);
		*lat_P = latP;
		*long_P = longP;
		return;
	}

	void convertit_coordonnees_x_y_locales_en_i_j_grande_carte(double lat_orig_carte, double long_orig_carte, double latitude_GPS_Master_babord, double longitude_GPS_Master_babord, double cap_GPS_babord_vers_tribord, double x_dans_repere_robot, double y_dans_repere_robot, int* i_carte_globale, int* j_carte_globale)
	{
		//cette fonction renvoie i = -1 et j =-1 si l'une des coordonnées est hors de la carte globale, sinon ça renvoie les coordonnées i,j de la carte globale en fonction des x,y du repère robot.
		//origine de la carte en haut à gauche (nord ouest). Axe i orienté vers la droite (est) et Axe j orienté vers le bas (sud)
		double lat_P_temp, long_P_temp;
		coordonnees_geo_d_un_point_P_x_y_du_repere_robot(latitude_GPS_Master_babord, longitude_GPS_Master_babord, x_dans_repere_robot, y_dans_repere_robot, &lat_P_temp, &long_P_temp, cap_GPS_babord_vers_tribord);
		calcul_coordonnees_i_j_du_point_P_dans_carte_globale(lat_orig_carte, long_orig_carte, lat_P_temp, long_P_temp, NB_CASES_COTE_CARTE_GLOBALE, LONGUEUR_COTE_CARTE_GLOBALE, i_carte_globale, j_carte_globale);
	}

	void convertit_coordonnees_x_y_locales_en_i_j_tres_grande_carte(double lat_orig_carte, double long_orig_carte, double latitude_GPS_Master_babord, double longitude_GPS_Master_babord, double cap_GPS_babord_vers_tribord, double x_dans_repere_robot, double y_dans_repere_robot, int* i_carte_globale, int* j_carte_globale)
	{
		//cette fonction renvoie i = -1 et j =-1 si l'une des coordonnées est hors de la carte globale, sinon ça renvoie les coordonnées i,j de la carte globale en fonction des x,y du repère robot.
		//origine de la carte en haut à gauche (nord ouest). Axe i orienté vers la droite (est) et Axe j orienté vers le bas (sud)
		double lat_P_temp, long_P_temp;
		coordonnees_geo_d_un_point_P_x_y_du_repere_robot(latitude_GPS_Master_babord, longitude_GPS_Master_babord, x_dans_repere_robot, y_dans_repere_robot, &lat_P_temp, &long_P_temp, cap_GPS_babord_vers_tribord);
		calcul_coordonnees_i_j_du_point_P_dans_carte_globale(lat_orig_carte, long_orig_carte, lat_P_temp, long_P_temp, NB_CASES_COTE_CARTE_GLOBALE_VERY_HIGH, LONGUEUR_COTE_CARTE_GLOBALE, i_carte_globale, j_carte_globale);
	}

	//void get_i_j_from_gps_pos() {

	//	calcul_coordonnees_i_j_du_point_P_dans_carte_globale(double lat_orig_carte, double long_orig_carte, 
	//		double lat_P, double long_P, 
	//		int nb_cases_cote_carte, int longueur_cote_carte, 
	//		int* i, int* j)

	//}

};

