#pragma once
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <vector>
#include <algorithm>

class Robot {

private:
	// Position
	float x, y, z;
	// State
	bool waiting=true; // Waiting=true - Moving=false
	// Drawing
	float size=50;
	float radius = 0.5;

public:

	/*------------------------------

		GETTER

	-------------------------------*/

	Robot(float x, float y, float z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	/*------------------------------

		GETTER

	-------------------------------*/
	~Robot() {}

	/*------------------------------

		GETTER

	-------------------------------*/

	int* get_pos() {
		int* pos = new int[3];
		pos[0] = this->x;
		pos[1] = this->y;
		pos[2] = this->z;
		return pos;
	}

	/*------------------------------
	
		SETTER
	
	-------------------------------*/

	void set_pos(float x, float y, float z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}
};

