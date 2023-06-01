#pragma once
#include "PracticalSocket.h"
#include "config.h"

class Bathymetry {
private:
	
	int target_x = 0, target_y = 0;
public:
	bool flag_bathy_is_clicked = false; //0 not clicked
	


public:
	/*------------------------------
		CONSTRUCTOR / DESTRUCTOR
	-------------------------------*/
	Bathymetry() {
		
	}

	~Bathymetry() {

	}

	

	void set_target(int target_x , int target_y){
		this->target_x = target_x;
		this->target_y = target_y;
	}
	cv::Point get_target() {
		
		cv::Point target = cv::Point(this->target_x, this->target_y);
		return target;
	}

};

