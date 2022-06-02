#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"


class DepthMap {
public:
	struct map_elmt {
		int time;
		int depth;
		int altitude;
	};

private:
	DepthMap() {
	}

	~DepthMap() {
	}


};

