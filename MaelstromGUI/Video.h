#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "Bbx.h"
#include "Bboxes.h"
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>

class Video {
public:
	cv::VideoCapture video;
	cv::Mat frame;
	cv::Mat edited_frame;
	int frames_nr;
	int current_frame;
	double fps;
	int fps_factor = 1;
	int wait_timer;
	std::vector<Bboxes> video_bboxes;
	std::string mode = "pause";

	cv::Size window_size;

	
public:
	/*--------------------------------------------------
	//
	//		CONSTRUCTOR
	//	
	----------------------------------------------------*/
	Video() {}

	/*--------------------------------------------------
	//
	//		DESTRUCTOR
	//
	----------------------------------------------------*/
	~Video() {}

	/*--------------------------------------------------
	//
	//		INITIALIZATION
	//
	----------------------------------------------------*/
	void init(std::string video_path, cv::Size window_size/*, std::string bboxes_path*/) {
		this->video.open(video_path);
		this->frames_nr = this->video.get(cv::CAP_PROP_FRAME_COUNT);
		this->fps = this->video.get(cv::CAP_PROP_FPS);
		this->wait_timer = int(1000 / this->fps / this->fps_factor);
		this->current_frame = 0;
		this->window_size = window_size;
		// TODO: Read bboxes file and assign it to each frame
		cv::Mat null_img = cv::Mat::zeros(cv::Size(1, 1), CV_8UC1);
		std::vector<Bbx> null_bbx_vector;
		Bboxes null_bboxes(null_img, null_bbx_vector);
		// Assign bboxes to each frame if exist
		for (size_t i = 0; i < this->frames_nr; i++) {
			this->video_bboxes.push_back(null_bboxes);
		}
	}

	/*--------------------------------------------------
	//
	//		GETTER
	//
	----------------------------------------------------*/
public:

	std::string getMode() {
		return this->mode;
	}

	bool isOpened() {
		return this->video.isOpened();
	}
	int getFpsFactor() {
		return this->fps_factor;
	}

	int getWaitTimer() {
		return this->wait_timer;
	}

	void nextFrame() {
		this->video.read(this->frame);
		// FPS drop... Might have to adapt to the original resolution
		//resize(this->frame, this->frame, this->window_size, cv::INTER_CUBIC);
	}

	cv::Mat getFrame() {
		return this->frame;
	}

	cv::Mat getEditedFrame() {
		// Draw bboxes
		this->edited_frame = this->video_bboxes[this->getFrameId()].draw(this->frame.clone());
		return this->edited_frame;
	}

	int getFrameId() {
		this->current_frame = this->video.get(cv::CAP_PROP_POS_FRAMES) - 1;
		return this->current_frame;
	}

	int getFramesNr() {
		return this->frames_nr;
	}

private:
	double getFps() {
		return this->fps;
	}
	
	/*--------------------------------------------------
	//
	//		SETTER
	//
	----------------------------------------------------*/
public:

	void addBbx(cv::Point center) {
		this->video_bboxes[getFrameId()].add(center);
	}

	void setEditedFrame() {
		this->edited_frame = this->frame.clone();
	}

	void setFpsFactor(int factor) {
		this->fps_factor = factor;
		setWaitTimer();
	}

	void setFrame(int frame_indice) {
		this->video.set(cv::CAP_PROP_POS_FRAMES, frame_indice);
	}

	void setMode(std::string mode) {
		// Should be "pause", "play", or "replay"
		this->mode = mode;
	}

private:
	void setWaitTimer() {
		this->wait_timer = int(1000 / getFps() / getFpsFactor());
	}

	/*--------------------------------------------------
	//
	//		UPDATE
	//
	----------------------------------------------------*/
public:
	void updateBboxes(int x_mouse, int y_mouse, bool* click, bool* hold) {
		this->video_bboxes[getFrameId()].update(x_mouse, y_mouse, click, hold);
	}
};