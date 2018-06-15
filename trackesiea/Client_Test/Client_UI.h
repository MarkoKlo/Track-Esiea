#pragma once
#include<opencv2\opencv.hpp>
#include"Tracker.h"
#include "cvui.h"

#define WINDOW_NAME "Track'ESIEA"

class Client_UI
{
public:
	/*Constructeurs*/
	Client_UI();
	~Client_UI();

	/*Variables publiques*/
	Tracker* tracker;

	/*Fonctions publiques*/
	void initialize();
	void run(bool& exit,int input);

private:

	cv::Mat frame;

	cv::Mat tracking_status_image[3]; // 0 OK 1 Average 2 BAD
	cv::Mat tab_button_img[2]; // 0 Idle 1 Over/Down

	enum tab{tracking,camera,options};
	tab current_tab;

	void status_field();
	void tab_column();

	void tracking_tab();
	Mat grid;
	Mat windowGrid;
	bool calibration_mode;
	int calibration_step;
	Mat calibration_image[4];

	void camera_tab();
	int currentBall;
	Mat windowVideo;
	cv::Point3i filterRange[2];
	int gain;
	int exposure;
	bool showBinary;
	bool showCircle;
	unsigned int getHexaColor(int index);

	void options_tab(bool &exit);
	bool hqTracking;
};

