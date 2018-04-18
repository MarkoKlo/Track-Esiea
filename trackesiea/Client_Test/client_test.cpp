//#include<opencv2/opencv.hpp>
//#include<opencv2\core\core.hpp>
#include "client_test.h"
#include<iostream>
#include<cstdio>
#include<math.h>
#include<opencv2\opencv.hpp>

#include"Tracker.h"
#include"counter_test.h"

#define PI 3.1415926536
#define BALL_SIZE 4.0
#define BALL_RADIUS 2.0
#define TEST_FOCAL_LENGTH 263.75
#define FOCAL_LENGTH_PIXEL 550.0

using namespace std;
using namespace cv;

client_test::client_test()//
{
}

client_test::~client_test()
{
}

int currentMode;
Mat debugGraph;
int counter = 0;
Vec3i hsvRange;

int showFilter = 0;
bool showCircle = false;
Point3f trackedPos;

Tracker tracker;
counter_test counte;

void onMouseEventMenu(int event, int x, int y, int flags, void* userdata) //
{
	if (event == EVENT_LBUTTONUP) {
		if (x > 11 && x < 628 && y>267 && y < 374) { currentMode = 3; destroyWindow("image_show"); // Afficher la camera
		namedWindow("cam_show", WINDOW_AUTOSIZE);
		}
		
		if (x > 12 && x < 301 && y>409 && y < 516) { showFilter = !showFilter; } // Activer/Desactiver le filtre
		if (x > 339 && x < 628 && y>409 && y < 516) { showCircle = !showCircle; } // Afficher le cercle
		if (x > 387 && x < 601 && y>571 && y < 608) { currentMode = -1;} // Quitter
	}
}
void onMouseEventFilteredCam(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONUP)
	{
		Vec3i color = tracker.get_hsv_color(Point2i(x, y));
		printf_s("Couleur changee : %d %d %d \n", color.val[0], color.val[1], color.val[2]);
		tracker.set_filter_color(color);
	}
}

void showGraph()
{
	Point3f speed = tracker.get_speed();
	Mat debugGraphFrame = debugGraph.clone();
	circle(debugGraphFrame, Point((trackedPos.x*5.0 + 500), trackedPos.z*5.0), 2, Scalar(0, 0, 255), 2, -1, 0); // XZ Debug
	char debugText[100]; sprintf_s(debugText, "X:%.1fcm  Y:%.1fcm Z: %.1fcm dX:%.1fcm  dY:%.1fcm dZ: %.1fcm", trackedPos.x, trackedPos.y, trackedPos.z,speed.x,speed.y,speed.z);
	putText(debugGraphFrame, debugText, Point2i(10, 975), CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 2, LINE_AA);
	imshow("position_graph", debugGraphFrame);
}

void showfilteredCam(Tracker tracker)
{
	Mat frame,thresholdedFrame;

	setMouseCallback("cam_show", onMouseEventFilteredCam, &frame);
	createTrackbar("Hue Range", "cam_show", &hsvRange[0], 255, NULL);
	createTrackbar("Saturation Range", "cam_show", &hsvRange[1], 255, NULL);
	createTrackbar("Value Range", "cam_show", &hsvRange[2], 255, NULL);
	tracker.set_filter_range(hsvRange);
	tracker.track();
	Point3f trackedPos2D = tracker.get_2D_position();
	trackedPos = tracker.get_position();
	frame = tracker.get_video_frame();
	thresholdedFrame = tracker.get_binary_frame();
	//tracker.get_delta_time();
	//printf("curr:%I64i last:%I64i\n", tracker.m_currentTick, tracker.m_lastTick);
	counte.track();
	if (showCircle) { circle(frame, Point(trackedPos2D.x, trackedPos2D.y), trackedPos2D.z, Scalar(0, 0, 255), 1, CV_AA, 0); }
	if (showFilter == 0)
	{imshow("cam_show", frame);}
	else if (showFilter == 1) {imshow("cam_show", thresholdedFrame);}
	//showGraph();
}

int main(int argc, char** argv)
{
	// cout est le flux de texte du terminal, '<<' permet d'injecter un élément dans cout, endl correspond à la fin de ligne
	cout << "Track'ESIEA est un pst de 2A qui vise a effectuer le tracking 3D d'un objet dans un espace delimite." << endl; 
	cout << "Librairie utilisée : OpenCV." << endl;
	waitKey(0);

	Mat menuBackground = imread("images/menu.jpg"); // Chargement de l'image
	if (!menuBackground.data) // Sécurité
	{
		cout << "Impossible d'ouvrir le menu." << endl;
		waitKey(1000);
		return -1;
	}

	debugGraph = imread("images/repere.jpg"); // Chargement de l'image
	if (!menuBackground.data) // Sécurité
	{
		cout << "Impossible d'ouvrir l'image." << endl;
		waitKey(1000);
		return -1;
	}
	int cameraIndex = 0;
	cout << "Entrer le numero de la camera : " << endl;
	scanf_s("%d", &cameraIndex);

	namedWindow("client_test", WINDOW_AUTOSIZE);
	imshow("client_test", menuBackground);

	//tracker = Tracker(BALL_RADIUS, FOCAL_LENGTH_PIXEL);
	tracker.init_tracker(cameraIndex, false);

	int input = 0;
	hsvRange.val[0] = 25; hsvRange.val[1] = 60; hsvRange.val[2] = 80;
	trackedPos.x = 0; trackedPos.y = 0;
	
	setMouseCallback("client_test", onMouseEventMenu, NULL);
	float e1, e2, time;

	while (input != 'q' && currentMode != -1 )
	{showfilteredCam(tracker);input = waitKey(13);
		/*
		e1 = getTickCount();
		
		
		e2 = getTickCount();
		time = (e2 - e1) / getTickFrequency();
		*/
		//cout << "time:" << time * 1000 << endl;
	}
	return 0;
}
