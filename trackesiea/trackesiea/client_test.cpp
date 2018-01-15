#include<opencv2/opencv.hpp>
//#include<opencv2\core\core.hpp>
#include "client_test.h"
#include<iostream>

using namespace std;
using namespace cv;

client_test::client_test()
{
}

client_test::~client_test()
{
}

Mat frame;

int currentMode;
Vec3b filterColour;

void onMouseEventMenu(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONUP) {

		if (x > 385 && x < 599 && y>571 && y < 607) { currentMode = -1;}
		if (x > 89 && x < 550 && y>165 && y < 265) { currentMode = 1; destroyWindow("cam_show");}
		if (x > 89 && x < 550 && y>296 && y < 374) { currentMode = 2; destroyWindow("image_show"); }
		if (x > 89 && x < 550 && y>426 && y < 505) { currentMode = 3; }
	}
}

void showCam(VideoCapture cap)
{
	cap.read(frame);
	if (!frame.empty())
	{
		imshow("cam_show", frame);
	}
}

void onMouseEventFilteredCam(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONUP)
	{
		if (filterColour.val[0] == 0 && filterColour.val[1] == 0 && filterColour.val[2] == 0) { filterColour = frame.at<Vec3b>(Point(x, y)); }
		else { filterColour.val[0] = 0; filterColour.val[1] = 0; filterColour.val[2] = 0; }
	}
}

void showfilteredCam(VideoCapture cap)
{
	cap.read(frame);
	setMouseCallback("cam_show", onMouseEventFilteredCam, &frame);
	if (!frame.empty())
	{
		Mat filteredFrame;
		Mat erodedFrame;
		Mat finalFrame;

		inRange(frame, Scalar(filterColour.val[0]-45, filterColour.val[1] - 45, filterColour.val[2] - 45),
			Scalar(filterColour.val[0] + 45, filterColour.val[1] + 45, filterColour.val[2] + 45), filteredFrame);
		erode(filteredFrame, finalFrame, Mat(), Point(-1, -1), 2, 1, 1);
		//dilate(erodedFrame, finalFrame, Mat(), Point(-1, -1), 2, 1, 1);

		if (filterColour.val[0] == 0 && filterColour.val[1] == 0 && filterColour.val[2] == 0) 
		{
			imshow("cam_show", frame);
		}
		else {
			imshow("cam_show", finalFrame);
		}
	}
}

int main(int argc, char** argv)
{
	// cout est le flux de texte du terminal, '<<' permet d'injecter un élément dans cout, endl correspond à la fin de ligne
	cout << "Track'ESIEA est un pst de 2A qui vise a effectuer le tracking 3D d'un objet dans un espace delimite." << endl; 
	cout << "Librairie utilisée : OpenCV." << endl;

	Mat menuBackground = imread("images/menu.jpg"); // Chargement de l'image
	if (!menuBackground.data) // Sécurité
	{
		cout << "Impossible d'ouvrir le menu." << endl;
		waitKey(0);
		return -1;
	}

	Mat marko = imread("images/marko.jpg"); // Chargement de l'image
	if (!menuBackground.data) // Sécurité
	{
		cout << "Impossible d'ouvrir l'image." << endl;
		waitKey(0);
		return -1;
	}

	VideoCapture cap(0);
	cap.set(CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CAP_PROP_FRAME_HEIGHT, 480);

	namedWindow("client_test", WINDOW_AUTOSIZE);
	imshow("client_test", menuBackground);

	int input = 0;
	currentMode = 0;
	filterColour.val[0] = 0; filterColour.val[1] = 0; filterColour.val[2] = 0;
	setMouseCallback("client_test", onMouseEventMenu, NULL);

	while (input != 'q' && currentMode != -1 )
	{
		

		switch (currentMode)
		{
		case 1:
			imshow("image_show", marko);
			break;
		case 2:
			showCam(cap);
			break;
		case 3:
			showfilteredCam(cap);
			break;
		}
		input = waitKey(33);

	}
	return 0;
}
