#include<opencv2/opencv.hpp>
//#include<opencv2\core\core.hpp>
#include "client_test.h"
#include<iostream>
//#include "windows.h"

using namespace std;
using namespace cv;

client_test::client_test()
{
}

client_test::~client_test()
{
}

Mat frame;
Mat finalFrame;

int currentMode;
Vec3b filterColour;
int fthreshold;
int showFilter;
Vec2i trackedPos;

void onMouseEventMenu(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONUP) {

		if (x > 385 && x < 599 && y>571 && y < 607) { currentMode = -1;}
		if (x > 89 && x < 550 && y>165 && y < 265) { currentMode = 1; destroyWindow("cam_show");}
		if (x > 89 && x < 550 && y>296 && y < 374) { currentMode = 2; destroyWindow("image_show");
		namedWindow("cam_show", WINDOW_AUTOSIZE);
		}
		if (x > 89 && x < 550 && y>426 && y < 505) { currentMode = 3; destroyWindow("image_show");
		namedWindow("cam_show", WINDOW_AUTOSIZE);
		}
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
		if (!showFilter) { filterColour = frame.at<Vec3b>(Point(x, y)); }
		
	}
}

Vec2i getWhitePos()
{
	int x = 0;
	int y = 0;
	Vec3b pColor;
	for (int i = 0; i < finalFrame.rows; i++) //360
	{
		for (int j = 0; j < finalFrame.cols; j++) //640
		{
			if (finalFrame.at<uchar>(i, j) >= 255) { x += i; y += j; }
		}
	}
	x /= 360;
	y /= 640;
	return Vec2i(x, y);
}

void showfilteredCam(VideoCapture cap)
{
	Mat fullframe;
	cap.read(fullframe);
	resize(fullframe, frame, Size(640, 360));
	setMouseCallback("cam_show", onMouseEventFilteredCam, &frame);
	
	if (!fullframe.empty())
	{
		
		Mat filteredFrame;
		Mat erodedFrame;
		createTrackbar("Tolerance", "cam_show", &fthreshold, 255, NULL);
		createTrackbar("filter", "cam_show", &showFilter, 1, NULL);
		inRange(frame, Scalar(filterColour.val[0]- fthreshold, filterColour.val[1] - fthreshold, filterColour.val[2] - fthreshold),
		Scalar(filterColour.val[0] + fthreshold, filterColour.val[1] + fthreshold, filterColour.val[2] + fthreshold), filteredFrame);
		erode(filteredFrame, finalFrame, Mat(), Point(-1, -1), 2, 1, 1);
		//dilate(erodedFrame, finalFrame, Mat(), Point(-1, -1), 2, 1, 1);
		trackedPos = getWhitePos();
		//Vec3b pColor = finalFrame.at<Vec3b>(0, 0);
		cout << trackedPos.val[0] << " " << trackedPos.val[1] << endl;
		if (showFilter) 
		{
			imshow("cam_show", finalFrame);
		}
		else {
			imshow("cam_show", frame);
		}
	}
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

	Mat marko = imread("images/marko.jpg"); // Chargement de l'image
	if (!menuBackground.data) // Sécurité
	{
		cout << "Impossible d'ouvrir l'image." << endl;
		waitKey(1000);
		return -1;
	}

	VideoCapture cap(0);
	cap.set(CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CAP_PROP_FRAME_HEIGHT, 480);

	namedWindow("client_test", WINDOW_AUTOSIZE);
	imshow("client_test", menuBackground);

	int input = 0;
	currentMode = 0;
	fthreshold = 40;
	showFilter = 0;
	filterColour.val[0] = 0; filterColour.val[1] = 0; filterColour.val[2] = 0;
	trackedPos.val[0] = 0; trackedPos.val[1] = 0;
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
