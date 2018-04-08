#include<opencv2/opencv.hpp>
//#include<opencv2\core\core.hpp>
#include "client_test.h"
#include<iostream>
#include<math.h>
//#include "windows.h"

using namespace std;
using namespace cv;

client_test::client_test()//
{
}

client_test::~client_test()
{
}

Mat frame;
Mat hsvFrame;
Mat thresholdedFrame;

int currentMode;
Vec3b filterColour;
int fthreshold;

Vec3i hsvRange;

int showFilter;
Vec3i trackedPos;

int exposure;
int gain;
int parameter;

void onMouseEventMenu(int event, int x, int y, int flags, void* userdata) //
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
		createTrackbar("Parameter", "cam_show", &parameter, 20, NULL);
		createTrackbar("Exposure", "cam_show", &exposure, 100, NULL);
		createTrackbar("Gain", "cam_show", &gain, 100, NULL);
		cap.set(parameter, exposure / 10.0);
		cap.set(CAP_PROP_BRIGHTNESS, exposure/10.0);
		cap.set(CAP_PROP_GAIN, gain/10.0);
		imshow("cam_show", frame);
	}
}

void onMouseEventFilteredCam(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONUP)
	{
		if (!showFilter) { filterColour = hsvFrame.at<Vec3b>(Point(x, y)); }
		
	}
}

Vec3i getBallPos() // Les deux premieres valeurs sont les coordonnées x y et la troisieme est le rayon
{
	Point2f position;
	float radius;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(thresholdedFrame, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // On détecte les contours de l'image seuil
	
	if (contours.size() <= 0) {return Vec3i(0, 0, 0);} // Si aucun contour on retourne rien
	int largestContourIndex = 0;
	int largestContourArea = 0;
	for (int i=0; i < contours.size(); i++)// On trouve le contour le plus grand
	{
		if (double c = contourArea(contours[i]) > largestContourArea) { largestContourIndex = i; largestContourArea = c; }
	}
	minEnclosingCircle(contours[largestContourIndex], position, radius);
	return Vec3i(position.x, position.y, radius);
}

void showfilteredCam(VideoCapture cap)
{
	Mat fullframe;
	cap.read(fullframe);
	resize(fullframe, frame, Size(640, 480));
	setMouseCallback("cam_show", onMouseEventFilteredCam, &frame);
	
	if (!fullframe.empty())
	{
		Mat filteredFrame;
		Mat erodedFrame;
		createTrackbar("Hue Range", "cam_show", &hsvRange[0], 255, NULL);
		createTrackbar("Saturation Range", "cam_show", &hsvRange[1], 255, NULL);
		createTrackbar("Value Range", "cam_show", &hsvRange[2], 255, NULL);
		createTrackbar("filter", "cam_show", &showFilter, 1, NULL);
		cvtColor(frame, hsvFrame, CV_BGR2HSV);
		inRange(hsvFrame, Scalar(filterColour.val[0]- hsvRange[1], filterColour.val[1] - hsvRange[1], filterColour.val[2] - hsvRange[2]),
		Scalar(filterColour.val[0] + fthreshold, filterColour.val[1] + fthreshold, filterColour.val[2] + fthreshold), filteredFrame);

		erode(filteredFrame, erodedFrame, Mat(), Point(-1, -1), 2);
		dilate(erodedFrame, thresholdedFrame, Mat(), Point(-1, -1), 2);
		trackedPos = getBallPos();
		//Vec3b pColor = finalFrame.at<Vec3b>(0, 0);
		//cout << trackedPos.val[0] << " " << trackedPos.val[1] << endl;
		circle(frame,Point(trackedPos.val[0], trackedPos.val[1]), trackedPos.val[2],Scalar(0,0,255),2,8,0);
		if (showFilter) 
		{
			imshow("cam_show", thresholdedFrame);
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
	cap.set(CV_CAP_PROP_GAIN, 5);
	cap.set(CV_CAP_PROP_EXPOSURE, 500);
	//cap.set(CV_CAP_PROP_SETTINGS, 0);
	cap.set(CV_CAP_DSHOW, 0);

	exposure = 5;
	gain = 5;
	cout << "Get Exposure : " << cap.get(CV_CAP_PROP_EXPOSURE) << endl;
	cout << "Get Gain : " << cap.get(CV_CAP_PROP_GAIN) << endl;
	cout << "Get FPS : " << cap.get(CV_CAP_PROP_FPS) << endl;
	cout << "Get Brightness : " << cap.get(CV_CAP_PROP_BRIGHTNESS) << endl;
	cout << "Get Width : " << cap.get(CV_CAP_PROP_FRAME_WIDTH) << endl;

	cout << "Exposure :" << cap.set(CAP_PROP_EXPOSURE, exposure) << endl;
	//cap.set(CAP_PROP_GAIN, gain);

	namedWindow("client_test", WINDOW_AUTOSIZE);
	imshow("client_test", menuBackground);

	int input = 0;
	currentMode = 0;
	fthreshold = 40;
	showFilter = 0;
	filterColour.val[0] = 0; filterColour.val[1] = 0; filterColour.val[2] = 0;
	hsvRange.val[0] = 90; hsvRange.val[1] =90; hsvRange.val[2] = 90;
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
