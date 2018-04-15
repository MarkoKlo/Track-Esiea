//#include<opencv2/opencv.hpp>
//#include<opencv2\core\core.hpp>
#include "client_test.h"
#include<iostream>
#include<cstdio>
#include<math.h>
#include<opencv2\opencv.hpp>

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

Mat frame;
Mat hsvFrame;
Mat thresholdedFrame;
Mat contoursFrame;

int currentMode;
Mat debugGraph;
int counter = 0;
Point3f lastPositions[20];

Vec3i hsvRange;
Vec3i filterColour;

int showFilter = 0;
bool showCircle = false;
Point3f trackedPos;

int exposure;
int gain;
int parameter;

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
		if (!showFilter) {
			
			int count = 0;
			filterColour = Vec3i(0, 0, 0);
			for (int i = -3; i < 4; i++) {
				for (int j = -3; j < 4; j++) { filterColour += hsvFrame.at<Vec3b>(Point(x+i, y+j)); count++; }
			}
			filterColour /= count;
			printf_s("Couleur changee : %d %d %d \n", filterColour.val[0], filterColour.val[1], filterColour.val[2]);
		}
		
	}
}

float getHorizontalFoV(float dfov,float aspect) { // retourne le fov en degrés
	float radAngle = dfov * ( PI / 180);//dfov en degrés
	float radHFOV = 2 * atan(tan(radAngle / 2.0) * aspect );
	float hFOV = (180 / PI) * radHFOV;
	return hFOV;
}

Point3f get2DBallPos() // Les deux premieres valeurs sont les coordonnées x y et la troisieme est le rayon
{
	Point2f position;
	float radius;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(thresholdedFrame, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
	if (contours.size() <= 0) {return Vec3i(0, 0, 0);} // Si aucun contour on retourne rien
	int largestContourIndex = 0;
	double largestContourArea = 0;
	for (int i=0; i < (int)contours.size(); i++)// On trouve le contour le plus grand
	{
		double c = contourArea(contours[i]);
		if (c > largestContourArea) { largestContourIndex = i; largestContourArea = c; }
	}
	minEnclosingCircle(contours[largestContourIndex], position, radius); // On calcule le cercle englobant le plus petit
	
	return Point3f(position.x, position.y, radius);
}

Point3f get3DPosition(float x, float y, float radius,float focal)
{
	focal = FOCAL_LENGTH_PIXEL;
	
	// Centrage de la position
	float x_px = x - 320;
	float y_px = 240 - y;

	// Calcul de la distance l_px entre le centre et le cercle
	float l_px = sqrt(x_px * x_px + y_px * y_px);

	// Calcul des rapports
	float K = l_px / focal;
	float J = (l_px + radius) / focal;
	float L = (J - K) / (1 + J * K);

	// Calcul de la distance relle D_cm entre le centre et la sphere
	float D_cm = (BALL_RADIUS * sqrt(1 + L * L)) / L;

	// Calcul de Z_cm la profondeur
	float fl = focal / l_px;
	float Z_cm = (D_cm*fl) / sqrt(1 + fl * fl);

	// Calcul de X_cm et Y_cm
	float L_cm = Z_cm * K;
	float X_cm = (L_cm*x_px) / l_px;
	float Y_cm = (L_cm*y_px) / l_px;

	return Point3f(X_cm, Y_cm, Z_cm);
}

void showfilteredCam(VideoCapture cap)
{
	cap.read(frame);
	setMouseCallback("cam_show", onMouseEventFilteredCam, &frame);
	
	if (!frame.empty())
	{
		Mat filteredFrame;
		Mat erodedFrame;
		createTrackbar("Hue Range", "cam_show", &hsvRange[0], 255, NULL);
		createTrackbar("Saturation Range", "cam_show", &hsvRange[1], 255, NULL);
		createTrackbar("Value Range", "cam_show", &hsvRange[2], 255, NULL);
		cvtColor(frame, hsvFrame, CV_BGR2HSV);
		inRange(hsvFrame, Scalar(filterColour.val[0]- hsvRange[0], filterColour.val[1] - hsvRange[1], filterColour.val[2] - hsvRange[2]),
		Scalar(filterColour.val[0] + hsvRange[0], filterColour.val[1] + hsvRange[1], filterColour.val[2] + hsvRange[2]), filteredFrame);

		erode(filteredFrame, erodedFrame, Mat(), Point(-1, -1), 2);
		dilate(erodedFrame, thresholdedFrame, Mat(), Point(-1, -1), 2);
		Point3f trackedPos2D = get2DBallPos();
		trackedPos = get3DPosition(trackedPos2D.x, trackedPos2D.y, trackedPos2D.z,FOCAL_LENGTH_PIXEL);
		counter++;
		if (counter%10==0) { cout << "x:" << trackedPos.x << " y:" << trackedPos.y << " z:" << trackedPos.z << endl; }

		if (showCircle) { circle(frame, Point(trackedPos2D.x, trackedPos2D.y), trackedPos2D.z, Scalar(0, 0, 255),1, CV_AA, 0); }
		if (showFilter==0) 
		{
			imshow("cam_show", frame);
		}
		else if(showFilter==1){
			imshow("cam_show", thresholdedFrame);
		}
		Mat debugGraphFrame = debugGraph.clone();
		circle(debugGraphFrame, Point( (trackedPos.x*5.0+500) , trackedPos.z*5.0 ), 2, Scalar(0, 0, 255), 2, -1, 0); // XZ Debug
		char debugText[100]; sprintf_s(debugText, "X:%.1fcm  Y:%.1fcm Z: %.1fcm",trackedPos.x,trackedPos.y,trackedPos.z);
		putText(debugGraphFrame, debugText, Point2i(10, 975), CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 2, LINE_AA);
		imshow("position_graph", debugGraphFrame);
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

	VideoCapture cap(cameraIndex);
	cap.set(CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CAP_PROP_FRAME_HEIGHT, 480);
	cap.set(CV_CAP_PROP_GAIN, 5);
	cap.set(CV_CAP_PROP_EXPOSURE, 2);
	cap.set(CV_CAP_PROP_FPS, 60);
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
	filterColour.val[0] = 0; filterColour.val[1] = 0; filterColour.val[2] = 0;
	hsvRange.val[0] = 25; hsvRange.val[1] =60; hsvRange.val[2] = 80;
	trackedPos.x = 0; trackedPos.y = 0;
	setMouseCallback("client_test", onMouseEventMenu, NULL);
	debugGraphFrame = debugGraph.clone();

	while (input != 'q' && currentMode != -1 )
	{
		

		switch (currentMode)
		{
		case 1:
			imshow("image_show", debugGraph);
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
