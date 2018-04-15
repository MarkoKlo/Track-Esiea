#include "Tracker.h"

using namespace std;
using namespace cv;

Tracker::Tracker(float ballRadius, float cameraFocalLength, Point3f* position, Point3f* speed)
{
	m_position = position;
	m_speed = speed;
	m_focalLength = cameraFocalLength;
	m_ballRadius = ballRadius;
	set_filter_range(Vec3i(25,80,80) );
	set_filter_color(Scalar(255,255,255) );
}


void Tracker::init_tracker(int cameraIndex, bool stereo) // Initialisation du tracker
{
	m_videoCap = VideoCapture(cameraIndex);
	m_videoCap.set(CAP_PROP_FRAME_WIDTH, 640);
	m_videoCap.set(CAP_PROP_FRAME_HEIGHT, 480);
	m_videoCap.set(CAP_PROP_FPS, 480);
	track();
}

void Tracker::track()
{
	m_videoCap.read(videoFrame); // Lecture du flux camera

	if (!videoFrame.empty)
	{
		Point3f circle2Dcoord = Point3f(0, 0, 0);
		Point3f raw_position = Point3f(0, 0, 0);
		color_filtering(); // On filtre la couleur
		circle_fitting(circle2Dcoord);
		mono_position_estimation(circle2Dcoord, raw_position);
		*m_position = raw_position;
	}
}

void Tracker::set_filter_color(Scalar color) // Règle la couleur du filtre
{
	m_filterColor = color;
}

void Tracker::set_filter_range(Vec3i m_hsvRange) // Règle la tolérance du filtre
{
	m_hsvRange = m_hsvRange;
}

void Tracker::color_filtering() // Filtre la couleur
{
	if (!videoFrame.empty())
	{
		Mat binaryFrame;
		Mat erodedFrame;
		Mat hsvFrame;
		cvtColor(videoFrame, hsvFrame, CV_BGR2HSV);
		inRange(hsvFrame, Scalar(m_filterColor.val[0] - m_hsvRange[0], m_filterColor.val[1] - m_hsvRange[1], m_filterColor.val[2] - m_hsvRange[2]),
			Scalar(m_filterColor.val[0] + m_hsvRange[0], m_filterColor.val[1] + m_hsvRange[1], m_filterColor.val[2] + m_hsvRange[2]), binaryFrame);

		erode(binaryFrame, erodedFrame, Mat(), Point(-1, -1), 2);
		dilate(erodedFrame, filteredFrame, Mat(), Point(-1, -1), 2);
	}
}

vector<Point> Tracker::get_largest_contour(vector<vector<Point> > contours) // Retourne le contour le plus grand
{
	int largestContourIndex = 0;
	double largestContourArea = 0;
	for (int i = 0; i < (int)contours.size(); i++)
	{
		double c = contourArea(contours[i]);
		if (c > largestContourArea) { largestContourIndex = i; largestContourArea = c; }
	}
	return contours[largestContourIndex];
}

void Tracker::circle_fitting(Point3f& circleCoord) // Donne la position 2D et le rayon du cercle dans circleCoord
{
	Point2f position;
	float radius;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	// Recherche des contours dans l'image binaire
	findContours(filteredFrame, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
	// Si aucun contour, ne rien faire
	if (contours.size() <= 0) { return; }
	// Sinon trouver le contour le plus grand et calculer le cercle englobant
	vector<Point> largestContour = get_largest_contour(contours);
	minEnclosingCircle(largestContour, position, radius);

	circleCoord.x = position.x;
	circleCoord.y = position.y;
	circleCoord.z = radius;
}

void Tracker::mono_position_estimation(Point3f circleCoord, Point3f& outPosition)
{
	float focal = m_focalLength;
	float radius = circleCoord.z;

	// Centrage de la position
	float x_px = circleCoord.x - 320; // Résolution hardcodée -> à changer
	float y_px = 240 - circleCoord.y;

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

	// Remplissage de la variable de sortie
	outPosition.x = X_cm;
	outPosition.y = Y_cm;
	outPosition.z = Z_cm;
}
