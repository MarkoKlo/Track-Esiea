#include "Tracker.h"

using namespace std;
using namespace cv;
using namespace libconfig;

/*
Constructeurs
*/

Tracker::Tracker() : m_focalLength(PSEYE_FOCAL), m_ballRadius(BALL_RADIUS), m_filteringType(multi_channel_lowpass), m_exposure(10),
m_gain(10), initialized(false), resolutionX(640), resolutionY(480) // Constructeur par défaut
{
	set_filter_range(Vec3i(25, 80, 80));
	set_filter_color(Vec3i(255, 255, 255));
	m_last_world_position = Point3f(1, 1, 1);
	m_last_cam_position = Point3f(1, 1, 1);
}

Tracker::Tracker(float ballRadius, float cameraFocalLength) : m_focalLength(cameraFocalLength), m_ballRadius(ballRadius) // Constructeur
, m_exposure(10), m_gain(10), m_filteringType(simple_lowpass), initialized(false), resolutionX(640), resolutionY(480)
{
	set_filter_range(Vec3i(25,80,80) );
	set_filter_color(Vec3i(255,255,255) );
	m_last_world_position = Point3f(1, 1, 1);
	m_last_cam_position = Point3f(1, 1, 1);
}

Tracker::~Tracker()
{
	//free(m_currentTick); free(m_lastTick);
}

/*
FONCTIONS PUBLIQUES
*/

#ifdef USE_PS3EYEDRIVER
ps3eye_context* ctx;
#endif

void Tracker::init_tracker(int cameraIndex, bool stereo) // Initialisation du tracker
{
	load_params();
#ifdef USE_PS3EYEDRIVER
	ctx = new ps3eye_context(resolutionX, resolutionY, 60);
	if (!ctx->hasDevices()) {printf("Aucune PS3Eye connectée !\n"); return;}
	ctx->eye->setFlip(false); /* miroir left-right */
	ctx->eye->setExposure(m_exposure);
	ctx->eye->setGain(m_gain);
	ctx->eye->setAutoWhiteBalance(false);
	ctx->eye->setBlueBalance(127); ctx->eye->setRedBalance(127); ctx->eye->setGreenBalance(127);
	ctx->eye->setFrameRate(60);
	m_videoFrame = Mat(480, 640, CV_8UC3, Scalar(0, 0, 0)); // Allocation d'un mat (8U : unsigned int 8bit), (C3 3 canaux) et initialisé au noir
	ctx->eye->start();
#else
	m_videoCap = VideoCapture(cameraIndex);
	if (!m_videoCap.isOpened()) { printf("Aucune caméra connectée !\n"); return; }
	m_videoCap.set(CAP_PROP_FRAME_WIDTH, 640);
	m_videoCap.set(CAP_PROP_FRAME_HEIGHT, 480);
#endif
	//m_currentTick = (int64*)malloc(sizeof(int64));
	//m_lastTick = (int64*)malloc(sizeof(int64));
	initialized = true;
	calibrate_camera_pose();

	track();
}

void Tracker::track()
{
#ifdef USE_PS3EYEDRIVER
	ctx->eye->getFrame(m_videoFrame.data); // Lecture du flux camera
#else
	m_videoCap.read(m_videoFrame); // Lecture du flux camera
#endif
	if (!m_videoFrame.empty())
	{
		
		// Calculs de positions
		Point3f raw_position = Point3f(0, 0, 0);
		color_filtering(m_videoFrame, m_hsvRange, m_filterColor, m_filteredFrame); // Filtrage de la couleur
		circle_fitting(m_2Dposition,m_filteredFrame); // Détection de la couleur
		if (m_hqTracking) { circle_refining(m_2Dposition, m_hsvFrame, m_filterColor, m_hsvRange); }
		mono_position_estimation(m_focalLength, m_2Dposition, raw_position); // Estimation

		set_delta_time(m_currentTick,m_lastTick);
		float alpha = LOWPASS_ALPHA;
		if (m_isTrackingValid)
		{
			switch (m_filteringType)
			{
			case simple_lowpass:
				m_cam_position = LOWPASS_ALPHA * raw_position + (1.0 - LOWPASS_ALPHA) * (m_last_cam_position);
				break;
			case multi_channel_lowpass:
				m_cam_position.x = alpha * raw_position.x + (1.0 - alpha) * (m_last_cam_position).x;
				m_cam_position.y = alpha * raw_position.y + (1.0 - alpha) * (m_last_cam_position).y;
				m_cam_position.z = alpha * Z_LOWPASS_SMOOTHING * raw_position.z + (1.0 - alpha * Z_LOWPASS_SMOOTHING) * (m_last_cam_position).z;
				break;
			case noFiltering:
				m_cam_position = raw_position;
				break;
			}
			m_world_position = get_world_position(m_cam_position);
			m_speed = (m_last_world_position - m_world_position) / m_deltaTime;
		}
		// Assignations de fin
		stackPositions(m_world_position, m_last_world_positions, 300);
		get_variance();
		m_last_world_position = m_world_position;
		m_last_cam_position = m_cam_position;

		
	}
}

void Tracker::set_filter_color(Vec3i color) // Règle la couleur du filtre
{
	m_filterColor = color;
}

void Tracker::set_filter_range(Vec3i hsvRange) // Règle la tolérance du filtre
{
	m_hsvRange = hsvRange;
}

void Tracker::set_gain(int gain)
{
#ifdef USE_PS3EYEDRIVER
	m_gain = gain;
	ctx->eye->setGain(m_gain);
#endif
}

void Tracker::set_exposure(int exposure)
{
#ifdef USE_PS3EYEDRIVER
	m_exposure = exposure;
	ctx->eye->setExposure(m_exposure);
#endif
}

void Tracker::set_hq_tracking(bool hqTracking)
{
	m_hqTracking = hqTracking;
}

Vec3i Tracker::get_hsv_color(Point2i coordinates)
{
	Vec3i col = m_hsvFrame.at<Vec3b>(Point(coordinates.x, coordinates.y));
	col += m_hsvFrame.at<Vec3b>(Point(coordinates.x - 0, coordinates.y - 1));
	col += m_hsvFrame.at<Vec3b>(Point(coordinates.x - 0, coordinates.y + 1));
	col += m_hsvFrame.at<Vec3b>(Point(coordinates.x - 1, coordinates.y - 0));
	col += m_hsvFrame.at<Vec3b>(Point(coordinates.x + 1, coordinates.y - 0));
	return col / 5;
}

Vec3i Tracker::get_fullscreen_average_hsv_color()
{
	Vec3i color = Vec3i(0,0,0);
	int divider = 0;
	for (int i = 0; i < 640; i++)
	{
		for (int j = 0; j < 480; j++)
		{
			Vec3i col = m_hsvFrame.at<Vec3b>(Point(i, j));
			if(col.val[2] > 10 ){
			color += col;
			divider++;}
			
		}
	}
	color /= divider;
	return color;
}

Vec3i Tracker::get_filter_color()
{
	return m_filterColor;
}

Vec3i Tracker::get_filter_range()
{
	return m_hsvRange;
}

Mat& Tracker::get_video_frame()
{
	return m_videoFrame;
}

Mat& Tracker::get_hsv_frame()
{
	return m_hsvFrame;
}

Mat& Tracker::get_binary_frame()
{
	return m_filteredFrame;
}

void Tracker::set_delta_time(int64& cur, int64& las)
{
	las = cur;
	cur = (int64)getTickCount();
	double delta = (double)( (cur) - (las) ) ;
	delta = delta / getTickFrequency();
	m_deltaTime = delta;
}

Point3f Tracker::get_2D_position()
{
	return m_2Dposition;
}

Point3f Tracker::get_cam_position()
{
	return m_cam_position;
}

Point3f Tracker::get_world_position()
{
	return m_world_position;
}

Point3f Tracker::get_speed()
{
	return m_speed;
}

Point3f Tracker::get_variance()
{
	int i; Point3f average = Point3f(0,0,0); Point3f variance = Point3f(0,0,0);
	for (i = 0; i < m_last_world_positions.size(); i++)
	{
		average += m_last_world_positions[i];
	}
	average /= (int)m_last_world_positions.size();
	
	for (i = 0; i < m_last_world_positions.size(); i++)
	{
		variance.x = std::pow(m_last_world_positions[i].x - average.x, 2);
		variance.y = std::pow(m_last_world_positions[i].y - average.y, 2);
		variance.z = std::pow(m_last_world_positions[i].z - average.z, 2);
	}
	variance /= (int)m_last_world_positions.size();

	return variance;
}

int Tracker::get_gain()
{
	return m_gain;
}

int Tracker::get_exposure()
{
	return m_exposure;
}

Point3f Tracker::get_camera_world_position()
{
	return m_cam_world_position;
}

void Tracker::set_world_origin()
{
	m_world_origin = m_cam_position;
	cout << "Set world origin : " << m_world_origin << endl;
}

void Tracker::set_world_xaxis()
{
	m_world_x_axis = m_cam_position - m_world_origin;
	m_world_x_axis = normalize((Vec3f)m_world_x_axis);
	cout << "X+ Point : " << m_cam_position << endl;
	cout << "Set world X axis : " << m_world_x_axis << endl;
}

void Tracker::set_world_zaxis() // On récupère la position actuelle sachant que l'origine a déjà été sauvegardée
{
	m_world_z_axis = m_cam_position - m_world_origin;
	m_world_z_axis = normalize( (Vec3f) m_world_z_axis);
	cout << "Z+ Point : " << m_cam_position << endl;
	cout << "Set world Z axis : " << m_world_z_axis << endl;
}

void Tracker::calibrate_camera_pose()
{
	compute_camToWorld_rotation_matrix(m_world_z_axis,m_world_x_axis,m_world_origin);
	m_cam_world_position = m_world_origin;
}

float Tracker::get_tracking_rate()
{
	return 1.0 / m_deltaTime;
}

bool Tracker::is_hq_tracking()
{
	return m_hqTracking;
}

bool Tracker::is_tracking_valid()
{
	return m_isTrackingValid;
}

void Tracker::save_params()
{
	// Création de l'objet Configuration et ajout des groupes de paramètres
	Config cfg;
	Setting& root = cfg.getRoot();
	Setting& camParams = root.add("camera", Setting::TypeGroup);
	Setting& trackingParams = root.add("tracking", Setting::TypeGroup);
	Setting& filterParams = trackingParams.add("filter", Setting::TypeGroup);

	// Ajout des paramètres pour la partie caméra
	camParams.add("resolutionX", Setting::TypeInt) = resolutionX;
	camParams.add("resolutionY", Setting::TypeInt) = resolutionY;
	camParams.add("gain", Setting::TypeInt) = m_gain;
	camParams.add("exposure", Setting::TypeInt) = m_exposure;

	Setting& world_x_axis_setting = camParams.add("world_x_axis", Setting::TypeGroup);
	world_x_axis_setting.add("x", Setting::TypeFloat) = m_world_x_axis.x;
	world_x_axis_setting.add("y", Setting::TypeFloat) = m_world_x_axis.y;
	world_x_axis_setting.add("z", Setting::TypeFloat) = m_world_x_axis.z;

	Setting& world_z_axis_setting = camParams.add("world_z_axis", Setting::TypeGroup);
	world_z_axis_setting.add("x", Setting::TypeFloat) = m_world_z_axis.x;
	world_z_axis_setting.add("y", Setting::TypeFloat) = m_world_z_axis.y;
	world_z_axis_setting.add("z", Setting::TypeFloat) = m_world_z_axis.z;

	Setting& world_z_origin_setting = camParams.add("world_origin", Setting::TypeGroup);
	world_z_origin_setting.add("x", Setting::TypeFloat) = m_world_origin.x;
	world_z_origin_setting.add("y", Setting::TypeFloat) = m_world_origin.y;
	world_z_origin_setting.add("z", Setting::TypeFloat) = m_world_origin.z;

	// Ajout des paramètres par défaut pour la partie tracking
	if (m_filteringType == simple_lowpass){ trackingParams.add("filterType", Setting::TypeString) = "simple_lowpass"; }
	else if (m_filteringType == multi_channel_lowpass) { trackingParams.add("filterType", Setting::TypeString) = "multi_channel_lowpass"; }
	else if (m_filteringType == noFiltering) { trackingParams.add("filterType", Setting::TypeString) = "noFiltering"; }
	trackingParams.add("hq_tracking", Setting::TypeBoolean) = m_hqTracking;

	filterParams.add("colorHue", Setting::TypeInt) = m_filterColor.val[0];
	filterParams.add("colorSat", Setting::TypeInt) = m_filterColor.val[1];
	filterParams.add("colorVal", Setting::TypeInt) = m_filterColor.val[2];

	filterParams.add("toleranceHue", Setting::TypeInt) = m_hsvRange.val[0];
	filterParams.add("toleranceSat", Setting::TypeInt) = m_hsvRange.val[1];
	filterParams.add("toleranceVal", Setting::TypeInt) = m_hsvRange.val[2];

	try
	{
		cfg.writeFile(CONFIG_FILE_PATH);
		cout << "Fichier configuration remplace : " << CONFIG_FILE_PATH << endl;

	}
	catch (const FileIOException &fioex)
	{
		cout << "Erreur d'ecriture de : " << CONFIG_FILE_PATH << endl;
	}
}

/*
FONCTIONS DE PRIVEES
*/
typedef cv::Point3_<uint8_t> Pixel;

struct SmoothInRange {

	Vec3i color;
	Vec3i colorRange;
	void operator ()(Pixel &pixel, const int * position) const {

		const int lowerBoundary = color.val[2] - colorRange.val[2];

		if (pixel.x > color.val[0] - colorRange.val[0] && pixel.x < color.val[0] + colorRange.val[0]
			&& pixel.y > color.val[1] - colorRange.val[1] && pixel.y < color.val[1] + colorRange.val[1])
		{
			pixel.y = 0;
			if (pixel.z > color.val[2] + colorRange.val[2]) { pixel.z = 0; }
			else if (pixel.z < lowerBoundary) {
				int p = 255 - 2 * (lowerBoundary - pixel.z);
				p = p < 0 ? 0 : p;
				pixel.z = p;
			}
			else { pixel.z = 255;}
		}
		else { pixel.x = 0; pixel.y = 0; pixel.z = 0; }
	}
};

void Tracker::circle_refining(Point3f & circleCoord, Mat & hsvFrame, Vec3i color, Vec3i colorRange)
{
	if (!m_isTrackingValid) return;
	// Calcul de la position et de la taille du ROI
	int X = circleCoord.x - circleCoord.z * 2.0f; X = X < 0 ? 0 : X;
	int Y = circleCoord.y - circleCoord.z * 2.0f; Y = Y < 0 ? 0 : Y;
	int width = 2.0 * circleCoord.z * 2.0f;
	if (X + width > hsvFrame.cols || Y + width > hsvFrame.rows) { return; }

	Rect roiPos = Rect(X, Y, width, width);
	Mat ROI = hsvFrame(roiPos);

	// Threshold non binaire de l'image
	SmoothInRange inRange = SmoothInRange();
	inRange.color = color;
	inRange.colorRange = colorRange;
	ROI.forEach<Pixel>(inRange);

	// Calcul du centre de la masse
	Point2f refinedPos = Point2f(0,0);
	float areaPixel = 0;
	int x = 0;int y = 0;
	for (Pixel &p : cv::Mat_<Pixel>(ROI)) {
		refinedPos.x += x * (p.z/255);
		refinedPos.y += y * (p.z / 255);
		areaPixel += (p.z / 255);

		x++;
		if (x >= ROI.cols) {x = 0; y++;}
	}
	y = y - 1;
	refinedPos /= areaPixel;
	float radius = sqrt(areaPixel/PI);
	
	cvtColor(ROI, ROI, CV_HSV2BGR);
	if(circleCoord.z/radius > 0.5 || circleCoord.z / radius < 1.2)
	{circleCoord = Point3f(X + refinedPos.x, Y + refinedPos.y, radius);}
	//{circleCoord = Point3f(circleCoord.x, circleCoord.y, radius);}
}

void Tracker::color_filtering(Mat& videoFrame, Vec3i hsvRange, Scalar filterColor,Mat& filteredFrame) // Filtre la couleur
{
	if (!videoFrame.empty())
	{
		Mat binaryFrame;
		Mat erodedFrame;
		cvtColor(videoFrame, m_hsvFrame, CV_BGR2HSV);
		inRange(m_hsvFrame, Scalar(filterColor.val[0] - m_hsvRange[0], filterColor.val[1] - hsvRange[1], filterColor.val[2] - hsvRange[2]),
		Scalar(filterColor.val[0] + hsvRange[0], filterColor.val[1] + hsvRange[1], filterColor.val[2] + hsvRange[2]), binaryFrame);

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

void Tracker::circle_fitting(Point3f& circleCoord,Mat& filteredFrame) // Donne la position 2D et le rayon du cercle dans circleCoord
{
	Point2f position;
	float radius;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	// Recherche des contours dans l'image binaire
	findContours(filteredFrame, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
	// Si aucun contour, ne rien faire
	if (contours.size() <= 0) { m_isTrackingValid = false; return; }
	// Sinon trouver le contour le plus grand et calculer le cercle englobant
	vector<Point> largestContour = get_largest_contour(contours);
	minEnclosingCircle(largestContour, position, radius);

	// Vérification si le rayon est valide (supérieur à 5 pixel et inférieur à 240 pixels)
	if (radius < 5 || radius > 240) { m_isTrackingValid = false; return; }
	else { m_isTrackingValid = true; }
	circleCoord.x = position.x;
	circleCoord.y = position.y;
	circleCoord.z = radius;
}

void Tracker::mono_position_estimation(float focal, Point3f circleCoord, Point3f& outPosition)
{
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

void Tracker::stackPositions(Point3f pos, std::deque<Point3f>& q, int size)
{
	q.push_front(pos);
	if (q.size() > size) { q.pop_back(); }
}

void Tracker::new_file_params()
{
	// Création de l'objet Configuration et ajout des groupes de paramètres
	Config cfg;
	Setting& root = cfg.getRoot();
	Setting& camParams = root.add("camera", Setting::TypeGroup);
	Setting& trackingParams = root.add("tracking", Setting::TypeGroup);
	Setting& filterParams = trackingParams.add("filter", Setting::TypeGroup);

	// Ajout des paramètres par défaut pour la partie caméra
	camParams.add("resolutionX", Setting::TypeInt) = 640;
	camParams.add("resolutionY", Setting::TypeInt) = 480;
	camParams.add("gain", Setting::TypeInt) = 10;
	camParams.add("exposure", Setting::TypeInt) = 10;

	Setting& world_x_axis_setting = camParams.add("world_x_axis", Setting::TypeGroup);
	world_x_axis_setting.add("x", Setting::TypeFloat) = 1.0;
	world_x_axis_setting.add("y", Setting::TypeFloat) = 0.0;
	world_x_axis_setting.add("z", Setting::TypeFloat) = 0.0;

	Setting& world_z_axis_setting = camParams.add("world_z_axis", Setting::TypeGroup);
	world_z_axis_setting.add("x", Setting::TypeFloat) = 0.0;
	world_z_axis_setting.add("y", Setting::TypeFloat) = 0.0;
	world_z_axis_setting.add("z", Setting::TypeFloat) = 1.0;

	Setting& world_z_origin_setting = camParams.add("world_origin", Setting::TypeGroup);
	world_z_origin_setting.add("x", Setting::TypeFloat) = 0.0;
	world_z_origin_setting.add("y", Setting::TypeFloat) = 0.0;
	world_z_origin_setting.add("z", Setting::TypeFloat) = 0.0;

	// Ajout des paramètres par défaut pour la partie tracking
	trackingParams.add("filterType", Setting::TypeString) = "multi_channel_lowpass";
	trackingParams.add("hq_tracking", Setting::TypeBoolean) = false;

	filterParams.add("colorHue", Setting::TypeInt) = 0;
	filterParams.add("colorSat", Setting::TypeInt) = 100;
	filterParams.add("colorVal", Setting::TypeInt) = 100;

	filterParams.add("toleranceHue", Setting::TypeInt) = 25;
	filterParams.add("toleranceSat", Setting::TypeInt) = 80;
	filterParams.add("toleranceVal", Setting::TypeInt) = 80;

	try
	{
		cfg.writeFile(CONFIG_FILE_PATH);
		cout << "Nouveau fichier configuration : " << CONFIG_FILE_PATH << endl;

	}
	catch (const FileIOException &fioex)
	{
		cout << "Erreur d'ecriture de : " << CONFIG_FILE_PATH << endl;
	}

}

void Tracker::load_params()
{
	Config cfg;

	try
	{
		cfg.readFile(CONFIG_FILE_PATH);
		cout << "Fichier configuration ouvert : " << CONFIG_FILE_PATH << endl;
	}
	catch (const FileIOException &fioex)
	{
		cout << "Erreur de lecture... creation d'un nouveau fichier... " << CONFIG_FILE_PATH << endl;
		new_file_params();
		return;
	}
	Setting& root = cfg.getRoot();
	try 
	{
		Setting& camParams = root["camera"];
		Setting& trackingParams = root["tracking"];
		Setting& filterParams = trackingParams["filter"];

		// Assignation des variables camera
		camParams.lookupValue("resolutionX", resolutionX);
		camParams.lookupValue("resolutionY", resolutionY);
		camParams.lookupValue("gain", m_gain);
		camParams.lookupValue("exposure", m_exposure);

		Setting& world_x_axis_setting = camParams["world_x_axis"];
		world_x_axis_setting.lookupValue("x", m_world_x_axis.x);
		world_x_axis_setting.lookupValue("y", m_world_x_axis.y);
		world_x_axis_setting.lookupValue("z", m_world_x_axis.z);

		Setting& world_z_axis_setting = camParams["world_z_axis"];
		world_z_axis_setting.lookupValue("x", m_world_z_axis.x);
		world_z_axis_setting.lookupValue("y", m_world_z_axis.y);
		world_z_axis_setting.lookupValue("z", m_world_z_axis.z);

		Setting& world_z_origin_setting = camParams["world_origin"];
		world_z_origin_setting.lookupValue("x", m_world_origin.x);
		world_z_origin_setting.lookupValue("y", m_world_origin.y);
		world_z_origin_setting.lookupValue("z", m_world_origin.z);

		// Assignation des variables filtre
		string filtertype;
		trackingParams.lookupValue("filterType", filtertype); // simple_lowpass,multi_channel_lowpass,noFiltering
		if (filtertype == "simple_lowpass") { m_filteringType = simple_lowpass; }
		else if (filtertype == "multi_channel_lowpass") { m_filteringType = multi_channel_lowpass; }
		else if (filtertype == "noFiltering") { m_filteringType = noFiltering; }
		trackingParams.lookupValue("hq_tracking", m_hqTracking);

		int h, s, v;
		filterParams.lookupValue("colorHue", h);
		filterParams.lookupValue("colorSat", s);
		filterParams.lookupValue("colorVal", v);
		set_filter_color( Vec3i(h, s, v) );

		filterParams.lookupValue("toleranceHue", h);
		filterParams.lookupValue("toleranceSat", s);
		filterParams.lookupValue("toleranceVal", v);
		set_filter_range( Vec3i(h, s, v) );

		cout << "Valeur couleur correctement chargées : " << "H:" << m_filterColor.val[0] << " S:" << m_filterColor[1] << " V:" << m_filterColor[2] << endl;
	}
	catch (const SettingNotFoundException &nfex)
	{
		cout << "Erreur de paramètres dans le fichier." << endl;
	}
}


// Maths : https://fr.wikipedia.org/wiki/Matrice_de_rotation
Matx33f Tracker::rotation_matrix(Point3f axis, float angle)
{
	float c = cos(angle); // en radians
	float s = sin(angle);

	//Matx33f mat = 
	
	Matx33f m(axis.x * axis.x * (1 - c) + c         , axis.x * axis.y * (1 - c) - axis.z * s , axis.x * axis.z * (1 - c) + axis.y * s,
			  axis.x * axis.y * (1 - c) + axis.z * s, axis.y * axis.y * (1 - c) + c			 , axis.y * axis.z * (1 - c) - axis.x * s,
			  axis.x * axis.z * (1 - c) - axis.y * s, axis.y * axis.z * (1 - c) + axis.x * s , axis.z * axis.z * (1 - c) + c			);
	/* La matrice est stockée dans cet ordre : 1 2 3 4 5 6 7 8 9
	Matx33f m(1,2,3,
			  4,5,6,
			  7,8,9);
	*/

	return m;
}

Vec3f Tracker::crossProduct(Point3f a, Point3f b)
{
	return Vec3f(a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x);
}

void Tracker::compute_camToWorld_rotation_matrix(Vec3f z_world_axis, Vec3f x_world_axis, Vec3f world_origin)
{
	// Axes de la caméra en coordonnées locales (base Caméra)
	Vec3f cameraXAxisInCameraCoord = Point3f(1,0,0);
	Vec3f cameraZAxisInCameraCoord = Point3f(0,0,1);
	Vec3f cameraYAxisInCameraCoord = normalize( crossProduct(cameraZAxisInCameraCoord,cameraXAxisInCameraCoord) );

	// On assigne l'origine en coordonnées locales
	Vec3f originInCameraCoord = world_origin;

	// On assigne l'axe Z en coordonnées locales
	Point3f worldZAxisInCameraCoord = normalize(z_world_axis);

	// On assigne l'axe X selon la calibration et on calcule l'axe Y
	Vec3f notOrthoWorldXAxisInCameraCoord = normalize(x_world_axis);
	Point3f worldYAxisInCameraCoord = normalize( crossProduct(worldZAxisInCameraCoord, notOrthoWorldXAxisInCameraCoord) );
	// On calcule le vrai axe X, orthonormal à l'axe Y et Z
	Point3f worldXAxisInCameraCoord = normalize( crossProduct(worldYAxisInCameraCoord,worldZAxisInCameraCoord) );

	// On calcule la matrice de passage de la base locale (base caméra) à la base absolue (base monde calibrée)
	Matx33f transferMatrix(worldXAxisInCameraCoord.x, worldYAxisInCameraCoord.x, worldZAxisInCameraCoord.x,
						  worldXAxisInCameraCoord.y, worldYAxisInCameraCoord.y, worldZAxisInCameraCoord.y,
						  worldXAxisInCameraCoord.z, worldYAxisInCameraCoord.z, worldZAxisInCameraCoord.z );

	// On transpose la matrice car dans OpenCV, les vecteurs sont des vecteurs lignes et non colonne
	transpose(transferMatrix, m_camToWorld_rotation);

	// Affichage du Debug
	cout << "\n*** CALCUL DE LA MATRICE DE PASSAGE ***\n" << endl;

	cout << "Camer X : " << cameraXAxisInCameraCoord << endl;
	cout << "Camer Y : " << cameraYAxisInCameraCoord << endl;
	cout << "Camer Z : " << cameraZAxisInCameraCoord << endl;

	cout << "World X : " << worldXAxisInCameraCoord << endl;
	cout << "World Y : " << worldYAxisInCameraCoord << endl;
	cout << "World Z : " << worldZAxisInCameraCoord << endl;

	cout << "Matrice de passage :" << endl;
	cout << m_camToWorld_rotation << endl;

	cout << endl;

}

Point3f Tracker::get_world_position(Point3f localPosition)
{
	Point3f worldPos = localPosition - m_world_origin;
	worldPos = m_camToWorld_rotation * worldPos;
	return worldPos;
}