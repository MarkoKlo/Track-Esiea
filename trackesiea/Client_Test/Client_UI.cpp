#pragma once
#include "Client_UI.h"

#define RESX 1024
#define RESY 768
#define TEXT_SCALE (RESX/800.0)

Client_UI::Client_UI() : filterRange(-1,-1,-1), gain(-1),exposure(-1),showBinary(false), showCircle(false), calibration_mode(false), calibration_step(0),
current_tab(tracking)
{
}


Client_UI::~Client_UI()
{
}

void Client_UI::initialize()
{
	cvui::init(WINDOW_NAME);
	frame = cv::Mat(cv::Size(RESX, RESY),CV_8UC3);

	cv::resize(cv::imread("images/tracking_state_ok.jpg", cv::IMREAD_COLOR), tracking_status_image[0], Size(RESX*0.1825, RESY*0.243));
	cv::resize(cv::imread("images/tracking_state_average.jpg", cv::IMREAD_COLOR), tracking_status_image[1], Size(RESX*0.1825, RESY*0.243));
	cv::resize(cv::imread("images/tracking_state_bad.jpg", cv::IMREAD_COLOR), tracking_status_image[2], Size(RESX*0.1825, RESY*0.243));

	cv::resize(cv::imread("images/tab_button_idle.jpg", cv::IMREAD_COLOR), tab_button_img[0], Size(RESX*0.2625, RESY*0.234));
	cv::resize(cv::imread("images/tab_button_hover.jpg", cv::IMREAD_COLOR), tab_button_img[1], Size(RESX*0.2625, RESY*0.234));

	grid = imread("images/2x2_grid.png", cv::IMREAD_COLOR);

	cv::resize(cv::imread("images/calib_image1.jpg", cv::IMREAD_COLOR), calibration_image[0], Size(RESX*0.4, RESX*0.4));
	cv::resize(cv::imread("images/calib_image2.jpg", cv::IMREAD_COLOR), calibration_image[1], Size(RESX*0.4, RESX*0.4));
	cv::resize(cv::imread("images/calib_image3.jpg", cv::IMREAD_COLOR), calibration_image[2], Size(RESX*0.4, RESX*0.4));
	cv::resize(cv::imread("images/calib_image4.jpg", cv::IMREAD_COLOR), calibration_image[3], Size(RESX*0.4, RESX*0.4));

	hqTracking = tracker->is_hq_tracking();
}

void Client_UI::run(bool & exit, int input)
{
	
	frame = cv::Scalar(255, 247, 247);
	status_field();
	tab_column();

	switch (current_tab)
	{
	case tracking :
		tracking_tab();
		break;
	case camera :
		camera_tab();
		break;
	case options :
		options_tab(exit);
		break;
	}

	// Show window content
	cvui::imshow(WINDOW_NAME, frame);

	if (input == 27){exit = true;}
	
}

void Client_UI::status_field()
{
	float tracking_speed = tracker->get_tracking_rate();

	// Rendu du fond
	cvui::rect(frame, 0, 0, RESX, RESY*0.3, 0xe8e9f3, 0xe8e9f3);

	// Rendu du texte du statut
	cvui::beginColumn(frame, RESX*0.25, RESY*0.029, -1, -1, RESY*0.0583);
	cvui::text("Frequence de suivi", TEXT_SCALE, 0xa6a6a8);
	cvui::text("Nombre de spheres", TEXT_SCALE, 0xa6a6a8);
	cvui::text("Suivi de position", TEXT_SCALE, 0xa6a6a8);
	cvui::endColumn();

	cvui::beginColumn(frame, RESX*0.725, RESY*0.029, -1, -1, RESY*0.0583);
	cvui::text(":", TEXT_SCALE, 0xa6a6a8);
	cvui::text(":", TEXT_SCALE, 0xa6a6a8);
	cvui::text(":", TEXT_SCALE, 0xa6a6a8);
	cvui::endColumn();

	cvui::beginColumn(frame, RESX*0.8, RESY*0.029, -1, -1, RESY*0.0583);
	cvui::printf(TEXT_SCALE, 0xa6a6a8,"%.1f", tracking_speed);
	cvui::printf(TEXT_SCALE, 0xa6a6a8, "1");
		if (!tracker->is_tracking_valid()) { cvui::printf(TEXT_SCALE, 0xa6a6a8, "AUCUN"); }
		else if (tracking_speed > 50){cvui::printf(TEXT_SCALE, 0xa6a6a8, "BON");}
		else if (tracking_speed > 25) { cvui::printf(TEXT_SCALE, 0xa6a6a8, "MOYEN"); }
		else { cvui::printf(TEXT_SCALE, 0xa6a6a8, "MAUVAIS"); }
	cvui::endColumn();

	// Rendu de l'icone statut
	if(!tracker->is_tracking_valid()){ cvui::image(frame, RESX*0.02125, RESY*0.029, tracking_status_image[2]); }
	else if(tracking_speed > 49){cvui::image(frame, RESX*0.02125, RESY*0.029, tracking_status_image[0]); }
	else { cvui::image(frame, RESX*0.02125, RESY*0.029, tracking_status_image[1]); }
	

}

void Client_UI::tab_column()
{
	cvui::image(frame, 0, RESY*0.3, tab_button_img[0]);
	cvui::image(frame, 0, RESY*0.533, tab_button_img[0]);
	cvui::image(frame, 0, RESY*0.766, tab_button_img[0]);

	if (cvui::button(frame, 0, RESY*0.3, tab_button_img[0], tab_button_img[1], tab_button_img[1])) { current_tab = tracking; };
	if (cvui::button(frame, 0, RESY*0.533, tab_button_img[0], tab_button_img[1], tab_button_img[1])) { current_tab = camera; };
	if (cvui::button(frame, 0, RESY*0.766, tab_button_img[0], tab_button_img[1], tab_button_img[1])) { current_tab = options; };

	cvui::text(frame, RESX*0.0375, RESY*0.3 + TEXT_SCALE * 56,"Suivi 3D", TEXT_SCALE, 0xf7f7ff);
	cvui::text(frame, RESX*0.0375, RESY*0.534 + TEXT_SCALE * 56, "Camera", TEXT_SCALE, 0xf7f7ff);
	cvui::text(frame, RESX*0.0375, RESY*0.767 + TEXT_SCALE * 56, "Avance", TEXT_SCALE, 0xf7f7ff);

}

void Client_UI::tracking_tab()
{
	const int videoX = RESX * 0.3;
	const int videoY = RESY * 0.35;
	const float videoScale = 0.4;

	if (calibration_mode)
	{
		switch (calibration_step)
		{
		case 0 :
			cvui::image(frame, RESX*0.3, RESY*0.35, calibration_image[0]);
			cvui::printf(frame, RESX*0.7375, RESY*0.36, TEXT_SCALE * 0.5, 0xa6a6a8, "Placez la balle au");
			cvui::printf(frame, RESX*0.7375, RESY*0.36 + TEXT_SCALE * 30, TEXT_SCALE * 0.5, 0xa6a6a8, "centre de la zone");
			cvui::printf(frame, RESX*0.7375, RESY*0.36 + TEXT_SCALE * 60, TEXT_SCALE * 0.5, 0xa6a6a8, "de jeu, puis cliquez");
			cvui::printf(frame, RESX*0.7375, RESY*0.36 + TEXT_SCALE * 90, TEXT_SCALE * 0.5, 0xa6a6a8, "sur Calibrer l'origine.");
			if (cvui::button(frame, RESX*0.7375, RESY*0.55, RESX*0.1875, RESY*0.05, "Calibrer l'origine"))
			{
				tracker->set_world_origin();
				calibration_step = 1;
			}
			if (cvui::button(frame, RESX*0.78, RESY*0.92, RESX*0.1875, RESY*0.05, "Annuler")) { calibration_mode = false; }
			break;
		case 1 :
			cvui::image(frame, RESX*0.3, RESY*0.35, calibration_image[1]);
			cvui::printf(frame, RESX*0.7375, RESY*0.36, TEXT_SCALE * 0.5, 0xa6a6a8,                   "Maintenant, placez");
			cvui::printf(frame, RESX*0.7375, RESY*0.36 + TEXT_SCALE * 30, TEXT_SCALE * 0.5, 0xa6a6a8, "la balle a droite");
			cvui::printf(frame, RESX*0.7375, RESY*0.36 + TEXT_SCALE * 60, TEXT_SCALE * 0.5, 0xa6a6a8, "du centre, puis cliquez");
			cvui::printf(frame, RESX*0.7375, RESY*0.36 + TEXT_SCALE * 90, TEXT_SCALE * 0.5, 0xa6a6a8, "sur Calibrer l'axe X.");
			if (cvui::button(frame, RESX*0.7375, RESY*0.55, RESX*0.1875, RESY*0.05, "Calibrer l'axe X"))
			{
				tracker->set_world_xaxis();
				calibration_step = 2;
			}
			if (cvui::button(frame, RESX*0.78, RESY*0.92, RESX*0.1875, RESY*0.05, "Annuler")) { calibration_mode = false; }
			break;
		case 2:
			cvui::image(frame, RESX*0.3, RESY*0.35, calibration_image[2]);
			cvui::printf(frame, RESX*0.7375, RESY*0.36, TEXT_SCALE * 0.5, 0xa6a6a8, "Ensuite, placez");
			cvui::printf(frame, RESX*0.7375, RESY*0.36 + TEXT_SCALE * 30, TEXT_SCALE * 0.5, 0xa6a6a8, "la balle a l'avant");
			cvui::printf(frame, RESX*0.7375, RESY*0.36 + TEXT_SCALE * 60, TEXT_SCALE * 0.5, 0xa6a6a8, "du centre, puis cliquez");
			cvui::printf(frame, RESX*0.7375, RESY*0.36 + TEXT_SCALE * 90, TEXT_SCALE * 0.5, 0xa6a6a8, "sur Calibrer l'axe Z.");
			if (cvui::button(frame, RESX*0.7375, RESY*0.55, RESX*0.1875, RESY*0.05, "Calibrer l'axe Z"))
			{
				tracker->set_world_zaxis();
				tracker->calibrate_camera_pose();
				calibration_step = 3;
			}
			if (cvui::button(frame, RESX*0.78, RESY*0.92, RESX*0.1875, RESY*0.05, "Annuler")) { calibration_mode = false; }
			break;
		case 3:
			cvui::image(frame, RESX*0.3, RESY*0.35, calibration_image[3]);
			cvui::printf(frame, RESX*0.7375, RESY*0.36, TEXT_SCALE * 0.5, 0xa6a6a8,                   "Calibration terminee !");
			cvui::printf(frame, RESX*0.7375, RESY*0.36 + TEXT_SCALE * 30, TEXT_SCALE * 0.5, 0xa6a6a8, "Vous pouvez maintenant");
			cvui::printf(frame, RESX*0.7375, RESY*0.36 + TEXT_SCALE * 60, TEXT_SCALE * 0.5, 0xa6a6a8, "cliquer sur le bouton");
			cvui::printf(frame, RESX*0.7375, RESY*0.36 + TEXT_SCALE * 90, TEXT_SCALE * 0.5, 0xa6a6a8, "Termine pour quitter.");
			if (cvui::button(frame, RESX*0.78, RESY*0.92, RESX*0.1875, RESY*0.05, "Terminer")) { tracker->save_params(); calibration_mode = false; }
			break;
		}
	}
	else
	{
		Mat debugGraphFrame = grid.clone();
		Point3f cam_position = tracker->get_camera_world_position();
		Point3f ball_position = tracker->get_world_position();
		Point3f ball_speed = tracker->get_speed();
		circle(debugGraphFrame, Point((ball_position.x*2.5 + 250), -ball_position.z*2.5 + 250), 2, Scalar(0, 0, 255), 2, -1, 0); // XZ Balle
		circle(debugGraphFrame, Point((cam_position.x*2.5 + 250), -cam_position.z*2.5 + 250), 2, Scalar(255, 255, 0), 2, -1, 0); // CameraPosition
		cv::resize(debugGraphFrame, debugGraphFrame, Size(RESX*videoScale, RESX*videoScale));
		cvui::image(frame, videoX, videoY, debugGraphFrame);

		cvui::printf(frame, RESX*0.3, RESY*0.9, TEXT_SCALE * 0.5, 0xa6a6a8, "Position Monde (cm) : %.f %.f %.f", ball_position.x, ball_position.y, ball_position.z);
		cvui::printf(frame, RESX*0.3, RESY*0.9 + TEXT_SCALE * 30, TEXT_SCALE * 0.5, 0xa6a6a8, "Vitesse (m/s) : %.f %.f %.f", ball_speed.x / 100, ball_speed.y / 100, ball_speed.z / 100);
	
		if (cvui::button(frame, RESX*0.7375, RESY*0.36, RESX*0.1875, RESY*0.05, "Calibrer la position 3D"))
		{
			calibration_mode = true; calibration_step = 0;
		}
	}

}

unsigned int Client_UI::getHexaColor()
{
	Mat col = Mat(1,1,CV_8UC3);
	col.at<Vec3b>(0, 0) = tracker->get_filter_color();
	cv::cvtColor(col,col,CV_HSV2RGB);
	Vec3b new_col = col.at<Vec3b>(0, 0);
	unsigned int col_hex = (( new_col.val[0] & 0xff) << 16) + ((new_col.val[1] & 0xff) << 8) + (new_col.val[2] & 0xff);
	return col_hex;
}

void Client_UI::camera_tab()
{
	if (filterRange == Point3i(-1, -1, -1)) { filterRange = tracker->get_filter_range(); }
	if (gain == -1 || exposure == -1) { gain = tracker->get_gain(); exposure = tracker->get_exposure(); }

	cvui::beginColumn(frame, RESX*0.7375, RESY*0.35, -1, -1,RESY*0.025);
		cvui::beginColumn(RESX*0.125, RESY*0.083, 0);
		cvui::text("Gain :", TEXT_SCALE*0.5, 0xa6a6a8);
		cvui::trackbar(RESX*0.1875, &gain, (int)0, (int)30,1,"",cvui::TRACKBAR_HIDE_LABELS);
		cvui::endColumn();

		cvui::beginColumn(RESX*0.125, RESY*0.083, 0);
		cvui::text("Exposition :", TEXT_SCALE*0.5, 0xa6a6a8);
		cvui::trackbar(RESX*0.1875, &exposure, (int)0, (int)30, 1, "", cvui::TRACKBAR_HIDE_LABELS);
		cvui::endColumn();

		cvui::beginColumn(RESX*0.125, RESY*0.083, RESY*0.008);
		cvui::text("Tolerance teinte :", TEXT_SCALE*0.5, 0xa6a6a8);
		cvui::trackbar(RESX*0.1875, &filterRange.x,(int)0,(int)255);
		cvui::endColumn();

		cvui::beginColumn(RESX*0.125, RESY*0.083, RESY*0.008);
		cvui::text("Tolerance saturation :", TEXT_SCALE*0.5, 0xa6a6a8);
		cvui::trackbar(RESX*0.1875, &filterRange.y, (int)0, (int)255);
		cvui::endColumn();

		cvui::beginColumn(RESX*0.125, RESY*0.083, RESY*0.008);
		cvui::text("Tolerance valeur :", TEXT_SCALE*0.5, 0xa6a6a8);
		cvui::trackbar(RESX*0.1875, &filterRange.z, (int)0, (int)255);
		cvui::endColumn();
		
	cvui::endColumn();

	cvui::text(frame, RESX*0.31, RESY*0.8,"Montrer le filtre :", TEXT_SCALE*0.5, 0xa6a6a8);
	cvui::checkbox(frame, RESX*0.31 + TEXT_SCALE * 200, RESY*0.8 + TEXT_SCALE * 2, "", &showBinary, 0xa6a6a8);

	cvui::text(frame, RESX*0.31, RESY*0.8 + TEXT_SCALE * 30, "Apercu du suivi 2D :", TEXT_SCALE*0.5, 0xa6a6a8);
	cvui::checkbox(frame, RESX*0.31 + TEXT_SCALE * 200, RESY*0.8 + TEXT_SCALE * 30 + TEXT_SCALE * 2, "", &showCircle, 0xa6a6a8);

	cvui::text(frame, RESX*0.31, RESY*0.8 + TEXT_SCALE * 60, "Couleur du filtre :", TEXT_SCALE*0.5, 0xa6a6a8);
	cvui::rect(frame, RESX*0.31 + TEXT_SCALE * 200, RESY*0.8 + TEXT_SCALE * 60 + TEXT_SCALE * 2, 15,15, 0xa6a6a8, getHexaColor() );

	if (cvui::button(frame, RESX*0.78, RESY*0.92, RESX*0.1875, RESY*0.05, "Sauvegarder")) { tracker->save_params();}

	// Image caméra
	const int videoX = RESX * 0.3;
	const int videoY = RESY * 0.35;
	const float videoScale = 0.4;
	if(showBinary){cv::cvtColor(tracker->get_binary_frame(), windowVideo, CV_GRAY2BGR);}
	else { windowVideo = tracker->get_video_frame(); }
	if(showCircle && tracker->is_tracking_valid() )
	{ circle(windowVideo, Point(tracker->get_2D_position().x, tracker->get_2D_position().y), tracker->get_2D_position().z, Scalar(0, 0, 255), 1, CV_AA, 0); }
	cv::resize( windowVideo , windowVideo, Size(RESX*videoScale, RESY*videoScale));

	cvui::image(frame, videoX, videoY, windowVideo);

	// Réglage Couleur
	cv::Point cursor = cvui::mouse();
	if (cursor.x > videoX && cursor.x < videoX + RESX * videoScale && cursor.y > videoY && cursor.y < videoY + RESY * videoScale)
	{
		if (cvui::mouse(cvui::CLICK))
		{
			Point2i mouse_video_pos = Point2i((cursor.x - videoX) * 640 / (RESX*videoScale), (cursor.y - videoY) * 480 / (RESY*videoScale));
			Vec3i color = tracker->get_hsv_color(mouse_video_pos);
			printf_s("Couleur changee : %d %d %d à %d %d \n", color.val[0], color.val[1], color.val[2], mouse_video_pos.x, mouse_video_pos.y);
			tracker->set_filter_color(color);
		}
	}
	
	tracker->set_filter_range(filterRange);
	tracker->set_gain(gain);
	tracker->set_exposure(exposure);
}

void Client_UI::options_tab(bool &exit)
{
	cvui::beginColumn(frame, RESX*0.3, RESY*0.35, -1, -1, RESY*0.025);
	cvui::checkbox("Tracking haute qualite (experimental) ", &hqTracking, 0xa6a6a8);
	tracker->set_hq_tracking(hqTracking);
	cvui::endColumn();
	if (cvui::button(frame, RESX*0.78, RESY*0.92, RESX*0.1875, RESY*0.05, "Fermer Track'ESIEA")) { tracker->save_params(); exit = true; }
}
