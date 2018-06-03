#pragma once
#include "client_test.h"
#include<iostream>
#include<cstdio>
#include<math.h>

#include"Tracker.h"

#define CVUI_IMPLEMENTATION
#include"Client_UI.h"

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

Client_UI client;
Tracker tracker;
int alpha;

void initialization()
{
	tracker.init_tracker(0,false);
	client.initialize();
	client.tracker = &tracker;
}

void runTracker()
{
	tracker.track();
}

int main(int argc, char** argv)
{
	// cout est le flux de texte du terminal, '<<' permet d'injecter un élément dans cout, endl correspond à la fin de ligne
	cout << "Track'ESIEA est un pst de 2A qui vise a effectuer le tracking 3D d'un objet dans un espace delimite." << endl; 
	cout << "Librairie utilisée : OpenCV." << endl;
	
	bool exit = false;
	int input;
	initialization();
	while (!exit )
	{
		input = waitKey(1);
		runTracker();
		client.run(exit,input);
	}
	
	return 0;
}
