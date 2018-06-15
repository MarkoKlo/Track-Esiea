#pragma once
#include "client_test.h"
#include<iostream>
#include"Tracker.h"

#pragma comment (lib, "ws2_32.lib")

#define CVUI_IMPLEMENTATION
#include"Client_UI.h"

using namespace std;

client_test::client_test()//
{
}

client_test::~client_test()
{
}

Client_UI client_ui;
Tracker tracker;
void initialization()
{
	cout << "\nInitialisation du tracker..." << endl;
	tracker.init_tracker(0,false);
	cout << "Initialisation du tracker avec succes !\n" << endl;

	cout << "Initialisation de l'interface..." << endl;
	client_ui.tracker = &tracker;
	client_ui.initialize();
	
	cout << "Succes !" << endl;
}

void runTracker()
{
	tracker.track();
}

WSADATA ws_data;
sockaddr_in server;
SOCKET out;

bool initSocket()
{
	
	WORD version = MAKEWORD(2, 2);
	int wsOk = WSAStartup(version, &ws_data);
	if (wsOk != 0) { cout << "Impossible de demarrer Winsock! " << wsOk; return false; }

	server.sin_family = AF_INET; // AF_INET = IPv4 addresses
	server.sin_port = htons(1010); // Little to big endian conversion
	inet_pton(AF_INET, "127.0.0.1", &server.sin_addr); // Convert from string to byte array

	// Socket creation, note that the socket type is datagram
	out = socket(AF_INET, SOCK_DGRAM, 0);
	cout << "Creation du socket reussie !" << endl;
	return true;
}

void runSocket()
{
		char msg[82];

		cv::Point3f pos1 = tracker.get_world_position(0); cv::Point3f pos2 = tracker.get_world_position(1);
		cv::Point3f speed1 = tracker.get_speed(0); cv::Point3f speed2 = tracker.get_speed(1);
		sprintf_s(msg, "%.1f %.1f %.1f %.1f %.1f %.1f %d %.1f %.1f %.1f %.1f %.1f %.1f %d", pos1.x, pos1.y, pos1.z, speed1.x, speed1.y, speed1.z, tracker.is_tracking_valid(0),
			pos2.x, pos2.y, pos2.z, speed2.x, speed2.y, speed2.z, tracker.is_tracking_valid(1) );
		// Write out to that socket
		string s(msg);
		int sendOk = sendto(out, s.c_str(), s.size() + 1, 0, (sockaddr*)&server, sizeof(server));

		if (sendOk == SOCKET_ERROR)
		{
			cout << "Erreur d'envoi : " << WSAGetLastError() << endl;
		}
	
}

int main(int argc, char** argv)
{
	// cout est le flux de texte du terminal, '<<' permet d'injecter un élément dans cout, endl correspond à la fin de ligne
	cout << "Track'ESIEA est un pst de 2A qui vise a effectuer le tracking 3D d'un objet dans un espace delimite." << endl; 
	cout << "Librairie utilisée : OpenCV." << endl;
	
	bool exit = false;
	int input;
	initialization();
	initSocket();
	cout << "Initialisation termine" << endl;

	while (!exit )
	{
		input = waitKey(1); //if (input == 'q') { exit = true; }
		runTracker();
		client_ui.run(exit,input);
		runSocket();
		
	}
	
	closesocket(out);
	WSACleanup();

	return 0;
}

