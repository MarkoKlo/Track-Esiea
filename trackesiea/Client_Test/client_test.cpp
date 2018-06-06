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
	client_ui.initialize();
	client_ui.tracker = &tracker;
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
	char msg[64];
	cv::Point3f pos = tracker.get_world_position();
	cv::Point3f speed = tracker.get_speed();
	sprintf_s(msg, "%.2f %.2f %.2f %.2f %.2f %.2f %d", pos.x, pos.y, pos.z,speed.x,speed.y,speed.z,tracker.is_tracking_valid() );
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

