#include "client_test.h"
#include<iostream>

using namespace std;

client_test::client_test()
{
}

client_test::~client_test()
{
}

int main(int argc, char** argv)
{
	// cout est le flux de texte du terminal, '<<' permet d'injecter un élément dans cout, endl correspond à la fin de ligne
	cout << "Track'ESIEA est un pst de 2A qui vise a effectuer le tracking 3D d'un objet dans un espace delimite." << endl; 
	cout << "Librairie utilisée : OpenCV." << endl;
	cout << "Un grand merci à Antoine pour avoir créé le projet." << endl;
	system("pause");
	return 0;
}
