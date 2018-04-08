#include "WxMainApp.h"



IMPLEMENT_APP(WxMainApp);


bool WxMainApp::OnInit() {
	
	mainfrm = new wxFrame(NULL, wxID_ANY, _T("TrackESIEA: Menu de Callibration"), wxDefaultPosition, wxSize(800, 600), wxRESIZE_BORDER | wxSYSTEM_MENU | wxCAPTION | wxCLOSE_BOX | wxCLIP_CHILDREN);


	mainfrm->Show();

	return true;
}
