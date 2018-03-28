#include "WxMainApp.h"



IMPLEMENT_APP(WxMainApp);


bool WxMainApp::OnInit() {
	wxMessageBox(_T("Bienvenue sur wxWidgets !"));
	return false;
}
