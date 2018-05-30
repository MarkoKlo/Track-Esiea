#include "WxMainApp.h"



IMPLEMENT_APP(WxMainApp);


bool WxMainApp::OnInit() {
	
	mainfrm = new wxFrame(NULL, wxID_ANY, _T("TrackESIEA: Menu de Callibration"), wxDefaultPosition, wxSize(800, 600), wxRESIZE_BORDER | wxSYSTEM_MENU | wxCAPTION | wxCLOSE_BOX | wxCLIP_CHILDREN);

	sizerVmain = new wxBoxSizer(wxVERTICAL);

	panelCalibration = new wxPanel(mainfrm);
		boxSizerCalibration = new wxBoxSizer(wxHORIZONTAL);
			staticBoxSizerCalibration = new wxStaticBoxSizer(wxHORIZONTAL, panelCalibration, _T("Calibration Camera:"));
			boxSizerCalibration->Add(staticBoxSizerCalibration, 1, wxALL | wxEXPAND, 5);

			boxSizerCameraOption = new wxBoxSizer(wxVERTICAL);
			staticBoxSizerCalibration->Add(boxSizerCameraOption);

			comboBoxSelectCamera = new wxComboBox(panelCalibration, -1, _T("Selectionnez votre Camera:"), wxDefaultPosition, wxSize(150, 0));
			boxSizerCameraOption->Add(comboBoxSelectCamera);

		panelCalibration->SetSizer(boxSizerCalibration);

	sizerVmain->Add(panelCalibration, 1, wxALL | wxEXPAND, 0);

	mainfrm->Show();

	return true;
}
