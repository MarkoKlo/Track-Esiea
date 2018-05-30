#pragma once


#include <wx/wx.h>



class WxMainApp : public wxApp {
public:
	virtual bool OnInit();


private:
	wxFrame *mainfrm;
	wxBoxSizer *sizerVmain;


	wxPanel *panelCalibration;
	wxBoxSizer *boxSizerCalibration;
	wxStaticBoxSizer *staticBoxSizerCalibration;

	wxBoxSizer *boxSizerCameraOption;
	wxComboBox *comboBoxSelectCamera;
};


DECLARE_APP(WxMainApp);
