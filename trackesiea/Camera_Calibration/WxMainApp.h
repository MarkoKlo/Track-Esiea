#pragma once


#include <wx/wx.h>



class WxMainApp : public wxApp {
public:
	virtual bool OnInit();


private:
	wxFrame *mainfrm;
};


DECLARE_APP(WxMainApp);
