#pragma once


#include <wx/wx.h>



class WxMainApp : public wxApp {
public:
	virtual bool OnInit();
};


DECLARE_APP(WxMainApp);
