#pragma once
#include <wx/thread.h>
#include "AppLogger.h"
#include "SerialComm.h"
class MeasReceptionThrea : public wxThread
{
public:
    MeasReceptionThrea(AppLogger& appLogger, wxEvtHandler* parent);
    virtual ~MeasReceptionThrea();


    virtual void* Entry();

private:
    int m_count;
    wxEvtHandler* m_parent;
    AppLogger& appLogger;
};
// --------------------------------------------------------
// SerialComThread