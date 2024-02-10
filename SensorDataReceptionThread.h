#pragma once
#include <wx/thread.h>
#include "AppLogger.h"
#include "SerialComm.h"

class SensorDataReceptionThread : public wxThread
{
public:
    SensorDataReceptionThread(AppLogger& appLogger, wxEvtHandler* parent);
    virtual ~SensorDataReceptionThread();


    virtual void* Entry();

private:
    int m_count;
    wxEvtHandler* m_parent;
    AppLogger& appLogger;
};
// --------------------------------------------------------
// SerialComThread