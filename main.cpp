#include <wx/wx.h>
#include <wx/button.h>
#include <wx/log.h>
#include <wx/notebook.h>
#include <wx/panel.h>
#include <wx/frame.h>
#include <wx/thread.h>


#include <wx/chartpanel.h>
#include <wx/chart.h>
#include <wx/chartype.h>
#include <wx/wxfreechartdefs.h>
#include <wx/dataset.h>
#include <wx/colorscheme.h>
#include <wx/category/categorydataset.h>
#include <wx/xy/xyplot.h>
#include <wx/xy/xylinerenderer.h>
#include <wx/xy/xysimpledataset.h>
#include <wx/xy/vectordataset.h>
#include "wx/chartpanel.h"

#include "SerialComm.h"
#include "KalmanFilter.h"
#include "AppLogger.h"
#include "RawMeasurements.h"
#include "VelocityCalculator.h"


//int main() {
//
//    boost::asio::io_context io;
//
//    std::string com = "COM7";
//    SerialComm serialComm(io, com);
//
//
//    io.run();
//
//    return 0;
//}


#include <chrono>


#include "wx/wx.h"
#include <stdio.h>
using namespace std;

// --------------------------------------------------------
// a simple thread
//wxDECLARE_EVENT(wxEVT_MY_THREAD_EVENT, wxThreadEvent);
//wxDEFINE_EVENT(wxEVT_MY_THREAD_EVENT, wxThreadEvent);

class SerialComThread : public wxThread
{
public:
    SerialComThread(AppLogger& appLogger, wxEvtHandler* parent);
    virtual ~SerialComThread();

    
    virtual void* Entry();

private:
    int m_count;
    wxEvtHandler* m_parent;
    AppLogger& appLogger;
};
// --------------------------------------------------------
// SerialComThread

SerialComThread::SerialComThread(AppLogger& appLogger, wxEvtHandler* parent) : wxThread(), appLogger(appLogger), m_parent(parent)
{
    //this->appLogger = appLogger;
    m_count = 0;
}

SerialComThread::~SerialComThread()
{

}

wxThread::ExitCode SerialComThread::Entry()
{
    boost::asio::io_context io;
    std::string com = "COM7";

    //wxThreadEvent* event = new wxThreadEvent(wxEVT_MY_THREAD_EVENT);
    //event->SetString("Data from thread");
    //wxQueueEvent(m_parent, event);

    SerialComm serialComm(io, com, m_parent);
    io.run();

    const std::string threadFinished{ "SerialComm Thread finished successfully.\n" };
    appLogger.logSerialCommStartThread(threadFinished);

    return NULL;
}

class MyTabPanel : public wxPanel {
public:
    MyTabPanel(wxWindow* parent) {
        // Create a wxMathPlot component
        //mpWindow* graph = new mpWindow(parent, wxID_ANY);
        //graph->EnableDoubleBuffer(true);
    }

    // Other functions and variables you may need
};

LPCWSTR CharToLPCWSTR(const char* charString) {
    int charStringLength = strlen(charString) + 1;
    int sizeRequired = MultiByteToWideChar(CP_UTF8, 0, charString, charStringLength, NULL, 0);
    wchar_t* wideString = new wchar_t[sizeRequired];
    MultiByteToWideChar(CP_UTF8, 0, charString, charStringLength, wideString, sizeRequired);
    return wideString;
}


// --------------------------------------------------------
class MyFrame : public wxFrame
{
public:
    MyFrame(const wxString& title);
    ~MyFrame()
    {
        //serialComThread->Delete();
    }

    //Bind(wxEVT_MY_THREAD_EVENT, &MyFrame::OnThreadEvent, this);

private:
    void OnButtonClick(wxCommandEvent& event) {
        // Handle the button click event
        int myVariable = 42;

        // Use OutputDebugString to print the variable value
        wchar_t buffer[256];
        swprintf(buffer, 256, L"On button click The value of myVariable is %d", myVariable);
        OutputDebugString(buffer);
    }

    void OnBaudRateChoice(wxCommandEvent& event) {
        int selection = event.GetInt(); // Get the selected index
        wxLogMessage("Selected baudrate: %s", event.GetString().c_str());
    }
    void OnStopBitsChoice(wxCommandEvent& event) {
        int selection = event.GetInt(); // Get the selected index
        wxLogMessage("Selected number of stop bits: %s", event.GetString().c_str());
    }
    void OnParityChoice(wxCommandEvent& event) {
        int selection = event.GetInt(); // Get the selected index
        wxLogMessage("Selected parity: %s", event.GetString().c_str());
    }
    void OnStartReceptionClick(wxCommandEvent& event) {
        wxLogMessage("Data reception starts - new thread will be created!");
        m_timer.Start(200);
        //createAndStartDataReceptionThread();
    }
    void OnThreadEvent(wxThreadEvent& event) {
        
        MeasurementCustomizator* myEvent = dynamic_cast<MeasurementCustomizator*>(&event);
        if (myEvent)
        {
            const std::vector<std::string>& measurements = myEvent->GetStringVector();
            appLogger.logReceivedDataOnMainThread(measurements);
            if (measurements.size() == 7)
            {
                RawMeasurements rawMeasurement(appLogger);
                if (rawMeasurement.assign(measurements))
                {
                    rawMeasurementsSet.push_back(rawMeasurement);

                    updateMagnChart(rawMeasurement.getMagn());
                    VelocityCalculator velocityCalculator;
                }
            }
        }
        else
        {
            const std::string errThreadEvent{ "ERR when handling data from thread!!!" };
            appLogger.logErrThreadDataHandling(errThreadEvent);
        }
        //wxString data = event.GetString();
        //std::string dataReceivedOnMainThread{ "Received data in the main thread: " };
        //dataReceivedOnMainThread.append(data.ToStdString());
        //appLogger.logSerialCommStartThread(dataReceivedOnMainThread);
        //wxMessageBox("Received data in the main thread: " + data, "Thread Event");
    }

    void OnTimer(wxTimerEvent& event)
    {
        //wxLogMessage("Timer runs!");


        std::ofstream outputFile;
        outputFile.open("new_file.txt", std::ios::app);
        std::stringstream ss;
        //auto currentTime = getCurrentTimeWithMilliSeconds();
        ss << "The timer exppired and values updated: " << xNewPoint << " " << yNewPoint << '\n';
        outputFile << ss.str();
        outputFile.close();
    }

    void updateMagnChart(const int16_t magn)
    {
        points.push_back(wxRealPoint(xNewPoint, magn));
        xNewPoint += 1;
        yNewPoint = static_cast<double>(magn);
        XYPlot* plot = new XYPlot();
        XYSimpleDataset* dataset = new XYSimpleDataset();
        dataset->AddSerie(new XYSerie(points));
        dataset->SetRenderer(new XYLineRenderer());
        NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
        NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
        leftAxis->SetTitle(wxT("NR"));
        bottomAxis->SetTitle(wxT("magn"));
        plot->AddObjects(dataset, leftAxis, bottomAxis);

        Chart* chart = new Chart(plot, "Magnetometr");

        chartPanel->SetChart(chart);
    }

    std::vector<RawMeasurements> rawMeasurementsSet{};

    bool isDataReceptionStarted{ false };
    wxVector <wxRealPoint> points;

    //frame
    wxNotebook* m_notebook = nullptr;
    wxPanel* comSetupPanel = nullptr;
    wxPanel* dataReceptionPanel = nullptr;
    wxPanel* kalmanParamsSetupPanel = nullptr;
    wxChartPanel* chartPanel = nullptr;

    XYPlot* plot = nullptr;
    XYSimpleDataset* dataset = nullptr;
    NumberAxis* leftAxis = nullptr;
    NumberAxis* bottomAxis = nullptr;
    Chart* chart = nullptr;

    SerialComThread* serialComThread = nullptr;

    wxTimer m_timer;
    double xNewPoint = 0.0;
    double yNewPoint = 36.0;

    void prepareGui();
    void createDataReceptionThread();
    AppLogger appLogger;

};
MyFrame::MyFrame(const wxString& title)
    : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(900, 600))
{
    Centre();
    prepareGui();

    ///THREAD TO RECEIVE DATA
    if (isDataReceptionStarted)
    {
        //createDataReceptionThread();
    }

    createDataReceptionThread();
}

void MyFrame::createDataReceptionThread()
{
    serialComThread = new SerialComThread(appLogger, this);
    if (serialComThread->Create() != wxTHREAD_NO_ERROR)
    {
        const std::string threadNotCreated{ "Can't create SerialComThread thread! \n" };
        appLogger.logSerialCommStartThread(threadNotCreated);
    }
    else {
        if (serialComThread->Run() != wxTHREAD_NO_ERROR)
        {
            const std::string cantStartThread{ "Can't start SerialComThread thread! \n" };
            appLogger.logSerialCommStartThread(cantStartThread);
        }
        else
        {
            const std::string threadStarted{ "New thread SerialComThread started.\n" };
            appLogger.logSerialCommStartThread(threadStarted);
            Bind(wxEVT_MY_THREAD_EVENT, &MyFrame::OnThreadEvent, this);
        }
    }
}

void MyFrame::prepareGui()
{
    m_notebook = new wxNotebook(this, 1);
    comSetupPanel = new wxPanel(m_notebook);
    m_notebook->AddPage(comSetupPanel, "Serial port setup");
    dataReceptionPanel = new wxPanel(m_notebook);
    m_notebook->AddPage(dataReceptionPanel, "Data reception");
    kalmanParamsSetupPanel = new wxPanel(m_notebook);
    m_notebook->AddPage(kalmanParamsSetupPanel, "KF setup");
    chartPanel = new wxChartPanel(m_notebook);

    points.push_back(wxRealPoint(3.2, 23.2));
    points.push_back(wxRealPoint(4.2, 23.2));
    points.push_back(wxRealPoint(6.2, 28.2));
    points.push_back(wxRealPoint(9.2, 35.2));
    plot = new XYPlot();
    dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(points));
    dataset->SetRenderer(new XYLineRenderer());
    leftAxis = new NumberAxis(AXIS_LEFT);
    bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("X"));
    bottomAxis->SetTitle(wxT("Y"));
    plot->AddObjects(dataset, leftAxis, bottomAxis);

    chart = new Chart(plot, "DATA SET");

    chartPanel->SetChart(chart);
    m_notebook->AddPage(chartPanel, "Chart");

    wxSize size(100, 20);

    wxBoxSizer* panelSizer = new wxBoxSizer(wxVERTICAL);

    wxStaticText* LBcom = new wxStaticText(comSetupPanel, wxID_ANY, "COM port number:");
    wxTextCtrl* INcomName = new wxTextCtrl(comSetupPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, size);
    LBcom->SetPosition({ 40,100 });
    INcomName->SetPosition({ 200,100 });


    wxFont font(16, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
    wxStaticText* LBheaderText = new wxStaticText(comSetupPanel, wxID_ANY, "Enter serial communication settings", wxDefaultPosition, wxDefaultSize);
    LBheaderText->SetFont(font);
    LBheaderText->SetPosition({ 50,50 });


    wxArrayString baudRateChoices;
    baudRateChoices.Add("9600");
    baudRateChoices.Add("115200");
    baudRateChoices.Add("330400");
    wxChoice* baudRateChoice = new wxChoice(comSetupPanel, wxID_ANY, wxDefaultPosition, size, baudRateChoices);
    baudRateChoice->SetPosition({ 200, 150 });
    baudRateChoice->Bind(wxEVT_CHOICE, &MyFrame::OnBaudRateChoice, this);

    wxStaticText* LBbaudRate = new wxStaticText(comSetupPanel, wxID_ANY, "Baud rate:");
    //wxTextCtrl* INbaudRate = new wxTextCtrl(comSetupPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize);
    LBbaudRate->SetPosition({ 40,150 });
    //INbaudRate->SetPosition({ 200,150 });


    wxArrayString stopBitsChoices;
    stopBitsChoices.Add("0");
    stopBitsChoices.Add("1");
    wxChoice* stopBitsChoice = new wxChoice(comSetupPanel, wxID_ANY, wxDefaultPosition, size, stopBitsChoices);
    stopBitsChoice->SetPosition({ 200, 200 });
    stopBitsChoice->Bind(wxEVT_CHOICE, &MyFrame::OnStopBitsChoice, this);
    wxStaticText* LBstopBits = new wxStaticText(comSetupPanel, wxID_ANY, "Stop bits:");
    //wxTextCtrl* INstopBits = new wxTextCtrl(comSetupPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize);
    LBstopBits->SetPosition({ 40,200 });
    stopBitsChoice->SetPosition({ 200,200 });

    wxArrayString parityChoices;
    parityChoices.Add("NONE");
    parityChoices.Add("EVEN");
    parityChoices.Add("ODD");

    wxChoice* parityChoice = new wxChoice(comSetupPanel, wxID_ANY, wxDefaultPosition, size, parityChoices);
    parityChoice->SetPosition({ 200, 250 });
    parityChoice->Bind(wxEVT_CHOICE, &MyFrame::OnParityChoice, this);
    wxStaticText* LBparity = new wxStaticText(comSetupPanel, wxID_ANY, "Parity:", wxDefaultPosition, wxDefaultSize);
    wxTextCtrl* INparity = new wxTextCtrl(comSetupPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize);
    LBparity->SetPosition({ 40,250 });
    //INparity->SetPosition({ 200,250 });

    wxButton* BTconfirmSetupAndStartReception = new wxButton(comSetupPanel, wxID_ANY, "Start data reception");
    BTconfirmSetupAndStartReception->SetPosition({ 400, 500 });
    BTconfirmSetupAndStartReception->Bind(wxEVT_BUTTON, &MyFrame::OnStartReceptionClick, this);

    m_timer.Bind(wxEVT_TIMER, &MyFrame::OnTimer, this);
}

class MyApp : public wxApp
{
public:
    virtual bool OnInit();
};

IMPLEMENT_APP(MyApp)
bool MyApp::OnInit()
{
    MyFrame* frame = new MyFrame(wxT("Data visualization"));
    frame->Show(true);

    return true;
}