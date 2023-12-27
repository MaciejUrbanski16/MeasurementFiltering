#include "MainWindow.h"

MeasReceptionThrea::MeasReceptionThrea(AppLogger& appLogger, wxEvtHandler* parent) : wxThread(), appLogger(appLogger), m_parent(parent)
{
    //this->appLogger = appLogger;
    m_count = 0;
}

MeasReceptionThrea::~MeasReceptionThrea()
{

}

wxThread::ExitCode MeasReceptionThrea::Entry()
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


MyWindow::MyWindow(const wxString& title)
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

void MyWindow::OnBaudRateChoice(wxCommandEvent& event) 
{
    int selection = event.GetInt(); // Get the selected index
    wxLogMessage("Selected baudrate: %s", event.GetString().c_str());
}

void MyWindow::OnButtonClick(wxCommandEvent& event)
{
    // Handle the button click event
    int myVariable = 42;

    // Use OutputDebugString to print the variable value
    wchar_t buffer[256];
    swprintf(buffer, 256, L"On button click The value of myVariable is %d", myVariable);
    OutputDebugString(buffer);
}

void MyWindow::OnStopBitsChoice(wxCommandEvent& event)
{
    int selection = event.GetInt(); // Get the selected index
    wxLogMessage("Selected number of stop bits: %s", event.GetString().c_str());
}

void MyWindow::OnParityChoice(wxCommandEvent& event)
{
    int selection = event.GetInt(); // Get the selected index
    wxLogMessage("Selected parity: %s", event.GetString().c_str());
}

void MyWindow::OnStartReceptionClick(wxCommandEvent& event) 
{
    wxLogMessage("Data reception starts - new thread will be created!");
    m_timer.Start(200);
    //createAndStartDataReceptionThread();
}

void MyWindow::OnResetAccChart(wxCommandEvent& event)
{
    wxLogMessage("Reset acceleration chart!");
    xAccPoints.clear();
    yAccPoints.clear();
    zAccPoints.clear();
    timeNewAccPoint = 0;
}

/// <summary>

void MyWindow::OnThreadEvent(wxThreadEvent& event) {

    MeasurementCustomizator* myEvent = dynamic_cast<MeasurementCustomizator*>(&event);
    if (myEvent)
    {
        const std::vector<std::string>& measurements = myEvent->GetStringVector();
        appLogger.logReceivedDataOnMainThread(measurements);
        if (measurements.size() == 7)
        {
            const uint32_t deltaTimeMs = deltaTimeCalculator.getDurationInMs();
            MeasurementsController rawMeasurement(appLogger);
            if (rawMeasurement.assign(measurements, deltaTimeMs))
            {

                const uint32_t totalTimeMs = deltaTimeCalculator.getTotalTimeMs();

                //rawMeasurement.setDeltaTimeMs(deltaTimeMs);
                //rawMeasurementsSet.push_back(rawMeasurement);

                updateMagnChart(rawMeasurement.getMagn());
                updateAccChart(rawMeasurement.getXaccMPerS2(),
                    rawMeasurement.getYaccMPerS2(),
                    rawMeasurement.getZaccMPerS2(),
                    totalTimeMs);
                updateVelChart(rawMeasurement.getXvelocityMperS());
                updatePositionChart(rawMeasurement.getXDistance(), rawMeasurement.getYDistance());
                updateAngleVelocityChart(rawMeasurement.getXangleVelocityDegreePerS(),
                    rawMeasurement.getYangleVelocityDegreePerS(),
                    rawMeasurement.getZangleVelocityDegreePerS());

                //kalman filter experiment
                kalmanFilter.setInitialState(rawMeasurement.getXDistance(), rawMeasurement.getXvelocityMperS(), rawMeasurement.getXaccMPerS2(),
                    rawMeasurement.getYDistance(), rawMeasurement.getYvelocityMperS(), rawMeasurement.getYaccMPerS2());

                experimentKf(rawMeasurement.getXaccMPerS2(), rawMeasurement.getYaccMPerS2(), deltaTimeMs);

                kalmanFilterGyro.setInitialStateForGyro(rawMeasurement.getXangleVelocityDegreePerS(),
                    rawMeasurement.getYangleVelocityDegreePerS(),
                    rawMeasurement.getZangleVelocityDegreePerS());

                experimentGyroKf(rawMeasurement.getXangleVelocityDegreePerS(),
                    rawMeasurement.getYangleVelocityDegreePerS(),
                    rawMeasurement.getZangleVelocityDegreePerS(),
                    deltaTimeMs);

                const double calculatedPositionX = rawMeasurement.getXDistance();
                const double calculatedPositionY = rawMeasurement.getYDistance();

                const double filteredPositionX = kalmanFilter.vecX()(0);//PosX
                const double filteredVelocityX = kalmanFilter.vecX()(1);
                const double filteredAccX = kalmanFilter.vecX()(1);
                const double filteredPositionY = kalmanFilter.vecX()(3);//PosY
                const double filteredVelocityY = kalmanFilter.vecX()(3);

                updateFilteredPositionChart(filteredPositionX, filteredPositionY);
                updateFilteredVelocityChart(filteredVelocityX, filteredVelocityY);

                const double filteredXangle = kalmanFilterGyro.vecX()(0);

                updateFilteredAngleXVelocityChart(filteredXangle, rawMeasurement.getXangleVelocityDegreePerS(), deltaTimeMs);

                //

                relativePositionCalculator.calculateActualRelativePosition(rawMeasurement.getXvelocityMperS(), deltaTimeMs, rawMeasurement.getMagn());

                //VelocityCalculator velocityCalculator;
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

/// </summary>


void MyWindow::OnTimer(wxTimerEvent& event)
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

void MyWindow::updateMagnChart(const int16_t magn)
{
    magnPoints.push_back(wxRealPoint(xNewPoint, magn));
    xNewPoint += 1;
    yNewPoint = static_cast<double>(magn);
    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(magnPoints));
    //dataset->AddSerie(new XYSerie(accPoints));
    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("NR"));
    bottomAxis->SetTitle(wxT("magn"));
    plot->AddObjects(dataset, leftAxis, bottomAxis);

    Chart* chart = new Chart(plot, "Magnetometr");

    chartPanel->SetChart(chart);
}

void MyWindow::updateAccChart(const double xAccMPerS2, const double yAccMPerS2, const double zAccMPerS2, const uint32_t totalTimeMs)
{
    xAccPoints.push_back(wxRealPoint(timeNewAccPoint, xAccMPerS2));
    yAccPoints.push_back(wxRealPoint(timeNewAccPoint, yAccMPerS2));
    zAccPoints.push_back(wxRealPoint(timeNewAccPoint, zAccMPerS2));
    timeNewAccPoint += totalTimeMs;
    yNewPoint = static_cast<double>(xAccMPerS2);
    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(xAccPoints));
    dataset->AddSerie(new XYSerie(yAccPoints));
    dataset->AddSerie(new XYSerie(zAccPoints));
    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("Acceleration [m/s2]"));
    bottomAxis->SetTitle(wxT("time [ms]"));
    plot->AddObjects(dataset, leftAxis, bottomAxis);

    Chart* chart = new Chart(plot, "X Acceleration");

    accChartPanel->SetChart(chart);
}

void MyWindow::updateVelChart(const double xVelocity)
{
    velPoints.push_back(wxRealPoint(xNewPoint, xVelocity));
    xNewPoint += 1;
    yNewPoint = static_cast<double>(xVelocity);
    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(velPoints));
    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("X velocity [m/s]"));
    bottomAxis->SetTitle(wxT("time [ms]"));
    plot->AddObjects(dataset, leftAxis, bottomAxis);

    Chart* chart = new Chart(plot, "X Velocity");

    velChartPanel->SetChart(chart);
}

void MyWindow::updatePositionChart(const double xDistance, const double yDistance)
{
    currentXPos = currentXPos + xDistance;
    currentYPos = currentYPos + yDistance;
    positionPoints.push_back(wxRealPoint(currentXPos, currentYPos));

    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();

    dataset->AddSerie(new XYSerie(positionPoints));
    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("Y position [m]"));
    bottomAxis->SetTitle(wxT("X position [m]"));
    plot->AddObjects(dataset, leftAxis, bottomAxis);

    Chart* chart = new Chart(plot, "Position");

    positionChartPanel->SetChart(chart);
}

void MyWindow::updateAngleVelocityChart(const double xAngleVel, const double yAngleVel, const double zAngleVel)
{

    xAngleVelocityPoints.push_back(wxRealPoint(xAngleVelNewPoint, xAngleVel));
    yAngleVelocityPoints.push_back(wxRealPoint(xAngleVelNewPoint, yAngleVel));
    zAngleVelocityPoints.push_back(wxRealPoint(xAngleVelNewPoint, zAngleVel));
    xAngleVelNewPoint += 1;

    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(xAngleVelocityPoints));
    dataset->AddSerie(new XYSerie(yAngleVelocityPoints));
    dataset->AddSerie(new XYSerie(zAngleVelocityPoints));
    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("Angle velocity [deg/s]"));
    bottomAxis->SetTitle(wxT("time [ms]"));
    DatasetArray datasetArray();
    //datasetArray
    Legend* lengend = new Legend(10, 10);
    wxRect rect(wxSize(10, 10));
    //lengend.Draw(this, rect, datasetArray);

    plot->AddObjects(dataset, leftAxis, bottomAxis);
    //plot->SetLegend(lengend);
    Chart* chart = new Chart(plot, "Angle velocity");

    angleVelocityChartPanel->SetChart(chart);
}

void MyWindow::updateFilteredPositionChart(const double filteredPositionX, const double filteredPositionY)
{
    //xAngleVelocityPoints.push_back(wxRealPoint(xAngleVelNewPoint, xAngleVel));
    //yAngleVelocityPoints.push_back(wxRealPoint(xAngleVelNewPoint, yAngleVel));
    //zAngleVelocityPoints.push_back(wxRealPoint(xAngleVelNewPoint, zAngleVel));
    //xAngleVelNewPoint += 1;

    currentFilteredXPosition += filteredPositionX;
    currentFilteredYPosition += filteredPositionY;

    filteredPositionPoints.push_back(wxRealPoint(currentFilteredXPosition, currentFilteredYPosition));

    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(positionPoints));
    dataset->AddSerie(new XYSerie(filteredPositionPoints));

    //dataset->AddSerie(new XYSerie(yAngleVelocityPoints));
    //dataset->AddSerie(new XYSerie(zAngleVelocityPoints));
    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("Filtered X position [m]"));
    bottomAxis->SetTitle(wxT("Filtered Y position [m]"));
    DatasetArray datasetArray();
    //datasetArray
    Legend* lengend = new Legend(10, 10);
    wxRect rect(wxSize(10, 10));
    //lengend.Draw(this, rect, datasetArray);

    plot->AddObjects(dataset, leftAxis, bottomAxis);
    //plot->SetLegend(lengend);
    Chart* chart = new Chart(plot, "Filtered position");

    //wxButton* button = new wxButton(filteredPositionChartPanel, wxID_ANY, "Click me");
    //splitter = new wxSplitterWindow(filteredPositionChartPanel, wxID_ANY);

    //wxPanel* panel1 = new wxPanel(splitter, wxID_ANY);
    //wxPanel* panel2 = new wxPanel(splitter, wxID_ANY);
    //panel1->SetMinSize(wxSize(100, 100));
    //splitter->SplitVertically(filteredPositionChartPanel, panel2);
    //sizerPositionPlot->Add(splitter, 1, wxEXPAND | wxALL, 5);
    //sizerPositionPlot->Add(button, 0, wxALIGN_RIGHT | wxALL, 5);

    //filteredPositionChartPanel->SetSizer(sizerPositionPlot);

    filteredPositionChartPanel->SetChart(chart);
}

void MyWindow::updateFilteredAngleXVelocityChart(const double filteredXangle, const double measuredXangle, const uint32_t time)
{
    currentXangleFiltered += filteredXangle;
    currentXangleMeasured += measuredXangle;

    filteredXangleVelocity.push_back(wxRealPoint(xNewPoint, filteredXangle));
    measuredXangleVelocity.push_back(wxRealPoint(xNewPoint, measuredXangle));

    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(measuredXangleVelocity));
    dataset->AddSerie(new XYSerie(filteredXangleVelocity));

    //dataset->AddSerie(new XYSerie(yAngleVelocityPoints));
    //dataset->AddSerie(new XYSerie(zAngleVelocityPoints));
    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("X angle velocity [degree/s]"));
    bottomAxis->SetTitle(wxT("Time [ms]"));
    DatasetArray datasetArray();
    //datasetArray
    Legend* lengend = new Legend(10, 10);
    wxRect rect(wxSize(10, 10));
    //lengend.Draw(this, rect, datasetArray);

    plot->AddObjects(dataset, leftAxis, bottomAxis);
    //plot->SetLegend(lengend);
    Chart* chart = new Chart(plot, "Filtered/measured angle velocity");

    filteredAngleXVelocity->SetChart(chart);
}

void MyWindow::updateFilteredVelocityChart(const double filteredVelocityX, const double filteredVelocityY)
{

}

void MyWindow::createDataReceptionThread()
{
    serialComThread = new MeasReceptionThrea(appLogger, this);
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
            Bind(wxEVT_MY_THREAD_EVENT, &MyWindow::OnThreadEvent, this);
        }
    }
}

void MyWindow::prepareAccChart()
{
    

    wxPanel* panel = new wxPanel(m_notebook, wxID_ANY);
    accPanelSplitter = new wxSplitterWindow(panel, wxID_ANY);
    wxPanel* controlPanel = new wxPanel(accPanelSplitter, wxID_ANY);

    accChartPanel = new wxChartPanel(accPanelSplitter);

    sizerAccPlot = new wxBoxSizer(wxVERTICAL);
    accChartPanel->SetMinSize(wxSize(600, 600));

    wxBoxSizer* controlPanelSizer = new wxBoxSizer(wxVERTICAL);

    wxSpinCtrl* spinCtrlXacc = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -100, 100, 0);
    wxStaticText* xAccText = new wxStaticText(controlPanel, wxID_ANY, "Adjust X acc");

    wxSpinCtrl* spinCtrlYacc = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -100, 100, 6);
    wxStaticText* yAccText = new wxStaticText(controlPanel, wxID_ANY, "Adjust Y acc");

    wxSpinCtrl* spinCtrlZacc = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -100, 100, 6);
    wxStaticText* zAccText = new wxStaticText(controlPanel, wxID_ANY, "Adjust Z acc");

    controlPanelSizer->Add(xAccText, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->Add(spinCtrlXacc, 0, wxALL| wxALIGN_CENTER, 5);

    controlPanelSizer->Add(yAccText, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->Add(spinCtrlYacc, 0, wxALL| wxALIGN_CENTER, 5);

    controlPanelSizer->Add(zAccText, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->Add(spinCtrlZacc, 0, wxALL| wxALIGN_CENTER, 5);


    wxBoxSizer* hboxSizer = new wxBoxSizer(wxHORIZONTAL);

    wxButton* resetButton = new wxButton(controlPanel, wxID_ANY, "Reset chart");
    hboxSizer->Add(resetButton, 0, wxALL|wxALIGN_LEFT, 5);
    resetButton->Bind(wxEVT_BUTTON, &MyWindow::OnResetAccChart, this);

    wxButton* submitButton = new wxButton(controlPanel, wxID_ANY, "Submit adjustments");
    hboxSizer->Add(submitButton, 0, wxALL | wxALIGN_LEFT, 5);

    controlPanelSizer->Add(hboxSizer, 0, wxALL, 5);

    controlPanel->SetSizer(controlPanelSizer);


    accPanelSplitter->SplitVertically(accChartPanel, controlPanel);
    sizerAccPlot->Add(accPanelSplitter, 1, wxEXPAND | wxALL, 5);


    //CreateStatusBar();
    //SetStatusText("wxWidgets Splitter Example");
    panel->SetSizer(sizerAccPlot);
   // m_notebook->AddPage(panel, "Filtered position");

    m_notebook->AddPage(panel, "Acc chart");
}

void MyWindow::prepareVelChart()
{
    velChartPanel = new wxChartPanel(m_notebook);
    m_notebook->AddPage(velChartPanel, "Vel chart");
}

void MyWindow::preparePositionChart()
{
    positionChartPanel = new wxChartPanel(m_notebook);
    m_notebook->AddPage(positionChartPanel, "Pos chart");
}

void MyWindow::prepareAngleVelocityChart()
{
    angleVelocityChartPanel = new wxChartPanel(m_notebook);
    m_notebook->AddPage(angleVelocityChartPanel, "Angle velocity");
}

void MyWindow::prepareFilteredVelocityChart()
{
    filteredVelocityChartPanel = new wxChartPanel(m_notebook);
    m_notebook->AddPage(filteredVelocityChartPanel, "Filtered velocity");
}

void MyWindow::prepareFilteredAngleXVelocityChart()
{
    filteredAngleXVelocity = new wxChartPanel(m_notebook);
    m_notebook->AddPage(filteredAngleXVelocity, "Filtered X angle velocity");
}

void MyWindow::prepareFilteredPositionChart()
{
    wxPanel* panel = new wxPanel(m_notebook, wxID_ANY);
    splitter = new wxSplitterWindow(panel, wxID_ANY);


    // Create two panels to be placed in the splitter window
    //wxPanel* panel1 = new wxPanel(splitter, wxID_ANY);
    wxPanel* controlPanel = new wxPanel(splitter, wxID_ANY);

    filteredPositionChartPanel = new wxChartPanel(splitter);
    sizerPositionPlot = new wxBoxSizer(wxVERTICAL);
    filteredPositionChartPanel->SetMinSize(wxSize(600, 600));
    //filteredVelocityChartPanel->SetSize(200, 200);
    //sizer->Add(filteredPositionChartPanel, 1, wxEXPAND | wxALL, 5);
    wxButton* button = new wxButton(controlPanel, wxID_ANY, "Click me");
    //sizerPositionPlot->Add(button, 0, wxALIGN_RIGHT | wxALL, 5);
    wxBoxSizer* controlPanelSizer = new wxBoxSizer(wxVERTICAL);
    wxSpinCtrl* spinCtrlPCoefficient = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -100, 100, 0);
    wxStaticText* pCoefficientText = new wxStaticText(controlPanel, wxID_ANY, "Spin Control for p coefficient:");

    wxSpinCtrl* spinCtrlSCoefficient = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -100, 100, 6);
    wxStaticText* sCoefficientText = new wxStaticText(controlPanel, wxID_ANY, "Spin Control for s coefficient:");

    matrixGrid = new wxGrid(controlPanel, wxID_ANY);
    matrixGrid->CreateGrid(6, 6);

    matrixGrid->HideRowLabels();
    matrixGrid->HideColLabels();
    //matrixGrid->SetSize(100, 100);

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            matrixGrid->SetCellValue(i, j, wxString::Format("%d", i * 6 + j));
        }
    }

    for (int i = 0; i < 6; ++i) {
        matrixGrid->SetRowSize(i, 30);  // Set the height of each row
        matrixGrid->SetColSize(i, 50);  // Set the width of each column
    }

    matrixRCovariance = new wxGrid(controlPanel, wxID_ANY);
    matrixRCovariance->CreateGrid(2, 2);

    matrixRCovariance->HideRowLabels();
    matrixRCovariance->HideColLabels();
    //matrixGrid->SetSize(100, 100);

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            matrixRCovariance->SetCellValue(i, j, wxString::Format("%d", i * 6 + j));
        }
    }

    for (int i = 0; i < 2; ++i) {
        matrixRCovariance->SetRowSize(i, 30);  // Set the height of each row
        matrixRCovariance->SetColSize(i, 50);  // Set the width of each column
    }

    controlPanelSizer->Add(pCoefficientText, 0, wxALL, 5);
    controlPanelSizer->Add(spinCtrlPCoefficient, 0, wxALL | wxALIGN_RIGHT, 5);
    controlPanelSizer->Add(sCoefficientText, 0, wxALL, 5);
    controlPanelSizer->Add(spinCtrlSCoefficient, 0, wxALL | wxALIGN_RIGHT, 5);
    controlPanelSizer->Add(button, 0, wxALL, 5);
    controlPanelSizer->Add(matrixGrid, 1, wxEXPAND | wxALL, 5);
    controlPanelSizer->Add(matrixRCovariance, 1, wxALIGN_CENTER | wxALL, 5);
    controlPanel->SetSizer(controlPanelSizer);


    splitter->SplitVertically(filteredPositionChartPanel, controlPanel);
    sizerPositionPlot->Add(splitter, 1, wxEXPAND | wxALL, 5);


    //CreateStatusBar();
    //SetStatusText("wxWidgets Splitter Example");
    panel->SetSizer(sizerPositionPlot);
    m_notebook->AddPage(panel, "Filtered position");
}

void MyWindow::prepareGui()
{
    m_notebook = new wxNotebook(this, 1);
    comSetupPanel = new wxPanel(m_notebook);
    m_notebook->AddPage(comSetupPanel, "Serial port setup");
    dataReceptionPanel = new wxPanel(m_notebook);
    m_notebook->AddPage(dataReceptionPanel, "Data reception");
    kalmanParamsSetupPanel = new wxPanel(m_notebook);
    innerNotebook = new wxNotebook(kalmanParamsSetupPanel, 2);
    wxPanel* innerPanel = new wxPanel(innerNotebook);
    innerNotebook->AddPage(innerPanel, "Inner");
    m_notebook->AddPage(kalmanParamsSetupPanel, "KF setup");
    chartPanel = new wxChartPanel(m_notebook);

    //splitter = new wxSplitterWindow(, wxID_ANY);

    prepareAccChart();
    prepareVelChart();
    preparePositionChart();
    prepareAngleVelocityChart();
    prepareFilteredPositionChart();
    prepareFilteredVelocityChart();
    prepareFilteredAngleXVelocityChart();

    // Create a notebook for outer tabs
    //wxNotebook* outerNotebook = new wxNotebook(this, wxID_ANY);

    // Add outer tabs to the notebook
    m_notebook->AddPage(new OuterTab(m_notebook, "Outer Tab 1"), "Outer Tab 1");
    m_notebook->AddPage(new OuterTab(m_notebook, "Outer Tab 2"), "Outer Tab 2");
    //m_notebook->AddPage(new OuterTab(m_notebook, "Outer Tab 2"), "Outer Tab 2");


    //OuterNotebook* outerNotebook = new OuterNotebook(this, 2);
    //m_notebook->AddPage(outerNotebook, "CHARTS");

    magnPoints.push_back(wxRealPoint(3.2, 23.2));
    magnPoints.push_back(wxRealPoint(4.2, 23.2));
    magnPoints.push_back(wxRealPoint(6.2, 28.2));
    magnPoints.push_back(wxRealPoint(9.2, 35.2));
    plot = new XYPlot();
    dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(magnPoints));
    dataset->SetRenderer(new XYLineRenderer());
    leftAxis = new NumberAxis(AXIS_LEFT);
    bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("X"));
    bottomAxis->SetTitle(wxT("Y"));
    plot->AddObjects(dataset, leftAxis, bottomAxis);

    chart = new Chart(plot, "DATA SET");

    chartPanel->SetChart(chart);
    //m_notebook->AddPage(new OuterTab(m_notebook, "Outer Tab 2"), "Outer Tab 2");
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
    baudRateChoice->Bind(wxEVT_CHOICE, &MyWindow::OnBaudRateChoice, this);

    wxStaticText* LBbaudRate = new wxStaticText(comSetupPanel, wxID_ANY, "Baud rate:");
    //wxTextCtrl* INbaudRate = new wxTextCtrl(comSetupPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize);
    LBbaudRate->SetPosition({ 40,150 });
    //INbaudRate->SetPosition({ 200,150 });


    wxArrayString stopBitsChoices;
    stopBitsChoices.Add("0");
    stopBitsChoices.Add("1");
    wxChoice* stopBitsChoice = new wxChoice(comSetupPanel, wxID_ANY, wxDefaultPosition, size, stopBitsChoices);
    stopBitsChoice->SetPosition({ 200, 200 });
    stopBitsChoice->Bind(wxEVT_CHOICE, &MyWindow::OnStopBitsChoice, this);
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
    parityChoice->Bind(wxEVT_CHOICE, &MyWindow::OnParityChoice, this);
    wxStaticText* LBparity = new wxStaticText(comSetupPanel, wxID_ANY, "Parity:", wxDefaultPosition, wxDefaultSize);
    wxTextCtrl* INparity = new wxTextCtrl(comSetupPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize);
    LBparity->SetPosition({ 40,250 });
    //INparity->SetPosition({ 200,250 });

    wxButton* BTconfirmSetupAndStartReception = new wxButton(comSetupPanel, wxID_ANY, "Start data reception");
    BTconfirmSetupAndStartReception->SetPosition({ 400, 500 });
    BTconfirmSetupAndStartReception->Bind(wxEVT_BUTTON, &MyWindow::OnStartReceptionClick, this);

    m_timer.Bind(wxEVT_TIMER, &MyWindow::OnTimer, this);
}
