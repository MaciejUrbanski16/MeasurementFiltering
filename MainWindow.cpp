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

    //SerialComm serialComm(io, com, m_parent);
    Server server(io, 8081, appLogger, m_parent);
    io.run();

    const std::string threadFinished{ "SerialComm Thread finished successfully.\n" };
    appLogger.logSerialCommStartThread(threadFinished);

    return NULL;
}


MyWindow::MyWindow(const wxString& title)
    : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(1080, 600))
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

void MyWindow::OnResetAngleVelChart(wxCommandEvent& event)
{
    xAngleVelocityPoints.clear();
    yAngleVelocityPoints.clear();
    zAngleVelocityPoints.clear();
    xAngleVelNewPoint = 0;
    wxLogMessage("Reset angle velocity chart!");
}

void MyWindow::OnSubmitAccAdjustments(wxCommandEvent& event)
{
    const int xAccCtrlValue = spinCtrlXacc->GetValue();
    const int yAccCtrlValue = spinCtrlYacc->GetValue();
    const int zAccCtrlValue = spinCtrlZacc->GetValue();

    rawGrawity = zAccCtrlValue;
    xBias = xAccCtrlValue;
    yBias = yAccCtrlValue;
    wxLogMessage("New acc adj X:%d Y:%d Z:%d!", xAccCtrlValue, yAccCtrlValue, zAccCtrlValue);
}

void MyWindow::OnSpinXAccIncrUpdate(wxSpinEvent& event)
{
    const int newIncrement = spinCtrlXaccMultiplicator->GetValue();
    spinCtrlXacc->SetIncrement(newIncrement);
}

void MyWindow::OnSpinXAccUpdate(wxSpinEvent& event)
{
    const int xAccCtrlValue = spinCtrlXacc->GetValue();
    xBias = xAccCtrlValue;
}

void MyWindow::OnSpinYAccIncrUpdate(wxSpinEvent& event)
{
    const int newIncrement = spinCtrlYaccMultiplicator->GetValue();
    spinCtrlYacc->SetIncrement(newIncrement);
}

void MyWindow::OnSpinYAccUpdate(wxSpinEvent& event)
{
    const int yAccCtrlValue = spinCtrlYacc->GetValue();
    yBias = yAccCtrlValue;
}

void MyWindow::OnSpinZAccIncrUpdate(wxSpinEvent& event)
{
    const int newIncrement = spinCtrlZaccMultiplicator->GetValue();
    spinCtrlZacc->SetIncrement(newIncrement);
}

void MyWindow::OnSpinZAccUpdate(wxSpinEvent& event)
{
    const int zAccCtrlValue = spinCtrlZacc->GetValue();
    rawGrawity = zAccCtrlValue;
}


void MyWindow::OnSubmitAngleVelAdjustments(wxCommandEvent& event)
{
    const int xAngleVelCtrlValue = spinCtrlXangleVel->GetValue();
    const int yAngleVelCtrlValue = spinCtrlYangleVel->GetValue();
    const int zAngleVelCtrlValue = spinCtrlZangleVel->GetValue();

    xGyroBias = xAngleVelCtrlValue;
    yGyroBias = yAngleVelCtrlValue;
    zGyroBias = zAngleVelCtrlValue;
    wxLogMessage("New acc adj X:%d Y:%d Z:%d!", xAngleVelCtrlValue, yAngleVelCtrlValue, zAngleVelCtrlValue);
}

void MyWindow::OnSpinXAnglVelIncrUpdate(wxSpinEvent& event)
{
    const int newIncrement = spinCtrlXangleVelMultiplicator->GetValue();
    spinCtrlXangleVel->SetIncrement(newIncrement);
}

void MyWindow::OnSpinXAngleVelUpdate(wxSpinEvent& event)
{
    const int xAngleVelCtrlValue = spinCtrlXangleVel->GetValue();
    xGyroBias = xAngleVelCtrlValue;
}

void MyWindow::OnSpinYAnglVelIncrUpdate(wxSpinEvent& event)
{
    const int newIncrement = spinCtrlYangleVelMultiplicator->GetValue();
    spinCtrlYangleVel->SetIncrement(newIncrement);
}

void MyWindow::OnSpinYAngleVelUpdate(wxSpinEvent& event)
{
    const int yAngleVelCtrlValue = spinCtrlYangleVel->GetValue();
    yGyroBias = yAngleVelCtrlValue;
}

void MyWindow::OnSpinZAnglVelIncrUpdate(wxSpinEvent& event)
{
    const int newIncrement = spinCtrlZangleVelMultiplicator->GetValue();
    spinCtrlZangleVel->SetIncrement(newIncrement);
}

void MyWindow::OnSpinZAngleVelUpdate(wxSpinEvent& event)
{
    const int zAngleVelCtrlValue = spinCtrlZangleVel->GetValue();
    zGyroBias = zAngleVelCtrlValue;
}

void MyWindow::OnResetMagnChart(wxCommandEvent& event)
{
    magnPoints.clear();
    //yAngleVelocityPoints.clear();
    //zAngleVelocityPoints.clear();
    azimuthXPoint = 0;
    wxLogMessage("Reset angle velocity chart!");
}

void MyWindow::OnApplyKFTunning(wxCommandEvent& event)
{
    bool conversionOK = true;
   
    for (int i = 0; i < DIM_Z; i++)
    {
        for (int j = 0; j < DIM_Z; j++)
        {
            wxString cellValue = matrixRCovariance->GetCellValue(i, j);
            double cellValueAsDouble{ 0.0 };
            if (cellValue.ToDouble(&cellValueAsDouble)) 
            {
                //wxPrintf("Double value: %lf\n", cellValueAsDouble);
                matR(i * DIM_Z + j) = cellValueAsDouble;    
            }
            else 
            {
                conversionOK = false;
                wxLogError("Wiersz %d kolumna %d z wartoscia: %s dla macierzy R nie moze zostac przekonwertowana na liczbe zmiennoprzecinkowa!", i, j, cellValue);
            }
            
        }
    }
    if (conversionOK)
    {
        std::stringstream msgToLog;
        msgToLog << "Apply KF tunning for R matrix";

        for (int i = 0; i < DIM_Z * DIM_Z; i++)
        {
            msgToLog << " matR[" << i << "]: " << matR(i);
        }
        wxString wxStringValue(msgToLog.str());
        wxLogMessage(wxStringValue);
    }

    bool conversionHOK = true;

    for (int i = 0; i < DIM_Z; i++)
    {
        for (int j = 0; j < DIM_X; j++)
        {
            wxString cellValue = matrixH->GetCellValue(i, j);
            double cellValueAsDouble{ 0.0 };
            if (cellValue.ToDouble(&cellValueAsDouble))
            {
                //wxPrintf("Double value: %lf\n", cellValueAsDouble);
                matH(i * DIM_Z + j) = cellValueAsDouble;
            }
            else
            {
                conversionHOK = false;
                wxLogError("Wiersz %d kolumna %d dla macierzy H z wartoscia: %s nie moze zostac przekonwertowana na liczbe zmiennoprzecinkowa!", i, j, cellValue);
            }

        }
    }
    if (conversionHOK)
    {
        std::stringstream msgToLog;
        msgToLog << "Apply KF tunning for H matrix";

        for (int i = 0; i < DIM_X * DIM_Z; i++)
        {
            msgToLog << " matH[" << i << "]: " << matH(i);
        }
        wxString wxStringValue(msgToLog.str());
        wxLogMessage(wxStringValue);
    }
}

void MyWindow::OnSubmitMagnAdjustments(wxCommandEvent& event)
{

}

/// <summary>

void MyWindow::OnThreadEvent(wxThreadEvent& event) {

    MeasurementCustomizator* myEvent = dynamic_cast<MeasurementCustomizator*>(&event);
    if (myEvent)
    {
        const std::vector<std::string>& measurements = myEvent->GetStringVector();
        appLogger.logReceivedDataOnMainThread(measurements);
        if (measurements.size() == 10)
        {
            if (isFirstMeasurement)
            {
                deltaTimeCalculator.startTimer();
                isFirstMeasurement = false;
                return;
            }
            const uint32_t deltaTimeMs = deltaTimeCalculator.getDurationInMs();
            MeasurementsController rawMeasurement(appLogger, rawGrawity, xBias, yBias, xGyroBias, yGyroBias, zGyroBias);
            totalTimeMs += static_cast<double>(deltaTimeMs);
            if (rawMeasurement.assign(measurements, deltaTimeMs))
            {

                //const uint32_t totalTimeMs = deltaTimeCalculator.getTotalTimeMs();

                //const double distance = haversineConverter.calculateDistance(rawMeasurement.getLongitude(), 70.2, rawMeasurement.getLatitude(), 30.2);
                //const auto xyPoint = haversineConverter.convertToXY_({70.234, 30.234});
                ////rawMeasurement.setDeltaTimeMs(deltaTimeMs);
                ////rawMeasurementsSet.push_back(rawMeasurement);
                if (kalmanFilterSetupGui.getIsCallibrationDone() == false)
                {
                    updateMagnChart(rawMeasurement.getRawXMagn(), rawMeasurement.getRawYMagn(), rawMeasurement.getAzimuth(), 1.0, totalTimeMs);
                    updateAccChart(rawMeasurement.getXaccMPerS2(),
                        rawMeasurement.getYaccMPerS2(),
                        rawMeasurement.getZaccMPerS2(),
                        totalTimeMs, deltaTimeMs);
                    ////updateVelChart(rawMeasurement.getXvelocityMperS());
                    ////updatePositionChart(rawMeasurement.getXDistance(), rawMeasurement.getYDistance(), totalTimeMs);
                    updateAngleVelocityChart(rawMeasurement.getXangleVelocityDegreePerS(),
                        rawMeasurement.getYangleVelocityDegreePerS(),
                        rawMeasurement.getZangleVelocityDegreePerS(), 1.0, totalTimeMs);

                    latitude = latitude + 0.003;
                    longitude = longitude - 0.004;
                    const auto gpsBasedPosition = haversineConverter.calculateCurrentPosition(longitude, latitude);
                    updateGpsBasedPositionChart(gpsBasedPosition);
                }
                else
                {
                    if (measurementCounter == 0)
                    {
                        totalTimeMs = 0;
                        resetChartsAfterCallibration();

                    }
                    measurementCounter++;


                    kalmanFilterAzimuth.setInitialStateForAzimuth(rawMeasurement.getAzimuth());
                    experimentKfAzimuth(rawMeasurement.getXangleVelocityDegreePerS(), rawMeasurement.getYangleVelocityDegreePerS(),
                        rawMeasurement.getZangleVelocityDegreePerS(), rawMeasurement.getAzimuth(), deltaTimeMs);
                    updatePositionChart(rawMeasurement.getXDistance(), rawMeasurement.getYDistance(), totalTimeMs);
                    //roll (X)
                    const double filteredAzimuth = kalmanFilterAzimuth.vecX()[0];
                    const double filteredZAngleVel = kalmanFilterAzimuth.vecX()[5];

                    updateMagnChart(rawMeasurement.getRawXMagn(), rawMeasurement.getRawYMagn(), rawMeasurement.getAzimuth(), filteredAzimuth, totalTimeMs);
                    updateAccChart(rawMeasurement.getXaccMPerS2(),
                        rawMeasurement.getYaccMPerS2(),
                        rawMeasurement.getZaccMPerS2(),
                        totalTimeMs, deltaTimeMs);
                    updateAngleVelocityChart(rawMeasurement.getXangleVelocityDegreePerS(),
                        rawMeasurement.getYangleVelocityDegreePerS(),
                        rawMeasurement.getZangleVelocityDegreePerS(), filteredZAngleVel, totalTimeMs);

                    ////kalman filter experiment
                    kalmanFilter.setInitialState(rawMeasurement.getXDistance(), rawMeasurement.getXvelocityMperS(), rawMeasurement.getXaccMPerS2(),
                        rawMeasurement.getYDistance(), rawMeasurement.getYvelocityMperS(), rawMeasurement.getYaccMPerS2());

                    experimentKf(rawMeasurement.getXaccMPerS2(), rawMeasurement.getYaccMPerS2(), deltaTimeMs);


                    positionUpdater.updatePosition(filteredPositionX, filteredPositionY, rawMeasurement.getXDistance(), rawMeasurement.getYDistance(),
                        filteredAzimuth);

                    filteredPositionX = kalmanFilter.vecX()(0);//PosX
                    //const double filteredVelocityX = kalmanFilter.vecX()(1);
                    //const double filteredAccX = kalmanFilter.vecX()(1);
                    filteredPositionY = kalmanFilter.vecX()(3);//PosY
                    //const double filteredVelocityY = kalmanFilter.vecX()(3);

                    const auto calculatedPosition{ positionUpdater.getCurrentPosition() };

                    updateFilteredPositionChart(filteredPositionX, filteredPositionY, calculatedPosition, totalTimeMs);
                }



                //kalmanFilterGyro.setInitialStateForGyro(rawMeasurement.getXangleVelocityDegreePerS(),
                //    rawMeasurement.getYangleVelocityDegreePerS(),
                //    rawMeasurement.getZangleVelocityDegreePerS());

                //experimentGyroKf(rawMeasurement.getXangleVelocityDegreePerS(),
                //    rawMeasurement.getYangleVelocityDegreePerS(),
                //    rawMeasurement.getZangleVelocityDegreePerS(),
                //    deltaTimeMs);

                //const double calculatedPositionX = rawMeasurement.getXDistance();
                //const double calculatedPositionY = rawMeasurement.getYDistance();


                //updateFilteredVelocityChart(filteredVelocityX, filteredVelocityY, totalTimeMs);

                //const double filteredXangle = kalmanFilterGyro.vecX()(0);

                //updateFilteredAngleXVelocityChart(filteredXangle, rawMeasurement.getXangleVelocityDegreePerS(), totalTimeMs);

                //

                //relativePositionCalculator.calculateActualRelativePosition(rawMeasurement.getXvelocityMperS(), deltaTimeMs, rawMeasurement.getAzimuth());

                //VelocityCalculator velocityCalculator;
            }
        }
        else
        {
            const std::string errThreadEvent{ "ERR when handling data from thread - wrong size of data - should be 10!!!" };
            appLogger.logErrThreadDataHandling(errThreadEvent);
         }
    }
    else
    {
        const std::string errThreadEvent{ "ERR when handling data from thread - no event received!!!" };
        appLogger.logErrThreadDataHandling(errThreadEvent);
    }
    //wxString data = event.GetString();
    //std::string dataReceivedOnMainThread{ "Received data in the main thread: " };
    //dataReceivedOnMainThread.append(data.ToStdString());
    //appLogger.logSerialCommStartThread(dataReceivedOnMainThread);
    //wxMessageBox("Received data in the main thread: " + data, "Thread Event");
}

void MyWindow::resetChartsAfterCallibration()
{
    magnPointsBuffer.Clear();
    filteredAzimuthBuffer.Clear();

    xAccBuffer.Clear();
    yAccBuffer.Clear();
    zAccBuffer.Clear();

    xAngleVelocityBuffer.Clear();
    yAngleVelocityBuffer.Clear();
    zAngleVelocityBuffer.Clear();
    filteredZangleVelocityBuffer.Clear();

    rawPositionBuffer.Clear();
    filteredPositionBuffer.Clear();
    calculatedPositionBuffer.Clear();
    gpsBasedPositionBuffer.Clear();
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

void MyWindow::updateMagnChart(const int16_t xMagn, const int16_t yMagn, const double azimuth, const double filteredAzimuth, const double timeMs)
{
    xMagnValue->SetLabel(std::to_string(xMagn));
    yMagnValue->SetLabel(std::to_string(yMagn));
    orientationValue->SetLabel(std::to_string(azimuth));

    magnPointsBuffer.AddElement(wxRealPoint(timeMs, azimuth));
    filteredAzimuthBuffer.AddElement(wxRealPoint(timeMs, filteredAzimuth));

    //magnPoints.push_back(wxRealPoint(timeMs, azimuth));
    //azimuthXPoint += 1;
    yNewPoint = static_cast<double>(azimuth);
    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(magnPointsBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(filteredAzimuthBuffer.getBuffer()));
    //dataset->AddSerie(new XYSerie(accPoints));
    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("Azimuth [deg]"));
    bottomAxis->SetTitle(wxT("Time [ms]"));
    if (magnPointsBuffer.getBuffer().size() >= 100)
    {
        bottomAxis->SetFixedBounds(magnPointsBuffer.getBuffer()[0].x, magnPointsBuffer.getBuffer()[99].x);
    }
    plot->AddObjects(dataset, leftAxis, bottomAxis);

    Chart* chart = new Chart(plot, "Magnetometr");

    azimuthChartPanel->SetChart(chart);
}

void MyWindow::updateAccChart(const double xAccMPerS2, const double yAccMPerS2, const double zAccMPerS2, const double timeMs, const uint32_t deltaTime)
{
    xAccValue->SetLabel(std::to_string(xAccMPerS2));
    yAccValue->SetLabel(std::to_string(yAccMPerS2));
    zAccValue->SetLabel(std::to_string(zAccMPerS2));

    deltaTimeValue->SetLabel(std::to_string(deltaTime));
    totalTimeValue->SetLabel(std::to_string(timeMs));

    //xAccPoints.push_back(wxRealPoint(timeMs, xAccMPerS2));
    //yAccPoints.push_back(wxRealPoint(timeMs, yAccMPerS2));
    //zAccPoints.push_back(wxRealPoint(timeMs, zAccMPerS2));
    xAccBuffer.AddElement(wxRealPoint(timeMs, xAccMPerS2));
    yAccBuffer.AddElement(wxRealPoint(timeMs, yAccMPerS2));
    zAccBuffer.AddElement(wxRealPoint(timeMs, zAccMPerS2));
    //timeNewAccPoint += totalTimeMs / 1000;
    //yNewPoint = static_cast<double>(xAccMPerS2);
    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(xAccBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(yAccBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(zAccBuffer.getBuffer()));
    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("Acceleration [m/s2]"));
    bottomAxis->SetTitle(wxT("time [ms]"));
    if (xAccBuffer.getBuffer().size() >= 100)
    {
        bottomAxis->SetFixedBounds(xAccBuffer.getBuffer()[0].x, xAccBuffer.getBuffer()[99].x);
    }
    plot->AddObjects(dataset, leftAxis, bottomAxis);

    Chart* chart = new Chart(plot, "X Acceleration");

    accChartPanel->SetChart(chart);
}

//void MyWindow::updateVelChart(const double xVelocity)
//{
//    velPoints.push_back(wxRealPoint(xNewPoint, xVelocity));
//    xNewPoint += 1;
//    yNewPoint = static_cast<double>(xVelocity);
//    XYPlot* plot = new XYPlot();
//    XYSimpleDataset* dataset = new XYSimpleDataset();
//    dataset->AddSerie(new XYSerie(velPoints));
//    dataset->SetRenderer(new XYLineRenderer());
//    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
//    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
//    leftAxis->SetTitle(wxT("X velocity [m/s]"));
//    bottomAxis->SetTitle(wxT("time [ms]"));
//    plot->AddObjects(dataset, leftAxis, bottomAxis);
//
//    Chart* chart = new Chart(plot, "X Velocity");
//
//    velChartPanel->SetChart(chart);
//}

void MyWindow::updatePositionChart(const double xDistance, const double yDistance, const double timeMs)
{
    currentXPos = currentXPos + xDistance;
    currentYPos = currentYPos + yDistance;
    //rawPositionPoints.push_back(wxRealPoint(currentXPos, currentYPos));
    rawPositionBuffer.AddElement(wxRealPoint(currentXPos, currentYPos));

    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    

    dataset->AddSerie(new XYSerie(rawPositionBuffer.getBuffer()));
    dataset->SetRenderer(new XYLineRenderer());
    dataset->GetSerie(0)->SetName("raw position");
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("Y position [m]"));
    bottomAxis->SetTitle(wxT("X position [m]"));
    //if (rawPositionBuffer.getBuffer().size() >= 100)
    //{
    //    bottomAxis->SetFixedBounds(rawPositionBuffer.getBuffer()[0].x, rawPositionBuffer.getBuffer()[99].x);
    //}
    Legend* legend = new Legend(wxTOP, wxLEFT);
    plot->SetLegend(legend);

    plot->AddObjects(dataset, leftAxis, bottomAxis);


    Chart* chart = new Chart(plot, "Position");

    positionChartPanel->SetChart(chart);
}

void MyWindow::updateGpsBasedPositionChart(std::pair<double, double> gpsBasedPosition)
{
    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    gpsBasedPositionBuffer.AddElement(wxRealPoint(gpsBasedPosition.first, gpsBasedPosition.second));
    dataset->AddSerie(new XYSerie(gpsBasedPositionBuffer.getBuffer()));
    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("Y [m]"));
    bottomAxis->SetTitle(wxT("X [m]"));
    plot->AddObjects(dataset, leftAxis, bottomAxis);
    Chart* chart = new Chart(plot, "GPS based position");
    gpsBasedPositionChartPanel->SetChart(chart);
}

void MyWindow::updateAngleVelocityChart(const double xAngleVel, const double yAngleVel, const double zAngleVel, const double filteredZangleVelocity, const double timeMs)
{
    xAngleVelValue->SetLabel(std::to_string(xAngleVel));
    yAngleVelValue->SetLabel(std::to_string(yAngleVel));
    zAngleVelValue->SetLabel(std::to_string(zAngleVel));

    //xAngleVelocityPoints.push_back(wxRealPoint(timeMs, xAngleVel));
    //yAngleVelocityPoints.push_back(wxRealPoint(timeMs, yAngleVel));
    //zAngleVelocityPoints.push_back(wxRealPoint(timeMs, zAngleVel));
    //xAngleVelNewPoint += 1;
    xAngleVelocityBuffer.AddElement(wxRealPoint(timeMs, xAngleVel));
    yAngleVelocityBuffer.AddElement(wxRealPoint(timeMs, yAngleVel));
    zAngleVelocityBuffer.AddElement(wxRealPoint(timeMs, zAngleVel));
    filteredZangleVelocityBuffer.AddElement(wxRealPoint(timeMs, filteredZangleVelocity));

    XYPlot* plot = new XYPlot();    
    XYSimpleDataset* dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(xAngleVelocityBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(yAngleVelocityBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(zAngleVelocityBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(filteredZangleVelocityBuffer.getBuffer()));
    dataset->GetSerie(1)->SetName("yAngleVelocity");
    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("Angle velocity [deg/s]"));
    bottomAxis->SetTitle(wxT("time [ms]"));
    if (xAngleVelocityBuffer.getBuffer().size() >= 100)
    {
        bottomAxis->SetFixedBounds(xAngleVelocityBuffer.getBuffer()[0].x, xAngleVelocityBuffer.getBuffer()[99].x);
    }
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

void MyWindow::updateFilteredPositionChart(const double filteredPositionX, const double filteredPositionY,
                                           const std::pair<double, double> calculatedPosition, const double timeMs)
{
    //xAngleVelocityPoints.push_back(wxRealPoint(xAngleVelNewPoint, xAngleVel));
    //yAngleVelocityPoints.push_back(wxRealPoint(xAngleVelNewPoint, yAngleVel));
    //zAngleVelocityPoints.push_back(wxRealPoint(xAngleVelNewPoint, zAngleVel));
    //xAngleVelNewPoint += 1;

    currentFilteredXPosition += filteredPositionX;
    currentFilteredYPosition += filteredPositionY;

    //filteredPositionPoints.push_back(wxRealPoint(currentFilteredXPosition, currentFilteredYPosition));
    filteredPositionBuffer.AddElement(wxRealPoint(currentFilteredXPosition, currentFilteredYPosition));
    calculatedPositionBuffer.AddElement(wxRealPoint(calculatedPosition.first, calculatedPosition.second));


    //updateMatQGrid();

    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    //dataset->AddSerie(new XYSerie(rawPositionBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(rawPositionBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(filteredPositionBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(calculatedPositionBuffer.getBuffer()));

    //dataset->AddSerie(new XYSerie(yAngleVelocityPoints));
    //dataset->AddSerie(new XYSerie(zAngleVelocityPoints));
    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("Filtered Y position [m]"));
    bottomAxis->SetTitle(wxT("Filtered X position [m]"));
    //if (rawPositionBuffer.getBuffer()[0].x < filteredPositionBuffer.getBuffer()[0].x && rawPositionBuffer.getBuffer().size() >= 100)
    //{
    //    bottomAxis->SetFixedBounds(rawPositionBuffer.getBuffer()[0].x, rawPositionBuffer.getBuffer()[99].x);
    //}
    //if(calculatedPositionBuffer.getBuffer().size() >= 100)
    //{
    //    bottomAxis->SetFixedBounds(calculatedPositionBuffer.getBuffer()[0].x, calculatedPositionBuffer.getBuffer()[99].x);
    //}
    

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

void MyWindow::updateFilteredAngleXVelocityChart(const double filteredXangle, const double measuredXangle, const double time)
{
    currentXangleFiltered += filteredXangle;
    currentXangleMeasured += measuredXangle;
    //angleTimeMeasurementsMs += static_cast<double>(time);

    //filteredXangleVelocity.push_back(wxRealPoint(time, filteredXangle));
    //measuredXangleVelocity.push_back(wxRealPoint(time, measuredXangle));

    filteredXAngleVelocityBuffer.AddElement(wxRealPoint(time, filteredXangle));

    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(xAngleVelocityBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(filteredXAngleVelocityBuffer.getBuffer()));

    //dataset->AddSerie(new XYSerie(yAngleVelocityPoints));
    //dataset->AddSerie(new XYSerie(zAngleVelocityPoints));
    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("X angle velocity [degree/s]"));
    bottomAxis->SetTitle(wxT("Time [ms]"));
    if (filteredXAngleVelocityBuffer.getBuffer().size() >= 100)
    {
        bottomAxis->SetFixedBounds(filteredXAngleVelocityBuffer.getBuffer()[0].x, filteredXAngleVelocityBuffer.getBuffer()[99].x);
    }
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

void MyWindow::updateFilteredVelocityChart(const double filteredVelocityX, const double filteredVelocityY, const double timeMs)
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
    wxBoxSizer* controlPanelSizerForXAdj = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* controlPanelSizerForYAdj = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* controlPanelSizerForZAdj = new wxBoxSizer(wxHORIZONTAL);

    spinCtrlXacc = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -17000, 17000, 530);
    spinCtrlXaccMultiplicator = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -1000, 1000, 10);
    wxStaticText* xAccText = new wxStaticText(controlPanel, wxID_ANY, "Adjust X acc");
    spinCtrlXacc->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinXAccUpdate, this);
    spinCtrlXaccMultiplicator->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinXAccIncrUpdate, this);

    spinCtrlYacc = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -17000, 17000, 530);
    spinCtrlYaccMultiplicator = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -1000, 1000, 10);
    wxStaticText* yAccText = new wxStaticText(controlPanel, wxID_ANY, "Adjust Y acc");
    spinCtrlYacc->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinYAccUpdate, this);
    spinCtrlYaccMultiplicator->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinYAccIncrUpdate, this);

    spinCtrlZacc = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -17000, 17000, 16100);
    spinCtrlZaccMultiplicator = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -1000, 1000, 10);
    wxStaticText* zAccText = new wxStaticText(controlPanel, wxID_ANY, "Adjust Z acc");
    spinCtrlZacc->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinZAccUpdate, this);
    spinCtrlZaccMultiplicator->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinZAccIncrUpdate, this);

    controlPanelSizer->Add(xAccText, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForXAdj->Add(spinCtrlXacc, 0, wxALL| wxALIGN_CENTER, 5);
    controlPanelSizerForXAdj->Add(spinCtrlXaccMultiplicator, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->Add(controlPanelSizerForXAdj, 0, wxALL | wxALIGN_CENTER, 5);

    controlPanelSizer->Add(yAccText, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForYAdj->Add(spinCtrlYacc, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForYAdj->Add(spinCtrlYaccMultiplicator, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->Add(controlPanelSizerForYAdj, 0, wxALL | wxALIGN_CENTER, 5);

    controlPanelSizer->Add(zAccText, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForZAdj->Add(spinCtrlZacc, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForZAdj->Add(spinCtrlZaccMultiplicator, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->Add(controlPanelSizerForZAdj, 0, wxALL | wxALIGN_CENTER, 5);


    wxBoxSizer* accSetupButtonsSizer = new wxBoxSizer(wxHORIZONTAL);

    wxButton* resetButton = new wxButton(controlPanel, wxID_ANY, "Reset chart");
    accSetupButtonsSizer->Add(resetButton, 0, wxALL|wxALIGN_LEFT, 5);
    resetButton->Bind(wxEVT_BUTTON, &MyWindow::OnResetAccChart, this);

    wxButton* submitButton = new wxButton(controlPanel, wxID_ANY, "Submit adjustments");
    accSetupButtonsSizer->Add(submitButton, 0, wxALL | wxALIGN_LEFT, 5);
    submitButton->Bind(wxEVT_BUTTON, &MyWindow::OnSubmitAccAdjustments, this);


    wxBoxSizer* xAccLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* yAccLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* zAccLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* deltaTimeLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* totalTimeLabelsSizer = new wxBoxSizer(wxHORIZONTAL);

    wxStaticText* xAccName = new wxStaticText(controlPanel, wxID_ANY, "Current X acc[ms/s^2]: ");
    wxStaticText* yAccName = new wxStaticText(controlPanel, wxID_ANY, "Current Y acc[ms/s^2]: ");
    wxStaticText* zAccName = new wxStaticText(controlPanel, wxID_ANY, "Current Z acc[ms/s^2]: ");
    xAccValue = new wxStaticText(controlPanel, wxID_ANY, "0");
    yAccValue = new wxStaticText(controlPanel, wxID_ANY, "0");
    zAccValue = new wxStaticText(controlPanel, wxID_ANY, "0");

    wxStaticText* deltaTimeName = new wxStaticText(controlPanel, wxID_ANY, "Delta time[ms]: ");
    wxStaticText* totalTimeName = new wxStaticText(controlPanel, wxID_ANY, "Total time[ms]: ");
    deltaTimeValue = new wxStaticText(controlPanel, wxID_ANY, "0");
    totalTimeValue = new wxStaticText(controlPanel, wxID_ANY, "0");

    xAccLabelsSizer->Add(xAccName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    xAccLabelsSizer->Add(xAccValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(xAccLabelsSizer, 0, wxALL, 5);

    yAccLabelsSizer->Add(yAccName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    yAccLabelsSizer->Add(yAccValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(yAccLabelsSizer, 0, wxALL, 5);

    zAccLabelsSizer->Add(zAccName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    zAccLabelsSizer->Add(zAccValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(zAccLabelsSizer, 0, wxALL, 5);

    deltaTimeLabelsSizer->Add(deltaTimeName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    deltaTimeLabelsSizer->Add(deltaTimeValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(deltaTimeLabelsSizer, 0, wxALL, 5);

    totalTimeLabelsSizer->Add(totalTimeName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    totalTimeLabelsSizer->Add(totalTimeValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(totalTimeLabelsSizer, 0, wxALL, 5);

    controlPanelSizer->Add(accSetupButtonsSizer, 0, wxALL, 5);

    controlPanel->SetSizer(controlPanelSizer);


    accPanelSplitter->SplitVertically(accChartPanel, controlPanel);
    sizerAccPlot->Add(accPanelSplitter, 1, wxEXPAND | wxALL, 5);


    //CreateStatusBar();
    //SetStatusText("wxWidgets Splitter Example");
    panel->SetSizer(sizerAccPlot);
   // m_notebook->AddPage(panel, "Filtered position");

    m_notebook->AddPage(panel, "Acc chart");
}

void MyWindow::prepareAngleVelocityChart()
{
    wxPanel* panel = new wxPanel(m_notebook, wxID_ANY);
    angleVelPanelSplitter = new wxSplitterWindow(panel, wxID_ANY);
    wxPanel* controlPanel = new wxPanel(angleVelPanelSplitter, wxID_ANY);

    angleVelocityChartPanel = new wxChartPanel(angleVelPanelSplitter);

    sizerAngleVelPlot = new wxBoxSizer(wxVERTICAL);
    angleVelocityChartPanel->SetMinSize(wxSize(600, 600));

    wxBoxSizer* controlPanelSizer = new wxBoxSizer(wxVERTICAL);
    wxBoxSizer* controlPanelSizerForXSpins = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* controlPanelSizerForYSpins = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* controlPanelSizerForZSpins = new wxBoxSizer(wxHORIZONTAL);

    wxStaticText* callibrationMultiplicatorLabel = new wxStaticText(controlPanel, wxID_ANY, "Set callibration multiplicator");

    spinCtrlXangleVel = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -33000, 33000, -18600);
    spinCtrlXangleVelMultiplicator = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -1000, 1000, 10);
    wxStaticText* xAngleVelText = new wxStaticText(controlPanel, wxID_ANY, "Adjust X angle velocity");
    spinCtrlXangleVel->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinXAngleVelUpdate, this);
    spinCtrlXangleVelMultiplicator->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinXAnglVelIncrUpdate, this);

    spinCtrlYangleVel = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -33000, 33000, 13450);
    spinCtrlYangleVelMultiplicator = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -1000, 1000, 10);
    wxStaticText* yAngleVelText = new wxStaticText(controlPanel, wxID_ANY, "Adjust Y angle velocity");
    spinCtrlYangleVel->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinYAngleVelUpdate, this);
    spinCtrlYangleVelMultiplicator->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinYAnglVelIncrUpdate, this);

    spinCtrlZangleVel = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -33000, 33000, 16500);
    spinCtrlZangleVelMultiplicator = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -1000, 1000, 10);
    wxStaticText* zAngleVelText = new wxStaticText(controlPanel, wxID_ANY, "Adjust Z angle velocity");
    spinCtrlZangleVel->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinZAngleVelUpdate, this);
    spinCtrlZangleVelMultiplicator->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinZAnglVelIncrUpdate, this);

    controlPanelSizer->Add(xAngleVelText, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForXSpins->Add(spinCtrlXangleVel, 0, wxALL | wxALIGN_CENTER, 5);
    //controlPanelSizerForXSpins->Add(spinCtrlXangleVel, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForXSpins->Add(spinCtrlXangleVelMultiplicator, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->Add(controlPanelSizerForXSpins, 0, wxALL | wxALIGN_CENTER, 5);


    controlPanelSizer->Add(yAngleVelText, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForYSpins->Add(spinCtrlYangleVel, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForYSpins->Add(spinCtrlYangleVelMultiplicator, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->Add(controlPanelSizerForYSpins, 0, wxALL | wxALIGN_CENTER, 5);

    controlPanelSizer->Add(zAngleVelText, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForZSpins->Add(spinCtrlZangleVel, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForZSpins->Add(spinCtrlZangleVelMultiplicator, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->Add(controlPanelSizerForZSpins, 0, wxALL | wxALIGN_CENTER, 5);

    wxBoxSizer* angleVelSetupButtonsSizer = new wxBoxSizer(wxHORIZONTAL);

    wxButton* resetButton = new wxButton(controlPanel, wxID_ANY, "Reset chart");
    angleVelSetupButtonsSizer->Add(resetButton, 0, wxALL | wxALIGN_LEFT, 5);
    resetButton->Bind(wxEVT_BUTTON, &MyWindow::OnResetAngleVelChart, this);

    wxButton* submitButton = new wxButton(controlPanel, wxID_ANY, "Submit adjustments");
    angleVelSetupButtonsSizer->Add(submitButton, 0, wxALL | wxALIGN_LEFT, 5);
    submitButton->Bind(wxEVT_BUTTON, &MyWindow::OnSubmitAngleVelAdjustments, this);

    wxBoxSizer* xAngleVelLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* yAngleVelLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* zAngleVelLabelsSizer = new wxBoxSizer(wxHORIZONTAL);

    wxStaticText* xAngleVelName = new wxStaticText(controlPanel, wxID_ANY, "Current X angle vel [degree/s]: ");
    wxStaticText* yAngleVelName = new wxStaticText(controlPanel, wxID_ANY, "Current Y angle vel [degree/s]: ");
    wxStaticText* zAngleVelName = new wxStaticText(controlPanel, wxID_ANY, "Current Z angle vel [degree/s]: ");
    xAngleVelValue = new wxStaticText(controlPanel, wxID_ANY, "0");
    yAngleVelValue = new wxStaticText(controlPanel, wxID_ANY, "0");
    zAngleVelValue = new wxStaticText(controlPanel, wxID_ANY, "0");

    xAngleVelLabelsSizer->Add(xAngleVelName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    xAngleVelLabelsSizer->Add(xAngleVelValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(xAngleVelLabelsSizer, 0, wxALL, 5);

    yAngleVelLabelsSizer->Add(yAngleVelName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    yAngleVelLabelsSizer->Add(yAngleVelValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(yAngleVelLabelsSizer, 0, wxALL, 5);

    zAngleVelLabelsSizer->Add(zAngleVelName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    zAngleVelLabelsSizer->Add(zAngleVelValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(zAngleVelLabelsSizer, 0, wxALL, 5);

    controlPanelSizer->Add(angleVelSetupButtonsSizer, 0, wxALL, 5);

    controlPanel->SetSizer(controlPanelSizer);

    angleVelPanelSplitter->SplitVertically(angleVelocityChartPanel, controlPanel);
    sizerAngleVelPlot->Add(angleVelPanelSplitter, 1, wxEXPAND | wxALL, 5);
    panel->SetSizer(sizerAngleVelPlot);

    m_notebook->AddPage(panel, "Angle velocity");
}


void MyWindow::prepareAzimuthChart()
{
    plot = new XYPlot();
    dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(magnPoints));
    dataset->SetRenderer(new XYLineRenderer());
    leftAxis = new NumberAxis(AXIS_LEFT);
    bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("X"));
    bottomAxis->SetTitle(wxT("Y"));
    plot->AddObjects(dataset, leftAxis, bottomAxis);

    azimuthChart = new Chart(plot, "DATA SET");

    wxPanel* panel = new wxPanel(m_notebook, wxID_ANY);
    azimuthPanelSplitter = new wxSplitterWindow(panel, wxID_ANY);
    wxPanel* controlPanel = new wxPanel(azimuthPanelSplitter, wxID_ANY);

    azimuthChartPanel = new wxChartPanel(azimuthPanelSplitter);

    sizerAzimuthPlot = new wxBoxSizer(wxVERTICAL);
    azimuthChartPanel->SetMinSize(wxSize(600, 600));

    wxBoxSizer* controlPanelSizer = new wxBoxSizer(wxVERTICAL);

    spinCtrlXmagn = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -33000, 33000, 1);
    wxStaticText* xMagnText = new wxStaticText(controlPanel, wxID_ANY, "Adjust X magn");

    spinCtrlYmagn = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -33000, 33000, 2);
    wxStaticText* yMagnText = new wxStaticText(controlPanel, wxID_ANY, "Adjust Y magn");

    controlPanelSizer->Add(xMagnText, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->Add(spinCtrlXmagn, 0, wxALL | wxALIGN_CENTER, 5);

    controlPanelSizer->Add(yMagnText, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->Add(spinCtrlYmagn, 0, wxALL | wxALIGN_CENTER, 5);

    wxBoxSizer* azimuthSetupButtonsSizer = new wxBoxSizer(wxHORIZONTAL);

    wxButton* resetButton = new wxButton(controlPanel, wxID_ANY, "Reset chart");
    azimuthSetupButtonsSizer->Add(resetButton, 0, wxALL | wxALIGN_LEFT, 5);
    resetButton->Bind(wxEVT_BUTTON, &MyWindow::OnResetMagnChart, this);

    wxButton* submitButton = new wxButton(controlPanel, wxID_ANY, "Submit adjustments");
    azimuthSetupButtonsSizer->Add(submitButton, 0, wxALL | wxALIGN_LEFT, 5);
    submitButton->Bind(wxEVT_BUTTON, &MyWindow::OnSubmitMagnAdjustments, this);

    wxBoxSizer* xMagnLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* yMagnLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* orientationSizer = new wxBoxSizer(wxHORIZONTAL);

    wxStaticText* xMagnName = new wxStaticText(controlPanel, wxID_ANY, "Current X magn: ");
    wxStaticText* yMagnName = new wxStaticText(controlPanel, wxID_ANY, "Current Y magn: ");
    wxStaticText* orientationName = new wxStaticText(controlPanel, wxID_ANY, "Orientation [deg]: ");
    xMagnValue = new wxStaticText(controlPanel, wxID_ANY, "0");
    yMagnValue = new wxStaticText(controlPanel, wxID_ANY, "0");
    orientationValue = new wxStaticText(controlPanel, wxID_ANY, "0");

    xMagnLabelsSizer->Add(xMagnName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    xMagnLabelsSizer->Add(xMagnValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(xMagnLabelsSizer, 0, wxALL, 5);

    yMagnLabelsSizer->Add(yMagnName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    yMagnLabelsSizer->Add(yMagnValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(yMagnLabelsSizer, 0, wxALL, 5);

    orientationSizer->Add(orientationName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    orientationSizer->Add(orientationValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(orientationSizer, 0, wxALL, 5);

    controlPanelSizer->Add(azimuthSetupButtonsSizer, 0, wxALL, 5);

    controlPanel->SetSizer(controlPanelSizer);

    azimuthPanelSplitter->SplitVertically(azimuthChartPanel, controlPanel);
    sizerAzimuthPlot->Add(azimuthPanelSplitter, 1, wxEXPAND | wxALL, 5);
    panel->SetSizer(sizerAzimuthPlot);

    m_notebook->AddPage(panel, "Azimuth");

    //azimuthChartPanel->SetChart(azimuthChart);
    ////m_notebook->AddPage(new OuterTab(m_notebook, "Outer Tab 2"), "Outer Tab 2");
    //m_notebook->AddPage(azimuthChartPanel, "Chart");
}


void MyWindow::preparePositionChart()
{
    positionChartPanel = new wxChartPanel(m_notebook);
    m_notebook->AddPage(positionChartPanel, "Pos chart");
}

void MyWindow::prepareGpsBasedPositionChart()
{
    wxPanel* panel = new wxPanel(m_notebook, wxID_ANY);
    gpsBasedPositionPanelSplitter = new wxSplitterWindow(panel, wxID_ANY);
    wxPanel* controlPanel = new wxPanel(gpsBasedPositionPanelSplitter, wxID_ANY);

    gpsBasedPositionChartPanel = new wxChartPanel(gpsBasedPositionPanelSplitter);

    wxBoxSizer* sizerGpsBasedPositionPlot = new wxBoxSizer(wxVERTICAL);
    gpsBasedPositionChartPanel->SetMinSize(wxSize(600, 600));

    wxBoxSizer* controlPanelSizer = new wxBoxSizer(wxVERTICAL);

    gpsBasedPositionPanelSplitter->SplitVertically(gpsBasedPositionChartPanel, controlPanel);
    sizerGpsBasedPositionPlot->Add(gpsBasedPositionPanelSplitter, 1, wxEXPAND | wxALL, 5);

    panel->SetSizer(sizerGpsBasedPositionPlot);

    m_notebook->AddPage(panel, "GPS position");

}



void MyWindow::prepareVelChart()
{
    //velChartPanel = new wxChartPanel(m_notebook);
    //m_notebook->AddPage(velChartPanel, "Vel chart");
}

void MyWindow::prepareFilteredVelocityChart()
{
    //filteredVelocityChartPanel = new wxChartPanel(m_notebook);
    //m_notebook->AddPage(filteredVelocityChartPanel, "Filtered velocity");
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

    matR << 0.1F, 0.0F,
            0.0F, 0.1F;

    matH << 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F;

    // Create two panels to be placed in the splitter window
    //wxPanel* panel1 = new wxPanel(splitter, wxID_ANY);
    wxPanel* controlPanel = new wxPanel(splitter, wxID_ANY);

    filteredPositionChartPanel = new wxChartPanel(splitter);
    sizerPositionPlot = new wxBoxSizer(wxVERTICAL);
    filteredPositionChartPanel->SetMinSize(wxSize(600, 600));
    //filteredVelocityChartPanel->SetSize(200, 200);
    //sizer->Add(filteredPositionChartPanel, 1, wxEXPAND | wxALL, 5);
    wxButton* applyKFtunningChangesButton = new wxButton(controlPanel, wxID_ANY, "Apply changes!");
    //sizerPositionPlot->Add(button, 0, wxALIGN_RIGHT | wxALL, 5);
    wxBoxSizer* controlPanelSizer = new wxBoxSizer(wxVERTICAL);
    wxSpinCtrl* spinCtrlPCoefficient = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -100, 100, 0);
    wxStaticText* pCoefficientText = new wxStaticText(controlPanel, wxID_ANY, "Spin Control for p coefficient:");

    wxSpinCtrl* spinCtrlSCoefficient = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -100, 100, 6);
    wxStaticText* sCoefficientText = new wxStaticText(controlPanel, wxID_ANY, "Spin Control for s coefficient:");


    wxStaticText* qMatrixText = new wxStaticText(controlPanel, wxID_ANY, "Matrix Q");
    wxStaticText* rMatrixText = new wxStaticText(controlPanel, wxID_ANY, "Matrix R");
    wxStaticText* hMatrixText = new wxStaticText(controlPanel, wxID_ANY, "Matrix H");

    matrixGrid = new wxGrid(controlPanel, wxID_ANY);
    matrixGrid->CreateGrid(DIM_X, DIM_X);

    matrixGrid->HideRowLabels();
    matrixGrid->HideColLabels();
    matrixGrid->SetSize(100, 100);


    updateMatQGrid();


    for (int i = 0; i < DIM_X; ++i) {
        matrixGrid->SetRowSize(i, 20);  // Set the height of each row
        matrixGrid->SetColSize(i, 50);  // Set the width of each column
    }

    matrixRCovariance = new wxGrid(controlPanel, wxID_ANY);
    matrixRCovariance->CreateGrid(DIM_Z, DIM_Z);

    matrixRCovariance->HideRowLabels();
    matrixRCovariance->HideColLabels();
    matrixGrid->SetSize(100, 100);

    for (int i = 0; i < DIM_Z; ++i) {
        for (int j = 0; j < DIM_Z; ++j) {
            matrixRCovariance->SetCellValue(i, j, wxString::Format("%f", matR(i*DIM_Z+j)));
        }
    }

    for (int i = 0; i < DIM_Z; ++i) {
        matrixRCovariance->SetRowSize(i, 20);  // Set the height of each row
        matrixRCovariance->SetColSize(i, 50);  // Set the width of each column
    }



    matrixH = new wxGrid(controlPanel, wxID_ANY);
    matrixH->CreateGrid(DIM_Z, DIM_X);

    matrixH->HideRowLabels();
    matrixH->HideColLabels();
    matrixH->SetSize(100, 100);

    for (int i = 0; i < DIM_Z; ++i) {
        for (int j = 0; j < DIM_X; ++j) {
            matrixH->SetCellValue(i, j, wxString::Format("%f", matH(j * DIM_Z + i)));
        }
    }

    for (int i = 0; i < DIM_Z; ++i) {
        matrixH->SetRowSize(i, 20);  // Set the height of each row
        matrixH->SetColSize(i, 50);  // Set the width of each column
    }



    applyKFtunningChangesButton->Bind(wxEVT_BUTTON, &MyWindow::OnApplyKFTunning, this);

    controlPanelSizer->Add(pCoefficientText, 0, wxALL, 5);
    controlPanelSizer->Add(spinCtrlPCoefficient, 0, wxALL | wxALIGN_RIGHT, 5);
    controlPanelSizer->Add(sCoefficientText, 0, wxALL, 5);
    controlPanelSizer->Add(spinCtrlSCoefficient, 0, wxALL | wxALIGN_RIGHT, 5);
    controlPanelSizer->Add(qMatrixText, 0, wxALIGN_CENTER|wxALL, 5);
    controlPanelSizer->Add(matrixGrid, 0, wxALIGN_CENTER | wxALL, 5);
    controlPanelSizer->Add(rMatrixText, 0, wxALIGN_CENTER|wxALL, 5);
    controlPanelSizer->Add(matrixRCovariance, 0, wxALIGN_CENTER | wxALL, 5);
    controlPanelSizer->Add(hMatrixText, 0, wxALIGN_CENTER | wxALL, 5);
    controlPanelSizer->Add(matrixH, 0, wxALIGN_CENTER | wxALL, 5);
    controlPanelSizer->Add(applyKFtunningChangesButton, 0, wxALIGN_CENTER|wxALL, 3);
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
    
    kalmanFilterSetupGui.setup(kalmanParamsSetupPanel);

    csvMeasurementLoadPanel = new wxPanel(m_notebook);
    m_notebook->AddPage(csvMeasurementLoadPanel, "Load CSV");
    csvMeasurementLoadGui.setup(csvMeasurementLoadPanel);
    
    

    //splitter = new wxSplitterWindow(, wxID_ANY);

    prepareAccChart();
    prepareVelChart();
    preparePositionChart();
    prepareGpsBasedPositionChart();
    prepareAngleVelocityChart();
    prepareFilteredPositionChart();
    prepareFilteredVelocityChart();
    prepareFilteredAngleXVelocityChart();
    prepareAzimuthChart();

    // Create a notebook for outer tabs
    //wxNotebook* outerNotebook = new wxNotebook(this, wxID_ANY);

    // Add outer tabs to the notebook
    m_notebook->AddPage(new OuterTab(m_notebook, "Outer Tab 1"), "Outer Tab 1");
    m_notebook->AddPage(new OuterTab(m_notebook, "Outer Tab 2"), "Outer Tab 2");
    //m_notebook->AddPage(new OuterTab(m_notebook, "Outer Tab 2"), "Outer Tab 2");


    //OuterNotebook* outerNotebook = new OuterNotebook(this, 2);
    //m_notebook->AddPage(outerNotebook, "CHARTS");

    //magnPoints.push_back(wxRealPoint(3.2, 23.2));
    //magnPoints.push_back(wxRealPoint(4.2, 23.2));
    //magnPoints.push_back(wxRealPoint(6.2, 28.2));
    //magnPoints.push_back(wxRealPoint(9.2, 35.2));


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

void MyWindow::updateMatQGrid()
{
    for (int i = 0; i < DIM_X; ++i) {
        for (int j = 0; j < DIM_X; ++j) {
            matrixGrid->SetCellValue(i, j, wxString::Format("%f", matQ(i * DIM_X + j)));
        }
    }
}
