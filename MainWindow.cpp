#include "MainWindow.h"


SensorDataReceptionThread::SensorDataReceptionThread(AppLogger& appLogger, wxEvtHandler* parent) : wxThread(), appLogger(appLogger), m_parent(parent)
{
    //this->appLogger = appLogger;
    m_count = 0;
}

SensorDataReceptionThread::~SensorDataReceptionThread()
{

}

wxThread::ExitCode SensorDataReceptionThread::Entry()
{
    boost::asio::io_context io;
    //std::string com = "COM7";

    //wxThreadEvent* event = new wxThreadEvent(wxEVT_MY_THREAD_EVENT);
    //event->SetString("Data from thread");
    //wxQueueEvent(m_parent, event);

    //SerialComm serialComm(io, com, m_parent);
    Server server(io, 8081, appLogger, m_parent);
    io.run();

    const std::string threadFinished{ "SensorDataReceptionThread Thread finished successfully.\n" };
    appLogger.logSerialCommStartThread(threadFinished);

    return NULL;
}

////////////////////
GpsDataReceptionThread::GpsDataReceptionThread(AppLogger& appLogger, wxEvtHandler* parent) : wxThread(), appLogger(appLogger), m_parent(parent)
{
    //this->appLogger = appLogger;
    m_count = 0;
}

GpsDataReceptionThread::~GpsDataReceptionThread()
{

}

wxThread::ExitCode GpsDataReceptionThread::Entry()
{
    boost::asio::io_context io;
    std::string com = "COM12";

    //wxThreadEvent* event = new wxThreadEvent(wxEVT_MY_THREAD_EVENT);
    //event->SetString("Data from thread");
    //wxQueueEvent(m_parent, event);

    //SerialComm serialComm(io, com, m_parent);
    //Server server(io, 8081, appLogger, m_parent);
    io.run();

    const std::string threadFinished{ "GpsDataReceptionThread Thread finished successfully.\n" };
    appLogger.logSerialCommStartThread(threadFinished);

    return NULL;
}

////////////////////////


MyWindow::MyWindow(const wxString& title)
    : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(1080, 600))
{
    Centre();
    prepareGui();

    if (isDataReceptionStarted)
    {
        //createDataReceptionThread();
    }

    createSensorDataReceptionThread();
    createGpsDataReceptionThread();
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
    //filterFileMeasTimer.Start(100);
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


//void MyWindow::OnSubmitAngleVelAdjustments(wxCommandEvent& event)
//{
//    const int xAngleVelCtrlValue = spinCtrlXangleVel->GetValue();
//    const int yAngleVelCtrlValue = spinCtrlYangleVel->GetValue();
//    const int zAngleVelCtrlValue = spinCtrlZangleVel->GetValue();
//
//    xGyroBias = xAngleVelCtrlValue;
//    yGyroBias = yAngleVelCtrlValue;
//    zGyroBias = zAngleVelCtrlValue;
//    wxLogMessage("New acc adj X:%d Y:%d Z:%d!", xAngleVelCtrlValue, yAngleVelCtrlValue, zAngleVelCtrlValue);
//}
//
//void MyWindow::OnSpinXAnglVelIncrUpdate(wxSpinEvent& event)
//{
//    const int newIncrement = spinCtrlXangleVelMultiplicator->GetValue();
//    spinCtrlXangleVel->SetIncrement(newIncrement);
//}
//
//void MyWindow::OnSpinXAngleVelUpdate(wxSpinEvent& event)
//{
//    const int xAngleVelCtrlValue = spinCtrlXangleVel->GetValue();
//    xGyroBias = xAngleVelCtrlValue;
//}
//
//void MyWindow::OnSpinYAnglVelIncrUpdate(wxSpinEvent& event)
//{
//    const int newIncrement = spinCtrlYangleVelMultiplicator->GetValue();
//    spinCtrlYangleVel->SetIncrement(newIncrement);
//}
//
//void MyWindow::OnSpinYAngleVelUpdate(wxSpinEvent& event)
//{
//    const int yAngleVelCtrlValue = spinCtrlYangleVel->GetValue();
//    yGyroBias = yAngleVelCtrlValue;
//}
//
//void MyWindow::OnSpinZAnglVelIncrUpdate(wxSpinEvent& event)
//{
//    const int newIncrement = spinCtrlZangleVelMultiplicator->GetValue();
//    spinCtrlZangleVel->SetIncrement(newIncrement);
//}
//
//void MyWindow::OnSpinZAngleVelUpdate(wxSpinEvent& event)
//{
//    const int zAngleVelCtrlValue = spinCtrlZangleVel->GetValue();
//    zGyroBias = zAngleVelCtrlValue;
//}

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

/// <summary>
void MyWindow::processFiltration(const std::vector<std::string>& measurements, const bool isRealTimeMeasurement)
{
    if (measurements.size() == 10)
    {
        if (isFirstMeasurement)
        {
            deltaTimeCalculator.startTimer();
            isFirstMeasurement = false;
            return;
        }
        const uint32_t deltaTimeMs = deltaTimeCalculator.getDurationInMs();
        MeasurementsController rawMeasurement(appLogger, rawGrawity, xBias, yBias,
            angleVelocityChartGui.getXgyroBias(),
            angleVelocityChartGui.getYgyroBias(),
            angleVelocityChartGui.getZgyroBias());
        totalTimeMs += static_cast<double>(deltaTimeMs);
        if (rawMeasurement.assign(measurements, deltaTimeMs, isRealTimeMeasurement))
        {

            //const uint32_t totalTimeMs = deltaTimeCalculator.getTotalTimeMs();

            //const double distance = haversineConverter.calculateDistance(rawMeasurement.getLongitude(), 70.2, rawMeasurement.getLatitude(), 30.2);
            //const auto xyPoint = haversineConverter.convertToXY_({70.234, 30.234});
            ////rawMeasurement.setDeltaTimeMs(deltaTimeMs);
            ////rawMeasurementsSet.push_back(rawMeasurement);
            if (kalmanFilterSetupGui.getIsCallibrationDone() == false)
            {
                magnChartGui.updateChart(magnPointsBuffer, filteredAzimuthBuffer, rollBuffer, pitchBuffer,
                    rawMeasurement.getRawXMagn(), rawMeasurement.getRawYMagn(), rawMeasurement.getAzimuth(), 1.0, totalTimeMs);
                //updateMagnChart(rawMeasurement.getRawXMagn(), rawMeasurement.getRawYMagn(), rawMeasurement.getAzimuth(), 1.0, totalTimeMs);
                updateAccChart(rawMeasurement.getXaccMPerS2(),
                    rawMeasurement.getYaccMPerS2(),
                    rawMeasurement.getZaccMPerS2(),
                    rawMeasurement.getCompensatedAccData(),
                    1.0, 1.0, 0.01, 0.01,
                    totalTimeMs, deltaTimeMs);
                ////updateVelChart(rawMeasurement.getXvelocityMperS());
                ////updatePositionChart(rawMeasurement.getXDistance(), rawMeasurement.getYDistance(), totalTimeMs);
                angleVelocityChartGui.updateChart(xAngleVelocityBuffer, yAngleVelocityBuffer, zAngleVelocityBuffer, filteredZangleVelocityBuffer,
                    rawMeasurement.getXangleVelocityDegreePerS(),
                    rawMeasurement.getYangleVelocityDegreePerS(),
                    rawMeasurement.getZangleVelocityDegreePerS(), 1.0, totalTimeMs);

                const double lon{ gpsDataConverter.getLongitude() };
                const double lat{ gpsDataConverter.getLatitude() };
                const auto gpsBasedPosition = haversineConverter.calculateCurrentPosition(lon, lat);
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

                //if (kalmanFilterSetupGui.getIsFiltrationRestarted())
                //{
                //    kalmanFilterSetupGui.setIsFiltrationRestarted(false);
                //    //isFirstMeasurement = true;
                //    resetChartsAfterCallibration();
                //    totalTimeMs = 0;

                //    kalmanFilter.vecX().setZero();
                //    kalmanFilter.matP().setZero();
                //    kalmanFilterAzimuth.vecX().setZero();
                //    kalmanFilterAzimuth.matP().setZero();
                //    kalmanFilterGyro.vecX().setZero();
                //    kalmanFilterGyro.matP().setZero();
                //    
                //}

                kalmanFilterAzimuth.setInitialStateForAzimuth(rawMeasurement.getAzimuth());
                experimentKfAzimuth(rawMeasurement.getXangleVelocityDegreePerS(), rawMeasurement.getYangleVelocityDegreePerS(),
                    rawMeasurement.getZangleVelocityDegreePerS(), rawMeasurement.getAzimuth(), deltaTimeMs);
                positionChartGui.updateChart(rawPositionBuffer, rawMeasurement.getXDistance(), rawMeasurement.getYDistance(), totalTimeMs);
                //roll (X)
                const double filteredAzimuth = kalmanFilterAzimuth.vecX()[0];
                //roll += rawMeasurement.getXangleVelocityDegreePerS() * (deltaTimeMs / 1000.0);
                //pitch += rawMeasurement.getYangleVelocityDegreePerS() * (deltaTimeMs / 1000.0);

                experimentGyroKf(rawMeasurement.getXangleVelocityDegreePerS(), rawMeasurement.getYangleVelocityDegreePerS(), rawMeasurement.getZangleVelocityDegreePerS()
                    , deltaTimeMs);

                roll += kalmanFilterGyro.vecX()[1];
                pitch += kalmanFilterGyro.vecX()[3];

                const double filteredXAngleVel = kalmanFilterAzimuth.vecX()[3];
                const double filteredYAngleVel = kalmanFilterAzimuth.vecX()[4];
                const double filteredZAngleVel = kalmanFilterAzimuth.vecX()[5];

                rollPitchChartGui.updateChart(rollBuffer, pitchBuffer, roll, pitch, totalTimeMs);

                const double xAccGyroCompensation = (rawMeasurement.getXaccMPerS2() * cos(roll)) - (rawMeasurement.getZaccMPerS2() * sin(pitch));
                const double yAccGyroCompensation = (rawMeasurement.getYaccMPerS2() * cos(pitch)) + (rawMeasurement.getZaccMPerS2() * sin(roll));
                


                magnChartGui.updateChart(magnPointsBuffer, filteredAzimuthBuffer, rollBuffer, pitchBuffer,
                    rawMeasurement.getRawXMagn(), rawMeasurement.getRawYMagn(), rawMeasurement.getAzimuth(), filteredAzimuth, totalTimeMs);
                //updateMagnChart(rawMeasurement.getRawXMagn(), rawMeasurement.getRawYMagn(), rawMeasurement.getAzimuth(), filteredAzimuth, totalTimeMs);

                angleVelocityChartGui.updateChart(xAngleVelocityBuffer, yAngleVelocityBuffer, zAngleVelocityBuffer, filteredZangleVelocityBuffer,
                    rawMeasurement.getXangleVelocityDegreePerS(),
                    rawMeasurement.getYangleVelocityDegreePerS(),
                    rawMeasurement.getZangleVelocityDegreePerS(), filteredZAngleVel, totalTimeMs);
                //roll pitch

                
                kalmanFilter.setInitialState(rawMeasurement.getXDistance(), rawMeasurement.getXvelocityMperS(), rawMeasurement.getCompensatedAccData().xAcc,
                    rawMeasurement.getYDistance(), rawMeasurement.getYvelocityMperS(), rawMeasurement.getCompensatedAccData().yAcc);

                experimentKf(rawMeasurement.getXaccMPerS2(), rawMeasurement.getYaccMPerS2(), deltaTimeMs);


                positionUpdater.updatePosition(filteredPositionX, filteredPositionY, rawMeasurement.getXDistance(), rawMeasurement.getYDistance(),
                    filteredAzimuth);

                filteredPositionX = kalmanFilter.vecX()(0);//PosX
                filteredPositionY = kalmanFilter.vecX()(3);//PosY
                const double filteredXacc = kalmanFilter.vecX()(2); //xAcc
                const double filteredYacc = kalmanFilter.vecX()(5); //yAcc
                //const double filteredVelocityX = kalmanFilter.vecX()(1);
                //const double filteredAccX = kalmanFilter.vecX()(1);
                
                //const double filteredVelocityY = kalmanFilter.vecX()(3);
                updateAccChart(rawMeasurement.getXaccMPerS2(),
                    rawMeasurement.getYaccMPerS2(),
                    rawMeasurement.getZaccMPerS2(),
                    rawMeasurement.getCompensatedAccData(),
                    filteredXacc, filteredYacc, xAccGyroCompensation, yAccGyroCompensation,
                    totalTimeMs, deltaTimeMs);

                const auto calculatedPosition{ positionUpdater.getCurrentPosition() };

                updateFilteredPositionChart(filteredPositionX, filteredPositionY, calculatedPosition, totalTimeMs);

                const double lon{ gpsDataConverter.getLongitude() };
                const double lat{ gpsDataConverter.getLatitude() };
                const auto gpsBasedPosition = haversineConverter.calculateCurrentPosition(lon, lat);
                updateGpsBasedPositionChart(gpsBasedPosition);
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
        const std::string errThreadEvent{ "ERR when handling data from thread/timer - wrong size of data - should be 10!!!" };
        appLogger.logErrThreadDataHandling(errThreadEvent);
    }
}

void MyWindow::OnFilterFileMeasTimer(wxTimerEvent& event)
{
    //po wcisnieciu przycisku "Zatwirdz kalibracje - rozpocznij filtracje" wczytywanie danych z plikow od poczatku
    //reset wykresów, czasu...
    if (kalmanFilterSetupGui.getIsCallibrationDone() == true && kalmanFilterSetupGui.getIsRestartFiltrationNeeded() == true)
    {
        csvMeasurementReader.setReadMeasurementFromBegining();
        csvMeasurementReader.setReadGpsDataFromBegining();
        kalmanFilterSetupGui.setIsRestartFiltrationNeeded(false);
    }
    const std::vector<std::string>& measurements = csvMeasurementReader.readCSVrow();
    const std::vector<std::string>& gpsData = csvMeasurementReader.readCSVrowGpsData();

    gpsDataConverter.handleGpsData(gpsData);

    processFiltration(measurements, false);
    //every 100ms
    //read new line
    //call filtration
}

void MyWindow::OnSensorsDataThreadEvent(wxThreadEvent& event) {

    MeasurementCustomizator* myEvent = dynamic_cast<MeasurementCustomizator*>(&event);
    if (myEvent)
    {
        const std::vector<std::string>& measurements = myEvent->GetStringVector();

        appLogger.logReceivedDataOnMainThread(measurements);
        processFiltration(measurements, true);
    }
    else
    {
        const std::string errThreadEvent{ "ERR when handling data from thread - no event received!!!" };
        appLogger.logErrThreadDataHandling(errThreadEvent);
    }
}

void MyWindow::OnGpsDataThreadEvent(wxThreadEvent& event)
{
    GpsDataCustomizator* myEvent = dynamic_cast<GpsDataCustomizator*>(&event);
    if (myEvent)
    {
        const std::vector<std::string>& measurements = myEvent->GetStringVector();

        gpsDataConverter.handleGpsData(measurements);


        appLogger.logReceivedDataOnMainThread(measurements, " GPS");
        appLogger.logGpsCsvData(measurements);
        //processFiltration(measurements, true);
    }
    else
    {
        const std::string errThreadEvent{ "ERR when handling data from thread - no event received!!!" };
        appLogger.logErrThreadDataHandling(errThreadEvent);
    }
}

void MyWindow::resetChartsAfterCallibration()
{
    magnPointsBuffer.Clear();
    filteredAzimuthBuffer.Clear();

    xAccBuffer.Clear();
    yAccBuffer.Clear();
    zAccBuffer.Clear();
    compensatedXAccDataBuffer.Clear();
    compensatedYAccDataBuffer.Clear();
    filteredXaccBuffer.Clear();
    filteredYaccBuffer.Clear();
    xAccWithGyroCompensation.Clear();
    yAccWithGyroCompensation.Clear();

    xAngleVelocityBuffer.Clear();
    yAngleVelocityBuffer.Clear();
    zAngleVelocityBuffer.Clear();
    filteredZangleVelocityBuffer.Clear();

    rollBuffer.Clear();
    pitchBuffer.Clear();

    rawPositionBuffer.Clear();
    filteredPositionBuffer.Clear();
    calculatedPositionBuffer.Clear();
    gpsBasedPositionBuffer.Clear();
}

void MyWindow::updateAccChart(const double xAccMPerS2, const double yAccMPerS2, const double zAccMPerS2,
    const CompensatedAccData& compensatedAccData,
    const double filteredXacc, const double filteredYacc,
    const double xAccGyroCompens, const double yAccGyroCompens, const double timeMs, const uint32_t deltaTime)
{
    xAccValue->SetLabel(std::to_string(xAccMPerS2));
    yAccValue->SetLabel(std::to_string(yAccMPerS2));
    zAccValue->SetLabel(std::to_string(zAccMPerS2));

    deltaTimeValue->SetLabel(std::to_string(deltaTime));
    totalTimeValue->SetLabel(std::to_string(timeMs));

    xAccBuffer.AddElement(wxRealPoint(timeMs, xAccMPerS2));
    yAccBuffer.AddElement(wxRealPoint(timeMs, yAccMPerS2));
    zAccBuffer.AddElement(wxRealPoint(timeMs, zAccMPerS2));


    filteredXaccBuffer.AddElement(wxRealPoint(timeMs, filteredXacc));
    filteredYaccBuffer.AddElement(wxRealPoint(timeMs, filteredYacc));

    compensatedXAccDataBuffer.AddElement(wxRealPoint(timeMs, compensatedAccData.xAcc));
    compensatedYAccDataBuffer.AddElement(wxRealPoint(timeMs, compensatedAccData.yAcc));

    xAccWithGyroCompensation.AddElement(wxRealPoint(timeMs, xAccGyroCompens));
    yAccWithGyroCompensation.AddElement(wxRealPoint(timeMs, yAccGyroCompens));

    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(xAccBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(yAccBuffer.getBuffer()));
    //dataset->AddSerie(new XYSerie(zAccBuffer.getBuffer()));
    //dataset->AddSerie(new XYSerie(filteredXaccBuffer.getBuffer()));
    //dataset->AddSerie(new XYSerie(filteredYaccBuffer.getBuffer()));

    dataset->AddSerie(new XYSerie(compensatedXAccDataBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(compensatedYAccDataBuffer.getBuffer()));

    //dataset->AddSerie(new XYSerie(xAccWithGyroCompensation.getBuffer()));
    //dataset->AddSerie(new XYSerie(yAccWithGyroCompensation.getBuffer()));

    dataset->GetSerie(0)->SetName("raw X acceleration");
    dataset->GetSerie(1)->SetName("raw Y acceleration");
    dataset->GetSerie(2)->SetName("compensated X acceleration");
    dataset->GetSerie(3)->SetName("compensated Y acceleration");

    //dataset->GetSerie(4)->SetName("gyro compensated X");
    //dataset->GetSerie(5)->SetName("gyro compensated Y");

    //dataset->GetSerie(2)->SetName("filtered X acceleration");
    //dataset->GetSerie(3)->SetName("filtered Y acceleration");

    //dataset->GetSerie(4)->SetName("compensated X acceleration");
    //dataset->GetSerie(5)->SetName("compensated Y acceleration");

    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("Acceleration [m/s2]"));
    bottomAxis->SetTitle(wxT("time [ms]"));
    if (xAccBuffer.getBuffer().size() >= 100)
    {
        bottomAxis->SetFixedBounds(xAccBuffer.getBuffer()[0].x, xAccBuffer.getBuffer()[99].x);
    }

    Legend* legend = new Legend(wxTOP, wxRIGHT);
    plot->SetLegend(legend);
    plot->AddObjects(dataset, leftAxis, bottomAxis);

    Chart* chart = new Chart(plot, "Acceleration");

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
    //dataset->AddSerie(new XYSerie(rawPositionBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(filteredPositionBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(rawPositionBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(calculatedPositionBuffer.getBuffer()));

    dataset->GetSerie(0)->SetName("filtered position");
    //dataset->GetSerie(1)->SetName("calculated position");
    dataset->GetSerie(1)->SetName("raw position");
    dataset->GetSerie(2)->SetName("calculated position");

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
    Legend* legend = new Legend(wxTOP, wxRIGHT);
    plot->SetLegend(legend);

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

void MyWindow::createSensorDataReceptionThread()
{
    sensorDataReceptionThread = new SensorDataReceptionThread(appLogger, this);
    if (sensorDataReceptionThread->Create() != wxTHREAD_NO_ERROR)
    {
        const std::string threadNotCreated{ "Can't create SensorDataReceptionThread thread! \n" };
        appLogger.logSerialCommStartThread(threadNotCreated);
    }
    else {
        if (sensorDataReceptionThread->Run() != wxTHREAD_NO_ERROR)
        {
            const std::string cantStartThread{ "Can't start SensorDataReceptionThread thread! \n" };
            appLogger.logSerialCommStartThread(cantStartThread);
        }
        else
        {
            const std::string threadStarted{ "New thread SensorDataReceptionThread started.\n" };
            appLogger.logSerialCommStartThread(threadStarted);
            Bind(wxEVT_MY_THREAD_EVENT, &MyWindow::OnSensorsDataThreadEvent, this);
        }
    }
}

void MyWindow::createGpsDataReceptionThread()
{
    gpsDataReceptionThread = new GpsDataReceptionThread(appLogger, this);
    if (gpsDataReceptionThread->Create() != wxTHREAD_NO_ERROR)
    {
        const std::string threadNotCreated{ "Can't create GpsDataReceptionThread thread! \n" };
        appLogger.logSerialCommStartThread(threadNotCreated);
    }
    else {
        if (gpsDataReceptionThread->Run() != wxTHREAD_NO_ERROR)
        {
            const std::string cantStartThread{ "Can't start GpsDataReceptionThread thread! \n" };
            appLogger.logSerialCommStartThread(cantStartThread);
        }
        else
        {
            const std::string threadStarted{ "New thread GpsDataReceptionThread started.\n" };
            appLogger.logSerialCommStartThread(threadStarted);
            Bind(wxEVT_MY_THREAD_EVENT_1, &MyWindow::OnGpsDataThreadEvent, this);
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

    spinCtrlXacc = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -17000, 17000, 15500);
    spinCtrlXacc->SetIncrement(100);
    spinCtrlXaccMultiplicator = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -1000, 1000, 100);
    wxStaticText* xAccText = new wxStaticText(controlPanel, wxID_ANY, "Adjust X acc");
    spinCtrlXacc->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinXAccUpdate, this);
    spinCtrlXaccMultiplicator->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinXAccIncrUpdate, this);

    spinCtrlYacc = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -17000, 17000, 675);
    spinCtrlYacc->SetIncrement(100);
    spinCtrlYaccMultiplicator = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -1000, 1000, 100);
    wxStaticText* yAccText = new wxStaticText(controlPanel, wxID_ANY, "Adjust Y acc");
    spinCtrlYacc->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinYAccUpdate, this);
    spinCtrlYaccMultiplicator->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinYAccIncrUpdate, this);

    spinCtrlZacc = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -17000, 17000, 16100);
    spinCtrlZacc->SetIncrement(100);
    spinCtrlZaccMultiplicator = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -1000, 1000, 100);
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

    matR << 1.0F, 0.0F,
            0.0F, 1.0F;
            //acc vel  pos
    matH << 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 1.0F;

    // Create two panels to be placed in the splitter window
    //wxPanel* panel1 = new wxPanel(splitter, wxID_ANY);
    wxPanel* controlPanel = new wxPanel(splitter, wxID_ANY);

    filteredPositionChartPanel = new wxChartPanel(splitter);
    sizerPositionPlot = new wxBoxSizer(wxVERTICAL);
    filteredPositionChartPanel->SetMinSize(wxSize(900, 600));
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
    //comSetupPanel = new wxPanel(m_notebook);
    //m_notebook->AddPage(comSetupPanel, "Serial port setup");
    //dataReceptionPanel = new wxPanel(m_notebook);
    //m_notebook->AddPage(dataReceptionPanel, "Data reception");
    
    //innerNotebook = new wxNotebook(kalmanParamsSetupPanel, 2);
    //wxPanel* innerPanel = new wxPanel(innerNotebook);
    //innerNotebook->AddPage(innerPanel, "Inner");

    
    kalmanFilterSetupGui.setup(m_notebook);

    csvMeasurementLoadPanel = new wxPanel(m_notebook);
    m_notebook->AddPage(csvMeasurementLoadPanel, "Load CSV");
    csvMeasurementLoadGui.setup(csvMeasurementLoadPanel, &filterFileMeasTimer);
    
    

    //splitter = new wxSplitterWindow(, wxID_ANY);

    prepareAccChart();
    prepareVelChart();
    positionChartGui.setup(m_notebook);
    prepareGpsBasedPositionChart();
    angleVelocityChartGui.setup(m_notebook);
    rollPitchChartGui.setup(m_notebook);
    prepareFilteredPositionChart();
    prepareFilteredVelocityChart();
    prepareFilteredAngleXVelocityChart();
    magnChartGui.setup(m_notebook);

    // Add outer tabs to the notebook
    //m_notebook->AddPage(new OuterTab(m_notebook, "Outer Tab 1"), "Outer Tab 1");
   // m_notebook->AddPage(new OuterTab(m_notebook, "Outer Tab 2"), "Outer Tab 2");
    //m_notebook->AddPage(new OuterTab(m_notebook, "Outer Tab 2"), "Outer Tab 2");


    wxSize size(100, 20);

    wxBoxSizer* panelSizer = new wxBoxSizer(wxVERTICAL);

    //wxStaticText* LBcom = new wxStaticText(comSetupPanel, wxID_ANY, "COM port number:");
    //wxTextCtrl* INcomName = new wxTextCtrl(comSetupPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, size);
    //LBcom->SetPosition({ 40,100 });
    //INcomName->SetPosition({ 200,100 });


    //wxFont font(16, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
    //wxStaticText* LBheaderText = new wxStaticText(comSetupPanel, wxID_ANY, "Enter serial communication settings", wxDefaultPosition, wxDefaultSize);
    //LBheaderText->SetFont(font);
    //LBheaderText->SetPosition({ 50,50 });


    //wxArrayString baudRateChoices;
    //baudRateChoices.Add("9600");
    //baudRateChoices.Add("115200");
    //baudRateChoices.Add("330400");
    //wxChoice* baudRateChoice = new wxChoice(comSetupPanel, wxID_ANY, wxDefaultPosition, size, baudRateChoices);
    //baudRateChoice->SetPosition({ 200, 150 });
    //baudRateChoice->Bind(wxEVT_CHOICE, &MyWindow::OnBaudRateChoice, this);

    //wxStaticText* LBbaudRate = new wxStaticText(comSetupPanel, wxID_ANY, "Baud rate:");
    //wxTextCtrl* INbaudRate = new wxTextCtrl(comSetupPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize);
    //LBbaudRate->SetPosition({ 40,150 });
    //INbaudRate->SetPosition({ 200,150 });


    //wxArrayString stopBitsChoices;
    //stopBitsChoices.Add("0");
    //stopBitsChoices.Add("1");
    //wxChoice* stopBitsChoice = new wxChoice(comSetupPanel, wxID_ANY, wxDefaultPosition, size, stopBitsChoices);
    //stopBitsChoice->SetPosition({ 200, 200 });
    //stopBitsChoice->Bind(wxEVT_CHOICE, &MyWindow::OnStopBitsChoice, this);
    //wxStaticText* LBstopBits = new wxStaticText(comSetupPanel, wxID_ANY, "Stop bits:");
    //wxTextCtrl* INstopBits = new wxTextCtrl(comSetupPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize);
    //LBstopBits->SetPosition({ 40,200 });
    //stopBitsChoice->SetPosition({ 200,200 });

    //wxArrayString parityChoices;
    //parityChoices.Add("NONE");
    //parityChoices.Add("EVEN");
    //parityChoices.Add("ODD");

    //wxChoice* parityChoice = new wxChoice(comSetupPanel, wxID_ANY, wxDefaultPosition, size, parityChoices);
    //parityChoice->SetPosition({ 200, 250 });
    //parityChoice->Bind(wxEVT_CHOICE, &MyWindow::OnParityChoice, this);
    //wxStaticText* LBparity = new wxStaticText(comSetupPanel, wxID_ANY, "Parity:", wxDefaultPosition, wxDefaultSize);
    //wxTextCtrl* INparity = new wxTextCtrl(comSetupPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize);
    //LBparity->SetPosition({ 40,250 });
    //INparity->SetPosition({ 200,250 });

    //wxButton* BTconfirmSetupAndStartReception = new wxButton(comSetupPanel, wxID_ANY, "Start data reception");
    //BTconfirmSetupAndStartReception->SetPosition({ 400, 500 });
    //BTconfirmSetupAndStartReception->Bind(wxEVT_BUTTON, &MyWindow::OnStartReceptionClick, this);

    filterFileMeasTimer.Bind(wxEVT_TIMER, &MyWindow::OnFilterFileMeasTimer, this);
}

void MyWindow::updateMatQGrid()
{
    for (int i = 0; i < DIM_X; ++i) {
        for (int j = 0; j < DIM_X; ++j) {
            matrixGrid->SetCellValue(i, j, wxString::Format("%f", matQ(i * DIM_X + j)));
        }
    }
}
