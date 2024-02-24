#pragma once

#include <wx/wx.h>
#include <wx/button.h>
#include <wx/log.h>
#include <wx/notebook.h>
#include <wx/panel.h>
#include <wx/frame.h>
#include <wx/thread.h>


#include <wx/chartpanel.h>
#include <wx/chart.h>
#include <wx/legend.h>
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
#include <wx/splitter.h>
#include <wx/spinctrl.h> 
#include <wx/grid.h>

#include "SensorDataReceptionThread.h"
#include "GpsDataReceptionThread.h"
#include "SensorDataComReceptionThread.h"
#include "SerialComm.h"
#include "SerialComSensorDataReceiver.h"
#include "WifiDataReceiver.h"
//#include "KalmanFilter.h"
#include "AppLogger.h"
#include "RawMeasurements.h"
#include "VelocityCalculator.h"
#include "DeltaTimeCalculator.h"
#include "RelativePositionCalculator.h"
#include "PlotElementsBuffer.h"
#include "GpsDataConverter.h"
#include "HaversineConverter.h"
#include "PositionUpdater.h"
#include "KalmanFilterSetupGui.h"
#include "CsvMeasurementReader.h"
#include "CsvMeasurementLoadGui.h"
#include "PositionChartGui.h"
#include "MagnChartGui.h"
#include "AngleVelocityChartGui.h"
#include "RollPitchChartGui.h"
#include "AccelTransform.h"
#include "GyroCallibrator.h"
#include "KalmanFilters.h"
#include "kalman_filter/kalman_filter.h"
#include <chrono>
#include <stdio.h>

using namespace std;

class MyWindow : public wxFrame
{
public:
    MyWindow(const wxString& title);
    ~MyWindow()
    {
        //serialComThread->Delete();
    }

    //Bind(wxEVT_MY_THREAD_EVENT, &MyFrame::OnThreadEvent, this);

private:
    void OnButtonClick(wxCommandEvent& event);
    void OnBaudRateChoice(wxCommandEvent& event);
    void OnStopBitsChoice(wxCommandEvent& event);
    void OnParityChoice(wxCommandEvent& event);
    void OnStartReceptionClick(wxCommandEvent& event);
    void OnResetAccChart(wxCommandEvent& event);
    void OnSubmitAccAdjustments(wxCommandEvent& event);
    void OnSpinXAccUpdate(wxSpinEvent& event);
    void OnSpinXAccIncrUpdate(wxSpinEvent& event);
    void OnSpinYAccUpdate(wxSpinEvent& event);
    void OnSpinYAccIncrUpdate(wxSpinEvent& event);
    void OnSpinZAccUpdate(wxSpinEvent& event);
    void OnSpinZAccIncrUpdate(wxSpinEvent& event);

    void OnApplyKFTunning(wxCommandEvent& event);

    void processFiltration(MeasurementsController& rawMeasurement, const uint32_t deltaTimeMs, const bool isRealTimeMeasurement);

    void OnSensorsDataThreadEvent(wxThreadEvent& event);
    void OnGpsDataThreadEvent(wxThreadEvent& event);
    void OnSensorDataComThreadEvent(wxThreadEvent& event);

    void OnFilterFileMeasTimer(wxTimerEvent& event);
    void OnFilterReceivedDataProcessingTimer(wxTimerEvent& event);
    void OnFilterReceivedGpsProcessingTimer(wxTimerEvent& event);


    void resetChartsAfterCallibration();
    void updateAccChart(const TransformedAccel& transformedAccel, const double xAccMPerS2, const double yAccMPerS2, const double zAccMPerS2,
        const CompensatedAccData& compensatedAccData,
        const double filteredXacc, const double filteredYacc, const double xAccGyroCompens, const double yAccGyroCompens,
        const double timeMs, const uint32_t deltaTime);
    //void updateVelChart(const double xVelocity, const double timeMs);
    void updateGpsBasedPositionChart(std::pair<double, double> gpsBasedPosition);
    void updateFilteredPositionChart(const double filteredPositionX, const double filteredPositionY,
        const std::pair<double, double> calculatedPosition, const double timeMs);
    void updateFilteredAngleXVelocityChart(const double filteredXangle, const double measuredXangle, const double timeMs);
    void updateFilteredVelocityChart(const double filteredVelocityX, const double filteredVelocityY, const double timeMs);

    void updateMatQGrid();

    std::pair<bool, std::vector<std::string>> currentSensorMeasurements{ true, std::vector<std::string>{} };
    std::pair<bool, std::vector<std::string>> currentGpsMeasurements{ false, std::vector<std::string>{} };

    DeltaTimeCalculator deltaTimeCalculator;
    RelativePositionCalculator relativePositionCalculator{};
    std::vector<MeasurementsController> rawMeasurementsSet{};
    HaversineConverter haversineConverter{};
    GpsDataConverter gpsDataConverter{};
    double longitude{ 52.3244 };
    double latitude{ 19.3243 };

    AccelTransform accelTransform{};
    PositionUpdater positionUpdater{};
    GyroCallibrator gyroCallibrator{};

    double filteredPositionX{ 0.0 };
    double filteredPositionY{ 0.0 };

    wxTimer filterFileMeasTimer;
    wxTimer filterReceivedDataProcessingTimer;
    wxTimer filterReceivedGpsProcessingTimer;

    MagnetometerCallibrator magnetometerCallibrator{};
    MagnChartGui magnChartGui{ magnetometerCallibrator };
    KalmanFilterSetupGui kalmanFilterSetupGui{ filterReceivedDataProcessingTimer, filterReceivedGpsProcessingTimer, magnChartGui };
    CsvMeasurementReader csvMeasurementReader;
    CsvMeasurementLoadGui csvMeasurementLoadGui{ csvMeasurementReader };

    PositionChartGui positionChartGui;

    AngleVelocityChartGui angleVelocityChartGui;
    RollPitchChartGui rollPitchChartGui;

    bool isDataReceptionStarted{ false };
    wxVector <wxRealPoint> magnPoints;
    KalmanFilters kalmanFilters{kalmanFilterSetupGui};

    bool isFirstMeasurement{ true };

    //frame
    wxNotebook* m_notebook = nullptr;
    wxNotebook* innerNotebook = nullptr;
    wxPanel* comSetupPanel = nullptr;
    wxPanel* dataReceptionPanel = nullptr;
    wxPanel* kalmanParamsSetupPanel = nullptr;
    wxPanel* csvMeasurementLoadPanel = nullptr;

    wxChartPanel* gpsBasedPositionChartPanel = nullptr;
    wxChartPanel* accChartPanel = nullptr;
    wxChartPanel* velChartPanel = nullptr;
    

    wxChartPanel* filteredPositionChartPanel = nullptr;
    wxChartPanel* filteredVelocityChartPanel = nullptr;

    wxChartPanel* filteredAngleXVelocity = nullptr;

    wxSplitterWindow* gpsBasedPositionPanelSplitter = nullptr;

    XYPlot* plot = nullptr;
    XYSimpleDataset* dataset = nullptr;
    NumberAxis* leftAxis = nullptr;
    NumberAxis* bottomAxis = nullptr;
    Chart* azimuthChart = nullptr;


    XYPlot* accPlot = nullptr;
    XYSimpleDataset* accDataset = nullptr;
    NumberAxis* accLeftAxis = nullptr;
    NumberAxis* accBottomAxis = nullptr;
    Chart* accChart = nullptr;

    wxVector <wxRealPoint> velPoints;
    wxVector <wxRealPoint> rawPositionPoints;

    wxVector <wxRealPoint> xAccPoints;
    wxVector <wxRealPoint> yAccPoints;
    wxVector <wxRealPoint> zAccPoints;

    wxVector <wxRealPoint> xAngleVelocityPoints;
    wxVector <wxRealPoint> yAngleVelocityPoints;
    wxVector <wxRealPoint> zAngleVelocityPoints;

    wxVector <wxRealPoint> filteredVelocityPoints;
    wxVector <wxRealPoint> filteredPositionPoints;

    PlotElementsBuffer rawPositionBuffer{ 300 };
    PlotElementsBuffer filteredPositionBuffer{ 300 };
    PlotElementsBuffer calculatedPositionBuffer{ 300 };
    PlotElementsBuffer gpsBasedPositionBuffer{ 300 };

    PlotElementsBuffer magnPointsBuffer;
    PlotElementsBuffer filteredAzimuthBuffer;

    PlotElementsBuffer xAccBuffer;
    PlotElementsBuffer yAccBuffer;
    PlotElementsBuffer zAccBuffer;
    PlotElementsBuffer xAccGravityCompensationBuffer;
    PlotElementsBuffer yAccGravityCompensationBuffer;
    PlotElementsBuffer zAccGravityCompensationBuffer;

    PlotElementsBuffer compensatedXAccDataBuffer;
    PlotElementsBuffer compensatedYAccDataBuffer;
    PlotElementsBuffer filteredXaccBuffer;
    PlotElementsBuffer filteredYaccBuffer;
    PlotElementsBuffer xAccWithGyroCompensation;
    PlotElementsBuffer yAccWithGyroCompensation;

    PlotElementsBuffer xAngleVelocityBuffer;
    PlotElementsBuffer yAngleVelocityBuffer;
    PlotElementsBuffer zAngleVelocityBuffer;
    PlotElementsBuffer filteredZangleVelocityBuffer;

    PlotElementsBuffer filteredXAngleVelocityBuffer;

    PlotElementsBuffer rollBuffer;
    PlotElementsBuffer pitchBuffer;
    PlotElementsBuffer yawBuffer;

    PlotElementsBuffer rollBasedOnAccBuffer;
    PlotElementsBuffer pitchBasedOnAccBuffer;

    double roll{ 0.0 };
    double pitch{ 0.0 };
    double yaw{ 0.0 };
    

    wxVector <wxRealPoint> filteredXangleVelocity;
    wxVector <wxRealPoint> measuredXangleVelocity;

    wxSplitterWindow* splitter = nullptr;
    wxBoxSizer* sizerPositionPlot = nullptr;

    wxGrid* matrixGrid;
    wxGrid* matrixRCovariance;
    wxGrid* matrixH;

    wxSplitterWindow* accPanelSplitter = nullptr;
    wxBoxSizer* sizerAccPlot = nullptr;
    wxSpinCtrl* spinCtrlXacc = nullptr;
    wxSpinCtrl* spinCtrlXaccMultiplicator = nullptr;
    wxSpinCtrl* spinCtrlYacc = nullptr;
    wxSpinCtrl* spinCtrlYaccMultiplicator = nullptr;
    wxSpinCtrl* spinCtrlZacc = nullptr;
    wxSpinCtrl* spinCtrlZaccMultiplicator = nullptr;

    wxStaticText* xAccValue = nullptr;
    wxStaticText* yAccValue = nullptr;
    wxStaticText* zAccValue = nullptr;

    wxStaticText* deltaTimeValue = nullptr;
    wxStaticText* totalTimeValue = nullptr;

    bool isRestartFiltrationNeeded{ false };

    double rawGrawity{ 16100.0 };
    double xBias{ 600.0 };
    double yBias{ 675.0 };

    double totalTimeMs{ 0.0 };
    double azimuthXPoint{ 0.0 };


    double currentFilteredXPosition{ 0.0 };
    double currentFilteredYPosition{ 0.0 };
    

    double currentXangleFiltered{ 0.0 };
    double currentXangleMeasured{ 0.0 };
    double angleTimeMeasurementsMs{ 0.0 };

    SensorDataReceptionThread* sensorDataReceptionThread = nullptr;
    GpsDataReceptionThread* gpsDataReceptionThread = nullptr;
    SensorDataComReceptionThread* sensorDataComReceptionThread = nullptr;

    double xNewPoint = 0.0;
    double yNewPoint = 36.0;

    double timeNewAccPoint{ 0.0 };

    double xAngleVelNewPoint{ 0.0 };

    uint32_t measurementCounter{ 0 };

    void prepareGui();
    void prepareAccChart();
    void prepareVelChart();
    void preparePositionChart();
    void prepareGpsBasedPositionChart();
    void prepareFilteredPositionChart();
    void prepareFilteredVelocityChart();
    void prepareFilteredAngleXVelocityChart();

    void createSensorDataReceptionThread();
    void createGpsDataReceptionThread();
    void createSensorDataCOMReceptionThread();

    AppLogger appLogger;
};



