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

#include "MeasurementReceptionThread.h"
#include "SerialComm.h"
#include "WifiDataReceiver.h"
//#include "KalmanFilter.h"
#include "AppLogger.h"
#include "RawMeasurements.h"
#include "VelocityCalculator.h"
#include "DeltaTimeCalculator.h"
#include "RelativePositionCalculator.h"
#include "PlotElementsBuffer.h"
#include "HaversineConverter.h"
#include "PositionUpdater.h"
#include "KalmanFilterSetupGui.h"
#include "CsvMeasurementLoadGui.h"

#include "kalman_filter/kalman_filter.h"

#include <chrono>


//#include "wx/wx.h"
#include <stdio.h>

using namespace std;

class InnerTab : public wxPanel {
public:
    InnerTab(wxWindow* parent, const wxString& label)
        : wxPanel(parent, wxID_ANY)
    {
        // Add controls or content for the inner tab
        wxStaticText* text = new wxStaticText(this, wxID_ANY, label, wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE);

        wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);
        sizer->Add(text, 1, wxEXPAND | wxALL, 10);
        SetSizerAndFit(sizer);
    }
};

class OuterTab : public wxPanel {
public:
    OuterTab(wxWindow* parent, const wxString& label)
        : wxPanel(parent, wxID_ANY)
    {
        // Create a notebook for inner tabs
        wxNotebook* innerNotebook = new wxNotebook(this, wxID_ANY);

        // Add inner tabs to the notebook
        innerNotebook->AddPage(new InnerTab(innerNotebook, "Inner Tab 1"), "Inner Tab 1");
        innerNotebook->AddPage(new InnerTab(innerNotebook, "Inner Tab 2"), "Inner Tab 2");

        // Add more inner tabs as needed

        wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);
        sizer->Add(innerNotebook, 1, wxEXPAND | wxALL, 10);
        SetSizerAndFit(sizer);
    }
};

class MyTabPanel : public wxPanel {
public:
    MyTabPanel(wxWindow* parent) {
        // Create a wxMathPlot component
        //mpWindow* graph = new mpWindow(parent, wxID_ANY);
        //graph->EnableDoubleBuffer(true);
    }

    // Other functions and variables you may need
};






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

    void OnResetAngleVelChart(wxCommandEvent& event);
    void OnSubmitAngleVelAdjustments(wxCommandEvent& event);
    void OnSpinXAngleVelUpdate(wxSpinEvent& event);
    void OnSpinXAnglVelIncrUpdate(wxSpinEvent& event);
    void OnSpinYAngleVelUpdate(wxSpinEvent& event);
    void OnSpinYAnglVelIncrUpdate(wxSpinEvent& event);
    void OnSpinZAngleVelUpdate(wxSpinEvent& event);
    void OnSpinZAnglVelIncrUpdate(wxSpinEvent& event);

    void OnResetMagnChart(wxCommandEvent& event);
    void OnApplyKFTunning(wxCommandEvent& event);
    void OnSubmitMagnAdjustments(wxCommandEvent& event);

    void OnThreadEvent(wxThreadEvent& event);

    void experimentKf(const double Xacc, const double Yacc, uint32_t deltaTimeUint)
    {

        static constexpr kf::float32_t T{ 1.0F };
        static constexpr kf::float32_t Q11{ 0.1F }, Q22{ 0.1F };



        /*
        *  macierz A = [1, dt, 0.5 * dt^2  -- dla pozycji x
        *               0, 1,  dt          -- dla prêdkosci x
        *               0, 0, 1          ] -- dla przyspieszenia x
        *
        *
        *
        *
        *  X
        *  [x
            y
            vx
            vy]
        */

        //macierz F
        /*
        *  1 dt 1/2 dt^2 0 0 0
        *  0 1 dt 0 0 0
        *  0 0 1 0 0 0
        *  0 0 0 1 dt 1/2 dt^2
        *  0 0 0 0 1 dt
        *  0 0 0 0 0 1
        */

        /*
        * X =  [x
        *       vx
        *       ax
        *       y
        *       vy
        *       ay]
        *
        */

        //const uint32_t deltaT{ 100 };
        //const double teta{ 18.0 };



        //kalmanFilter.vecX() << 0.0F, 2.0F;
        //kalmanFilter.matP() << 0.1F, 0.0F, 0.0F, 0.1F;

        kf::Matrix<DIM_X, DIM_X> A; // macierz stanu
        //F << 1.0F, deltaT, 0.5F*cos(teta)*deltaT*deltaT, 0.0F,
        //    0, 1, 0.5F * sin(teta) * deltaT * deltaT, 0.0F,
        //    0, 0, 1, deltaT,
        //    0, 0, 0, 1;
        double deltaTimeMs = static_cast<double>(deltaTimeUint);
        A << 1.0F, deltaTimeMs, (deltaTimeMs * deltaTimeMs) / 2, 0.0F, 0.0F, 0.0F,
            0.0F, 1.0F, deltaTimeMs, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 1.0F, deltaTimeMs, (deltaTimeMs * deltaTimeMs) / 2,
            0.0F, 0.0F, 0.0F, 0.0F, 1.0F, deltaTimeMs,
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F;

        double process_variance = 0.002F;
        deltaTimeMs = deltaTimeMs / 100000.0F;

        //kowariancja szumu procesowego
        matQ << pow(deltaTimeMs, 6) / 36, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 6, 0, 0, 0,
            pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 4, pow(deltaTimeMs, 3) / 2, 0, 0, 0,
            pow(deltaTimeMs, 4) / 6, pow(deltaTimeMs, 3) / 2, pow(deltaTimeMs, 2), 0, 0, 0,
            0, 0, 0, pow(deltaTimeMs, 6) / 36, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 6,
            0, 0, 0, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 4, pow(deltaTimeMs, 3) / 2,
            0, 0, 0, pow(deltaTimeMs, 4) / 6, pow(deltaTimeMs, 3) / 2, pow(deltaTimeMs, 2);

        //Q << 1000, 2000, 25, 0, 0, 0,
        //    2000, 25, 20, 0, 0, 0,
        //    25, 20, 5, 0, 0, 0,
        //    0, 0, 0, 5, 20, 25,
        //    0, 0, 0, 20, 25, 2000,
        //    0, 0, 0, 25, 2000, 1000;

        //Q << 1, 1, 1, 0, 0, 0,
        //    1, 1, 1, 0, 0, 0,
        //    1, 1, 1, 0, 0, 0,
        //    0, 0, 0, 1, 1, 1,
        //    0, 0, 0, 1, 1, 1,
        //    0, 0, 0, 1, 1, 1;


        matQ *= process_variance;

        ////PREDICTION STEP
        kalmanFilter.predictLKF(A, matQ);
        appLogger.logKalmanFilterPredictionStep(kalmanFilter);
        ////

        kf::Vector<DIM_Z> vecZ;
        //float ax = newMeasurement + 1.1f;
        //float ay = newMeasurement + 2.1f;
        vecZ << Xacc, Yacc;

        //kf::Matrix<DIM_Z, DIM_Z> matR;
        //matR << 0.1F, 0.0F,
        //        0.0F, 0.1F;

        //kf::Matrix<DIM_Z, DIM_X> matH;
        //matH << 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        //        0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F;

        ////CORRECTION STEP
        kalmanFilter.correctLKF(vecZ, matR, matH);

        appLogger.logKalmanFilterCorrectionStep(kalmanFilter);
    }

    void experimentKfAzimuth(const double xAngleVelocityDegPerSec, const double yAngleVelocityDegPerSec, const double zAngleVelocityDegPerSec, const double azimuthFromMagn, const uint32_t deltaTime)
    {
        double deltaTimeMs = static_cast<double>(deltaTime);
        kf::Matrix<DIM_X_azimuth, DIM_X_azimuth> A;
        A << 1.0F, 0.0F, 0.0F, deltaTimeMs, 0.0F, 0.0F,
            0.0F, 1.0F, 0.0F, 0.0F, deltaTimeMs, 0.0F,
            0.0F, 0.0F, 1.0F,  0.0F, 0.0F, deltaTimeMs,
            0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F;

        kf::Matrix<DIM_X_azimuth, DIM_X_azimuth> Q;

        double process_variance = 0.02F;
        deltaTimeMs = deltaTimeMs / 1000.0F;
        Q << pow(deltaTimeMs, 6) / 36, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 6, 0, 0, 0,
            pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 4, pow(deltaTimeMs, 3) / 2, 0, 0, 0,
            pow(deltaTimeMs, 4) / 6, pow(deltaTimeMs, 3) / 2, pow(deltaTimeMs, 2), 0, 0, 0,
            0, 0, 0, pow(deltaTimeMs, 6) / 36, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 6,
            0, 0, 0, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 4, pow(deltaTimeMs, 3) / 2,
            0, 0, 0, pow(deltaTimeMs, 4) / 6, pow(deltaTimeMs, 3) / 2, pow(deltaTimeMs, 2);

        Q *= process_variance;

        kalmanFilterAzimuth.predictLKF(A, Q);

        kf::Vector<DIM_Z_azimuth> vecZ;

        vecZ <<  xAngleVelocityDegPerSec, zAngleVelocityDegPerSec, azimuthFromMagn;

        kf::Matrix<DIM_Z_azimuth, DIM_Z_azimuth> matR;
        matR << 0.0001F, 0, 0,
                0, 0.1F, 0,
                0, 0, 0.1F;
        kf::Matrix<DIM_Z_azimuth, DIM_X_azimuth> matH;
        matH << 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0;

        kalmanFilterAzimuth.correctLKF(vecZ, matR, matH);

    }

    void experimentGyroKf(const double xAngleVelocityDegPerSec, const double yAngleVelocityDegPerSec, const double zAngleVelocityDegPerSec, uint32_t deltaTimeMs)
    {
        //double deltaTimeMs = static_cast<double>(deltaTimeUint) * 10.0F;

        kf::Matrix<DIM_X_gyro, DIM_X_gyro> A;
        A << 1.0F, deltaTimeMs, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 1.0F, deltaTimeMs, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 0.0F, 1.0F, deltaTimeMs,
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F;

        double sigma_theta_x_sq = 1;  // Wariancja b³êdu k¹ta w osi x
        double sigma_theta_dot_x_sq = 10;  // Wariancja b³êdu prêdkoœci k¹towej w osi x
        double sigma_theta_y_sq = 1;  // Wariancja b³êdu k¹ta w osi y
        double sigma_theta_dot_y_sq = 10;  // Wariancja b³êdu prêdkoœci k¹towej w osi y
        double sigma_theta_z_sq = 1;  // Wariancja b³êdu k¹ta w osi z
        double sigma_theta_dot_z_sq = 10;

        kf::Matrix<DIM_X_gyro, DIM_X_gyro> Q;

        double process_variance = 0.02F;
        deltaTimeMs = deltaTimeMs / 1000.0F;
        Q << pow(deltaTimeMs, 6) / 36, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 6, 0, 0, 0,
            pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 4, pow(deltaTimeMs, 3) / 2, 0, 0, 0,
            pow(deltaTimeMs, 4) / 6, pow(deltaTimeMs, 3) / 2, pow(deltaTimeMs, 2), 0, 0, 0,
            0, 0, 0, pow(deltaTimeMs, 6) / 36, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 6,
            0, 0, 0, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 4, pow(deltaTimeMs, 3) / 2,
            0, 0, 0, pow(deltaTimeMs, 4) / 6, pow(deltaTimeMs, 3) / 2, pow(deltaTimeMs, 2);

        //double process_variance = 0.01;
        Q *= process_variance;

        kalmanFilterGyro.predictLKF(A, Q);

        kf::Vector<DIM_Z_gyro> vecZ;

        vecZ << xAngleVelocityDegPerSec, yAngleVelocityDegPerSec, zAngleVelocityDegPerSec;

        kf::Matrix<DIM_Z_gyro, DIM_Z_gyro> matR;
        matR << 0.01F, 0, 0,
            0, 0.01F, 0,
            0, 0, 0.01F;
        kf::Matrix<DIM_Z_gyro, DIM_X_gyro> matH;
        matH << 1, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 1, 0;

        kalmanFilterGyro.correctLKF(vecZ, matR, matH);
    }

    void OnTimer(wxTimerEvent& event);

    void resetChartsAfterCallibration();
    void updateMagnChart(const int16_t rawXMagn, const int16_t rawYMagn, const double rawAzimuth, const double filteredAzimuth, const double timeMs);
    void updateAccChart(const double xAccMPerS2, const double yAccMPerS2, const double zAccMPerS2, const double timeMs, const uint32_t deltaTime);
    //void updateVelChart(const double xVelocity, const double timeMs);
    void updatePositionChart(const double xDistance, const double yDistance, const double timeMs);
    void updateAngleVelocityChart(const double xAngleVel, const double yAngleVel, const double zAngleVel, const double filteredZangleVelocity, const double timeMs);
    void updateGpsBasedPositionChart(std::pair<double, double> gpsBasedPosition);
    void updateFilteredPositionChart(const double filteredPositionX, const double filteredPositionY,
        const std::pair<double, double> calculatedPosition, const double timeMs);
    void updateFilteredAngleXVelocityChart(const double filteredXangle, const double measuredXangle, const double timeMs);
    void updateFilteredVelocityChart(const double filteredVelocityX, const double filteredVelocityY, const double timeMs);

    void updateMatQGrid();

    DeltaTimeCalculator deltaTimeCalculator;
    RelativePositionCalculator relativePositionCalculator{};
    std::vector<MeasurementsController> rawMeasurementsSet{};
    HaversineConverter haversineConverter{};
    double longitude{ 52.3244 };
    double latitude{ 19.3243 };

    PositionUpdater positionUpdater{};

    double filteredPositionX{ 0.0 };
    double filteredPositionY{ 0.0 };

    KalmanFilterSetupGui kalmanFilterSetupGui;
    CsvMeasurementLoadGui csvMeasurementLoadGui;

    static constexpr size_t DIM_X{ 6 };
    static constexpr size_t DIM_Z{ 2 };
    kf::Matrix<DIM_X, DIM_X> matQ;
    kf::Matrix<DIM_Z, DIM_Z> matR;
    kf::Matrix<DIM_Z, DIM_X> matH;

    bool isDataReceptionStarted{ false };
    wxVector <wxRealPoint> magnPoints;


    kf::KalmanFilter<DIM_X, DIM_Z> kalmanFilter;

    static constexpr size_t DIM_X_gyro{ 6 };
    static constexpr size_t DIM_Z_gyro{ 3 };
    kf::KalmanFilter<DIM_X_gyro, DIM_Z_gyro> kalmanFilterGyro;

    static constexpr size_t DIM_X_azimuth{ 6 };
    static constexpr size_t DIM_Z_azimuth{ 3 };
    kf::KalmanFilter<DIM_X_azimuth, DIM_Z_azimuth> kalmanFilterAzimuth;


    bool isFirstMeasurement{ true };

    //frame
    wxNotebook* m_notebook = nullptr;
    wxNotebook* innerNotebook = nullptr;
    wxPanel* comSetupPanel = nullptr;
    wxPanel* dataReceptionPanel = nullptr;
    wxPanel* kalmanParamsSetupPanel = nullptr;
    wxPanel* csvMeasurementLoadPanel = nullptr;

    wxChartPanel* azimuthChartPanel = nullptr;
    wxChartPanel* gpsBasedPositionChartPanel = nullptr;
    wxChartPanel* accChartPanel = nullptr;
    wxChartPanel* velChartPanel = nullptr;
    wxChartPanel* positionChartPanel = nullptr;
    wxChartPanel* angleVelocityChartPanel = nullptr;

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

    PlotElementsBuffer xAngleVelocityBuffer;
    PlotElementsBuffer yAngleVelocityBuffer;
    PlotElementsBuffer zAngleVelocityBuffer;
    PlotElementsBuffer filteredZangleVelocityBuffer;

    PlotElementsBuffer filteredXAngleVelocityBuffer;
    

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

    double rawGrawity{ 16100.0 };
    double xBias{ 530.0 };
    double yBias{ 530.0 };


    wxSplitterWindow* angleVelPanelSplitter = nullptr;
    wxBoxSizer* sizerAngleVelPlot = nullptr;
    wxSpinCtrl* spinCtrlXangleVel = nullptr;
    wxSpinCtrl* spinCtrlXangleVelMultiplicator = nullptr;
    wxSpinCtrl* spinCtrlYangleVel = nullptr;
    wxSpinCtrl* spinCtrlYangleVelMultiplicator = nullptr;
    wxSpinCtrl* spinCtrlZangleVel = nullptr;
    wxSpinCtrl* spinCtrlZangleVelMultiplicator = nullptr;

    wxStaticText* xAngleVelValue = nullptr;
    wxStaticText* yAngleVelValue = nullptr;
    wxStaticText* zAngleVelValue = nullptr;

    double xGyroBias{ -18000.0 };
    double yGyroBias{ 12900.0 };
    double zGyroBias{ 15000.0 };

    wxSplitterWindow* azimuthPanelSplitter = nullptr;
    wxBoxSizer* sizerAzimuthPlot = nullptr;
    wxSpinCtrl* spinCtrlXmagn = nullptr;
    wxSpinCtrl* spinCtrlYmagn = nullptr;

    wxStaticText* xMagnValue = nullptr;
    wxStaticText* yMagnValue = nullptr;
    wxStaticText* orientationValue = nullptr;

    double totalTimeMs{ 0.0 };
    double azimuthXPoint{ 0.0 };

    double currentXPos{ 0.0 };
    double currentYPos{ 0.0 };

    double currentFilteredXPosition{ 0.0 };
    double currentFilteredYPosition{ 0.0 };
    

    double currentXangleFiltered{ 0.0 };
    double currentXangleMeasured{ 0.0 };
    double angleTimeMeasurementsMs{ 0.0 };

    MeasReceptionThrea* serialComThread = nullptr;

    wxTimer m_timer;
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
    void prepareAngleVelocityChart();
    void prepareFilteredPositionChart();
    void prepareFilteredVelocityChart();
    void prepareAzimuthChart();
    void prepareFilteredAngleXVelocityChart();

    void createDataReceptionThread();

    AppLogger appLogger;

};



