#include "AngleVelocityChartGui.h"

void AngleVelocityChartGui::setup(wxNotebook* m_notebook)
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
    spinCtrlXangleVel->Bind(wxEVT_SPINCTRL, &AngleVelocityChartGui::OnSpinXAngleVelUpdate, this);
    spinCtrlXangleVelMultiplicator->Bind(wxEVT_SPINCTRL, &AngleVelocityChartGui::OnSpinXAnglVelIncrUpdate, this);

    spinCtrlYangleVel = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -33000, 33000, 13450);
    spinCtrlYangleVelMultiplicator = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -1000, 1000, 10);
    wxStaticText* yAngleVelText = new wxStaticText(controlPanel, wxID_ANY, "Adjust Y angle velocity");
    spinCtrlYangleVel->Bind(wxEVT_SPINCTRL, &AngleVelocityChartGui::OnSpinYAngleVelUpdate, this);
    spinCtrlYangleVelMultiplicator->Bind(wxEVT_SPINCTRL, &AngleVelocityChartGui::OnSpinYAnglVelIncrUpdate, this);

    spinCtrlZangleVel = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -33000, 33000, 16500);
    spinCtrlZangleVelMultiplicator = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -1000, 1000, 10);
    wxStaticText* zAngleVelText = new wxStaticText(controlPanel, wxID_ANY, "Adjust Z angle velocity");
    spinCtrlZangleVel->Bind(wxEVT_SPINCTRL, &AngleVelocityChartGui::OnSpinZAngleVelUpdate, this);
    spinCtrlZangleVelMultiplicator->Bind(wxEVT_SPINCTRL, &AngleVelocityChartGui::OnSpinZAnglVelIncrUpdate, this);

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
    resetButton->Bind(wxEVT_BUTTON, &AngleVelocityChartGui::OnResetAngleVelChart, this);

    wxButton* submitButton = new wxButton(controlPanel, wxID_ANY, "Submit adjustments");
    angleVelSetupButtonsSizer->Add(submitButton, 0, wxALL | wxALIGN_LEFT, 5);
    submitButton->Bind(wxEVT_BUTTON, &AngleVelocityChartGui::OnSubmitAngleVelAdjustments, this);

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

void AngleVelocityChartGui::updateChart(
    PlotElementsBuffer& xAngleVelocityBuffer,
    PlotElementsBuffer& yAngleVelocityBuffer,
    PlotElementsBuffer& zAngleVelocityBuffer,
    PlotElementsBuffer& filteredZangleVelocityBuffer,
    const double xAngleVel, const double yAngleVel, const double zAngleVel, const double filteredZangleVelocity, const double timeMs)
{
    xAngleVelValue->SetLabel(std::to_string(xAngleVel));
    yAngleVelValue->SetLabel(std::to_string(yAngleVel));
    zAngleVelValue->SetLabel(std::to_string(zAngleVel));

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

double AngleVelocityChartGui::getXgyroBias() const
{
    return xGyroBias;
}

double AngleVelocityChartGui::getYgyroBias() const
{
    return yGyroBias;
}

double AngleVelocityChartGui::getZgyroBias() const
{
    return zGyroBias;
}

void AngleVelocityChartGui::OnResetAngleVelChart(wxCommandEvent& event)
{
    wxLogMessage("Reset angle velocity chart!");
}

void AngleVelocityChartGui::OnSubmitAngleVelAdjustments(wxCommandEvent& event)
{
    const int xAngleVelCtrlValue = spinCtrlXangleVel->GetValue();
    const int yAngleVelCtrlValue = spinCtrlYangleVel->GetValue();
    const int zAngleVelCtrlValue = spinCtrlZangleVel->GetValue();

    xGyroBias = xAngleVelCtrlValue;
    yGyroBias = yAngleVelCtrlValue;
    zGyroBias = zAngleVelCtrlValue;
    wxLogMessage("New gyro adj X:%d Y:%d Z:%d!", xAngleVelCtrlValue, yAngleVelCtrlValue, zAngleVelCtrlValue);
}

void AngleVelocityChartGui::OnSpinXAnglVelIncrUpdate(wxSpinEvent& event)
{
    const int newIncrement = spinCtrlXangleVelMultiplicator->GetValue();
    spinCtrlXangleVel->SetIncrement(newIncrement);
}

void AngleVelocityChartGui::OnSpinXAngleVelUpdate(wxSpinEvent& event)
{
    const int xAngleVelCtrlValue = spinCtrlXangleVel->GetValue();
    xGyroBias = xAngleVelCtrlValue;
}

void AngleVelocityChartGui::OnSpinYAnglVelIncrUpdate(wxSpinEvent& event)
{
    const int newIncrement = spinCtrlYangleVelMultiplicator->GetValue();
    spinCtrlYangleVel->SetIncrement(newIncrement);
}

void AngleVelocityChartGui::OnSpinYAngleVelUpdate(wxSpinEvent& event)
{
    const int yAngleVelCtrlValue = spinCtrlYangleVel->GetValue();
    yGyroBias = yAngleVelCtrlValue;
}

void AngleVelocityChartGui::OnSpinZAnglVelIncrUpdate(wxSpinEvent& event)
{
    const int newIncrement = spinCtrlZangleVelMultiplicator->GetValue();
    spinCtrlZangleVel->SetIncrement(newIncrement);
}

void AngleVelocityChartGui::OnSpinZAngleVelUpdate(wxSpinEvent& event)
{
    const int zAngleVelCtrlValue = spinCtrlZangleVel->GetValue();
    zGyroBias = zAngleVelCtrlValue;
}