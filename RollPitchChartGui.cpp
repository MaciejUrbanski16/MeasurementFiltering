#include "RollPitchChartGui.h"

void RollPitchChartGui::setup(wxNotebook* m_notebook)
{
	wxPanel* panel = new wxPanel(m_notebook, wxID_ANY);
	rollPitchPanelSplitter = new wxSplitterWindow(panel, wxID_ANY);
	wxPanel* controlPanel = new wxPanel(rollPitchPanelSplitter, wxID_ANY);

	rollPitchChartPanel = new wxChartPanel(rollPitchPanelSplitter);

	sizerRollPitchPlot = new wxBoxSizer(wxVERTICAL);
	rollPitchChartPanel->SetMinSize(wxSize(800, 600));

	wxBoxSizer* controlPanelSizer = new wxBoxSizer(wxVERTICAL);

	//spinCtrlXmagn = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -33000, 33000, 100);
	//wxStaticText* xMagnText = new wxStaticText(controlPanel, wxID_ANY, "Adjust X magn");

	//spinCtrlYmagn = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -33000, 33000, 100);
	//wxStaticText* yMagnText = new wxStaticText(controlPanel, wxID_ANY, "Adjust Y magn");

	//controlPanelSizer->Add(xMagnText, 0, wxALL | wxALIGN_CENTER, 5);
	//controlPanelSizer->Add(spinCtrlXmagn, 0, wxALL | wxALIGN_CENTER, 5);

	//controlPanelSizer->Add(yMagnText, 0, wxALL | wxALIGN_CENTER, 5);
	//controlPanelSizer->Add(spinCtrlYmagn, 0, wxALL | wxALIGN_CENTER, 5);

	wxBoxSizer* azimuthSetupButtonsSizer = new wxBoxSizer(wxHORIZONTAL);

	wxButton* resetButton = new wxButton(controlPanel, wxID_ANY, "Reset chart");
	azimuthSetupButtonsSizer->Add(resetButton, 0, wxALL | wxALIGN_LEFT, 5);
	//resetButton->Bind(wxEVT_BUTTON, &MagnChartGui::OnResetMagnChart, this);

	wxButton* submitButton = new wxButton(controlPanel, wxID_ANY, "Submit adjustments");
	azimuthSetupButtonsSizer->Add(submitButton, 0, wxALL | wxALIGN_LEFT, 5);
	//submitButton->Bind(wxEVT_BUTTON, &MagnChartGui::OnSubmitMagnAdjustments, this);

	wxBoxSizer* rollLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* pitchLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* yawLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
	//wxBoxSizer* orientationSizer = new wxBoxSizer(wxHORIZONTAL);
	//wxBoxSizer* checkBoxSizer = new wxBoxSizer(wxVERTICAL);

	wxStaticText* rollName = new wxStaticText(controlPanel, wxID_ANY, "Roll: ");
	wxStaticText* pitchName = new wxStaticText(controlPanel, wxID_ANY, "Pitch: ");
	wxStaticText* yawName = new wxStaticText(controlPanel, wxID_ANY, "Yaw: ");
	//wxStaticText* orientationName = new wxStaticText(controlPanel, wxID_ANY, "Orientation [deg]: ");
	rollValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	pitchValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	yawValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	//orientationValue = new wxStaticText(controlPanel, wxID_ANY, "0");

	//rawAzimuthCheckbox = new wxCheckBox(controlPanel, wxID_ANY, wxT("Plot raw azimuth"));
	//rawAzimuthCheckbox->SetValue(true);
	//rawAzimuthCheckbox->Bind(wxEVT_CHECKBOX, &MagnChartGui::OnRawAzimuthCheckBoxClicked, this);

	//filteredAzimuthCheckbox = new wxCheckBox(controlPanel, wxID_ANY, wxT("Plot filtered azimuth"));
	//filteredAzimuthCheckbox->SetValue(true);
	//filteredAzimuthCheckbox->Bind(wxEVT_CHECKBOX, &MagnChartGui::OnFilteredAzimuthCheckBoxClicked, this);

	rollLabelsSizer->Add(rollName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	rollLabelsSizer->Add(rollValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(rollLabelsSizer, 0, wxALL, 5);

	pitchLabelsSizer->Add(pitchName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	pitchLabelsSizer->Add(pitchValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(pitchLabelsSizer, 0, wxALL, 5);

	yawLabelsSizer->Add(yawName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	yawLabelsSizer->Add(yawValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(yawLabelsSizer, 0, wxALL, 5);

	//orientationSizer->Add(orientationName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	//orientationSizer->Add(orientationValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	//controlPanelSizer->Add(orientationSizer, 0, wxALL, 5);

	controlPanelSizer->Add(azimuthSetupButtonsSizer, 0, wxALL, 5);

	controlPanelSizer->AddSpacer(10);

	//checkBoxSizer->Add(rawAzimuthCheckbox, 0, wxALL, 5);
	//checkBoxSizer->Add(filteredAzimuthCheckbox, 0, wxALL, 5);
	//controlPanelSizer->Add(checkBoxSizer, 0, wxALL, 5);

	controlPanel->SetSizer(controlPanelSizer);

	rollPitchPanelSplitter->SplitVertically(rollPitchChartPanel, controlPanel);
	sizerRollPitchPlot->Add(rollPitchPanelSplitter, 1, wxEXPAND | wxALL, 5);
	panel->SetSizer(sizerRollPitchPlot);

	m_notebook->AddPage(panel, "Roll/pitch/yaw");
}

void RollPitchChartGui::updateChart(const MeasurementsController& rawMeasurement,
	PlotElementsBuffer& rollBasedOnAccBuffer, PlotElementsBuffer& pitchBasedOnAccBuffer, PlotElementsBuffer& magnPointsBuffer,
	PlotElementsBuffer& rollBuffer, PlotElementsBuffer& pitchBuffer, PlotElementsBuffer& yawBuffer,
	const double rollVal, const double pitchVal, const double yawVal, const double timeMs)
{
	rollValue->SetLabel(std::to_string(rollVal));
	pitchValue->SetLabel(std::to_string(pitchVal));
	yawValue->SetLabel(std::to_string(yawVal));

	//rollBuffer.AddElement(wxRealPoint(timeMs, rollVal));
	//pitchBuffer.AddElement(wxRealPoint(timeMs, pitchVal));
	//rollBuffer.AddElement(wxRealPoint(timeMs, rollVal));
	//pitchBuffer.AddElement(wxRealPoint(timeMs, pitchVal));
	//yawBuffer.AddElement(wxRealPoint(timeMs, yawVal));

	rollBasedOnAccBuffer.AddElement(wxRealPoint(timeMs, rawMeasurement.getRollFromAcc() * (360.0 / (2.0* M_PI))));
	pitchBasedOnAccBuffer.AddElement(wxRealPoint(timeMs, rawMeasurement.getPitchFromAcc() * (360.0 / (2.0 * M_PI))));

	XYPlot* plot = new XYPlot();
	XYSimpleDataset* dataset = new XYSimpleDataset();

	//dataset->AddSerie(new XYSerie(rollBuffer.getBuffer()));
	//dataset->AddSerie(new XYSerie(pitchBuffer.getBuffer()));
	//dataset->AddSerie(new XYSerie(yawBuffer.getBuffer()));
	dataset->AddSerie(new XYSerie(rollBasedOnAccBuffer.getBuffer()));
	dataset->AddSerie(new XYSerie(pitchBasedOnAccBuffer.getBuffer()));
	dataset->AddSerie(new XYSerie(magnPointsBuffer.getBuffer()));
	//dataset->GetSerie(0)->SetName("roll");
	//dataset->GetSerie(1)->SetName("pitch");
	//dataset->GetSerie(2)->SetName("yaw");
	dataset->GetSerie(0)->SetName("roll based on acc");
	dataset->GetSerie(1)->SetName("pitch based on acc");
	dataset->GetSerie(2)->SetName("yaw based on magn");

	dataset->SetRenderer(new XYLineRenderer());
	NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
	NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
	leftAxis->SetTitle(wxT("Angle"));
	bottomAxis->SetTitle(wxT("Time [ms]"));
	if (rollBasedOnAccBuffer.getBuffer().size() >= 100)
	{
		bottomAxis->SetFixedBounds(rollBasedOnAccBuffer.getBuffer()[0].x, rollBasedOnAccBuffer.getBuffer()[99].x);
	}
	Legend* legend = new Legend(wxTOP, wxRIGHT);
	plot->SetLegend(legend);
	plot->AddObjects(dataset, leftAxis, bottomAxis);

	Chart* chart = new Chart(plot, "Roll/pitch/yaw");
	rollPitchChartPanel->SetChart(chart);
}
