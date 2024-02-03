#include "MagnChartGui.h"

void MagnChartGui::setup(wxNotebook* m_notebook /*, MyWindow* window*/)
{
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
	resetButton->Bind(wxEVT_BUTTON, &MagnChartGui::OnResetMagnChart, this);

	wxButton* submitButton = new wxButton(controlPanel, wxID_ANY, "Submit adjustments");
	azimuthSetupButtonsSizer->Add(submitButton, 0, wxALL | wxALIGN_LEFT, 5);
	submitButton->Bind(wxEVT_BUTTON, &MagnChartGui::OnSubmitMagnAdjustments, this);

	wxBoxSizer* xMagnLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* yMagnLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* orientationSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* checkBoxSizer = new wxBoxSizer(wxVERTICAL);

	wxStaticText* xMagnName = new wxStaticText(controlPanel, wxID_ANY, "Current X magn: ");
	wxStaticText* yMagnName = new wxStaticText(controlPanel, wxID_ANY, "Current Y magn: ");
	wxStaticText* orientationName = new wxStaticText(controlPanel, wxID_ANY, "Orientation [deg]: ");
	xMagnValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	yMagnValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	orientationValue = new wxStaticText(controlPanel, wxID_ANY, "0");

	rawAzimuthCheckbox = new wxCheckBox(controlPanel, wxID_ANY, wxT("Plot raw azimuth"));
	rawAzimuthCheckbox->SetValue(true);
	rawAzimuthCheckbox->Bind(wxEVT_CHECKBOX, &MagnChartGui::OnRawAzimuthCheckBoxClicked, this);

	filteredAzimuthCheckbox = new wxCheckBox(controlPanel, wxID_ANY, wxT("Plot filtered azimuth"));
	filteredAzimuthCheckbox->SetValue(true);
	filteredAzimuthCheckbox->Bind(wxEVT_CHECKBOX, &MagnChartGui::OnFilteredAzimuthCheckBoxClicked, this);

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

	checkBoxSizer->Add(rawAzimuthCheckbox, 0, wxALL, 5);
	checkBoxSizer->Add(filteredAzimuthCheckbox, 0, wxALL, 5);
	controlPanelSizer->Add(checkBoxSizer, 0, wxALL, 5);

	controlPanel->SetSizer(controlPanelSizer);

	azimuthPanelSplitter->SplitVertically(azimuthChartPanel, controlPanel);
	sizerAzimuthPlot->Add(azimuthPanelSplitter, 1, wxEXPAND | wxALL, 5);
	panel->SetSizer(sizerAzimuthPlot);

	m_notebook->AddPage(panel, "Azimuth");
}

void MagnChartGui::updateChart(PlotElementsBuffer& magnPointsBuffer, PlotElementsBuffer& filteredAzimuthBuffer, 
	const int16_t xMagn, const int16_t yMagn, const double azimuth, const double filteredAzimuth, const double timeMs)
{
	xMagnValue->SetLabel(std::to_string(xMagn));
	yMagnValue->SetLabel(std::to_string(yMagn));
	orientationValue->SetLabel(std::to_string(azimuth));

	magnPointsBuffer.AddElement(wxRealPoint(timeMs, azimuth));
	filteredAzimuthBuffer.AddElement(wxRealPoint(timeMs, filteredAzimuth));

	XYPlot* plot = new XYPlot();
	XYSimpleDataset* dataset = new XYSimpleDataset();
	if (plotFilteredAzimuth and plotRawAzimuth)
	{
		dataset->AddSerie(new XYSerie(magnPointsBuffer.getBuffer()));
		dataset->AddSerie(new XYSerie(filteredAzimuthBuffer.getBuffer()));
		dataset->GetSerie(0)->SetName("Raw azimuth");
		dataset->GetSerie(1)->SetName("Filtered azimuth");
	}
	else if (plotFilteredAzimuth and not plotRawAzimuth)
	{
		dataset->AddSerie(new XYSerie(filteredAzimuthBuffer.getBuffer()));
		dataset->GetSerie(0)->SetName("Filtered azimuth");
	}
	else if(not plotFilteredAzimuth and plotRawAzimuth)
	{
		dataset->AddSerie(new XYSerie(magnPointsBuffer.getBuffer()));
		dataset->GetSerie(0)->SetName("Raw azimuth");
	}

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
	Legend* legend = new Legend(wxTOP, wxLEFT);
	plot->SetLegend(legend);
	plot->AddObjects(dataset, leftAxis, bottomAxis);

	Chart* chart = new Chart(plot, "Magnetometr");

	azimuthChartPanel->SetChart(chart);
}


void MagnChartGui::OnResetMagnChart(wxCommandEvent& event)
{
	wxMessageBox("OnResetMagnChart was clicked", "Informacja", wxOK | wxICON_INFORMATION);
}

void MagnChartGui::OnSubmitMagnAdjustments(wxCommandEvent& event)
{
	wxMessageBox("OnSubmitMagnAdjustments was clicked", "Informacja", wxOK | wxICON_INFORMATION);
}

void MagnChartGui::OnRawAzimuthCheckBoxClicked(wxCommandEvent& event)
{
	if (rawAzimuthCheckbox->IsChecked())
	{
		plotRawAzimuth = true;
		wxMessageBox("Raw azimuth checkbox zosta³ zaznaczony!", "Informacja", wxOK | wxICON_INFORMATION, this);
	}
	else
	{
		plotRawAzimuth = false;
		wxMessageBox("Raw azimuth checkbox zosta³ odznaczony!", "Informacja", wxOK | wxICON_INFORMATION, this);
	}
}

void MagnChartGui::OnFilteredAzimuthCheckBoxClicked(wxCommandEvent& event)
{
	if (filteredAzimuthCheckbox->IsChecked())
	{
		plotFilteredAzimuth = true;
		wxMessageBox("Filtered azimuth checkbox zosta³ zaznaczony!", "Informacja", wxOK | wxICON_INFORMATION, this);
	}
	else
	{
		plotFilteredAzimuth = false;
		wxMessageBox("Filtered azimuth checkbox zosta³ odznaczony!", "Informacja", wxOK | wxICON_INFORMATION, this);
	}
}