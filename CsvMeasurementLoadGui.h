#pragma once

#include <wx/wx.h>
#include <wx/button.h>
#include <wx/panel.h>
#include <wx/grid.h>
#include <wx/sizer.h>

class CsvMeasurementLoadGui
{
public:
	void setup(wxPanel* kalmanParamsSetupPanel);

private:
	void OnLoadSensorData(wxCommandEvent& event);
	void OnLoadGpsData(wxCommandEvent& event);
	void OnStartFiltration(wxCommandEvent& event);

	bool isFileExtensionCorrect(const wxString& filePath, const wxString& extension) const;

	wxButton* loadSensorsDataFromFileButton = nullptr;
	wxButton* loadGpsDataFromFileButton = nullptr;
	wxButton* startFiltrationButton = nullptr;

	wxTextCtrl* filePathSensorDataTextCtrl = nullptr;
	wxTextCtrl* filePathGpsDataTextCtrl = nullptr;
	wxFileDialog* openFileDialog = nullptr;

	bool sensorDataPathGiven{ false };
	bool gpsDataPathGiven{ false };
	bool canFiltrationBeStarted{ false };


};

