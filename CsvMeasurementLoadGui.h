#pragma once

#include <wx/wx.h>
#include <wx/button.h>
#include <wx/panel.h>
#include <wx/grid.h>
#include <wx/sizer.h>
#include "CsvMeasurementReader.h"

class CsvMeasurementLoadGui
{
public:
	CsvMeasurementLoadGui(CsvMeasurementReader& csvMeasurementReader, wxTimer& filterFileMeasTimer, wxTimer& filterFileGpsTimer) :
		csvMeasurementReader(csvMeasurementReader), filterFileMeasTimer(filterFileMeasTimer), filterFileGpsTimer(filterFileGpsTimer)
	{}

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

	wxTimer& filterFileMeasTimer;
	wxTimer& filterFileGpsTimer;
	CsvMeasurementReader& csvMeasurementReader;

	bool sensorDataPathGiven{ false };
	bool gpsDataPathGiven{ false };
	bool canFiltrationBeStarted{ false };
};

