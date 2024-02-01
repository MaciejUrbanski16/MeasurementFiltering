#include "CsvMeasurementLoadGui.h"

void CsvMeasurementLoadGui::setup(wxPanel* kalmanParamsSetupPanel)
{

    wxBoxSizer* loadSensorDataSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* loadGpsDataSizer = new wxBoxSizer(wxHORIZONTAL);

    wxBoxSizer* controlPanelSizer = new wxBoxSizer(wxVERTICAL);

	loadSensorsDataFromFileButton = new wxButton(kalmanParamsSetupPanel, wxID_ANY, "Load sensor measurements");
    loadSensorsDataFromFileButton->SetMinSize(wxSize(350, -1));
    loadSensorsDataFromFileButton->Bind(wxEVT_BUTTON, &CsvMeasurementLoadGui::OnLoadSensorData, this);

    loadGpsDataFromFileButton = new wxButton(kalmanParamsSetupPanel, wxID_ANY, "Load GPS data");
    loadGpsDataFromFileButton->SetMinSize(wxSize(350, -1));
    loadGpsDataFromFileButton->Bind(wxEVT_BUTTON, &CsvMeasurementLoadGui::OnLoadGpsData, this);

    filePathSensorDataTextCtrl = new wxTextCtrl(kalmanParamsSetupPanel, wxID_ANY, wxT(""), wxPoint(100, 10), wxSize(500, 30), wxTE_READONLY);
    filePathGpsDataTextCtrl = new wxTextCtrl(kalmanParamsSetupPanel, wxID_ANY, wxT(""), wxPoint(100, 100), wxSize(500, 30), wxTE_READONLY);

    startFiltrationButton = new wxButton(kalmanParamsSetupPanel, wxID_ANY, "Start filtration");
    startFiltrationButton->SetMinSize(wxSize(300, 120));
    startFiltrationButton->Bind(wxEVT_BUTTON, &CsvMeasurementLoadGui::OnStartFiltration, this);

    loadSensorDataSizer->Add(loadSensorsDataFromFileButton, 0, wxALIGN_CENTER_VERTICAL, 5);
    loadSensorDataSizer->Add(filePathSensorDataTextCtrl, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

    loadGpsDataSizer->Add(loadGpsDataFromFileButton, 0, wxALIGN_CENTER_VERTICAL, 5);
    loadGpsDataSizer->Add(filePathGpsDataTextCtrl, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

    controlPanelSizer->AddSpacer(40);
    controlPanelSizer->Add(loadSensorDataSizer, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->Add(loadGpsDataSizer, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->AddSpacer(120);
    controlPanelSizer->Add(startFiltrationButton, 0, wxALL | wxALIGN_CENTER, 5);


    kalmanParamsSetupPanel->SetSizer(controlPanelSizer);
    //kalmanParamsSetupPanel->SetSizer(loadGpsDataSizer);

    openFileDialog = new wxFileDialog(kalmanParamsSetupPanel, _("Wybierz plik"), "", "", "All files (*.*)|*.*", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
        
}

void CsvMeasurementLoadGui::OnLoadSensorData(wxCommandEvent& event)
{
    if (openFileDialog->ShowModal() == wxID_OK) 
    {
        wxString filePath = openFileDialog->GetPath();
        if (isFileExtensionCorrect(filePath, wxT("txt")))
        {
            filePathSensorDataTextCtrl->SetValue(filePath);
            //wxMessageBox("The path for sensor data has CORRECT extension - should be .csv", "Informacja", wxOK | wxICON_INFORMATION);

        }
        else
        {
            wxMessageBox("The path for sensor data has incorrect extension - should be .csv", "Informacja", wxOK | wxICON_INFORMATION);
        }       
    }
}

void CsvMeasurementLoadGui::OnLoadGpsData(wxCommandEvent& event)
{
    if (openFileDialog->ShowModal() == wxID_OK)
    {
        wxString filePath = openFileDialog->GetPath();
        if (isFileExtensionCorrect(filePath, wxT("txt")))
        {
            filePathGpsDataTextCtrl->SetValue(filePath);
            //wxMessageBox("The path for GPS data has CORRECT extension - should be .csv", "Informacja", wxOK | wxICON_INFORMATION);
        }
        else
        {
            wxMessageBox("The path for GPS data has incorrect extension - should be .csv", "Informacja", wxOK | wxICON_INFORMATION);
        }  
    }
}

void CsvMeasurementLoadGui::OnStartFiltration(wxCommandEvent& event)
{
    if (filePathSensorDataTextCtrl->IsEmpty()) 
    {
        wxMessageBox("The path for sensor data was not given", "Informacja", wxOK | wxICON_INFORMATION);
        return;
    }
    else
    {
        sensorDataPathGiven = true;
    }

    if (filePathGpsDataTextCtrl->IsEmpty())
    {
        wxMessageBox("The path for GPS data was not given", "Informacja", wxOK | wxICON_INFORMATION);
        return;
    }
    else
    {
        gpsDataPathGiven = true;
    }

    canFiltrationBeStarted = true;
}

bool CsvMeasurementLoadGui::isFileExtensionCorrect(const wxString& filePath, const wxString& extension) const
{
    if (filePath.Contains("."))
    {
        wxString extensionFromPath = filePath.AfterLast('.');

        if (extensionFromPath == extension)
        {
            return true;
        }
    }
    return false;
}