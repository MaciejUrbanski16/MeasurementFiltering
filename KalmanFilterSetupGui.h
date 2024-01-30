#pragma once
#include <wx/wx.h>
#include <wx/button.h>
#include <wx/panel.h>
#include <wx/grid.h>
#include <wx/sizer.h>

enum class MovementModel
{
    PEDESTRIAN = 0,
    RC_CAR,
    CAR,
    NONE
};

class KalmanFilterSetupGui
{
public:
	void setup(wxPanel* kalmanParamsSetupPanel)
	{
		startFilteringButton = new wxButton(kalmanParamsSetupPanel, wxID_ANY, "Start!");
		startFilteringButton->SetPosition({ 400, 500 });
		startFilteringButton->Bind(wxEVT_BUTTON, &KalmanFilterSetupGui::OnStartFiltrationClick, this);


        wxStaticText* label = new wxStaticText(kalmanParamsSetupPanel, wxID_ANY, "KALIBRACJA CZUJNIKÓW");
        wxStaticText* chooseModel = new wxStaticText(kalmanParamsSetupPanel, wxID_ANY, "Model ruchu:");
        

        // Utwórz przyciski
        addButton = new wxButton(kalmanParamsSetupPanel, wxID_ANY, wxT("Dodaj wiersz"));
        removeButton = new wxButton(kalmanParamsSetupPanel, wxID_ANY, wxT("Usuñ wiersz"));
        addColumnButton = new wxButton(kalmanParamsSetupPanel, wxID_ANY, wxT("Dodaj kolumnê"));
        removeColumnButton = new wxButton(kalmanParamsSetupPanel, wxID_ANY, wxT("Usuñ kolumnê"));

        confirmCallibrationButton = new wxButton(kalmanParamsSetupPanel, wxID_ANY, wxT("ZatwierdŸ kalibracjê"));
        confirmCallibrationButton->SetSize({30,50});
        confirmCallibrationButton->SetBackgroundColour(wxColour(255, 0, 0)); // Ustaw czerwony kolor t³a

        pedestrianModelButton = new wxRadioButton(kalmanParamsSetupPanel, wxID_ANY, wxT("Pedestrian model"), wxDefaultPosition, wxDefaultSize, wxRB_GROUP);
        rcCarModelButton = new wxRadioButton(kalmanParamsSetupPanel, wxID_ANY, wxT("RC car model"));
        carModelButton = new wxRadioButton(kalmanParamsSetupPanel, wxID_ANY, wxT("Car model"));


        // Utwórz siatkê (tabelê)
        grid = new wxGrid(kalmanParamsSetupPanel, wxID_ANY);
        grid->CreateGrid(5, 5);  // Ustaw pocz¹tkowy rozmiar tabeli

        // Ustaw layout
        sizer = new wxBoxSizer(wxVERTICAL);
        sizer->Add(label, 0, wxALL | wxALIGN_CENTER, 5);
        sizer->Add(chooseModel, 0, wxALL | wxALIGN_LEFT, 5);
        sizer->Add(pedestrianModelButton, 0, wxALL | wxALIGN_LEFT, 5);
        sizer->Add(rcCarModelButton, 0, wxALL | wxALIGN_LEFT, 5);
        sizer->Add(carModelButton, 0, wxALL | wxALIGN_LEFT, 5);
        sizer->Add(confirmCallibrationButton, 0, wxALL | wxALIGN_CENTER, 5);

        sizer->Add(addButton, 0, wxALL, 5);
        sizer->Add(removeButton, 0, wxALL, 5);
        sizer->Add(addColumnButton, 0, wxALL, 5);
        sizer->Add(removeColumnButton, 0, wxALL, 5);
        sizer->Add(grid, 1, wxEXPAND | wxALL, 5);

        kalmanParamsSetupPanel->SetSizer(sizer);
        //Fit();

        // Po³¹cz przyciski z metodami obs³uguj¹cymi zdarzenia
        addButton->Bind(wxEVT_BUTTON, &KalmanFilterSetupGui::OnAddRow, this);
        removeButton->Bind(wxEVT_BUTTON, &KalmanFilterSetupGui::OnRemoveRow, this);
        addColumnButton->Bind(wxEVT_BUTTON, &KalmanFilterSetupGui::OnAddColumn, this);
        removeColumnButton->Bind(wxEVT_BUTTON, &KalmanFilterSetupGui::OnRemoveColumn, this);
        confirmCallibrationButton->Bind(wxEVT_BUTTON, &KalmanFilterSetupGui::OnConfirmCallibration, this);

	}

    bool getIsCallibrationDone() const
    {
        return isCallibrationDone;
    }

    MovementModel getMovementModel() const
    {
        return movementModel;
    }

    void OnConfirmCallibration(wxCommandEvent& event)
    {
        isCallibrationDone = true;

        if (pedestrianModelButton->GetValue())
        {
            movementModel = MovementModel::PEDESTRIAN;
            wxMessageBox(wxT("Model ruchu pieszego zosta³ wybrany"));
        }
        else if (rcCarModelButton->GetValue())
        {
            movementModel = MovementModel::RC_CAR;
            wxMessageBox(wxT("Model ruchu RC samochó zaosta³ wybrany"));
        }
        else if (carModelButton->GetValue())
        {
            movementModel = MovementModel::CAR;
            wxMessageBox(wxT("Model ruchu pojazd samochodowy zosta³ wybrany"));
        }
        else 
        {
            movementModel = MovementModel::NONE;
            wxMessageBox(wxT("¯aden z przycisków nie zosta³ zaznaczony"));
        }

        wxLogMessage("Kalibracja zakoñczona - zbieranie pomiarów rozpoczêto.");
    }

	void OnStartFiltrationClick(wxCommandEvent& event)
	{
		wxLogMessage("Data reception starts - new thread will be created!");
	}
    // Metoda obs³uguj¹ca dodawanie wierszy
    void OnAddRow(wxCommandEvent& event) {
        if (grid != nullptr)
        {
            grid->AppendRows(1);
        }
        
    }

    // Metoda obs³uguj¹ca usuwanie wierszy
    void OnRemoveRow(wxCommandEvent& event) {
        if (grid != nullptr)
        {
            int numRows = grid->GetNumberRows();
            if (numRows > 0) {
                grid->DeleteRows(numRows - 1);  // Usuñ ostatni wiersz
            }
        }
    }

    // Metoda obs³uguj¹ca dodawanie kolumn
    void OnAddColumn(wxCommandEvent& event) {
        grid->AppendCols(1);
    }

    // Metoda obs³uguj¹ca usuwanie kolumn
    void OnRemoveColumn(wxCommandEvent& event) {
        if (grid != nullptr)
        {
            int numCols = grid->GetNumberCols();
            if (numCols > 0) {
                grid->DeleteCols(numCols - 1);  // Usuñ ostatni¹ kolumnê
            }
        }
    }
private:
	wxButton* startFilteringButton = nullptr;

    wxButton* addButton = nullptr;
    wxButton* removeButton = nullptr;
    wxButton* addColumnButton = nullptr;
    wxButton* removeColumnButton = nullptr;
    wxButton* confirmCallibrationButton = nullptr;
    wxGrid* grid = nullptr;
    wxBoxSizer* sizer = nullptr;

    wxRadioButton* pedestrianModelButton;
    wxRadioButton* rcCarModelButton;
    wxRadioButton* carModelButton;

    MovementModel movementModel{ MovementModel::NONE };
    bool isCallibrationDone = false;
};

