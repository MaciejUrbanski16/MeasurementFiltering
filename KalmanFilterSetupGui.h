#pragma once
#include <wx/wx.h>
#include <wx/button.h>
#include <wx/panel.h>
#include <wx/grid.h>
#include <wx/sizer.h>

class KalmanFilterSetupGui
{
public:
	void setup(wxPanel* kalmanParamsSetupPanel)
	{
		startFilteringButton = new wxButton(kalmanParamsSetupPanel, wxID_ANY, "Start!");
		startFilteringButton->SetPosition({ 400, 500 });
		startFilteringButton->Bind(wxEVT_BUTTON, &KalmanFilterSetupGui::OnStartFiltrationClick, this);


        // Utw�rz przyciski
        addButton = new wxButton(kalmanParamsSetupPanel, wxID_ANY, wxT("Dodaj wiersz"));
        removeButton = new wxButton(kalmanParamsSetupPanel, wxID_ANY, wxT("Usu� wiersz"));
        addColumnButton = new wxButton(kalmanParamsSetupPanel, wxID_ANY, wxT("Dodaj kolumn�"));
        removeColumnButton = new wxButton(kalmanParamsSetupPanel, wxID_ANY, wxT("Usu� kolumn�"));

        // Utw�rz siatk� (tabel�)
        grid = new wxGrid(kalmanParamsSetupPanel, wxID_ANY);
        grid->CreateGrid(5, 5);  // Ustaw pocz�tkowy rozmiar tabeli

        // Ustaw layout
        sizer = new wxBoxSizer(wxVERTICAL);
        sizer->Add(addButton, 0, wxALL, 5);
        sizer->Add(removeButton, 0, wxALL, 5);
        sizer->Add(addColumnButton, 0, wxALL, 5);
        sizer->Add(removeColumnButton, 0, wxALL, 5);
        sizer->Add(grid, 1, wxEXPAND | wxALL, 5);

        kalmanParamsSetupPanel->SetSizer(sizer);
        //Fit();

        // Po��cz przyciski z metodami obs�uguj�cymi zdarzenia
        addButton->Bind(wxEVT_BUTTON, &KalmanFilterSetupGui::OnAddRow, this);
        removeButton->Bind(wxEVT_BUTTON, &KalmanFilterSetupGui::OnRemoveRow, this);
        addColumnButton->Bind(wxEVT_BUTTON, &KalmanFilterSetupGui::OnAddColumn, this);
        removeColumnButton->Bind(wxEVT_BUTTON, &KalmanFilterSetupGui::OnRemoveColumn, this);

	}

	void OnStartFiltrationClick(wxCommandEvent& event)
	{
		wxLogMessage("Data reception starts - new thread will be created!");
	}
    // Metoda obs�uguj�ca dodawanie wierszy
    void OnAddRow(wxCommandEvent& event) {
        if (grid != nullptr)
        {
            grid->AppendRows(1);
        }
        
    }

    // Metoda obs�uguj�ca usuwanie wierszy
    void OnRemoveRow(wxCommandEvent& event) {
        if (grid != nullptr)
        {
            int numRows = grid->GetNumberRows();
            if (numRows > 0) {
                grid->DeleteRows(numRows - 1);  // Usu� ostatni wiersz
            }
        }
    }

    // Metoda obs�uguj�ca dodawanie kolumn
    void OnAddColumn(wxCommandEvent& event) {
        grid->AppendCols(1);
    }

    // Metoda obs�uguj�ca usuwanie kolumn
    void OnRemoveColumn(wxCommandEvent& event) {
        if (grid != nullptr)
        {
            int numCols = grid->GetNumberCols();
            if (numCols > 0) {
                grid->DeleteCols(numCols - 1);  // Usu� ostatni� kolumn�
            }
        }
    }
private:
	wxButton* startFilteringButton = nullptr;

    wxButton* addButton = nullptr;
    wxButton* removeButton = nullptr;
    wxButton* addColumnButton = nullptr;
    wxButton* removeColumnButton = nullptr;
    wxGrid* grid = nullptr;
    wxBoxSizer* sizer = nullptr;
};

