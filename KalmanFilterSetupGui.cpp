#include "KalmanFilterSetupGui.h"

void KalmanFilterSetupGui::setupLeftPanel(wxPanel* mainPanel)
{
    wxBoxSizer* leftVerticalSizer = new wxBoxSizer(wxVERTICAL);
    leftPanel = new wxPanel(mainPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxBORDER_SIMPLE);

    chooseModel = new wxStaticText(leftPanel, wxID_ANY, "Model ruchu:");

    pedestrianModelButton = new wxRadioButton(leftPanel, wxID_ANY, wxT("Pedestrian model"), wxDefaultPosition, wxDefaultSize, wxRB_GROUP);
    rcCarModelButton = new wxRadioButton(leftPanel, wxID_ANY, wxT("RC car model"));
    carModelButton = new wxRadioButton(leftPanel, wxID_ANY, wxT("Car model"));

    pedestrianModelButton->Bind(wxEVT_RADIOBUTTON, &KalmanFilterSetupGui::OnRadioButtonClicked, this);
    rcCarModelButton->Bind(wxEVT_RADIOBUTTON, &KalmanFilterSetupGui::OnRadioButtonClicked, this);
    carModelButton->Bind(wxEVT_RADIOBUTTON, &KalmanFilterSetupGui::OnRadioButtonClicked, this);


    confirmCallibrationButton = new wxButton(leftPanel, wxID_ANY, wxT("ZatwierdŸ kalibracjê"));
    confirmCallibrationButton->SetSize({ 60,100 });
    confirmCallibrationButton->SetBackgroundColour(wxColour(128, 0, 64));

    wxBoxSizer* mainSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* leftSizer = new wxBoxSizer(wxVERTICAL);
    wxBoxSizer* rightSizer = new wxBoxSizer(wxVERTICAL);

    stateVectorAzimuthName = new wxStaticText(leftPanel, wxID_ANY, "Wektor stanu X dla rotacji");
    createAndFillStateVectorRotationGrid();

    stateVectorPositionName = new wxStaticText(leftPanel, wxID_ANY, "Wektor stanu X dla pozycji");
    createAndFillStateVectorPositionGrid();

    matrixHAzimuthName = new wxStaticText(leftPanel, wxID_ANY, "Macierz H dla rotacji");
    createAndFillMatrixHRotationGrid();

    matrixHPositionName = new wxStaticText(leftPanel, wxID_ANY, "Macierz H dla pozycji");
    createAndFillMatrixHPositionGrid();

    leftSizer->Add(stateVectorAzimuthName, 0, wxALL | wxALIGN_CENTER, 5);
    leftSizer->Add(stateVectorAzimuthGrid, 0, wxALL | wxALIGN_CENTER, 5);

    leftSizer->Add(matrixHAzimuthName, 0, wxALL | wxALIGN_CENTER, 5);
    leftSizer->Add(matrixHAzimuthGrid, 0, wxALL | wxALIGN_CENTER, 5);

    rightSizer->Add(stateVectorPositionName, 0, wxALL | wxALIGN_CENTER, 5);
    rightSizer->Add(stateVectorPositionGrid, 0, wxALL | wxALIGN_CENTER, 5);

    rightSizer->Add(matrixHPositionName, 0, wxALL | wxALIGN_CENTER, 5);
    rightSizer->Add(matrixHPositionGrid, 0, wxALL | wxALIGN_CENTER, 5);

    mainSizer->Add(leftSizer, 1, wxALL, 5);
    mainSizer->Add(rightSizer, 1, wxALL, 5);

    leftVerticalSizer->Add(chooseModel, 0, wxALL, 5);
    leftVerticalSizer->Add(pedestrianModelButton, 0, wxALL, 5);
    leftVerticalSizer->Add(rcCarModelButton, 0, wxALL, 5);
    leftVerticalSizer->Add(carModelButton, 0, wxALL, 5);
    leftVerticalSizer->AddSpacer(10);
    leftVerticalSizer->Add(mainSizer, 1, wxALL, 5);
 

    leftVerticalSizer->Add(confirmCallibrationButton, 0, wxALL | wxALIGN_CENTER, 5);

    confirmCallibrationButton->Bind(wxEVT_BUTTON, &KalmanFilterSetupGui::OnConfirmCallibration, this);

    leftPanel->SetSizer(leftVerticalSizer);
    leftPanel->Layout();
}

void KalmanFilterSetupGui::setupRightPanel(wxPanel* mainPanel)
{
    rightVerticalSizer = new wxBoxSizer(wxVERTICAL);
    
    rightPanel = new wxPanel(mainPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxBORDER_SIMPLE);
    rightPanel->SetBackgroundColour(*wxLIGHT_GREY);

    matrixRNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla k¹tu obrotu ruchu pieszego");
    matrixQNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla k¹tu obrotu ruchu pieszego");

    matrixRNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla po³o¿enia w ruchu pieszego");
    matrixQNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla po³o¿enia w ruchu pieszego");

    createGrids();

    fillMatRPedestrianAzimuth();
    fillMatQPedestrianAzimuth();
    fillMatRPedestrianAcc();
    fillMatQPedestrianAcc();

    setupSizer();
}

void KalmanFilterSetupGui::OnRadioButtonClicked(wxCommandEvent& event) 
{
    const int selectedId = event.GetId();
    if (isCallibrationDone)
    {
        wxLogMessage(wxT("Kalibracja zosta³a ju¿ zakoñczona - filtracja danych w toku"));
        return;
    }

    if (selectedId == pedestrianModelButton->GetId()) 
    {
        rightVerticalSizer->Clear();
        destroyWidgets();

        matrixRNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla k¹tu obrotu ruchu pieszego");
        matrixQNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla k¹tu obrotu ruchu pieszego");

        matrixRNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla po³o¿enia w ruchu pieszego");
        matrixQNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla po³o¿enia w ruchu pieszego");

        createGrids();

        fillMatRPedestrianAzimuth();
        fillMatQPedestrianAzimuth();
        fillMatRPedestrianAcc();
        fillMatQPedestrianAcc();

        setupSizer();

        wxLogMessage(wxT("Wybrano model ruchu pieszego."));
    }
    else if (selectedId == rcCarModelButton->GetId()) 
    {
        rightVerticalSizer->Clear();
        destroyWidgets();

        matrixRNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla k¹tu obrotu ruchu pojazdu zdalnie sterowanego");
        matrixQNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla k¹tu obrotu ruchu pojazdu zdalnie sterowanego");

        matrixRNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla po³o¿enia w ruchu pojazdu zdalnie sterowanego");
        matrixQNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla po³o¿enia ruchu pojazdu zdalnie sterowanego");

        createGrids();

        fillMatRPedestrianAzimuth();
        fillMatQPedestrianAzimuth();
        fillMatRPedestrianAcc();
        fillMatQPedestrianAcc();

        setupSizer();

        wxLogMessage(wxT("Wybrano model ruchu pojazdu zdalnie sterowanego."));
    }
    else if (selectedId == carModelButton->GetId()) 
    {
        rightVerticalSizer->Clear();
        destroyWidgets();

        matrixRNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla k¹tu obrotu ruchu pojazdu samochodowego");
        matrixQNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla k¹tu obrotu ruchu pojazdu samochodowego");

        matrixRNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla po³o¿enia w ruchu pojazdu samochodowego");
        matrixQNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla po³o¿enia w ruchu pojazdu samochodowego");

        createGrids();

        fillMatRPedestrianAzimuth();
        fillMatQPedestrianAzimuth();
        fillMatRPedestrianAcc();
        fillMatQPedestrianAcc();

        setupSizer();

        wxLogMessage(wxT("Wybrano model ruchu pojazdu samochodowego."));
    }
    else
    {
        wxLogMessage(wxT("Nieprawid³owy model ruchu"));
    }
}

void KalmanFilterSetupGui::createWidgetsForAcc()
{
    matrixRNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla po³o¿enia w ruchu pieszego");

    matrixRforAccFilterPedestrianModel = new wxGrid(rightPanel, wxID_ANY);
    matrixRforAccFilterPedestrianModel->HideRowLabels();
    matrixRforAccFilterPedestrianModel->HideColLabels();
    matrixRforAccFilterPedestrianModel->CreateGrid(3, 3);

    matrixRforAccFilterPedestrianModel->Show();

    matrixQNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla po³o¿enia w ruchu pieszego");

    matrixQforAccFilterPedestrianModel = new wxGrid(rightPanel, wxID_ANY);
    matrixQforAccFilterPedestrianModel->HideRowLabels();
    matrixQforAccFilterPedestrianModel->HideColLabels();
    matrixQforAccFilterPedestrianModel->CreateGrid(6, 6);

    matrixQforAccFilterPedestrianModel->Show();
}

void KalmanFilterSetupGui::fillMatRPedestrianAzimuth()
{
    if (matrixRforAzimuthFilterPedestrianModel)
    {
        kf::Matrix<DIM_Z_azimuth, DIM_Z_azimuth> matR;
        matR << 0.0001F, 0, 0,
            0, 0.1F, 0,
            0, 0, 0.1F;


        const int rows = matR.rows();
        const int cols = matR.cols();
        matrixRforAzimuthFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixRforAzimuthFilterPedestrianModel->SetCellValue
                    (row, col, wxString::Format("%.4f", matR(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::fillMatQPedestrianAzimuth()
{
    if (matrixQforAzimuthFilterPedestrianModel)
    {
        kf::Matrix<DIM_X_azimuth, DIM_X_azimuth> Q;

        double process_variance = 0.02F;
        double deltaTimeMs = 0.1F;
        Q << pow(deltaTimeMs, 6) / 36, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 6, 0, 0, 0,
            pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 4, pow(deltaTimeMs, 3) / 2, 0, 0, 0,
            pow(deltaTimeMs, 4) / 6, pow(deltaTimeMs, 3) / 2, pow(deltaTimeMs, 2), 0, 0, 0,
            0, 0, 0, pow(deltaTimeMs, 6) / 36, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 6,
            0, 0, 0, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 4, pow(deltaTimeMs, 3) / 2,
            0, 0, 0, pow(deltaTimeMs, 4) / 6, pow(deltaTimeMs, 3) / 2, pow(deltaTimeMs, 2);

        Q *= process_variance;

        int rows = Q.rows();
        int cols = Q.cols();
        matrixQforAzimuthFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixQforAzimuthFilterPedestrianModel->SetCellValue(row, col, wxString::Format("%.6f", Q(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::fillMatRPedestrianAcc()
{
    if (matrixRforAccFilterPedestrianModel)
    {
        kf::Matrix<DIM_Z, DIM_Z> matR;
        matR << 1.0F, 0.0F,
                0.0F, 1.0F;


        const int rows = matR.rows();
        const int cols = matR.cols();
        matrixRforAccFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixRforAccFilterPedestrianModel->SetCellValue
                (row, col, wxString::Format("%.4f", matR(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::fillMatQPedestrianAcc()
{
    if (matrixQforAccFilterPedestrianModel)
    {
        double process_variance = 0.002F;
        double deltaTimeMs = 100.0F;
        deltaTimeMs = deltaTimeMs / 100000000.0F;


        kf::Matrix<DIM_X, DIM_X> matQ;

        matQ << pow(deltaTimeMs, 6) / 36, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 6, 0, 0, 0,
            pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 4, pow(deltaTimeMs, 3) / 2, 0, 0, 0,
            pow(deltaTimeMs, 4) / 6, pow(deltaTimeMs, 3) / 2, pow(deltaTimeMs, 2), 0, 0, 0,
            0, 0, 0, pow(deltaTimeMs, 6) / 36, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 6,
            0, 0, 0, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 4, pow(deltaTimeMs, 3) / 2,
            0, 0, 0, pow(deltaTimeMs, 4) / 6, pow(deltaTimeMs, 3) / 2, pow(deltaTimeMs, 2);

        matQ *= process_variance;

        int rows = matQ.rows();
        int cols = matQ.cols();
        matrixQforAccFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixQforAccFilterPedestrianModel->SetCellValue(row, col, wxString::Format("%.6f", matQ(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::destroyWidgets()
{
    if (matrixRNameForAzimuth)
    {
        matrixRNameForAzimuth->Destroy();
    }
    if (matrixRforAzimuthFilterPedestrianModel)
    {
        matrixRforAzimuthFilterPedestrianModel->Destroy();
    }
    if (matrixQNameForAzimuth)
    {
        matrixQNameForAzimuth->Destroy();
    }
    if (matrixQforAzimuthFilterPedestrianModel)
    {
        matrixQforAzimuthFilterPedestrianModel->Destroy();
    }
    if (matrixRNameForAcc)
    {
        matrixRNameForAcc->Destroy();
    }
    if (matrixRforAccFilterPedestrianModel)
    {
        matrixRforAccFilterPedestrianModel->Destroy();
    }
    if (matrixQNameForAcc)
    {
        matrixQNameForAcc->Destroy();
    }
    if (matrixQforAccFilterPedestrianModel)
    {
        matrixQforAccFilterPedestrianModel->Destroy();
    }
}

void KalmanFilterSetupGui::createGrids()
{
    //if (matrixRforAzimuthFilterPedestrianModel == nullptr)
    {
        matrixRforAzimuthFilterPedestrianModel = new wxGrid(rightPanel, wxID_ANY);
        matrixRforAzimuthFilterPedestrianModel->HideRowLabels();
        matrixRforAzimuthFilterPedestrianModel->HideColLabels();
    }
    //if (matrixQforAzimuthFilterPedestrianModel == nullptr)
    {
        matrixQforAzimuthFilterPedestrianModel = new wxGrid(rightPanel, wxID_ANY);
        matrixQforAzimuthFilterPedestrianModel->HideRowLabels();
        matrixQforAzimuthFilterPedestrianModel->HideColLabels();
    }
    //if (matrixRforAccFilterPedestrianModel == nullptr)
    {
        matrixRforAccFilterPedestrianModel = new wxGrid(rightPanel, wxID_ANY);
        matrixRforAccFilterPedestrianModel->HideRowLabels();
        matrixRforAccFilterPedestrianModel->HideColLabels();
    }
    //if (matrixQforAccFilterPedestrianModel == nullptr)
    {
        matrixQforAccFilterPedestrianModel = new wxGrid(rightPanel, wxID_ANY);
        matrixQforAccFilterPedestrianModel->HideRowLabels();
        matrixQforAccFilterPedestrianModel->HideColLabels();
    }
}

void KalmanFilterSetupGui::setupSizer()
{
    rightVerticalSizer->Add(matrixRNameForAzimuth, 0, wxALL | wxALIGN_CENTER, 5);
    rightVerticalSizer->Add(matrixRforAzimuthFilterPedestrianModel, 0, wxALL | wxALIGN_CENTER, 5);
    rightVerticalSizer->AddSpacer(15);
    rightVerticalSizer->Add(matrixQNameForAzimuth, 0, wxALL | wxALIGN_CENTER, 5);
    rightVerticalSizer->Add(matrixQforAzimuthFilterPedestrianModel, 0, wxALL | wxALIGN_CENTER, 5);

    rightVerticalSizer->Add(matrixRNameForAcc, 0, wxALL | wxALIGN_CENTER, 5);
    rightVerticalSizer->Add(matrixRforAccFilterPedestrianModel, 0, wxALL | wxALIGN_CENTER, 5);
    rightVerticalSizer->AddSpacer(15);
    rightVerticalSizer->Add(matrixQNameForAcc, 0, wxALL | wxALIGN_CENTER, 5);
    rightVerticalSizer->Add(matrixQforAccFilterPedestrianModel, 0, wxALL | wxALIGN_CENTER, 5);

    rightPanel->SetSizer(rightVerticalSizer);
    rightPanel->Layout();
}

void KalmanFilterSetupGui::createAndFillStateVectorRotationGrid()
{
    stateVectorAzimuthGrid = new wxGrid(leftPanel, wxID_ANY);
    stateVectorAzimuthGrid->HideRowLabels();
    stateVectorAzimuthGrid->HideColLabels();
    stateVectorAzimuthGrid->CreateGrid(6, 1);

    attr = new wxGridCellAttr();
    attr->SetReadOnly(true);


    stateVectorAzimuthGrid->SetCellValue(0, 0, "x rotation");
    stateVectorAzimuthGrid->SetCellValue(1, 0, "y rotation");
    stateVectorAzimuthGrid->SetCellValue(2, 0, "z rotation");
    stateVectorAzimuthGrid->SetCellValue(3, 0, "x angle velocity");
    stateVectorAzimuthGrid->SetCellValue(4, 0, "y angle velocity");
    stateVectorAzimuthGrid->SetCellValue(5, 0, "z angle velocity");

    //stateVectorAzimuthGrid->SetAttr(attr)
    //stateVectorAzimuthGrid->SetAttr(0, 1, attr);
    //stateVectorAzimuthGrid->SetAttr(0, 2, attr);
    //stateVectorAzimuthGrid->SetAttr(0, 3, attr);
    //stateVectorAzimuthGrid->SetAttr(0, 4, attr);
    //stateVectorAzimuthGrid->SetAttr(0, 5, attr);

    stateVectorAzimuthGrid->SetColSize(0, 100);
}

void KalmanFilterSetupGui::createAndFillStateVectorPositionGrid()
{
    stateVectorPositionGrid = new wxGrid(leftPanel, wxID_ANY);
    stateVectorPositionGrid->HideRowLabels();
    stateVectorPositionGrid->HideColLabels();
    stateVectorPositionGrid->CreateGrid(6, 1);

    stateVectorPositionGrid->SetCellValue(0, 0, "x acceleration");
    stateVectorPositionGrid->SetCellValue(1, 0, "x velocity");
    stateVectorPositionGrid->SetCellValue(2, 0, "x distance");
    stateVectorPositionGrid->SetCellValue(3, 0, "y acceleration");
    stateVectorPositionGrid->SetCellValue(4, 0, "y velocity");
    stateVectorPositionGrid->SetCellValue(5, 0, "y distance");

    stateVectorPositionGrid->SetColSize(0, 100);
}

void KalmanFilterSetupGui::createAndFillMatrixHRotationGrid()
{
    matrixHAzimuthGrid = new wxGrid(leftPanel, wxID_ANY);
    matrixHAzimuthGrid->HideRowLabels();
    matrixHAzimuthGrid->HideColLabels();
    matrixHPositionGrid->SetDefaultColSize(25);

    kf::Matrix<DIM_Z_azimuth, DIM_X_azimuth> matH;
    matH << 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1,
            1, 0, 0, 0, 0, 0;

    matrixHAzimuthGrid->CreateGrid(matH.rows(), matH.cols());

    for (int row = 0; row < matH.rows(); ++row) {
        for (int col = 0; col < matH.cols(); ++col) {
            matrixHAzimuthGrid->SetCellValue(row, col, wxString::Format("%g", matH(row, col)));
        }
    }
}

void KalmanFilterSetupGui::createAndFillMatrixHPositionGrid()
{
    matrixHPositionGrid = new wxGrid(leftPanel, wxID_ANY);
    matrixHPositionGrid->HideRowLabels();
    matrixHPositionGrid->HideColLabels();
    matrixHPositionGrid->SetDefaultColSize(25);

    kf::Matrix<DIM_Z, DIM_X> matH;
    matH << 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 1.0F;

    matrixHPositionGrid->CreateGrid(matH.rows(), matH.cols());

    for (int row = 0; row < matH.rows(); ++row) {
        for (int col = 0; col < matH.cols(); ++col) {
            matrixHPositionGrid->SetCellValue(row, col, wxString::Format("%g", matH(row, col)));
        }
    }
}