#include "PositionChartGui.h"

void PositionChartGui::setup(wxNotebook* m_notebook)
{
	positionChartPanel = new wxChartPanel(m_notebook);
	m_notebook->AddPage(positionChartPanel, "Pos chart");
}

void PositionChartGui::updateChart(PlotElementsBuffer& rawPositionBuffer, const double xDistance, const double yDistance, const double timeMs)
{
    currentXPos = currentXPos + xDistance;
    currentYPos = currentYPos + yDistance;
    //rawPositionPoints.push_back(wxRealPoint(currentXPos, currentYPos));
    rawPositionBuffer.AddElement(wxRealPoint(currentXPos, currentYPos));

    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();


    dataset->AddSerie(new XYSerie(rawPositionBuffer.getBuffer()));
    dataset->SetRenderer(new XYLineRenderer());
    dataset->GetSerie(0)->SetName("raw position");
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("Y position [m]"));
    bottomAxis->SetTitle(wxT("X position [m]"));
    //if (rawPositionBuffer.getBuffer().size() >= 100)
    //{
    //    bottomAxis->SetFixedBounds(rawPositionBuffer.getBuffer()[0].x, rawPositionBuffer.getBuffer()[99].x);
    //}
    Legend* legend = new Legend(wxTOP, wxLEFT);
    plot->SetLegend(legend);

    plot->AddObjects(dataset, leftAxis, bottomAxis);


    Chart* chart = new Chart(plot, "Position");

    positionChartPanel->SetChart(chart);
}