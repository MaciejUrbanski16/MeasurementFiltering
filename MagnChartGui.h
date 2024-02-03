#pragma once

#include <wx/wx.h>
#include <wx/sizer.h>
#include <wx/notebook.h>
#include <wx/chartpanel.h>
#include <wx/chart.h>
#include <wx/chartype.h>
#include <wx/wxfreechartdefs.h>
#include <wx/legend.h>
#include <wx/dataset.h>
#include <wx/colorscheme.h>
#include <wx/category/categorydataset.h>
#include <wx/xy/xyplot.h>
#include <wx/xy/xylinerenderer.h>
#include <wx/xy/xysimpledataset.h>
#include <wx/xy/vectordataset.h>
#include <wx/splitter.h>
#include <wx/spinctrl.h> 

#include "PlotElementsBuffer.h"

class MagnChartGui
{
public:
	void setup(wxNotebook* m_notebook/*, wxWindow* window*/);
	void updateChart(PlotElementsBuffer& magnPointsBuffer, PlotElementsBuffer& filteredAzimuthBuffer,
		 const int16_t xMagn, const int16_t yMagn, const double azimuth, const double filteredAzimuth, const double timeMs);

private:
	void OnResetMagnChart(wxCommandEvent& event);
	void OnSubmitMagnAdjustments(wxCommandEvent& event);

	wxChartPanel* azimuthChartPanel = nullptr;
	wxSplitterWindow* azimuthPanelSplitter = nullptr;
	wxBoxSizer* sizerAzimuthPlot = nullptr;
	wxSpinCtrl* spinCtrlXmagn = nullptr;
	wxSpinCtrl* spinCtrlYmagn = nullptr;
	wxStaticText* xMagnValue = nullptr;
	wxStaticText* yMagnValue = nullptr;
	wxStaticText* orientationValue = nullptr;
};
