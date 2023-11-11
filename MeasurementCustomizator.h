#pragma once

#include <wx/wx.h>
#include <wx/thread.h>
#include <wx/event.h>
#include <vector>
#include <string>

class MeasurementCustomizator : public wxThreadEvent
{
public:
    MeasurementCustomizator(wxEventType eventType, int id = wxID_ANY)
        : wxThreadEvent(eventType, id) {}

    void SetStringVector(const std::vector<std::string>& strings) {
        measurements = strings;
    }

    const std::vector<std::string>& GetStringVector() const {
        return measurements;
    }

    wxEvent* Clone() const override {
        return new MeasurementCustomizator(*this);
    }

private:
    std::vector<std::string> measurements;
};