#include "MainWindow.h"


//int main() {
//
//    boost::asio::io_context io;
//
//    std::string com = "COM7";
//    SerialComm serialComm(io, com);
//
//
//    io.run();
//
//
//    return 0;
//}




// --------------------------------------------------------
// a simple thread
//wxDECLARE_EVENT(wxEVT_MY_THREAD_EVENT, wxThreadEvent);
//wxDEFINE_EVENT(wxEVT_MY_THREAD_EVENT, wxThreadEvent);







//LPCWSTR CharToLPCWSTR(const char* charString) {
//    int charStringLength = strlen(charString) + 1;
//    int sizeRequired = MultiByteToWideChar(CP_UTF8, 0, charString, charStringLength, NULL, 0);
//    wchar_t* wideString = new wchar_t[sizeRequired];
//    MultiByteToWideChar(CP_UTF8, 0, charString, charStringLength, wideString, sizeRequired);
//    return wideString;
//}


// --------------------------------------------------------



class MyApp : public wxApp
{
public:
    virtual bool OnInit();
};

IMPLEMENT_APP(MyApp)
bool MyApp::OnInit()
{
    MyWindow* frame = new MyWindow(wxT("Data visualization"));
    frame->Show(true);

    return true;
}