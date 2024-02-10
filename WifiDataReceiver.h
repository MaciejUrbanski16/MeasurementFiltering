#pragma once
//#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <vector>

#include "MeasurementCustomizator.h"
#include "AppLogger.h"

using namespace boost::asio;

wxDEFINE_EVENT(wxEVT_MY_THREAD_EVENT, wxThreadEvent);

class Session : public std::enable_shared_from_this<Session> {
public:
    Session(ip::tcp::socket socket, AppLogger& appLogger, wxEvtHandler* parent) : socket_(std::move(socket)), appLogger(appLogger), m_parent(parent) {}

    void start() {
        doRead();
    }

private:
    void doRead() {
        auto self(shared_from_this());

        socket_.async_read_some(buffer(data_),
            [this, self](boost::system::error_code ec, std::size_t length) {
                if (!ec) {

                    std::stringstream ss;
                    const std::string receivedMeasurement = std::string(data_.begin(), data_.begin() + length);
                    ss << "Session on: " << socket_.remote_endpoint().address().to_string() << " : port: " << socket_.remote_endpoint().port() <<
                        " Received data from client: " << receivedMeasurement;
                    const std::string msg = ss.str();

                    appLogger.logReceivedDataOnTcpPort(msg);

                    //DATA STRUCTURE Xacc 6 elements
           //               Yacc 6 elements
           //               Zacc 6 elements
           //               Xgyr 6 elements
           //               Ygyr 6 elements 
           //               Zgyr 6 elements 
           //               Xmagn 6 elements
           //               Ymagn 6 elements


                    //if (sizeof(remoteSensorData) != 40)
 //                   {
                        //simple logging mechanism
                        //std::stringstream ss;
                        //auto currentTime = getCurrentTimeWithMilliSeconds();
                        //ss << currentTime << "The remoteSensorData has size different from 40 " << sizeof(remoteSensorData) << '\n';
                        //outputFile << ss.str();
                    //}
                    //std::string remoteDataAsString(reinterpret_cast<char*>(remoteSensorData), sizeof(remoteSensorData));

                    const std::vector<std::string> exctractedMeasurements = exctractMeasurements(receivedMeasurement);

                    MeasurementCustomizator* event = new MeasurementCustomizator(wxEVT_MY_THREAD_EVENT);
                    event->SetStringVector(exctractedMeasurements);

                    //event->SetString(strToSendToMainThread);
                    //event->Set
                    wxQueueEvent(m_parent, event);

                    //logIntoFile(receivedMeasurement, exctractedMeasurements);
                    doRead();
                   }
                
            }
        );
    }

    std::vector<std::string> exctractMeasurements(const std::string & frameWithMeasurements)
    {
        std::istringstream ss(frameWithMeasurements);
        std::vector<std::string> tokens;
        std::string token;
        int nr = 0;
        while (std::getline(ss, token, measumrementsDelimiter))
        {
            //this will be extended for handling GPS data
            if (nr < 8)
            {
                nr++;
                tokens.push_back(token);
            }
        }
        tokens.push_back("70.234"); //Latitude
        tokens.push_back("30.234"); //Longitude

        return tokens;
    }

    ip::tcp::socket socket_;
    boost::array<char, 128> data_;

    wxEvtHandler* m_parent;
    AppLogger& appLogger;

    static constexpr char measumrementsDelimiter = '_';
};

class Server {
public:
    Server(io_service& io, short port, AppLogger& appLogger, wxEvtHandler* parent) : acceptor_(io, ip::tcp::endpoint(ip::tcp::v4(), port)), appLogger(appLogger) {
        doAccept(parent);
        //this->appLogger = appLogger;
    }

private:
    void doAccept(wxEvtHandler* parent) {
        acceptor_.async_accept(
            [this, parent](boost::system::error_code ec, ip::tcp::socket socket) {
                if (!ec) {
                    std::make_shared<Session>(std::move(socket), appLogger, parent)->start();
                }

                doAccept(parent);
            }
        );
    }

    ip::tcp::acceptor acceptor_;
    AppLogger& appLogger;


};

class WifiDataReceiver
{
};

