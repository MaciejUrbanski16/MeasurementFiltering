#include "SerialComm.h"


std::vector<std::string> SerialComm::exctractMeasurements(const std::string& frameWithMeasurements)
{
    std::istringstream ss(frameWithMeasurements);
    std::vector<std::string> tokens;
    std::string token;
    int nr = 0;
    while (std::getline(ss, token, measumrementsDelimiter)) {
        if (nr < 8)
        {
            nr++;
            //if (nr == 8)
            //{
            //    std::string tempToken{};
            //    for (const auto c : token)
            //    {
            //        if (c == '\r')
            //        {
            //            tokens.push_back(tempToken);
            //            break;
            //        }
            //        tempToken += c;
            //    }
            //}
            //else 
            {
                //token.append("++");
                //token.append(std::to_string(nr));
                tokens.push_back(token);
            }

        }
    }

    return tokens;
}

void SerialComm::logIntoFile(const std::string& frameWithMeasurements, const std::vector<std::string>& exctractedMeasurements)
{
    char delimiter = 'X';
    size_t position = frameWithMeasurements.find(delimiter);
    std::string rawRemoteDataString = frameWithMeasurements.substr(0, position);

    std::stringstream ssSensorData;
    const auto currentTime = getCurrentTimeWithMilliSeconds();
    ssSensorData << currentTime << " RAW SENSOR FRAME: ";/* << rawRemoteDataString << " "; /*" ---> The remoteSensorData was received: Xacc:" << XaccAsString << " Yacc:" <<
        YaccAsString << " Zacc:" << ZaccAsString << " Xgyr:" << XgyrAsString << " Ygyr:" << YgyrAsString
        << " Zgyr:" << ZgyrAsString << " magn: " << magnAsString << '\n'*/
    for (const auto& measumrentStr : exctractedMeasurements)
    {
        ssSensorData << measumrentStr << ", ";
    }
    ssSensorData << '\n';
    outputFile << ssSensorData.str();
}

void SerialComm::logErrReadRemoteData()
{
    std::stringstream errRead;
    const auto currentTime = getCurrentTimeWithMilliSeconds();
    errRead << currentTime << " The ERR occurred during read of measurement data!!!\n";
    outputFile << errRead.str();
}

std::string SerialComm::getCurrentTimeWithMilliSeconds()
{
    auto currentTime = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count();

    constexpr uint16_t msToSec{ 1000 };
    constexpr uint16_t usToMs{ 1000 };
    auto sec = ms / msToSec;
    auto us = (ms % usToMs) * msToSec;

    std::time_t now = std::chrono::system_clock::to_time_t(currentTime);
    std::tm tm_now = *std::localtime(&now);

    std::stringstream timeString;
    timeString << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");
    timeString << '.' << std::setfill('0') << std::setw(3) << us;

    std::string currentDateTimeWithMillis = timeString.str();
    return currentDateTimeWithMillis;
}