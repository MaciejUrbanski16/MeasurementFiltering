#pragma once
#include <fstream>
#include <string>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <vector>

#include "kalman_filter/kalman_filter.h"

class AppLogger
{
public:
	AppLogger() {
		outputFile.open(filePath, std::ios::app);
		kalmanOutputFile.open(kalmanFilePath, std::ios::app);
	}
	~AppLogger()
	{
		if (outputFile.is_open())
		{
			outputFile.close();
		}
		if (kalmanOutputFile.is_open())
		{
			kalmanOutputFile.close();
		}
	}
	void logSerialCommStartThread(const std::string& msg)
	{
		std::stringstream ss;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ss << currentTime << " " << msg << '\n';
		outputFile << ss.str();
	}
	void logReceivedDataOnMainThread(const std::vector<std::string>& measurements)
	{
		std::stringstream ss;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ss << currentTime <<" DATA ON MAIN THREAD: ";
		for (const auto& measurement : measurements)
		{
			ss << measurement << " ";
		}
		ss << '\n';
		outputFile << ss.str();
	}
	void logErrThreadDataHandling(const std::string& msg)
	{
		std::stringstream ss;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ss << currentTime << " " << msg << '\n';
		outputFile << ss.str();
	}

	void logHandledMeas(const int16_t xAcc, const int16_t yAcc,
		const int16_t zAcc, const int16_t xGyro, const int16_t yGyro, const int16_t zGyro,
		const int16_t magn, const double xAccMPerS2, const double yAccMPerS2, const double zAccMPerS2,
		const double xVelocity, const double yVelocity, const double xDistance, const double yDistance,
		const uint32_t deltaTimeMs)
	{
		std::stringstream ss;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ss << currentTime << " HANDLED MEASUREMENT DATA: " << "xAcc:" << xAcc << " yAcc:" << yAcc << " zAcc:"
			<< zAcc << " xGyro:" << xGyro << " yGyro:" << yGyro << " zGyro:" << zGyro << " magn:" << magn << " || " << "xAcc: " << xAccMPerS2
			<< "[m/s2]" << " yAcc: " << yAccMPerS2 << "[m/s2]" << " zAcc: " << zAccMPerS2 << "[m/s2] || xVelocity: " << xVelocity << "[m/s]"
			<< " yVelocity: " << yVelocity << "[m/s] || xDistance: " << xDistance << "[m] yDistance: " << yDistance << "[m] || Delta time : "
			<< deltaTimeMs<< "[ms]" << '\n';
		outputFile << ss.str();
	}
	void logErrMeasurementConversion(const std::string& msg)
	{
		std::stringstream ss;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ss << currentTime << " " << msg << '\n';
		outputFile << ss.str();
	}

	template <size_t DIM_X, size_t DIM_Z>
	void logKalmanFilterPredictionStep(const kf::KalmanFilter<DIM_X, DIM_Z>& kalmanFilter)
	{
		std::stringstream ss;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ss << currentTime << "\nKALMAN FILTER: Predicted state vector = \n" << kalmanFilter.vecX() << "\n Predicted state covariance = \n" << kalmanFilter.matP() << "  END\n";
		kalmanOutputFile << ss.str();
	}

	template <size_t DIM_X, size_t DIM_Z>
	void logKalmanFilterCorrectionStep(const kf::KalmanFilter<DIM_X, DIM_Z>& kalmanFilter)
	{
		std::stringstream ss;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ss << currentTime << "\nKALMAN FILTER: Corrected state vector = \n" << kalmanFilter.vecX() << "\nCorrected state covariance = \n" << kalmanFilter.matP() << "  END\n";
		kalmanOutputFile << ss.str();
	}
private:
	std::string getCurrentTimeWithMilliSeconds()
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


	std::ofstream outputFile;
	std::string filePath = "appLogs.txt";

	std::ofstream kalmanOutputFile;
	std::string kalmanFilePath = "kalmanFilterLogs.txt";
};
//std::string AppLogger::path = "appLogs.txt";
//std::ofstream AppLogger::outputFile;// = path;

