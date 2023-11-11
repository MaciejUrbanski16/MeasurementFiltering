#pragma once
#include <vector>
#include <string>
#include <utility>
#include <chrono>

#include "AppLogger.h"

class MeasurementsController
{
public:
	MeasurementsController(AppLogger& appLogger): appLogger(appLogger)
	{
		//assign(measurements);
	}

	bool assign(const std::vector<std::string>& measurements, const uint32_t deltaTimeMs)
	{
		if (measurements.size() == 7)
		{
			const auto isMagnetometr = isMagn(measurements[6]);
			if (isNumber(measurements[0]) && isNumber(measurements[1]) && isNumber(measurements[2]) &&
				isNumber(measurements[3]) && isNumber(measurements[4]) && isNumber(measurements[5]))

			{
				xAcc = std::stoi(measurements[0]);
				yAcc = std::stoi(measurements[1]);
				zAcc = std::stoi(measurements[2]);

				xGyro = std::stoi(measurements[3]);
				yGyro = std::stoi(measurements[4]);
				zGyro = std::stoi(measurements[5]);

				//
				magn = std::stoi(measurements[6]);

				this->deltaTimeMs = deltaTimeMs;

				calculateAccInMPerS2();

				appLogger.logHandledMeas(xAcc, yAcc, zAcc, xGyro, yGyro, zGyro, magn,
					xAccMPerS2, yAccMPerS2, zAccMPerS2, deltaTimeMs);

				return true;
			}
		}
		const std::string errMeasConversion{ "ERR when measurement conversion from string to int16_t" };
		appLogger.logErrMeasurementConversion(errMeasConversion);
		return false;
	}

	//void setDeltaTimeMs(const uint32_t deltaTimeMs)
	//{
	//	this->deltaTimeMs = deltaTimeMs; 
	//}

	double getXaccMPerS2() const { return xAccMPerS2; }
	double getYaccMPerS2() const { return yAccMPerS2; }
	double getZaccMPerS2() const { return zAccMPerS2; }
	int16_t getMagn() const { return magn; }

private:

	void calculateAccInMPerS2()
	{
		//in order to correctly calculate the calibration is required
		// 800 bias
		xAccMPerS2 = static_cast<double>(xAcc + xBias) * gPhysConst / rawGrawity;
		yAccMPerS2 = static_cast<double>(yAcc + yBias) * gPhysConst / rawGrawity;
		zAccMPerS2 = static_cast<double>(zAcc) * gPhysConst / rawGrawity;
	}

	bool isNumber(const std::string& meas)
	{
		for (const char c : meas)
		{
			if (not (c == '-' or c == '0' or c == '1' or c == '2' or c == '3' or c == '4' or
				c == '5' or c == '6' or c == '7' or c == '8' or c == '9'))
			{
				return false;
			}
		}
		return true;
	}

	std::pair<bool, std::string> isMagn(const std::string& amgnMeas)
	{
		std::string magnStr{};
		for (const char c : amgnMeas)
		{
			if (not (c == '-' or c == '0' or c == '1' or c == '2' or c == '3' or c == '4' or
				c == '5' or c == '6' or c == '7' or c == '8' or c == '9'))
			{
				if (c == '\r' or c == '\n')
				{
					return { true, magnStr };
				}
				return { false,"" };
			}
			magnStr += c;
		}
		return { true, magnStr };
	}

	int16_t xAcc{ 0xFF };
	int16_t yAcc{ 0xFF };
	int16_t zAcc{ 0xFF };
	int16_t xGyro{ 0xFF };
	int16_t yGyro{ 0xFF };
	int16_t zGyro{ 0xFF };
	int16_t magn{ 0xFF };

	float xAccMPerS2{ 0.f };
	float yAccMPerS2{ 0.f };
	float zAccMPerS2{ 0.f };

	uint32_t deltaTimeMs{ 0 };
	
	static constexpr double gPhysConst = 9.803;
	static constexpr double rawGrawity = 16000.0;
	static constexpr double xBias = 880.0;
	static constexpr double yBias = 800.0;



	AppLogger& appLogger;
};