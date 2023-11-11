#pragma once
#include <vector>
#include <string>
#include <utility>

#include "AppLogger.h"

class RawMeasurements
{
public:
	RawMeasurements(AppLogger& appLogger): appLogger(appLogger)
	{
		//assign(measurements);
	}

	bool assign(const std::vector<std::string>& measurements)
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

				appLogger.logHandledMeas(xAcc, yAcc, zAcc, xGyro, yGyro, zGyro, magn);
				return true;
			}
		}
		const std::string errMeasConversion{ "ERR when measurement conversion from string to int16_t" };
		appLogger.logErrMeasurementConversion(errMeasConversion);
		return false;
	}

	int16_t getMagn() const { return magn; }

private:

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

	AppLogger& appLogger;
};