#pragma once
#include <vector>
#include <string>
#include <utility>
#include <chrono>
#include <cmath>
#include <math.h>

#include "AppLogger.h"

class MeasurementsController
{
public:
	MeasurementsController(AppLogger& appLogger, const double newRawGrawity, const double newXBias, const double newYBias,
		const double newXGyroBias, const double newYGyroBias, const double newZGyroBias): appLogger(appLogger)
	{
		setRawGrawity(newRawGrawity);
		setXBias(newXBias);
		setYBias(newYBias);

		setXGyroBias(newXGyroBias);
		setYGyroBias(newYGyroBias);
		setZGyroBias(newZGyroBias);

		//assign(measurements);
	}

	bool assign(const std::vector<std::string>& measurements, const uint32_t deltaTimeMs)
	{
		if (measurements.size() == 8)
		{
			//const auto isMagnetometr = isMagn(measurements[6]);
			if (isNumber(measurements[0]) && isNumber(measurements[1]) && isNumber(measurements[2]) &&
				isNumber(measurements[3]) && isNumber(measurements[4]) && isNumber(measurements[5]) &&
				isNumber(measurements[6]) && isNumber(measurements[7]))

			{
				xAcc = std::stoi(measurements[0]);
				yAcc = std::stoi(measurements[1]);
				zAcc = std::stoi(measurements[2]);

				xGyro = std::stoi(measurements[3]);
				yGyro = std::stoi(measurements[4]);
				zGyro = std::stoi(measurements[5]);

				//
				//magn = std::stoi(measurements[6]);
				xMagn = std::stoi(measurements[6]);
				yMagn = std::stoi(measurements[7]);

				this->deltaTimeMs = deltaTimeMs;

				calculateAccInMPerS2();
				calculateAngleVelocity();

				calculateXYvelocity();
				calculateDistance();

				calculateOrientationDegree();

				appLogger.logHandledMeas(xAcc, yAcc, zAcc, xGyro, yGyro, zGyro, xMagn, yMagn,
					xAccMPerS2, yAccMPerS2, zAccMPerS2, xVelocity, yVelocity,
					xDistance, yDistance, orientationDegree, deltaTimeMs);

				return true;
			}
		}
		const std::string errMeasConversion{ "ERR when measurement conversion from string to int16_t" };
		appLogger.logErrMeasurementConversion(errMeasConversion);
		return false;
	}

	double getXaccMPerS2() const { return xAccMPerS2; }
	double getYaccMPerS2() const { return yAccMPerS2; }
	double getZaccMPerS2() const { return zAccMPerS2; }
	double getXangleVelocityDegreePerS() const { return xAngleVelocityDegPerS; }
	double getYangleVelocityDegreePerS() const { return yAngleVelocityDegPerS; }
	double getZangleVelocityDegreePerS() const { return zAngleVelocityDegPerS; }
	double getAzimuth() const { return orientationDegree; }
	int16_t getRawXMagn() const { return xMagn; }
	int16_t getRawYMagn() const { return yMagn; }

	double getXvelocityMperS() const { return xVelocity; }
	double getYvelocityMperS() const { return yVelocity; }

	double getXDistance() const { return xDistance; }
	double getYDistance() const { return yDistance; }



private:
	void setRawGrawity(const double newRawGrawity) { rawGrawity = newRawGrawity; }
	void setXBias(const double newXBias) { xBias = newXBias; }
	void setYBias(const double newYBias) { yBias = newYBias; }

	void setXGyroBias(const double newXBias) { xAngleVelBias = newXBias; }
	void setYGyroBias(const double newYBias) { yAngleVelBias = newYBias; }
	void setZGyroBias(const double newZBias) { zAngleVelBias = newZBias; }

	void calculateAccInMPerS2()
	{
		//in order to correctly calculate the calibration is required
		
		xAccMPerS2 = static_cast<double>(xAcc + xBias) * gPhysConst / rawGrawity;
		yAccMPerS2 = static_cast<double>(yAcc + yBias) * gPhysConst / rawGrawity;
		zAccMPerS2 = static_cast<double>(zAcc) * gPhysConst / rawGrawity;
	}

	void calculateAngleVelocity()
	{
		xAngleVelocityDegPerS = static_cast<double>(xGyro + xAngleVelBias) * 250.0f / 32768.0f; //the bias has to be set correctly
		yAngleVelocityDegPerS = static_cast<double>(yGyro + yAngleVelBias) * 250.0f / 32768.0f;
		zAngleVelocityDegPerS = static_cast<double>(zGyro + zAngleVelBias) * 250.0f / 32768.0f;
	}

	void calculateXYvelocity()
	{
		const double timeIntervalSec = static_cast<double>(deltaTimeMs)/105;
		xVelocity =  xAccMPerS2 * timeIntervalSec;
		yVelocity =  yAccMPerS2 * timeIntervalSec;

		previousXvelocity = xVelocity;
		previousYvelocity = yVelocity;
		// v=a*t  1m/s*s * 0.3s = 
		//const auto actualSample{ timeIntervalSec * (xAcc + previousxAcc) };
	}

	void calculateDistance()
	{
		const double timeIntervalSec = static_cast<double>(deltaTimeMs)/105;
		xDistance = xVelocity * timeIntervalSec;
		yDistance = yVelocity * timeIntervalSec;
	}

	void calculateOrientationDegree()
	{
		orientationDegree = atan2(static_cast<double>(yMagn),
			static_cast<double>(xMagn)) * (180.0f / M_PI);
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
	int16_t xMagn{ 0xFF };
	int16_t yMagn{ 0xFF };

	int16_t magn{ 0xFF };

	double xAccMPerS2{ 0.0 };
	double yAccMPerS2{ 0.0 };
	double zAccMPerS2{ 0.0 };

	double xAngleVelocityDegPerS{ 0.0 };
	double yAngleVelocityDegPerS{ 0.0 };
	double zAngleVelocityDegPerS{ 0.0 };

	double orientationDegree{ 0.0 };
	
	/// <summary>
	/// 
	/// </summary>
	double previousXvelocity{ 0.0 };
	double previousYvelocity{ 0.0 };
	double xVelocity{ 0.0 };
	double yVelocity{ 0.0 };

	double xDistance{ 0.0 };
	double yDistance{ 0.0 };

	uint32_t deltaTimeMs{ 0 };
	
	static constexpr double gPhysConst = 9.803;
	double rawGrawity = 2000.0;
	double xBias = 80.0;
	double yBias = 40.0;

	double xAngleVelBias{ -18000.0};
	double yAngleVelBias{ 12900.0};
	double zAngleVelBias{15000.0};



	AppLogger& appLogger;
};