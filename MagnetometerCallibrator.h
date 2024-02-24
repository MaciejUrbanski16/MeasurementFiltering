#pragma once

#include <cstdint>
#include <Eigen/Dense>

class MagnetometerCallibrator
{
public:
	void collectData(const int16_t xMagn, const int16_t yMagn);
	void callibrate();
	int16_t getXoffset() const { return xOffset; }
	int16_t getYoffset() const { return yOffset; }
private:
	int sumOfXMagn{ 0 };
	int sumOfYMagn{ 0 };
	int32_t numOfSamples{ 0 };

	Eigen::MatrixXd magnetometerData;

	int16_t xOffset{ 0 };
	int16_t yOffset{ 0 };
};

