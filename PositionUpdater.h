#pragma once

#include <cmath>
#include <utility>

class PositionUpdater
{
public:
	void updatePosition(const double previousX, const double previuosY, const double xDistance, const double yDistance, const double azimuth)
	{
		const double distance = calculateDistance(xDistance, yDistance);
		currentX += (previousX + distance * cos(azimuth));
		currentY += (previuosY + distance * (sin(azimuth)));
	}

	std::pair<double, double> getCurrentPosition() const
	{
		return std::make_pair(currentX, currentY);
	}

private:
	double calculateDistance(const double xDistance, const double yDistance)
	{
		const double temp{ pow(xDistance,2) + pow(yDistance,2) };
		return sqrt(temp);
	}

	double currentX{ 0.0 };
	double currentY{ 0.0 };
};

