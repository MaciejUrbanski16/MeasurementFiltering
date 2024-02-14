#pragma once
#include <cmath>
#include <utility>
#include "wx/math.h"

struct GPSPoint 
{
	double latitude;
	double longitude;
};

class HaversineConverter
{
public:
	std::pair<double, double> calculateCurrentPosition(const double longitude, const double latitude)
	{
		if (isFirstMeasurement)
		{
			actualGpsPoint.latitude = latitude;
			actualGpsPoint.longitude = longitude;
			isFirstMeasurement = false;
			return std::make_pair(0.0, 0.0);
		}

		previousGpsPoint = actualGpsPoint;
		actualGpsPoint.latitude = latitude;
		actualGpsPoint.longitude = longitude;

		const double distance = calculateDistance(longitude, latitude);

		const double angle = std::atan2(actualGpsPoint.longitude - previousGpsPoint.longitude,
			actualGpsPoint.latitude - previousGpsPoint.latitude);

		xPosition = xPosition + (distance * cos(angle));
		yPosition = yPosition + sin(angle);

		return std::make_pair(xPosition, yPosition);
	}

private:
	double calculateDistance(const double longitude, const double latitude)
	{

		//stare     nowe
		//double dLat = toRadians(latitude2 - latitude1);
		double deltaLat = toRadians(previousGpsPoint.latitude - actualGpsPoint.latitude);

		//double dLon = toRadians(longitude2 - longitude1);
		double deltaLon = toRadians(previousGpsPoint.longitude - actualGpsPoint.longitude);

		double a = sin(deltaLat / 2.0) * sin(deltaLat / 2.0) +
			cos(toRadians(actualGpsPoint.latitude)) * cos(toRadians(previousGpsPoint.latitude)) *
			sin(deltaLon / 2.0) * sin(deltaLon / 2.0);

		double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

		return earthRadius * c;
	}

	double toRadians(const double degree)
	{
		return degree* (M_PI / 180.0);
	}

	GPSPoint previousGpsPoint{ 0.0, 0.0 };
	GPSPoint actualGpsPoint{ 0.0, 0.0 };

	double xPosition{ 0.0 };
	double yPosition{ 0.0 };

	bool isFirstMeasurement{ true };

	static constexpr double earthRadius = 6371000.0;
};

