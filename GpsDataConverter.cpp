#include "GpsDataConverter.h"

void GpsDataConverter::handleGpsData(const std::vector<std::string>& measurements)
{
	if (measurements.size() == 4)
	{
					//latitude					  //longitude					//velocity					//satellites	
		if (isNumber(measurements[0]) && isNumber(measurements[1]) && isNumber(measurements[2]) /* && isNumber(measurements[3])*/)
		{
			latitude = std::stod(measurements[0]);
			longitude = std::stod(measurements[1]);
			velocityKmph = std::stod(measurements[2]);
			//satellites = std::stoi(measurements[3]);
		}
	}
	else
	{
		//00
	}
}

bool GpsDataConverter::isNumber(const std::string& meas)
{
	for (const char c : meas)
	{
		if (not (c == '.' or  c == '-' or c == '0' or c == '1' or c == '2' or c == '3' or c == '4' or
			c == '5' or c == '6' or c == '7' or c == '8' or c == '9'))
		{

			return false;
		}
	}
	return true;
}
