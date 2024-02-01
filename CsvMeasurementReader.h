#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class CsvMeasurementReader
{
public:
	std::vector<std::string> readCSVHeader(const std::string& filename, char delimiter = ',');
};

