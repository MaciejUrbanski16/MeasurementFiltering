#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class CsvMeasurementReader
{
public:
	bool openFile(const std::string filePath);
	std::vector<std::string> readCSVrow(char delimiter = ',');
	~CsvMeasurementReader();

private:
	std::ifstream file;
	std::vector<std::string> headers;
};

