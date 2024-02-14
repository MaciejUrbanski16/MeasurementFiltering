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
	bool openFileWithGpsData(const std::string filePath);

	std::vector<std::string> readCSVrow(char delimiter = ',');
	std::vector<std::string> readCSVrowGpsData(char delimiter = ',');
	~CsvMeasurementReader();

private:
	std::ifstream file;
	std::vector<std::string> headers;
	std::ifstream fileGpsData;
	std::vector<std::string> headersGpsData;
};

