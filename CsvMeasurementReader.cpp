#include "CsvMeasurementReader.h"


bool CsvMeasurementReader::openFile(const std::string filePath)
{
    file.open(filePath);
    if (!file.is_open())
    {
        return false;
    }

    std::string line;
    if (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string header;
        while (std::getline(ss, header, ','))
        {
            headers.push_back(header);
        }
    }

    return true;
}

CsvMeasurementReader::~CsvMeasurementReader()
{
    if (file.is_open())
    {
        file.close();
    }
}

std::vector<std::string> CsvMeasurementReader::readCSVrow(char delimiter)
{
    std::vector<std::string> row;
    std::string line;
    
    if (!file.is_open()) 
    {
        return row;
    }
    
    if (std::getline(file, line))
    { 
        std::stringstream ss(line);
        std::string cell;
        int cellNr{ 0 };

        while (std::getline(ss, cell, delimiter))
        {
            if (cellNr >= 1 && cellNr < 9)
            {
                //int16_t value = std::stoi(cell);
                row.push_back(cell);
            }
            cellNr++;
        }

        //GPS
        row.push_back("70.342");
        row.push_back("30.342");
    }
    return row;
}