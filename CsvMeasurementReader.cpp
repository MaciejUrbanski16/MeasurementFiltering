#include "CsvMeasurementReader.h"


bool CsvMeasurementReader::openFile(const std::string filePath)
{
    fileMeasurementsData.open(filePath);
    if (!fileMeasurementsData.is_open())
    {
        return false;
    }

    std::string line;
    if (std::getline(fileMeasurementsData, line))
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

bool CsvMeasurementReader::openFileWithGpsData(const std::string filePath)
{
    fileGpsData.open(filePath);
    if (!fileGpsData.is_open())
    {
        return false;
    }

    std::string line;
    if (std::getline(fileGpsData, line))
    {
        std::stringstream ss(line);
        std::string header;
        while (std::getline(ss, header, ','))
        {
            headersGpsData.push_back(header);
        }
    }

    return true;
}

CsvMeasurementReader::~CsvMeasurementReader()
{
    if (fileMeasurementsData.is_open())
    {
        fileMeasurementsData.close();
    }
    if (fileGpsData.is_open())
    {
        fileGpsData.close();
    }
}

std::vector<std::string> CsvMeasurementReader::readCSVrow(char delimiter)
{
    std::vector<std::string> row;
    std::string line;
    
    if (!fileMeasurementsData.is_open())
    {
        return row;
    }
    
    if (std::getline(fileMeasurementsData, line))
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

std::vector<std::string> CsvMeasurementReader::readCSVrowGpsData(char delimiter)
{
    std::vector<std::string> row;
    std::string line;

    if (!fileGpsData.is_open())
    {
        return row;
    }

    if (std::getline(fileGpsData, line))
    {
        std::stringstream ss(line);
        std::string cell;
        int cellNr{ 0 };

        while (std::getline(ss, cell, delimiter))
        {
            if (cellNr >= 1 && cellNr < 5)
            {
                //int16_t value = std::stoi(cell);
                row.push_back(cell);
            }
            cellNr++;
        }
    }
    return row;
}

void CsvMeasurementReader::setReadMeasurementFromBegining()
{
    fileMeasurementsData.clear();
    fileMeasurementsData.seekg(0, std::ios::beg);
}

void CsvMeasurementReader::setReadGpsDataFromBegining()
{
    fileGpsData.clear();
    fileGpsData.seekg(0, std::ios::beg);
}
