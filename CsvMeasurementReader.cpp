#include "CsvMeasurementReader.h"

std::vector<std::string> CsvMeasurementReader::readCSVHeader(const std::string& filename, char delimiter)
{
    std::vector<std::string> headers;
    std::ifstream file(filename);
    if (!file.is_open()) 
    {
        return headers;
    }

    std::string line;
    if (std::getline(file, line)) 
    {
        std::stringstream ss(line);
        std::string header;
        while (std::getline(ss, header, delimiter)) 
        {
            headers.push_back(header);
        }
    }

    
    std::vector<std::vector<std::string>> data;
    int count{ 0 };
    
    while (std::getline(file, line))
    {
        std::vector<std::string> row;
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

        //HERE CALL OF FILTRATION

        data.push_back(row);
        count++;
        if (count == 20)
        {
            break;
        }
    }

    file.close();
    return headers;
}