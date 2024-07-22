#include "barelangFile.h"
#include "icecream.hpp"

template <typename tipedata>
tipedata convert_to(std::string input)
{
    std::istringstream data(input);
    tipedata target;
    data >> target;
    return target;
}

template <typename tipedata>
bool barelangFile<tipedata>::parseData()
{
    std::fstream file(file_path);

    if (file.is_open())
    {
        std::string line;

        while (std::getline(file, line))
        {
            // IC(line);
            std::istringstream iss(line);
            std::string key, value;
            // IC(iss);
            if (std::getline(iss, key, '=') && std::getline(iss, value))
            {
                key.erase(key.end()-1,key.end());
                data[key] = convert_to<tipedata>(value);
            }
        }

        file.close();
        return true;
    }
    else
    {
        std::cout << "Failed to open the file for reading." << std::endl;
        return false;
    }
}

template <typename tipedata>
void barelangFile<tipedata>::saveDataToFile()
{
    std::ofstream file(file_path);

    if (file.is_open())
    {
        for (const auto &pair : data)
        {
            file << pair.first << " = " << pair.second << std::endl;
        }

        file.close();
        std::cout << "Data saved to file." << std::endl;
    }
    else
    {
        std::cout << "Failed to open the file for writing." << std::endl;
    }
}

template <typename tipedata>
std::map<std::string, tipedata> barelangFile<tipedata>::get()
{
    return data;
}

template <typename tipedata>
void barelangFile<tipedata>::put(std::string key, tipedata value)
{
    data[key] = value;
}

template class barelangFile<int>;
template class barelangFile<std::string>;
template class barelangFile<float>;