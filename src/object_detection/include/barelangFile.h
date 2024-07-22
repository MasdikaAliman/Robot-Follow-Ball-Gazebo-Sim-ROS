#ifndef BARELANG63_FILE_HEADER
#define BARELANG63_FILE_HEADER

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>

template <class tipedata>
class barelangFile
{
private:
    
    std::string file_path;

public:
    barelangFile() = default;
    std::map<std::string, tipedata> data;
    barelangFile(const std::string &filename){
        file_path = filename;
        parseData();
    }

    void init(const std::string &filename){
        file_path = filename;
        parseData();
    }

    void put(std::string, tipedata);

    
    tipedata* operator[](std::string key){
        return &data[key];
    }

    bool parseData();

    void saveDataToFile();

    std::map<std::string, tipedata> get();

    // ~barelangFile() = default;
};

#endif
