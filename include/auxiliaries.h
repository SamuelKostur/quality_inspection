#pragma once

#include <ros/ros.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

template<typename T>
int loadTxtTo2DVector(std::string path, std::vector<std::vector<T>>& vector, int numRowElements, int numRows){
    //if numRowElements/numRows is equal to -1, then reading until the end of the row/file
    //open file
    std::ifstream file;
    file.open(path);
    if(!file){
        ROS_ERROR("%s",std::string("Cannot open file " + path + " ...").c_str());
        return -1;
    }

    //parse file
    std::string line;
    int curRow = 0;
    int curElement = 0;
    T readData;
    while(std::getline(file, line)){
        if((numRows <= curRow) && (numRows != -1)){
            ROS_ERROR("%s",std::string("Non matching number of rows in file " + path + " expected " +
            std::to_string(numRows) + " present " + std::to_string(curRow + 1) +" ...").c_str());
            return -1;
        }
        vector.push_back(std::vector<T>());
        std::stringstream lineStream(line);

        while(lineStream >> readData){
            if((numRowElements <= curElement) && (numRowElements != -1)){
                ROS_ERROR("%s",std::string("Non matching number of row elements in file " + path + " expected " +
                std::to_string(numRowElements) + " present " + std::to_string(curElement + 1) +" ...").c_str());
                return -1;
            }
            vector.at(curRow).push_back(readData);
            curElement++;
        }
        if((numRowElements > curElement) && (numRowElements != -1)){
            ROS_ERROR("%s",std::string("Non matching number of row elements in file " + path + " expected " +
            std::to_string(numRowElements) + " present " + std::to_string(curElement) +" ...").c_str());
            return -1;
        }
        curElement = 0;
        curRow++;
    }
    if((numRows > curRow) && (numRows != -1)){
        ROS_ERROR("%s",std::string("Non matching number of rows in file " + path + " expected " +
        std::to_string(numRows) + " present " + std::to_string(curRow) +" ...").c_str());
        return -1;
    }

    file.close();
    return 0;
}