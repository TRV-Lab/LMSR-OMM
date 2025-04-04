#ifndef _HOLISTIC_PATH_H_
#define _HOLISTIC_PATH_H_
#include "utils.h"

using namespace H5;
class PointWithDeviation
{
public:
    float x;
    float y;
    float z;
    float x_deviation;
    float y_deviation;
    float z_deviation;
};
class Cxx_holistic_path
{
private:
    int length;    
    std::vector<uint64_t> zeader_timestamp_ns;
    std::vector<int64_t> timestamp;
    std::vector<uint8_t> n_valid_points;
    std::vector<std::vector<PointWithDeviation>> points;
public:
    void setLength(int l) {length=l;}
    int getLength() const {return length;}
    std::vector<uint64_t>& getZeaderTimestampNs() {return zeader_timestamp_ns;}
    std::vector<int64_t>& getTimestamp() {return timestamp;}
    std::vector<uint8_t>& getNumOfValidPoints() {return n_valid_points;}
    std::vector<std::vector<PointWithDeviation>>& getPoints() {return points;}
    void print()const;

};
using Cxx_hp=Cxx_holistic_path;
class H5_holistic_path
{
private:
    DataSet zeader_timestamp_ns;
    DataSet timestamp;
    DataSet n_valid_points;
    DataSet points_x;
    DataSet points_y;
    DataSet points_z;
    DataSet points_x_deviation;
    DataSet points_y_deviation;
    DataSet points_z_deviation;

    void parseZeaderTimestampNsTo(Cxx_hp &) const;
    void parseTimestampTo(Cxx_hp &) const;
    void parseNumOfValidPoints(Cxx_hp &) const;
    void parsePoints(Cxx_hp &) const;
 
public:
    void getDataSet(H5File&,bool);
    void parseTo(Cxx_hp &) const;
    DataSet getZeaderTimestampNs()const {return zeader_timestamp_ns;}
    DataSet getTimestamp(Cxx_hp &) const{return timestamp;}
    DataSet getNumOfValidPoints(Cxx_hp &) const{return n_valid_points;}
    DataSet getPointsX(Cxx_hp &) const{return points_x;}
    DataSet getPointsY(Cxx_hp &) const{return points_y;}
    DataSet getPointsZ(Cxx_hp &) const{return points_z;}
    DataSet getPointsXDev(Cxx_hp &) const{return points_x_deviation;}
    DataSet getPointsYDev(Cxx_hp &) const{return points_y_deviation;}
    DataSet getPointsZDev(Cxx_hp &) const{return points_z_deviation;}
};
using H5_hp=H5_holistic_path;
#endif