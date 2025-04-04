#ifndef _POSITIONING_SATELLITE_DEBUG_H_
#define _POSITIONING_SATELLITE_DEBUG_H_
#include "utils.h"

using namespace H5;
class Cxx_positioning_satellite_debug
{
private:
    int length;
    std::vector<uint64_t> zeader_timestamp_ns;
    std::vector<int64_t> lon;//nanodegrees
    std::vector<int64_t> lat;//nanodegrees
    std::vector<float> heading;//degrees
    std::vector<float> lon_variance;//meters2
    std::vector<float> lat_variance;//meters2
    std::vector<float> yaw_variance;//radians2
    std::vector<int32_t> filter_error_code;
    std::vector<int32_t> state;
    std::vector<uint8_t> valid;


public:
    void setLength(int l) {length=l;};
    int getLength() const {return length;}
    std::vector<uint64_t>& getZeaderTimestampNs() {return zeader_timestamp_ns;}
    std::vector<int64_t>& getLon() {return lon;}
    std::vector<int64_t>& getLat() {return lat;}
    std::vector<float>& getHeading() {return heading;}
    std::vector<float>& getLonVariance() {return lon_variance;}
    std::vector<float>& getLatVariance() {return lat_variance;}
    std::vector<float>& getYawVariance() {return yaw_variance;}
    std::vector<int32_t>& getFilterErrorCode() {return filter_error_code;}
    std::vector<int32_t>& getState() {return state;}
    std::vector<uint8_t>& getValid() {return valid;}
    void checkValid();

    void print()const;

};
using Cxx_psd=Cxx_positioning_satellite_debug;

class H5_positioning_satellite_debug
{
private:
    DataSet zeader_timestamp_ns;
    DataSet lon;//nanodegrees
    DataSet lat;//nanodegrees
    DataSet heading;//degrees
    DataSet lon_variance;//meters2
    DataSet lat_variance;//meters2
    DataSet yaw_variance;//radians2
    DataSet filter_error_code;
    DataSet state;
    void parseZeaderTimestampNsTo(Cxx_psd &) const;
    void parseLonTo(Cxx_psd &) const;
    void parseLatTo(Cxx_psd &) const;
    void parseHeadingTo(Cxx_psd &) const;
    void parseLonVarianceTo(Cxx_psd &) const;
    void parseLatVarianceTo(Cxx_psd &) const;
    void parseYawVarianceTo(Cxx_psd &) const;
    void parseFilterErrorCodeTo(Cxx_psd &) const;
    void parseStateTo(Cxx_psd &) const;

public:
    void getDataSet(H5File&);
    void parseTo(Cxx_psd &) const;
    DataSet getZeaderTimestampNs()const {return zeader_timestamp_ns;}
    DataSet getLon()const {return lon;}
    DataSet getLat()const {return lat;}
    DataSet getHeading()const {return heading;}
    DataSet getLonVariance()const {return lon_variance;}
    DataSet getLatVariance()const {return lat_variance;}
    DataSet getYawVariance()const {return yaw_variance;}
    DataSet getFilterErrorCode()const {return filter_error_code;}
    DataSet getState()const {return state;}
};
using H5_psd=H5_positioning_satellite_debug;

#endif