#ifndef _SATELLITE_H_
#define _SATELLITE_H_
#include "utils.h"

using namespace H5;
class Cxx_satellite
{
private:
    int length;
    std::vector<uint64_t> zeader_timestamp_ns;
    std::vector<uint8_t> valid;
    std::vector<int64_t> satellite_timestamp;
    std::vector<int64_t> longposn;
    std::vector<int64_t> latposn;
    std::vector<float> altitude;
    std::vector<float> heading;
    std::vector<float> speed;
    std::vector<float> horizontal_std;
    std::vector<float> heading_std;
    std::vector<float> speed_std;
    std::vector<uint8_t> isUpdate;
public:
    void setLength(int l) {length=l;}
    int getLength() const {return length;}
    std::vector<uint64_t>& getZeaderTimestampNs() {return zeader_timestamp_ns;}
    std::vector<uint8_t>& getValid() {return valid;}
    std::vector<int64_t>& getSatelliteTimestamp() {return satellite_timestamp;}
    std::vector<int64_t>& getLongPosN() {return longposn;}
    std::vector<int64_t>& getLatPosN() {return latposn;}
    std::vector<float>& getAltitude() {return altitude;}
    std::vector<float>& getHeading() {return heading;}
    std::vector<float>& getSpeed() {return speed;}
    std::vector<float>& getHorizontalStd() {return horizontal_std;}
    std::vector<float>& getHeadingStd() {return heading_std;}
    std::vector<float>& getSpeedStd() {return speed_std;}
    std::vector<uint8_t>& getIsUpdate() {return isUpdate;}
    void print()const;

};
class H5_satellite
{
private:
    DataSet zeader_timestamp_ns;
    DataSet valid;
    DataSet satellite_timestamp;
    DataSet longposn;
    DataSet latposn;
    DataSet altitude;
    DataSet heading;
    DataSet speed;
    DataSet horizontal_standard_deviation;
    DataSet heading_standard_deviation;
    DataSet speed_standard_deviation;
    void parseZeaderTimestampNsTo(Cxx_satellite &) const;
    void parseValidTo(Cxx_satellite &) const;
    void parseSatelliteTimestampTo(Cxx_satellite &) const;
    void parseLongPosNTo(Cxx_satellite &) const;
    void parseLatPosNTo(Cxx_satellite &) const;
    void parseAltitudeTo(Cxx_satellite &) const;
    void parseHeadingTo(Cxx_satellite &) const;
    void parseSpeedTo(Cxx_satellite &) const;
    void parseHorizontalStdTo(Cxx_satellite &) const;
    void parseHeadingStdTo(Cxx_satellite &) const;
    void parseSpeedStdTo(Cxx_satellite &) const;
    void checkUpdate(Cxx_satellite &) const;

public:
    void getDataSet(H5File&);
    void parseTo(Cxx_satellite &) const;
    DataSet getZeaderTimestampNs()const {return zeader_timestamp_ns;}
    DataSet getValid()const {return valid;}
    DataSet getSatelliteTimestamp()const {return satellite_timestamp;}
    DataSet getLongPosN()const {return longposn;}
    DataSet getLatPosN()const {return latposn;}
    DataSet getAltitude()const {return altitude;}
    DataSet getHeading()const {return heading;}
    DataSet getSpeed()const {return speed;}
    DataSet getHorizontalStd()const {return horizontal_standard_deviation;}
    DataSet getHeadingStd()const {return heading_standard_deviation;}
    DataSet getSpeedStd()const {return speed_standard_deviation;}
};
#endif