#ifndef _OXTS_H_
#define _OXTS_H_
#include "utils.h"

using namespace H5;
class Cxx_oxts
{
private:
    int length;
    std::vector<double> timestamp;
    std::vector<uint8_t> isValidLatLong;
    std::vector<uint8_t> isValidAltitude;
    std::vector<uint8_t> isValidHeading;
    std::vector<double> posLon;
    std::vector<double> posLat;
    std::vector<double> posAlt;
    std::vector<double> heading;
    std::vector<double> speed;
    std::vector<float> stdDevHeading;
    std::vector<float> stdDevPosDown;
    std::vector<float> stdDevPosEast;
    std::vector<float> stdDevPosNorth;
public:
    void setLength(int l) {length=l;}
    int getLength() const {return length;}
    std::vector<double>& getTimestamp() {return timestamp;}
    std::vector<uint8_t>& getIsValidLatLong() {return isValidLatLong;}
    std::vector<uint8_t>& getIsValidAltitude() {return isValidAltitude;}
    std::vector<uint8_t>& getIsValidHeading() {return isValidHeading;}
    std::vector<double>& getPosLon() {return posLon;}
    std::vector<double>& getPosLat() {return posLat;}
    std::vector<double>& getPosAlt() {return posAlt;}
    std::vector<double>& getHeading() {return heading;}
    std::vector<double>& getSpeed() {return speed;}
    std::vector<float>& getStdDevHeading() {return stdDevHeading;}
    std::vector<float>& getStdDevPosDown() {return stdDevPosDown;}
    std::vector<float>& getStdDevPosEast() {return stdDevPosEast;}
    std::vector<float>& getStdDevPosNorth() {return stdDevPosNorth;}
    void print() const;
    
};

class H5_oxts
{
private:
    DataSet timestamp;
    DataSet isValidLatLong;
    DataSet isValidAltitude;
    DataSet isValidHeading;
    DataSet posLon;
    DataSet posLat;
    DataSet posAlt;
    DataSet heading;
    DataSet speed;
    DataSet stdDevHeading;
    DataSet stdDevPosDown;
    DataSet stdDevPosEast;
    DataSet stdDevPosNorth;
    void parseTimestampTo(Cxx_oxts &) const;
    void parseIsValidLatLongTo(Cxx_oxts &) const;
    void parseIsValidAltitudeTo(Cxx_oxts &) const;
    void parseIsValidHeadingTo(Cxx_oxts &) const;
    void parsePosLonTo(Cxx_oxts &) const;
    void parsePosLatTo(Cxx_oxts &) const;
    void parsePosAltTo(Cxx_oxts &) const;
    void parseHeadingTo(Cxx_oxts &) const;
    void parseSpeedTo(Cxx_oxts &) const;
    void parseStdDevHeadingTo(Cxx_oxts &) const;
    void parseStdDevPosDownTo(Cxx_oxts &) const;
    void parseStdDevPosEastTo(Cxx_oxts &) const;
    void parseStdDevPosNorthTo(Cxx_oxts &) const;

public:
    void getDataSet(H5File&);
    void parseTo(Cxx_oxts &) const;
    DataSet getTimestamp() {return timestamp;}
    DataSet getIsValidLatLong() {return isValidLatLong;}
    DataSet getIsValidAltitude() {return isValidAltitude;}
    DataSet getIsValidHeading() {return isValidHeading;}
    DataSet getPosLon() {return posLon;}
    DataSet getPosLat() {return posLat;}
    DataSet getPosAlt() {return posAlt;}
    DataSet getHeading() {return heading;}
    DataSet getSpeed() {return speed;}
    DataSet getStdDevHeading() {return stdDevHeading;}
    DataSet getStdDevPosDown() {return stdDevPosDown;}
    DataSet getStdDevPosEast() {return stdDevPosEast;}
    DataSet getStdDevPosNorth() {return stdDevPosNorth;}
    void split(std::string,std::string,size_t,size_t)const;    
};
#endif