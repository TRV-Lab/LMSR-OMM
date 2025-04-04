#ifndef _VISUAL_ODOMETRY_VERIFICATION_H_
#define _VISUAL_ODOMETRY_VERIFICATION_H_
#include "utils.h"

using namespace H5;
class Cxx_visual_odometry_verification
{
private:
    int length;    
    std::vector<uint64_t> zeader_timestamp_ns;
    std::vector<int64_t> timestamp;
    std::vector<int64_t> previous_timestamp;
    std::vector<std::vector<float> > vehicle_odometry_pose;
    std::vector<std::vector<float> > visual_odometry_pose;
    std::vector<int32_t> vehicle_odometry_quality;
    std::vector<int32_t> visual_odometry_quality;
public:
    void setLength(int l) {length=l;}
    int getLength() const {return length;}
    std::vector<uint64_t>& getZeaderTimestampNs() {return zeader_timestamp_ns;}
    std::vector<int64_t>& getTimestamp() {return timestamp;}
    std::vector<int64_t>& getPreviousTimestamp() {return previous_timestamp;}
    std::vector<std::vector<float> >& getVehicleOdomPose() {return vehicle_odometry_pose;}
    std::vector<std::vector<float> >& getVisualOdomPose() {return vehicle_odometry_pose;}
    std::vector<int32_t>& getVehicleOdomQuality() {return vehicle_odometry_quality;}
    std::vector<int32_t>& getVisualOdomQuality() {return visual_odometry_quality;}
    void print()const;

};
using Cxx_vov=Cxx_visual_odometry_verification;
class H5_visual_odometry_verification
{
private:
    DataSet zeader_timestamp_ns;
    DataSet timestamp;
    DataSet previous_timestamp;
    DataSet vehicle_odometry_pose;
    DataSet visual_odometry_pose;
    DataSet vehicle_odometry_quality;
    DataSet visual_odometry_quality;
    void parseZeaderTimestampNsTo(Cxx_vov &) const;
    void parseTimestampTo(Cxx_vov &) const;
    void parsePreviousTimestampTo(Cxx_vov &) const;
    void parseVehicleOdomPoseTo(Cxx_vov &) const;
    void parseVisualOdomPoseTo(Cxx_vov &) const;
    void parseVehicleOdomQualityTo(Cxx_vov &) const;
    void parseVisualOdomQualityTo(Cxx_vov &) const;
public:
    void getDataSet(H5File&);
    void parseTo(Cxx_vov &) const;
    DataSet getZeaderTimestampNs()const {return zeader_timestamp_ns;}
    DataSet getTimestamp(Cxx_vov &) const{return timestamp;}
    DataSet getPreviousTimestamp(Cxx_vov &) const{return previous_timestamp;}
    DataSet getVehicleOdomPose(Cxx_vov &) const{return vehicle_odometry_pose;}
    DataSet getVisualOdomPose(Cxx_vov &) const{return visual_odometry_pose;}
    DataSet getVehicleOdomQuality(Cxx_vov &) const{return vehicle_odometry_quality;}
    DataSet getVisualOdomQuality(Cxx_vov &) const{return visual_odometry_quality;}
};
using H5_vov=H5_visual_odometry_verification;
#endif