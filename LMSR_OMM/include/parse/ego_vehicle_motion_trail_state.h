#ifndef _EGO_VEHICLE_MOTION_TRAIL_STATE_H_
#define _EGO_VEHICLE_MOTION_TRAIL_STATE_H_
#include "utils.h"

using namespace H5;
class Cxx_ego_vehicle_motion_trail_state
{
private:
    int length;
    std::vector<uint64_t> zeader_timestamp_ns;
    std::vector<int64_t> motion_timestamp;
    std::vector<uint32_t> number_of_trail_points;
    std::vector<std::vector<float> > delta_lat_position;
    std::vector<std::vector<float> > delta_lon_position;
    std::vector<std::vector<float> > heading;
    std::vector<std::vector<float> > lat_velocity;
    std::vector<std::vector<float> > lon_velocity;
    std::vector<std::vector<float> > yaw_rate;
    std::vector<uint8_t> isUpdate;

public:
    void setLength(int l){length=l;}
    int getLength() const {return length;}
    std::vector<uint64_t>& getZeaderTimestampNs() {return zeader_timestamp_ns;}
    std::vector<int64_t>& getMotionTimestamp() {return motion_timestamp;}
    std::vector<uint32_t>& getNumOfTrailPoints() {return number_of_trail_points;}
    std::vector<std::vector<float> >& getDeltaLatPos() {return delta_lat_position;}
    std::vector<std::vector<float> >& getDeltaLonPos() {return delta_lon_position;}
    std::vector<std::vector<float> >& getHeading() {return heading;}
    std::vector<std::vector<float> >& getLatVelocity() {return lat_velocity;}
    std::vector<std::vector<float> >& getLonVelocity() {return lon_velocity;}
    std::vector<std::vector<float> >& getYawRate() {return yaw_rate;}
    std::vector<uint8_t>& getIsUpdate() {return isUpdate;}
    void print()const;

};
using Cxx_motion_trail=Cxx_ego_vehicle_motion_trail_state;
class H5_ego_vehicle_motion_trail_state
{
private:
    DataSet zeader_timestamp_ns;
    DataSet motion_timestamp;
    DataSet number_of_trail_points;
    DataSet delta_lat_position;
    DataSet delta_lon_position;
    DataSet heading;
    DataSet lat_velocity;
    DataSet lon_velocity;
    DataSet yaw_rate;
    void parseZeaderTimestampNsTo(Cxx_motion_trail &) const;
    void parseMotionTimestampTo(Cxx_motion_trail &) const;
    void parseNumOfTrailPointsTo(Cxx_motion_trail &) const;
    void parseDeltaLatPosTo(Cxx_motion_trail &) const;
    void parseDeltaLonPosTo(Cxx_motion_trail &) const;
    void parseHeadingTo(Cxx_motion_trail &) const;
    void parseLatVelocityTo(Cxx_motion_trail &) const;
    void parseLonVelocityTo(Cxx_motion_trail &) const;
    void parseYawRateTo(Cxx_motion_trail &) const;  
public:
    void getDataSet(H5File&);
    void parseTo(Cxx_motion_trail &Cxx_motion_trail) const;
    DataSet getZeaderTimestampNs()const {return zeader_timestamp_ns;}
    DataSet getMotionTimestamp()const {return motion_timestamp;}
    DataSet getNumOfTrailPoints()const {return number_of_trail_points;}
    DataSet getDeltaLatPos()const {return delta_lat_position;}
    DataSet getDeltaLonPos()const {return delta_lon_position;}
    DataSet getHeading()const {return heading;}
    DataSet getLatVelocity()const {return lat_velocity;}
    DataSet getLonVelocity()const {return lon_velocity;}
    DataSet getYawRate()const {return yaw_rate;}

};
using H5_motion_trail=H5_ego_vehicle_motion_trail_state;

#endif