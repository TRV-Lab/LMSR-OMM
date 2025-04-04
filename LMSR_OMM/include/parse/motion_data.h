#ifndef _MOTION_DATA_H_
#define _MOTION_DATA_H_
#include "utils.h"

using namespace H5;
class Cxx_motion
{
private:
    int length;
    std::vector<uint64_t> zeader_timestamp_ns;
    std::vector<int64_t> motion_timestamp;
    std::vector<int32_t> quality;
    std::vector<int32_t> motion_state;
    std::vector<float> lon_acc;
    std::vector<float> lon_vel;
    std::vector<float> lat_acc;
    std::vector<float> lat_vel;
    std::vector<float> roll_rate;
    std::vector<float> pitch_rate;
    std::vector<float> yaw_rate;
    std::vector<uint8_t> isUpdate;


public:
    void setLength(int l){length=l;}
    int getLength() const {return length;}
    std::vector<uint64_t>& getZeaderTimestampNs() {return zeader_timestamp_ns;}
    std::vector<int64_t>& getMotionTimestamp() {return motion_timestamp;}
    std::vector<int32_t>& getQuality() {return quality;}
    std::vector<int32_t>& getMotionState() {return motion_state;}
    std::vector<float>& getLonAcc() {return lon_acc;}
    std::vector<float>& getLonVel() {return lon_vel;}
    std::vector<float>& getLatAcc() {return lat_acc;}
    std::vector<float>& getLatVel() {return lat_vel;}
    std::vector<float>& getRollRate() {return roll_rate;}
    std::vector<float>& getPitchRate() {return pitch_rate;}
    std::vector<float>& getYawRate() {return yaw_rate;}
    std::vector<uint8_t>& getIsUpdate() {return isUpdate;}
    void print()const;

};
class H5_motion
{
private:
    DataSet zeader_timestamp_ns;
    DataSet motion_timestamp;
    DataSet quality;
    DataSet motion_state;
    DataSet lon_acc;
    DataSet lon_vel;
    DataSet lat_acc;
    DataSet lat_vel;
    DataSet roll_rate;
    DataSet pitch_rate;
    DataSet yaw_rate;
    void parseZeaderTimestampNsTo(Cxx_motion &) const;
    void parseMotionTimestampTo(Cxx_motion &) const;
    void parseQualityTo(Cxx_motion &) const;
    void parseMotionStateTo(Cxx_motion &) const;
    void parseLonAccTo(Cxx_motion &) const;
    void parseLonVelTo(Cxx_motion &) const;
    void parseLatAccTo(Cxx_motion &) const;
    void parseLatVelTo(Cxx_motion &) const;
    void parseRollRateTo(Cxx_motion &) const;
    void parsePitchRateTo(Cxx_motion &) const;
    void parseYawRateTo(Cxx_motion &) const;   
public:
    void getDataSet(H5File&);
    void parseTo(Cxx_motion &cxx_motion) const;
    DataSet getZeaderTimestampNs()const {return zeader_timestamp_ns;}
    DataSet getMotionTimestamp()const {return motion_timestamp;}
    DataSet getQuality()const {return quality;}
    DataSet getMotionState()const {return motion_state;}
    DataSet getLonAcc()const {return lon_acc;}
    DataSet getLonVel()const {return lon_vel;}
    DataSet getLatAcc()const {return lat_acc;}
    DataSet getLatVel()const {return lat_vel;}
    DataSet getRollRate()const {return roll_rate;}
    DataSet getPitchRate()const {return pitch_rate;}
    DataSet getYawRate()const {return yaw_rate;}

};
#endif