#ifndef _SENSOR_FUSION_H_
#define _SENSOR_FUSION_H_
#include "parse/satellite.h"
#include "parse/positioning_satellite_debug.h"
#include "parse/motion_data.h"
#include "parse/lane_markers.h"
#include "parse/vision_traffic_signs.h"
#include "parse/ego_vehicle_motion_trail_state.h"
#include "parse/visual_odometry_verification.h"
#include "parse/localization_data.h"
#include "parse/oxts.h"
#include "parse/nibbler.h"

enum GT_TYPE{OXTS,Nibbler};
enum DATA_TYPE{PSD,Satellite,MRE,MRE2};
class GroundTruth
{
private:
    Cxx_oxts oxts;
    Cxx_nibbler nibbler;
public:
    bool oxts_flag=false,nibbler_flag=false;
    Cxx_oxts& getOxts() {return oxts;}
    Cxx_nibbler& getNibbler() {return nibbler;}
    GroundTruth(){}
    GroundTruth(std::string);
};
// class Output
// {
// public:
//     int length;
//     std::vector<double> timestamps;
//     std::vector<double> lon;
//     std::vector<double> lat;
//     std::vector<float> heading;
//     void fromPSD(Cxx_psd);
//     void fromSatellite(Cxx_satellite);
// };
class SensorFusion
{
private:
    int length;
    std::vector<double> timestamps;
    Cxx_satellite satellite;
    Cxx_psd psd;
    Cxx_motion motion;
    Cxx_lane_markers lane_markers;
    Cxx_vts vts;
    Cxx_vts motion_trail;
    Cxx_vov vov;
    Cxx_localization localization,localization2;
    GroundTruth gt;
    std::vector<uint8_t> nibbler_valid,oxts_valid;//,psd_valid,satellite_valid;
    double lonlat_init[2];
    

public:
    std::string fout_path;
    std::string time_string;
    SensorFusion(std::string);
    void calValidOutput(DATA_TYPE,GT_TYPE,std::vector<uint8_t>&);
    void checkValid(std::string save_path);
    void fout_result_psd(std::string,GT_TYPE,std::vector<uint8_t>& );
    void fout_result_satellite(std::string,GT_TYPE,std::vector<uint8_t>& );
    void fout_result_mre(std::string,GT_TYPE,std::vector<uint8_t>& );    
    void fout_oxts(std::string,std::vector<uint8_t>& );
    void fout_nibbler(std::string,std::vector<uint8_t>& );
    // Output psd_output;
    // Output satellite_output;
    // void fout_diff(Output,GT_TYPE,std::string);

};


#endif
