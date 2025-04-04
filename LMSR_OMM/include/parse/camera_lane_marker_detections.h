#ifndef _CAMERA_LANE_MARKER_DETECTIONS_H_
#define _CAMERA_LANE_MARKER_DETECTIONS_H_
#include "utils.h"

using namespace H5;

struct Edge_point
{
    uint8_t instance_id;
    int32_t color;
    int32_t type;
    float std_dev;
    float x;
    float y;
};

class Cxx_camera_lane_marker_detections
{
private:
    int length;
    std::vector<uint64_t> zeader_timestamp_ns;
    std::vector<int64_t> vision_timestamp;
    std::vector<uint32_t> n_valid_points;
    std::vector<std::vector<Edge_point>> edge_points;
public:
    void setLength(int l){length=l;}
    int getLength() const {return length;}
    std::vector<uint64_t>& getZeaderTimestampNs() {return zeader_timestamp_ns;}
    std::vector<int64_t>& getVisionTimestamp() {return vision_timestamp;}
    std::vector<std::vector<Edge_point>>& getEdgePoints(){return edge_points;}
    std::vector<uint32_t>& getNumOfValidPoints() {return n_valid_points;}

};
using Cxx_clmd=Cxx_camera_lane_marker_detections;
class H5_camera_lane_marker_detections
{
private:
    DataSet zeader_timestamp_ns;
    DataSet vision_timestamp;
    DataSet n_valid_points;
    Group points;
    void parseZeaderTimestampNsTo(Cxx_clmd&) const;
    void parseVisionTimestampTo(Cxx_clmd&)const ;
    void parseNumOfValidPointsTo(Cxx_clmd&)const ;
    void parsePointsTo(Cxx_clmd&)const;
public:
    void getDataSet(H5File&,bool);
    void parseTo(Cxx_clmd &) const;
    DataSet getZeaderTimestampNs()const {return zeader_timestamp_ns;}
    DataSet getVisionTimestamp()const {return vision_timestamp;}
    Group getPoints()const {return points;}
    DataSet getNumOfValidPoints(Cxx_clmd &) const{return n_valid_points;}

};
using H5_clmd=H5_camera_lane_marker_detections;

#endif