#ifndef _LANE_MARKERS_H_
#define _LANE_MARKERS_H_
#include "utils.h"

using namespace H5;

struct Double_polynomial
{
    float a;
    float b;
    float c;
    float d1;
    float d2;
    float end_distance;
    float start_distance;
    float transition_distance;

};
struct Sample_point
{
    float normal_std_dev_y;
    float x;
    float y;
};
struct Tracks
{
    int32_t current_color;
    int32_t next_color;
    float distance_to_next_color;
    float lateral_offset;
    int32_t current_type;
    int32_t next_type;
    float distance_to_next_type;
    float width;
};
struct Lane_info
{
    // float association_probability;
    uint8_t cardinality;
    int32_t current_color;
    int32_t next_color;
    float distance_to_next_color;
    int32_t current_type;
    int32_t next_type;
    float distance_to_next_type;
    float covariance[16];
    Double_polynomial polynomial;
    float existence_probability;
    uint64_t id;
    float marking_width;
    float model_error;
    uint8_t n_samplepoints_in_range;
    std::vector<Sample_point> sample_points;
    Tracks tracks[6];
    uint8_t valid;
};
class Cxx_lane_markers
{
private:
    int length;
    std::vector<uint64_t> zeader_timestamp_ns;
    std::vector<int32_t> lane_change;
    std::vector<Lane_info> left;
    std::vector<Lane_info> right;
    std::vector<Lane_info> second_left;
    std::vector<Lane_info> second_right;
    std::vector<int64_t> vision_timestamp;
    std::vector<uint8_t> isUpdate;

public:
    void setLength(int l){length=l;}
    int getLength() const {return length;}
    std::vector<uint64_t>& getZeaderTimestampNs() {return zeader_timestamp_ns;}
    std::vector<int32_t>& getLaneChange() {return lane_change;}
    std::vector<Lane_info>& getLeft() {return left;}
    std::vector<Lane_info>& getRight() {return right;}
    std::vector<Lane_info>& getSecondLeft() {return second_left;}
    std::vector<Lane_info>& getSecondRight() {return second_right;}
    std::vector<int64_t>& getVisionTimestamp() {return vision_timestamp;}
    std::vector<uint8_t>& getIsUpdate() {return isUpdate;}
    void print()const;
};
class H5_lane_markers
{
    enum Lane{Left,Right,Second_left,Second_right};
private:
    DataSet zeader_timestamp_ns;
    DataSet lane_change;
    DataSet vision_timestamp;
    Group left;
    Group right;
    Group second_left;
    Group second_right;
    void parseZeaderTimestampNsTo(Cxx_lane_markers &) const;
    void parseLaneChangeTo(Cxx_lane_markers &) const;
    void parseVisionTimestampTo(Cxx_lane_markers &) const;
    void parseLaneTo(const Group *, Cxx_lane_markers &,Lane) const;

public:
    void getDataSet(H5File&);
    void parseTo(Cxx_lane_markers &cxx_lane_markers) const;
    DataSet getZeaderTimestampNs()const {return zeader_timestamp_ns;}
    DataSet getLaneChange()const {return lane_change;}
    DataSet getVisionTimestamp()const {return vision_timestamp;}
    Group getLeft() const {return left;};
    Group getRight() const {return right;};
    Group getSecondLeft() const {return second_left;};
    Group getSecondRight() const {return second_right;};

};
#endif