#ifndef _HMM_LANE_H_
#define _HMM_LANE_H_
#include "utils.h"
#include "parse/camera_lane_marker_detections.h"
// #include "hmm_madmap.h"

#include <deque>
//func:车道线按实例去分割

// enum Direct
// {
//     UNKNOWN,
//     LEFT,
//     RIGHT,
//     FRONT,
//     BACK
// };


enum LaneCenterInfo
{
    Unknown,
    Normal,
    Narrow,
    Broad,
    NoLeft,
    NoRight,
    FollowLeft,
    FollowRight

};


//func:class车道
class LaneCenter{
public:
    LaneCenterInfo center_info; 
    int left_instance_id;
    int right_instance_id;
    int left_marker_type;
    int right_marker_type;
    float x;
    float y;
    float width;

    bool operator!=(const LaneCenter& l) const {
    return !(this->x==l.x && this->y==l.y );
    }    
};
class LaneCenters{
public:
    std::vector<LaneCenterInfo> center_info; 
    std::vector<int> left_instance_id;
    std::vector<int> right_instance_id;
    std::vector<int> left_marker_type;
    std::vector<int> right_marker_type;
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> width;
    LaneCenters(std::deque<LaneCenter>& lcs)
    {
        for(auto& lc:lcs)
        {
            this->center_info.push_back(lc.center_info);
            this->left_instance_id.push_back(lc.left_instance_id);
            this->right_instance_id.push_back(lc.right_instance_id);
            this->left_marker_type.push_back(lc.left_marker_type);
            this->right_marker_type.push_back(lc.right_marker_type);
            this->x.push_back(lc.x);
            this->y.push_back(lc.y);
            this->width.push_back(lc.width);
        }
    }
};
class LaneInstance{
public:
    int instance_id;
    int left_marker_id=-1;
    int right_marker_id=-1;
    int left_marker_type;
    int right_marker_type;
    std::deque<LaneCenter> lane_centers;
private:

};
class LaneDetection{
public:
    std::vector<int> preview_lane_markers_id;
};
//
class LaneMarkerInstance{
public:
    int instance_id;
    int color;
    int type;
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    bool track=false;
    double preview_avg_y;
    std::vector<float> std_dev;
    std::vector<float> x;
    std::vector<float> y;
    static void devideInstance(std::vector<Edge_point>& edge_points,std::unordered_map<int,LaneMarkerInstance>& lane_marker_instances);
    static void previewLaneMarkers(std::unordered_map<int,LaneMarkerInstance>& lane_marker_instances,double LANE_PREVIEW_DISTANCE,double LANE_PREVIEW_THRESHOLD);
    static void previewLanes(std::unordered_map<int,LaneMarkerInstance>& lane_marker_instances,std::unordered_map<int,LaneInstance>& lanes,double LANE_WIDTH,double LANE_FITTING_DEFINITION,double LANE_PREVIEW_DISTANCE,double LANE_PREVIEW_THRESHOLD,double LANE_WIDTH_ERROR_TOLERANCE_FACTOR,double LANE_PREVIEW_MIN,double LANE_PREVIEW_MAX);
    static LaneCenter laneCenterFitting(std::unordered_map<int,LaneMarkerInstance>& lane_marker_instances,double x,double y,double LANE_WIDTH,double LANE_WIDTH_ERROR_TOLERANCE_FACTOR,double LANE_FITTING_DEFINITION);
private:
};
#endif