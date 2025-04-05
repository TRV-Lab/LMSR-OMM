#ifndef _HMM_MADMAP_H_
#define _HMM_MADMAP_H_
#include <bits/stdc++.h>
#include <sys/resource.h>
#include <deque>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include "utils.h"
#include "matplotlibcpp.h"

#include "kalman_filter.h"
#include "MadMapStandard.h"
#include "Clock.h"
#include "Types.h"
#include "parse/positioning_satellite_debug.h"
#include "parse/holistic_path.h"
#include "parse/oxts.h"
#include "parse/satellite.h"
#include "parse/camera_lane_marker_detections.h"
#include "hmm_lane.h"
#include "hmm_icp.h"

namespace plt=matplotlibcpp;

// #define N 121380
// #define pi 3.1415926535897932384626433832795
// #define EARTH_RADIUS 6378.137 //地球半径 KM

class LatLon
{
public:
    double lat;
    double lon;
    LatLon(double lat_,double lon_)
    {
        lat=lat_;
        lon=lon_;
    }
    LatLon(){}
    static double distance(LatLon x, LatLon a, LatLon b );
    static LatLon projection_point(LatLon x, LatLon a, LatLon b );
    static double rad(double d);

};
struct hash_MapObjectId{
    size_t operator()(const mad::MapObjectId& p) const {//别忘记const
        return p.getPart1()+p.getPart2();
    }
};
class Road_MADMAP
{
private:
    
public:
    // int64_t id;
    mad::MapObjectId id;
    std::string road_class;
    std::string link_type;
    std::vector<mad::MapObjectId> start_node;
    std::vector<mad::MapObjectId> end_node;
    std::vector<LatLon> node; //from node 0
    mad::LinkDirection direction;
    double length;
    bool interpolated=false;
    bool isTunnel=false;
    bool isBridge=false;
    std::unordered_map<mad::MapObjectId,double,hash_MapObjectId> tranPsb;
    // std::unordered_map<mad::MapObjectId,double,hash_MapObjectId> preview_tranPsb;
    static void roadsCorrection(std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId>& roads, mad::MapObjectId fork_road_id,double INTERP_LENGTH,double INTERP_INTERVAL,double INTERP_DELTA_ANGLE_CONDITION);
};//路网
// class Fork
// {
// public:
//     mad::MapObjectId id;
//     std::unordered_map<mad::MapObjectId,double,hash_MapObjectId> tranPsb;
// };
class LanePoint: public LatLon
{
public:
    double alt;
    double heading;//rad, 0:north, +: clockwise
    int type;
    int num_hypothesis=0;
    std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId> candidate_roads;
    std::unordered_map<mad::MapObjectId, std::pair<double, mad::MapObjectId> ,hash_MapObjectId> joint_prob;
    std::unordered_map<mad::MapObjectId,LatLon,hash_MapObjectId > trace_projection;
    std::unordered_map<mad::MapObjectId,double,hash_MapObjectId > projection_distance;
    std::unordered_map<mad::MapObjectId,double,hash_MapObjectId > road_heading;
    std::unordered_map<mad::MapObjectId,double,hash_MapObjectId> distanceEmisPsb,angleEmisPsb;
    std::vector<std::pair<mad::MapObjectId, std::pair<double, mad::MapObjectId> > > sorted_joint_prob;
    LanePoint(double lat_,double lon_):LatLon(lat_,lon_){}

};
class LaneMarkerSDAssociation
{
public:
    std::vector<LanePoint> points;
    // std::unordered_map<mad::MapObjectId,float,hash_MapObjectId> associate_prob;
    std::unordered_map<mad::MapObjectId,double,hash_MapObjectId> road_probs;
    // std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId> candidate_roads;
    static void candidateRoads(mad::MadMapStandard &madmapSD,std::unordered_map<mad::MapObjectId,Road_MADMAP,hash_MapObjectId>& hypothesis_roads,LaneMarkerSDAssociation &lane_marker,double query_distance);
    static void projection(LaneMarkerSDAssociation &lane_marker);
    static void calDistanceEmisPsb(LaneMarkerSDAssociation &lane_marker,double LANE_PD_LATERAL_STD);
    static void calAngleEmisPsb(LaneMarkerSDAssociation &lane_marker,double ANGLE_POWER_EXPONENT,double ANGLE_POSSI_DEFAULT);  
    static void calPriorTranPsb(LaneMarkerSDAssociation &lane_marker);
    static void Viterbi(LaneMarkerSDAssociation &lane_marker,std::unordered_map<mad::MapObjectId,std::unordered_map<mad::MapObjectId,double,hash_MapObjectId>,hash_MapObjectId>& global_tranPsb,double MIN_PROBABILITY_THRESHOLD);
    static void validHypothesis(LanePoint& lp,double MIN_PROBABILITY_THRESHOLD,bool removeLowProb);
    static bool isSameSource(LanePoint& lane_point);
    static void refine(mad::MadMapStandard &madmapSD,LaneMarkerSDAssociation &lane_marker);
};
enum ProjectDirection
{
    Left,
    Right
};
// class HPNode: public LatLon
// {
// public:
//     double heading;//rad, 0:north, +: clockwise
//     double lateral_std;
//     int num_hypothesis=0;
//     mad::MapObjectId mm_result=mad::MapObjectId();
//     LatLon increment;

// };
class TraceNode: public LatLon
{
private:

public:
    double time;
    double heading;//rad, 0:north, +: clockwise
    double speed;
    bool isFiltered=false;
    bool isConverged=false;
    bool isLaneAvail=false;
    double icp_residual=-1;
    int icp_match_num;
    int icp_iter;
    LatLon corrected_pos;
    double corrected_heading;
    // double lateral_err;
    double lateral_std;
    double confidence;
    int num_hypothesis=0;
    mad::MapObjectId mm_result=mad::MapObjectId();
    std::vector<TraceNode> holistic_path;
    LatLon holistic_path_increment;
    mad::MapObjectId fork_road_id;
    std::vector<Edge_point> lane_markers;
    pcl::PointCloud<pcl::PointXYZL>::Ptr lane_marker_pointcloud;
    std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId> candidate_roads;
    std::unordered_map<mad::MapObjectId, std::pair<double, mad::MapObjectId> ,hash_MapObjectId> joint_prob;
    std::unordered_map<mad::MapObjectId,LatLon,hash_MapObjectId > trace_projection;
    std::unordered_map<mad::MapObjectId,double,hash_MapObjectId > projection_distance;
    std::unordered_map<mad::MapObjectId,double,hash_MapObjectId > road_heading;
    std::unordered_map<mad::MapObjectId,double,hash_MapObjectId> distanceEmisPsb;
    std::unordered_map<mad::MapObjectId,double,hash_MapObjectId> angleEmisPsb;
    std::unordered_map<mad::MapObjectId,double,hash_MapObjectId> OBDSC_EmisPsb;
    std::unordered_map<mad::MapObjectId,double,hash_MapObjectId> hpEmisPsb;

    // std::unordered_map<int,LaneMarkerSDAssociation > localLaneMap;
    std::unordered_set<int> local_lane_index;
    std::unordered_map<int,double > lane_projection_distance;
    std::unordered_map<int,double > lane_projection_heading;
    std::unordered_map<int,LatLon > lane_projection_point;
    std::unordered_map<int,ProjectDirection > lane_projection_direction;
    std::unordered_map<mad::MapObjectId,double,hash_MapObjectId> laneEmisPsb;
    std::vector<std::pair<mad::MapObjectId, std::pair<double, mad::MapObjectId> > > sorted_joint_prob;
    std::unordered_map<int,LaneMarkerInstance> lane_marker_instances;
    std::unordered_map<int,LaneInstance> lanes;
    TraceNode(double lat_,double lon_):LatLon(lat_,lon_){}
    TraceNode(){}
    static void candidateRoad(mad::MadMapStandard &madmapSD,TraceNode &trace_node,mad::MapObjectId fork_road_id);
    static void candidateRoads(mad::MadMapStandard &madmapSD,TraceNode &trace_node,std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId>& candidate_history,double query_distance,bool use_bias=false);
    static void calPriorTranPsb(TraceNode &trace_node,std::unordered_map<mad::MapObjectId,std::unordered_map<mad::MapObjectId,double,hash_MapObjectId>,hash_MapObjectId>& global_tranPsb,double MIN_PROBABILITY_THRESHOLD);//,double distance);
    static void calVisionTranPsb(mad::MadMapStandard &madmapSD,TraceNode &trace_node,std::unordered_map<int,LaneMarkerSDAssociation>& lane_marker_trackings,const std::unordered_map<mad::MapObjectId,std::unordered_map<int,double>,hash_MapObjectId>& lane_road_association,mad::MapObjectId fork_road_id,double fork_distance,std::ofstream& hp_out, double ANGLE_POWER_EXPONENT,double ANGLE_POSSI_DEFAULT,double MIN_PROBABILITY_THRESHOLD,double MAX_DISTANCE_THRESHOLD,bool USE_LANE_MARKER,double PD_LATERAL_STD,double QUERY_DISTANCE,double VISION_TRANS_DISCOUT_FACTOR,double LANE_PROB_SUM_LIMIT,double LANE_WIDTH);
    static void calVisionEmissPsb(mad::MadMapStandard &madmapSD,TraceNode &trace_node,std::unordered_map<int,LaneMarkerSDAssociation>& lane_marker_trackings,const std::unordered_map<mad::MapObjectId,std::unordered_map<int,double>,hash_MapObjectId>& lane_road_association,mad::MapObjectId fork_road_id,double fork_distance,std::ofstream& hp_out, double ANGLE_POWER_EXPONENT,double ANGLE_POSSI_DEFAULT,double MIN_PROBABILITY_THRESHOLD,double MAX_DISTANCE_THRESHOLD,bool USE_LANE_MARKER,double PD_LATERAL_STD,double QUERY_DISTANCE,double VISION_TRANS_DISCOUT_FACTOR,double LANE_PROB_SUM_LIMIT,double LANE_WIDTH);
    static void calDistanceEmisPsb(TraceNode &trace_node,double PD_LATERAL_STD);
    static void calAngleEmisPsb(TraceNode &trace_node,double ANGLE_POWER_EXPONENT,double ANGLE_POSSI_DEFAULT);  
    static double calAngleEmisPsb(double heading1,double heading2,double ANGLE_POWER_EXPONENT,double ANGLE_POSSI_DEFAULT);
    // static void localLaneMap(TraceNode &trace_node,std::unordered_map<mad::MapObjectId,std::unordered_map<int,double>,hash_MapObjectId>& lane_road_association,std::unordered_set<int>& local_lane_index);
    static void laneProjection(TraceNode &trace_node,std::unordered_map<int,LaneMarkerSDAssociation>& lane_marker_trackings,const std::unordered_map<mad::MapObjectId,std::unordered_map<int,double>,hash_MapObjectId>& lane_road_association);
    static void calLaneEmisPsb(TraceNode &trace_node,std::unordered_map<int,LaneMarkerSDAssociation>& lane_marker_trackings,const std::unordered_map<mad::MapObjectId,std::unordered_map<int,double>,hash_MapObjectId>& lane_road_association,double LANE_EMIS_STD,double QUERY_DISTANCE,double ANGLE_POWER_EXPONENT,double ANGLE_POSSI_DEFAULT,double LANE_PROB_SUM_LIMIT,double LANE_WIDTH);  
    static void projection(TraceNode &trace_node);
    static double calBearing(double lat1,double lng1,double lat2,double lng2);
    static double dAngle(double ang1,double ang2);
    static double angleInterp(double a1, double w1, double a2, double w2);
    // static double road_min_distance(double x, double y, Road_MADMAP &road);
    static double RealDistance(double lat1,double lng1,double lat2,double lng2);//lat1第一个点纬度,lng1第一个点经度,lat2第二个点纬度,lng2第二个点经度
    static double RealDistance(LatLon p1, LatLon p2 );
    static void validHypothesis(TraceNode& trace_node,double MIN_PROBABILITY_THRESHOLD,bool removeLowProb=true);
    // static void hypothesisFilter(TraceNode& trace_node,double MIN_PROBABILITY_THRESHOLD);
    static LatLon WGS84incre(double lat,double lng, double heading, double distance); 

};
// class localLaneMap
// {

// };

class Reset_Info
{
public:
    double time;
    double lat;
    double lon;
    mad::MapObjectId last_road;

};
class Jump_Info
{
public:
    std::string error_info;
    double time;
    double lat;
    double lon;
    mad::MapObjectId wrong_road;
    mad::MapObjectId corrected_road;

};
// class Detour_Info
// {
// public:
//     double lat;
//     double lon;
//     Road_MADMAP detour_road;
//     Road_MADMAP last_road;
//     Road_MADMAP new_road;

// };
struct OBDSC_result
{
    double p_normal;
    double p_express;
    double p_tunnel;
};

class HMM_MADMAP
{
private:
    // std::ofstream mm_out_hp;//,mm_out2;
    std::ofstream mm_out_standard;//,mm_out2;
    std::ofstream hp_out;//,mm_out2;
    std::ofstream gt_mm_out,new_gt_mm_out;
    std::ofstream projection_out;
    std::ofstream road_out;
    std::ofstream trace_out;
    std::ofstream trace_out_corrected;
    // std::ofstream projection_out;
    mad::MapObjectId last_mm_standard;//,last_mm_hp;
    mad::MapObjectId mm_output;
    std::unordered_map<mad::MapObjectId,Road_MADMAP,hash_MapObjectId> roads;
    // std::unordered_map<mad::MapObjectId,Fork,hash_MapObjectId> fork_info;
public:
    std::ofstream lane_out;

    // double highest_speed(Road_MADMAP &road);
    // double calculate_angle(std::deque<TraceNode> &trace,int p, int q);//第p条trace，第q条真路
    static void errorFilterInit(std::unique_ptr< KalmanFilter>& kf,double init_x);
    static void errorFilterUpdate(std::unique_ptr< KalmanFilter>& kf,double projection_error);
    static void filterBatchUpdate(std::unique_ptr< KalmanFilter>& kf,std::deque<TraceNode> &trace);
    static double errorFilterPredict(std::unique_ptr< KalmanFilter>& kf,double heading);
    void reset();
    static void updateMM(std::deque<TraceNode> &trace,bool skip_current);
    void fileOut(TraceNode& trace_node);
    static mad::MapObjectId multiHypoStrategy(mad::MadMapStandard &madmapSD,TraceNode& trace_node,mad::MapObjectId last_mm,std::ofstream& hp_out,double ANGLE_POWER_EXPONENT,double MIN_PROBABILITY_THRESHOLD);
    static void Viterbi(std::deque<TraceNode> &trace,bool USE_LANE_MARKER,bool COMPARE_P2C,bool COMPARE_OBDSC,double CONFIDENCE_THRESHOLD);
    static void Viterbi(TraceNode&trace_node,TraceNode& last_trace_node,bool USE_LANE_MARKER);

    static void Viterbi_HP(TraceNode& trace_node,int& num_valid_hp,double MIN_PROBABILITY_THRESHOLD);
    void stateMachine(mad::MadMapStandard &madmapSD,std::deque<TraceNode> &trace,std::unique_ptr< KalmanFilter>& kf);
    static bool isSameSource(TraceNode& trace_node);

    static mad::MapObjectId findForkedRoad(TraceNode& trace_node,mad::MapObjectId last_fork);
    static double distanceToFork(TraceNode& trace_node,mad::MapObjectId forked_road);
    void findJumps(std::vector<Jump_Info>& jumps,std::deque<TraceNode> &trace,mad::MapObjectId last_mm,mad::MapObjectId mm_output,std::string error_info,std::unordered_map<mad::MapObjectId,Road_MADMAP,hash_MapObjectId>& hypothesis_roads);
    void roadOut();
    void roadsUpdate(std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId>& candidate_roads);

    // void traceOut(std::deque<TraceNode> &trace);
    static void addHolisticPath(Cxx_hp& cxx_hp,TraceNode&  trace_node,int index,double PD_LATERAL_STD,double time,bool USE_HOLISTIC_PATH);
    static void addLaneMarker(Cxx_clmd& cxx_clmd,TraceNode&  trace_node,int index,double PD_LATERAL_STD,double ICP_CLOUD_PREVIEW,double time);
    bool traceUpdateSatellite(Cxx_satellite& cxx_satellite,std::deque<TraceNode>& trace,int index,std::unique_ptr< KalmanFilter>& kf,double PD_LATERAL_STD,bool ALLOW_PD_CORRECTION);
    bool traceUpdatePSD(Cxx_psd& cxx_psd,std::deque<TraceNode>& trace,int index,std::unique_ptr< KalmanFilter>& kf,double PD_LATERAL_STD,bool ALLOW_PD_CORRECTION);
    bool traceOXTSUpdate(Cxx_oxts& cxx_oxts,std::deque<TraceNode>& trace,int index,std::unique_ptr< KalmanFilter>& kf,double PD_LATERAL_STD);
    void mapMatch(mad::MadMapStandard &madmapSD,std::deque<TraceNode>& trace,std::unique_ptr< KalmanFilter>& kf,double QUERY_DISTANCE,double PD_LATERAL_STD);
    // void laneMatch(mad::MadMapStandard &madmapSD,std::deque<TraceNode>& trace,double QUERY_DISTANCE,double PD_LATERAL_STD);
    static void loadHDF5File(H5File sensorfusion_file,Cxx_psd& cxx_psd,Cxx_hp& cxx_hp);
    static void lanePlot(TraceNode &trace_node,bool ENABLE_PLOT_SHOW);
    void localPlot(std::deque<TraceNode>& trace,std::unordered_map<int,LaneMarkerSDAssociation>& lane_marker_trackings,bool ENABLE_PLOT_SHOW);
    static Eigen::Matrix4f ICP4D(TraceNode&  trace_node,std::shared_ptr< pcl::PointCloud<pcl::PointXYZL>> source_cloud,std::shared_ptr< pcl::PointCloud<pcl::PointXYZL>> target_cloud,Eigen::Matrix4f init_transform,double heading,double ICP_RADIUS,double ICP_TYPE_FACTOR,int ICP_MAX_ITER,double ICP_EPSILON ,double ICP_CONVERGE,int ICP_MIN_POINTS,double ICP_EARLY_STOP,double ICP_MIN_MATCH_RATIO,double ICP_MATCH_DIS,int ICP_MATCH_NUM,double ICP_HEADING_LIMIT,LatLon cloud_map_init );
    static void laneMarkerICP(std::deque<TraceNode>& trace,pcl::PointCloud<pcl::PointXYZL>::Ptr lane_marker_cloud_map,LatLon cloud_map_init,double CLOUD_MAP_QUERY_DISTANCE,double ICP_RADIUS,double ICP_TYPE_FACTOR,int ICP_MAX_ITER,double ICP_EPSILON,double ICP_CONVERGE ,int ICP_MIN_POINTS,double ICP_EARLY_STOP,double ICP_MIN_MATCH_RATIO,double ICP_MATCH_DIS,int ICP_MATCH_NUM,double ICP_HEADING_LIMIT,bool ENABLE_PLOT);
    void laneMatch(std::string lane_marker_tracking_file,mad::MadMapStandard &madmapSD,std::unordered_map<mad::MapObjectId,Road_MADMAP,hash_MapObjectId>& hypothesis_roads,std::unordered_map<int,LaneMarkerSDAssociation>& lane_marker_trackings,double QUERY_DISTANCE,double PD_LATERAL_STD);
    void holisticPath(std::deque<TraceNode> &trace);
    //void holisticPathEmiss(std::deque<TraceNode> &trace);

    // static void laneMarkerSDAssociation(TraceNode );

    // static Cxx_oxts loadOXTSFile(H5File oxts_file);

    std::deque<TraceNode>& getTrace() {return trace;}

    HMM_MADMAP()
    {
        time_t now = time(0);
        char timeString[20];
        strftime(timeString, sizeof(timeString), "%Y-%m-%d-%H:%M:%S", localtime(&now));
        // mm_out_hp.open("mm_out_hp.txt");
        mm_out_standard.open("mm_out_standard.txt");
        hp_out.open("hp_out.txt");
        lane_out.open("lane_out.txt");
        gt_mm_out.open("gt_mm_out");
        new_gt_mm_out.open(std::string(timeString)+".txt");
        projection_out.open("refer.txt");//(std::string(timeString)+"_refer.txt");
        trace_out.precision(10);
        trace_out.open("trace.txt");
        trace_out_corrected.precision(10);
        trace_out_corrected.open("trace_corrected.txt");
        // projection_out.precision(10);
        // projection_out.open("projection.txt");
        road_out.precision(10);
        road_out.open("madmap_candidate_road.txt");
    }
    ~HMM_MADMAP()
    {
        // mm_out_hp.close();  
        mm_out_standard.close();
        hp_out.close();
        gt_mm_out.close();
        new_gt_mm_out.close();
        projection_out.close();
        lane_out.close();
        road_out.close();   
        trace_out.close(); 
        trace_out_corrected.close();
        // projection_out.close();
    }
    std::deque<TraceNode> trace;
    std::unique_ptr< KalmanFilter> kf;
    int TRACE_HISTORY_NUM;
    int TRACE_INTERVAL;
    int TRACE_NUM;
    double PD_LATERAL_STD;
    // double TRANS_POSSI_DEFAULT;
    double QUERY_DISTANCE;
    double FORK_PREVIEW_DISTANCE;
    double ANGLE_POSSI_DEFAULT;
    double USE_RAW_GNSS;
    bool USE_HOLISTIC_PATH;
    bool USE_DELAY;
    bool USE_PD_CORRECTION;
    bool USE_ROAD_CORRECTION;
    bool USE_LANE_ICP;
    bool USE_LANE_MARKER;
    bool ENABLE_PLOT;
    bool ENABLE_PLOT_SHOW;

    double ERROR_MEAS_COV;
    double ERROR_PROC_COV;
    double MIN_PROBABILITY_THRESHOLD;
    double ANGLE_POWER_EXPONENT;
    double MAX_DISTANCE_THRESHOLD;
    double CONFIDENCE_THRESHOLD;
    double INTERP_LENGTH;
    double INTERP_INTERVAL;
    double INTERP_DELTA_ANGLE_CONDITION;
    double LANE_PD_LATERAL_STD;
    double LANE_QUERY_DISTANCE;
    double LANE_WIDTH;
    double LANE_FITTING_DEFINITION;
    double LANE_PREVIEW_DISTANCE;
    double LANE_PREVIEW_THRESHOLD;
    double LANE_WIDTH_ERROR_TOLERANCE_FACTOR;
    double LANE_PREVIEW_MIN;
    double LANE_PREVIEW_MAX;
    int LANE_SAMPLE_INTERP;
    double LANE_PROB_SUM_LIMIT;
    int LANE_MIN_POINTS;
    double LANE_EMIS_STD;
    double CLOUD_MAP_QUERY_DISTANCE;
    double ICP_RADIUS;
    double ICP_TYPE_FACTOR;
    int ICP_MAX_ITER;
    double ICP_EPSILON;
    double ICP_CONVERGE;
    int ICP_MIN_POINTS;
    double ICP_EARLY_STOP;
    double ICP_MIN_MATCH_RATIO;
    double ICP_MATCH_DIS;
    int ICP_MATCH_NUM;
    double ICP_HEADING_LIMIT;
    double ICP_CLOUD_PREVIEW;
    double VISION_TRANS_DISCOUT_FACTOR;
    bool ASSOCIATE_LANE_MODE;
    bool COMPARE_P2C;
    bool COMPARE_OBDSC;
    bool DECODE_GCJ02;
    std::string AMM_RESULT_PATH;
    std::vector<double > DEBUG_TIME;
    double RESULT_CONFIDENCE_TRHE;

    // int jump=0;
    std::vector<Jump_Info> jumps;
    int reset_num=0;
    // std::vector<Detour_Info> detours;
    std::deque<Road_MADMAP> history_mm;
    // std::unordered_set<mad::MapObjectId,hash_MapObjectId> gt0_mm;
    mad::MadMapStandard madmapSD;
    std::unordered_map<int,LaneMarkerSDAssociation> lane_marker_trackings;
    std::unordered_map<mad::MapObjectId,std::unordered_map<int,double>,hash_MapObjectId> lane_road_association;
    pcl::PointCloud<pcl::PointXYZL>::Ptr lane_marker_cloud_map;
    std::unordered_map<mad::MapObjectId,Road_MADMAP,hash_MapObjectId> hypothesis_roads;
    std::unordered_map<mad::MapObjectId,std::unordered_map<mad::MapObjectId,double,hash_MapObjectId>,hash_MapObjectId> global_tranPsb;
    std::unordered_set<mad::MapObjectId,hash_MapObjectId> mm_result_collection;
    std::vector<std::pair<mad::MapObjectId, LatLon>> projection_points;
    std::vector<OBDSC_result> OBDSC_results;
    int OBDSC_OFFSET;
    std::string ZOD_ID;
    
    LatLon cloud_map_init;
    double travel_length=0;

    double err_sum_correct=0;
    double err_sum_origin=0;
    // int projection_count=0;
    int count=0;
    // std::vector<double> unconverged_time;

};
#endif