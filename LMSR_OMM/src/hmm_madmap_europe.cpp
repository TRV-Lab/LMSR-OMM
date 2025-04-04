#include "hmm_madmap.h"
#include "hmm_icp.h"
#include "gcj02_wgs84.hpp"



// std::unordered_set<double> error_frames{10042.000};





/*History trajectory sliding window*/
bool HMM_MADMAP::traceUpdateSatellite(Cxx_satellite& cxx_satellite,std::deque<TraceNode>& trace,int index,std::unique_ptr< KalmanFilter>& kf,double PD_LATERAL_STD,bool ALLOW_PD_CORRECTION)
{

    static int last_valid=0;


    if(!cxx_satellite.getValid()[index])
    {
        return false;

    }
    TraceNode node{cxx_satellite.getLatPosN()[index]/1e9,cxx_satellite.getLongPosN()[index]/1e9};
    if(DECODE_GCJ02)
    {
        std::pair<double, double> new_crd = gcj02_to_wgs84(node.lat, node.lon);
        node.lat=new_crd.first;
        node.lon=new_crd.second;
    }
    node.time=(cxx_satellite.getZeaderTimestampNs()[index]-cxx_satellite.getZeaderTimestampNs()[0])/1e9+10000;
    node.heading=cxx_satellite.getHeading()[index]*M_PI/180;
    node.lateral_std=PD_LATERAL_STD;

    if (kf->isKalmanInit)
    {
        double lateral_error=HMM_MADMAP::errorFilterPredict(kf,node.heading);
        double coefficient=std::max(0.0,cos(node.heading-kf->last_filtered_heading));
        if(ALLOW_PD_CORRECTION)
        {
            node.corrected_pos=TraceNode::WGS84incre(node.lat,node.lon,node.heading-M_PI/2,-lateral_error*coefficient);
            node.corrected_heading=node.heading;
        }
        else{
            node.corrected_pos.lat=node.lat;
            node.corrected_pos.lon=node.lon;
            node.corrected_heading=node.heading;
        }
            
    }
    else{
        node.corrected_pos=LatLon(node.lat,node.lon);
        node.corrected_heading=node.heading;
    }
    trace.push_back(node);
    if(last_valid!=0)
    {
        travel_length+=TraceNode::RealDistance(cxx_satellite.getLatPosN()[index]/1e9,cxx_satellite.getLongPosN()[index]/1e9,cxx_satellite.getLatPosN()[last_valid]/1e9,cxx_satellite.getLongPosN()[last_valid]/1e9);

    }
    last_valid=index;
    return true;

}


bool HMM_MADMAP::traceUpdatePSD(Cxx_psd& cxx_psd,std::deque<TraceNode>& trace,int index,std::unique_ptr< KalmanFilter>& kf,double PD_LATERAL_STD,bool ALLOW_PD_CORRECTION)
{

    static int last_valid=0;

    if(!cxx_psd.getValid()[index])
    {
        return false;

    }
    TraceNode node{cxx_psd.getLat()[index]/1e9,cxx_psd.getLon()[index]/1e9};
    if(DECODE_GCJ02)
    {
        std::pair<double, double> new_crd = gcj02_to_wgs84(node.lat, node.lon);
        node.lat=new_crd.first;
        node.lon=new_crd.second;
    }
    node.time=(cxx_psd.getZeaderTimestampNs()[index]-cxx_psd.getZeaderTimestampNs()[0])/1e9+10000;
    node.heading=cxx_psd.getHeading()[index]*M_PI/180;
    node.lateral_std=PD_LATERAL_STD;

    if (kf->isKalmanInit)
    {
        double lateral_error=HMM_MADMAP::errorFilterPredict(kf,node.heading);
        double coefficient=std::max(0.0,cos(node.heading-kf->last_filtered_heading));
        if(ALLOW_PD_CORRECTION)
        {
            node.corrected_pos=TraceNode::WGS84incre(node.lat,node.lon,node.heading-M_PI/2,-lateral_error*coefficient);
            node.corrected_heading=node.heading;
        }
        else{
            node.corrected_pos.lat=node.lat;
            node.corrected_pos.lon=node.lon;
            node.corrected_heading=node.heading;

        }
            
    }
    else{
        node.corrected_pos=LatLon(node.lat,node.lon);
        node.corrected_heading=node.heading;
    }
    trace.push_back(node);
    if(last_valid!=0)
    {
        travel_length+=TraceNode::RealDistance(cxx_psd.getLat()[index]/1e9,cxx_psd.getLon()[index]/1e9,cxx_psd.getLat()[last_valid]/1e9,cxx_psd.getLon()[last_valid]/1e9);

    }
    last_valid=index;
    return true;

}


/*Test the effect of high precision positioning*/
bool HMM_MADMAP::traceOXTSUpdate(Cxx_oxts& cxx_oxts,std::deque<TraceNode>& trace,int index,std::unique_ptr< KalmanFilter>& kf,double PD_LATERAL_STD)
{
    static int last_valid=0;



    // if(cxx_oxts.getIsValidLatLong()[index] && cxx_oxts.getIsValidHeading()[index])
    // {
        TraceNode node{cxx_oxts.getPosLat()[index],cxx_oxts.getPosLon()[index]};
        if(DECODE_GCJ02)
        {
            std::pair<double, double> new_crd = gcj02_to_wgs84(node.lat, node.lon);
            node.lat=new_crd.first;
            node.lon=new_crd.second;
        }
        node.time=(cxx_oxts.getTimestamp()[index]-cxx_oxts.getTimestamp()[0])+10000;
        node.heading=cxx_oxts.getHeading()[index]*M_PI/180;
        node.lateral_std=PD_LATERAL_STD;
        if(node.lat<0.1||node.lon<0.1)
        {
            return false;
        }
        if (kf->isKalmanInit)
        {
            double lateral_error=HMM_MADMAP::errorFilterPredict(kf,node.heading);
            double coefficient=std::max(0.0,cos(node.heading-kf->last_filtered_heading));
            node.corrected_pos=TraceNode::WGS84incre(node.lat,node.lon,node.heading,-lateral_error*coefficient);
            //Is it still should be used
            // node.lat=node.corrected_pos.lat;
            // node.lon=node.corrected_pos.lon;

                
        }
        else{
            node.corrected_pos=LatLon(node.lat,node.lon);
        }
        trace.push_back(node);
        if(last_valid!=0)
        {
            travel_length+=TraceNode::RealDistance(cxx_oxts.getPosLat()[index],cxx_oxts.getPosLon()[index],cxx_oxts.getPosLat()[last_valid],cxx_oxts.getPosLon()[last_valid]);

        }
        last_valid=index;

        return true;
    // }
    // else{
    //     return false;
    // }

}

void Road_MADMAP::roadsCorrection(std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId>& roads, mad::MapObjectId fork_road_id,double INTERP_LENGTH,double INTERP_INTERVAL,double INTERP_DELTA_ANGLE_CONDITION)
{
    // for(auto& road:roads)
    // {
        if(roads[fork_road_id].direction==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
        {
            return;
        }
        LatLon fork;
        double heading_end;
        if(roads[fork_road_id].end_node.size()>1 &&roads[fork_road_id].direction==mad::LinkDirection::FROM_NODE_0)
        {
            fork=roads[fork_road_id].node.back();
            heading_end=TraceNode::calBearing(roads[fork_road_id].node[roads[fork_road_id].node.size()-2].lat,roads[fork_road_id].node[roads[fork_road_id].node.size()-2].lon,roads[fork_road_id].node.back().lat,roads[fork_road_id].node.back().lon);
 
        }
        else if(roads[fork_road_id].end_node.size()>1 &&roads[fork_road_id].direction==mad::LinkDirection::FROM_NODE_1)
        {
            fork=roads[fork_road_id].node[0];
            heading_end=TraceNode::calBearing(roads[fork_road_id].node[1].lat,roads[fork_road_id].node[1].lon,roads[fork_road_id].node[0].lat,roads[fork_road_id].node[0].lon);
 
        }
        for(auto end_id:roads[fork_road_id].end_node)
        {
            std::vector<LatLon> node_interp;
            if(roads.count(end_id)!=0 &&  !roads[end_id].interpolated && roads[end_id].direction==mad::LinkDirection::FROM_NODE_0)
            {
                LatLon node_length=roads[end_id].node.back();
                // double heading_length=roads[end_id].roads[fork_road_id].back();
                double heading_length=TraceNode::calBearing(roads[end_id].node[roads[end_id].node.size()-2].lat,roads[end_id].node[roads[end_id].node.size()-2].lon,roads[end_id].node.back().lat,roads[end_id].node.back().lon);

                int i=1;
                for(;i<roads[end_id].node.size();i++)
                {
                    if(TraceNode::RealDistance(fork, roads[end_id].node[i])>INTERP_LENGTH)
                    {
                        node_length=roads[end_id].node[i];
                        heading_length=TraceNode::calBearing(roads[end_id].node[i-1].lat,roads[end_id].node[i-1].lon,roads[end_id].node[i].lat,roads[end_id].node[i].lon);

                        break;
                    }
                    
                }
                if(i==roads[end_id].node.size())i--;
                LatLon last=node_length;
                double delta_heading_fork=TraceNode::dAngle(TraceNode::calBearing(roads[end_id].node[0].lat,roads[end_id].node[0].lon,roads[end_id].node[1].lat,roads[end_id].node[1].lon),heading_end);
                if(delta_heading_fork<INTERP_DELTA_ANGLE_CONDITION)
                {
                    continue;
                }
                double delta_heading=TraceNode::dAngle(heading_length,heading_end);

                double gain=(INTERP_LENGTH*sin(delta_heading)+INTERP_LENGTH/M_PI*2*cos(delta_heading))/INTERP_LENGTH*M_PI/2;

                for(int j=TraceNode::RealDistance(fork, last)/INTERP_INTERVAL;j>=0;j--)
                {
                    double dis_fork_last=TraceNode::RealDistance(fork, last);
                    if(dis_fork_last>j*INTERP_INTERVAL &&j<=INTERP_LENGTH/INTERP_INTERVAL)
                    {
                        double proportion=j/INTERP_LENGTH*INTERP_INTERVAL;
                        double heading=TraceNode::angleInterp(heading_length,proportion,heading_end,1-proportion);
                        LatLon interp=TraceNode::WGS84incre(last.lat,last.lon,heading,-INTERP_INTERVAL*gain);
                        // interp.lat=last.lat-(INTERP_INTERVAL*cos(heading))/LONLAT2METER;
                        // interp.lon=last.lon-(INTERP_INTERVAL*sin(heading))/LONLAT2METER/cos(last.lat*M_PI/180);
                        
                        node_interp.push_back(interp);
                        last=interp;
                    }
                    else if(j>INTERP_LENGTH/INTERP_INTERVAL)
                    {
                        LatLon interp=TraceNode::WGS84incre(last.lat,last.lon,heading_length,-(TraceNode::RealDistance(fork, last)-INTERP_LENGTH));
                        // interp.lat=last.lat-((TraceNode::RealDistance(fork, last)-INTERP_LENGTH)*cos(heading_length))/LONLAT2METER;
                        // interp.lon=last.lon-((TraceNode::RealDistance(fork, last)-INTERP_LENGTH)*sin(heading_length))/LONLAT2METER/cos(last.lat*M_PI/180);
                        j=INTERP_LENGTH/INTERP_INTERVAL;
                        node_interp.push_back(interp);
                        last=interp;
                    }

                }
                std::reverse(node_interp.begin(),node_interp.end());
                for(;i<roads[end_id].node.size();i++)
                {
                    node_interp.push_back(roads[end_id].node[i]);
                }


            }
            else if(roads.count(end_id)!=0 &&  !roads[end_id].interpolated && roads[end_id].direction==mad::LinkDirection::FROM_NODE_1)
            {
                LatLon node_length=roads[end_id].node[0];
                // double heading_length=roads[end_id].roads[fork_road_id].back();
                double heading_length=TraceNode::calBearing(roads[end_id].node[1].lat,roads[end_id].node[1].lon,roads[end_id].node[0].lat,roads[end_id].node[0].lon);

                int i=roads[end_id].node.size()-2;
                for(;i>=1;i--)
                {
                    if(TraceNode::RealDistance(fork, roads[end_id].node[i])>INTERP_LENGTH)
                    {
                        node_length=roads[end_id].node[i];
                        heading_length=TraceNode::calBearing(roads[end_id].node[i+1].lat,roads[end_id].node[i+1].lon,roads[end_id].node[i].lat,roads[end_id].node[i].lon);

                        break;
                    }
                    
                }
                // if(i==0)i++;
                LatLon last=node_length;
                double delta_heading_fork=TraceNode::dAngle(TraceNode::calBearing(roads[end_id].node.back().lat,roads[end_id].node.back().lon,roads[end_id].node[roads[end_id].node.size()-2].lat,roads[end_id].node[roads[end_id].node.size()-2].lon),heading_end);
                if(delta_heading_fork<INTERP_DELTA_ANGLE_CONDITION)
                {
                    continue;
                }
                double delta_heading=TraceNode::dAngle(heading_length,heading_end);
                double gain=(INTERP_LENGTH*sin(delta_heading)+INTERP_LENGTH/M_PI*2*cos(delta_heading))/INTERP_LENGTH*M_PI/2;


                for(int j=TraceNode::RealDistance(fork, last)/INTERP_INTERVAL;j>=0;j--)
                {
                    double dis_fork_last=TraceNode::RealDistance(fork, last);
                    if(dis_fork_last>j*INTERP_INTERVAL && j<=INTERP_LENGTH/INTERP_INTERVAL)
                    {
                        double proportion=j/INTERP_LENGTH*INTERP_INTERVAL;
                        double heading=TraceNode::angleInterp(heading_length,proportion,heading_end,1-proportion);
                        LatLon interp=TraceNode::WGS84incre(last.lat,last.lon,heading,-INTERP_INTERVAL*gain);
                        // interp.lat=last.lat-(INTERP_INTERVAL*cos(heading))/LONLAT2METER;
                        // interp.lon=last.lon-(INTERP_INTERVAL*sin(heading))/LONLAT2METER/cos(last.lat*M_PI/180);
                        node_interp.push_back(interp);
                        last=interp;
                    }
                    else if(j>INTERP_LENGTH/INTERP_INTERVAL)
                    {
                        LatLon interp=TraceNode::WGS84incre(last.lat,last.lon,heading_length,-(TraceNode::RealDistance(fork, last)-INTERP_LENGTH));
                        // interp.lat=last.lat-((TraceNode::RealDistance(fork, last)-INTERP_LENGTH)*cos(heading_length))/LONLAT2METER;
                        // interp.lon=last.lon-((TraceNode::RealDistance(fork, last)-INTERP_LENGTH)*sin(heading_length))/LONLAT2METER/cos(last.lat*M_PI/180);
                        j=INTERP_LENGTH/INTERP_INTERVAL;
                        node_interp.push_back(interp);
                        last=interp;
                    }

                }
                std::reverse(node_interp.begin(),node_interp.end());
                for(;i>=0;i--)
                {
                    node_interp.push_back(roads[end_id].node[i]);
                }
                std::reverse(node_interp.begin(),node_interp.end());


            
            }
            else{
                return;
            }
            std::swap(roads[end_id].node,node_interp);
            std::cout<<"interp"<<end_id.getPart1()<<"-"<<end_id.getPart2()<<", "<<roads[end_id].node.size()<<", "<<node_interp.size()<<std::endl;
            roads[end_id].interpolated=true;
        }

        
    // }
}





/*Hypothesis filter and Normalization*/
void TraceNode::validHypothesis(TraceNode& trace_node,double MIN_PROBABILITY_THRESHOLD,bool removeLowProb)
{
    auto& Pback=trace_node.sorted_joint_prob;
    if(trace_node.joint_prob.size()>0)
    {
        auto compare_joint_prob=[](const std::pair<mad::MapObjectId, std::pair<double, mad::MapObjectId>> &p1,const std::pair<mad::MapObjectId, std::pair<double, mad::MapObjectId>> &p2){return p1.second.first > p2.second.first ;};
        Pback.assign(trace_node.joint_prob.begin(), trace_node.joint_prob.end());
        std::sort(Pback.begin(),Pback.end(),compare_joint_prob);

    }
    // auto& Pback=trace_node.sorted_joint_prob;
    double sum=0;
    int valid_hypothesis=0;
    for(int i=0; i<Pback.size(); i++)
    {
        // double& prob=Pback[i].second.first;

        sum+=Pback[i].second.first;

    }
    if(sum==0)
    {
        trace_node.num_hypothesis=valid_hypothesis;
        Pback.clear();
        return;
    }

    //normalize
    for(int i=0; i<Pback.size(); i++)
    {
        Pback[i].second.first/=sum;
        if(removeLowProb && Pback[i].second.first<MIN_PROBABILITY_THRESHOLD)
        {
            Pback.resize(valid_hypothesis);
            break;
        }
        valid_hypothesis++;
    }

    for(auto& prob:trace_node.joint_prob)
    {
        if(prob.second.first>0)
        {
            prob.second.first/=sum;
        }
    }
    trace_node.num_hypothesis=valid_hypothesis;

    // if(valid_hypothesis==0 &&Pback.size()>0)
    // {
    //     printf(" ");
    //     assert(0);
    // }

}




void HMM_MADMAP::addLaneMarker(Cxx_clmd& cxx_clmd,TraceNode&  trace_node, int index,double PD_LATERAL_STD,double ICP_CLOUD_PREVIEW,double time)
{
    if(time!=cxx_clmd.getZeaderTimestampNs()[index])
    {
        // not synced
        int k=0;
        for(;k<cxx_clmd.getZeaderTimestampNs().size();k++)
        {
            if(cxx_clmd.getZeaderTimestampNs()[k]<time && cxx_clmd.getZeaderTimestampNs()[k+1]>time)
            {
                index=k;
                break;
            }
        }
        if(k==cxx_clmd.getZeaderTimestampNs().size())
        {
            return;
        }
    }

    trace_node.lane_markers=cxx_clmd.getEdgePoints()[index];
    trace_node.lane_marker_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZL>);
    trace_node.lane_marker_pointcloud->width = trace_node.lane_markers.size();  
    trace_node.lane_marker_pointcloud->height = 1;  
    trace_node.lane_marker_pointcloud->is_dense = true;  
    trace_node.lane_marker_pointcloud->points.resize(trace_node.lane_markers.size());
    for(size_t i=0;i<trace_node.lane_markers.size();i++)
    {
        if(trace_node.lane_markers[i].x>ICP_CLOUD_PREVIEW)
        {
            continue;
        }
        trace_node.lane_marker_pointcloud->points[i].x=trace_node.lane_markers[i].x;
        trace_node.lane_marker_pointcloud->points[i].y=trace_node.lane_markers[i].y;
        trace_node.lane_marker_pointcloud->points[i].z=0;
        trace_node.lane_marker_pointcloud->points[i].label=trace_node.lane_markers[i].type;
        trace_node.lane_marker_pointcloud->points[i].data[3]=trace_node.lane_markers[i].type;

    }
        // std::cout<<trace_node.heading<<", "<<node.heading<<std::endl;


}


/*Find Jumps*/
void HMM_MADMAP::findJumps(std::vector<Jump_Info>& jumps,std::deque<TraceNode> &trace,mad::MapObjectId last_mm,mad::MapObjectId mm_output,std::string error_info,std::unordered_map<mad::MapObjectId,Road_MADMAP,hash_MapObjectId>& hypothesis_roads)
{
    static bool first=true;

    if(trace.size()>1 &&!first && trace[trace.size()-2].candidate_roads[last_mm].tranPsb[mm_output]<=0 && trace[trace.size()-2].candidate_roads[mm_output].tranPsb[last_mm]<=0)
    {//last_mm!=mad::MapObjectId()
        
        Jump_Info new_jump_info;
        new_jump_info.time=trace.back().time;
        new_jump_info.error_info=error_info;
        new_jump_info.lat=trace.back().lat;
        new_jump_info.lon=trace.back().lon;
        new_jump_info.wrong_road=last_mm;
        new_jump_info.corrected_road=mm_output;
        jumps.push_back(new_jump_info);

        //delete wrong roads
        if(ASSOCIATE_LANE_MODE)
        {
            mad::MapObjectId source;
            {
                auto last_node=trace.end()-1;
                auto& current_road=last_mm;
                std::vector<mad::MapObjectId> delete_list;
                delete_list.push_back(last_mm);
                //last_mm的上一条路
                if(last_node->candidate_roads[current_road].start_node.size()==1)
                {
                    source=last_node->candidate_roads[last_mm].start_node[0];

                }
                else{
                    for(auto node:last_node->candidate_roads[current_road].start_node)
                    {
                        if(hypothesis_roads.count(node))
                        {
                            source=node;
                        }
                    }
                }
                
                while(hypothesis_roads.count(source))
                {
                    if(last_node!=trace.begin())last_node--;
                    if(global_tranPsb[source][mm_output]>0 || !last_node->candidate_roads.count(source))
                    {
                        break;
                    }
                    delete_list.push_back(source);
                    if(last_node->candidate_roads[source].start_node.size()==1)
                    {
                        source=last_node->candidate_roads[source].start_node[0];

                    }
                    else{
                        for(auto node:last_node->candidate_roads[source].start_node)
                        {
                            if(hypothesis_roads.count(node))
                            {
                                source=node;
                            }
                        }
                    }
                }
                for(auto r:delete_list)
                {
                    hypothesis_roads.erase(r);
                }
            }

            {
                auto last_node=trace.end()-1;
                auto& current_road=mm_output;
                if(last_node->candidate_roads[current_road].start_node.size()==1)
                {
                    source=last_node->candidate_roads[mm_output].start_node[0];
                }
                else{
                    for(auto node:hypothesis_roads[source].end_node)
                    {
                        if(node==last_node->candidate_roads[mm_output].start_node[0] || node==last_node->candidate_roads[mm_output].start_node[1])
                        {
                            source=node;
                        }
                        
                    }
                }
                
                while(!hypothesis_roads.count(source))
                {
                    if(last_node!=trace.begin())last_node--;
                    hypothesis_roads[source]=last_node->candidate_roads[source];
                    current_road=source;
                    if(hypothesis_roads[mm_output].start_node.size()==1)
                    {
                        source=last_node->candidate_roads[source].start_node[0];

                    }
                    else{
                        for(auto node:hypothesis_roads[source].end_node)
                        {
                            if(node==last_node->candidate_roads[current_road].start_node[0] || node==last_node->candidate_roads[current_road].start_node[1])
                            {
                                source=node;
                            }
                            
                        }
                    }
                } 
            }

        }

        // std::vector<mad::MapObjectId> delete_list;
        // delete_list.push_back(last_mm);
        // mad::MapObjectId source=hypothesis_roads[last_mm].start_node[0];
        // while(hypothesis_roads[last_mm].start_node.size()==1 &&hypothesis_roads.count(source)&&hypothesis_roads[source].end_node.size()==1)
        // {
        //     delete_list.push_back(source);
        //     source=hypothesis_roads[source].start_node[0];
        // }
        // for(auto r:delete_list)
        // {
        //     hypothesis_roads.erase(r);

        // }

        // int i=1;
        // auto last_node=trace.end()-i;
        // auto& current_road=mm_output;
        // if(last_node->candidate_roads[current_road].start_node.size()==1)
        // {
        //     source=last_node->candidate_roads[mm_output].start_node[0];
        // }
        // else{
        //     for(auto node:hypothesis_roads[source].end_node)
        //     {
        //         if(node==last_node->candidate_roads[mm_output].start_node[0] || node==last_node->candidate_roads[mm_output].start_node[1])
        //         {
        //             source=node;
        //         }
                
        //     }
        // }
        
        // while(!hypothesis_roads.count(source))
        // {
        //     if(i<trace.size())i++;
        //     hypothesis_roads[source]=last_node->candidate_roads[source];
        //     current_road=source;
        //     if(hypothesis_roads[mm_output].start_node.size()==1)
        //     {
        //         source=last_node->candidate_roads[source].start_node[0];

        //     }
        //     else{
        //         for(auto node:hypothesis_roads[source].end_node)
        //         {
        //             if(node==last_node->candidate_roads[mm_output].start_node[0] || node==last_node->candidate_roads[mm_output].start_node[1])
        //             {
        //                 source=node;
        //             }
                    
        //         }
        //     }
        // }
    }
    else if(trace.size()==1&&!first ){
        Jump_Info new_jump_info;
        new_jump_info.time=trace.back().time;
        new_jump_info.error_info=error_info;
        new_jump_info.lat=trace.back().lat;
        new_jump_info.lon=trace.back().lon;
        new_jump_info.wrong_road=last_mm;
        new_jump_info.corrected_road=mm_output;
        jumps.push_back(new_jump_info);
    }
    first=false;
    
}
/*Init kalman filter*/

/*Reinitialize map matching if system crash*/
void HMM_MADMAP::reset()
{
    //TODO: error, restart

    // assert(0);
    trace.resize(0);
    kf->isKalmanInit=false;
    // history_mm.resize(0);
    // last_mm_hp=mad::MapObjectId();
    // last_mm_standard=mad::MapObjectId();
}
/*Update history map matching to one hypothesis if confident enough*/
void HMM_MADMAP::updateMM(std::deque<TraceNode> &trace,bool skip_current=false)
{
    // std::cout<<"One hypothesis"<<std::endl;

    if(skip_current==false)
    {
        trace.back().mm_result = trace.back().sorted_joint_prob[0].first;
        for(int i = trace.size()-2; i>=0; i--)
        {
            trace[i].mm_result = trace[i+1].joint_prob[ trace[i+1].mm_result ].second ;
            trace[i].num_hypothesis=1;
            // std::cout<<b[i].getPart1()+b[i].getPart2()<<" "<<trace[i+1].joint_prob[ b[i+1] ].first<<"\nb i+1 info:"<<b[i+1].getPart1()+b[i+1].getPart2()<<std::endl;
        }        
    }
    else{
        if(trace.size()>=2)
        {
            trace[trace.size()-2].mm_result =trace.back().sorted_joint_prob[0].second.second;
            trace[trace.size()-2].num_hypothesis=1;
            for(int i = trace.size()-3; i>=0; i--)
            {
                trace[i].mm_result = trace[i+1].joint_prob[ trace[i+1].mm_result ].second ;
                trace[i].num_hypothesis=1;
                // std::cout<<b[i].getPart1()+b[i].getPart2()<<" "<<trace[i+1].joint_prob[ b[i+1] ].first<<"\nb i+1 info:"<<b[i+1].getPart1()+b[i+1].getPart2()<<std::endl;
            }              
        }

    }



}




bool HMM_MADMAP::isSameSource(TraceNode& trace_node)
{
    mad::MapObjectId& first_source_road=trace_node.sorted_joint_prob[0].second.second;
    for(int i=1;i<trace_node.sorted_joint_prob.size();i++)
    {
        if(trace_node.sorted_joint_prob[i].second.second!=first_source_road
            &&trace_node.candidate_roads.count(trace_node.sorted_joint_prob[i].second.second)!=0 && trace_node.candidate_roads[trace_node.sorted_joint_prob[i].second.second].tranPsb[first_source_road]==0
            &&trace_node.candidate_roads.count(first_source_road)!=0 && trace_node.candidate_roads[first_source_road].tranPsb[trace_node.sorted_joint_prob[i].second.second]==0 )
        {
            
            return false;
        }
    }
    return true;
}




/*Different process when facing different number of hypothesis*/
void HMM_MADMAP::stateMachine(mad::MadMapStandard &madmapSD,std::deque<TraceNode> &trace,std::unique_ptr< KalmanFilter>& kf)
{

    // std::cout<<"trace size"<<trace.size()<<std::endl;
    auto& Pback=trace.back().sorted_joint_prob;

    //modify 20241211
    // int pi=0;
    // while(pi<trace.back().num_hypothesis)
    // {
    //     for(auto it=Pback.begin()+1;it!=Pback.end();)
    //     {
    //         if((trace.back().candidate_roads[Pback[0].first].tranPsb[it->first]==1 && trace.back().candidate_roads[Pback[0].first].end_node.size()==1)
    //         || (trace.back().candidate_roads[it->first].tranPsb[Pback[0].first]==1 && trace.back().candidate_roads[it->first].end_node.size()==1))
    //         {
    //             // std::cout<<"Merge probability "<<": "<<Pback[0].first.getPart1()<<"-"<<Pback[0].first.getPart2()<<","<<it->first.getPart1()<<"-"<<it->first.getPart2()<<std::endl;
    //             Pback[pi].second.first+=it->second.first;
    //             it=Pback.erase(it);
    //             trace.back().num_hypothesis=Pback.size();
    //         }      
    //         else{
    //             it++;
    //         }  
    //     }    
    //     pi++;    
    // }
    std::cout<<trace.back().candidate_roads.size()<<std::endl;
    std::cout<<"position"<<std::setprecision(8)<<trace.back().lat<<","<<trace.back().lon<<std::endl;
    // if(trace.size()>1 && TraceNode::RealDistance(trace.back().corrected_pos,trace[trace.size()-2].corrected_pos)>QUERY_DISTANCE)
    // {
    //     std::cout<<"state -1"<<std::endl;
    //     reset();
    //     return;

    // }
    if(trace.back().num_hypothesis==0 )
    {
        std::cout<<"state 0"<<std::endl;
        std::cout<<"Zero hypothesis, ERROR, restart"<<std::endl;
        reset_num++;
        reset();
        return;

    }
    std::cout<<trace.back().candidate_roads.size()<<std::endl;

    if(trace.back().num_hypothesis==1)
    {
        // std::cout<<"state 1"<<std::endl;
        HMM_MADMAP::updateMM(trace);
        if(USE_PD_CORRECTION)
        {
            HMM_MADMAP::filterBatchUpdate(kf,trace);

        }


        // mm_output=Pback[0].first;
    }
    else if(HMM_MADMAP::isSameSource(trace.back()))
    {
        std::cout<<"state 2"<<std::endl;
        HMM_MADMAP::updateMM(trace,true);
        if(USE_PD_CORRECTION)
        {
            HMM_MADMAP::filterBatchUpdate(kf,trace);

        }

        // mad::MapObjectId fork_road_id= HMM_MADMAP::findForkedRoad(trace.back());
        // double fork_distance=fork_road_id==mad::MapObjectId()? 1000:HMM_MADMAP::distanceToFork(trace.back(),fork_road_id);
        // mm_output=HMM_MADMAP::multiHypoStrategy(madmapSD,trace.back(),last_mm_hp,hp_out,ANGLE_POWER_EXPONENT,MIN_PROBABILITY_THRESHOLD);

        // mm_output=Pback[0].first;
    }
    else
    {    
        std::cout<<"state 3"<<std::endl;
        // mm_output=HMM_MADMAP::multiHypoStrategy(madmapSD,trace.back(),last_mm_hp,hp_out,ANGLE_POWER_EXPONENT,MIN_PROBABILITY_THRESHOLD);
        // TraceNode::calVisionTranPsb(trace.back(),fork_road_id,ANGLE_POWER_EXPONENT,MIN_PROBABILITY_THRESHOLD,MAX_DISTANCE_THRESHOLD);
        // mad::MapObjectId fork_road_id= HMM_MADMAP::findForkedRoad(trace.back());
        // double fork_distance=fork_road_id==mad::MapObjectId()? 1000:HMM_MADMAP::distanceToFork(trace.back(),fork_road_id);
     
    }
    std::cout<<trace.back().candidate_roads.size()<<std::endl;

    while(trace.size()>TRACE_HISTORY_NUM && trace.front().num_hypothesis==1)
    {
        trace.pop_front(); 
    }


    std::cout<<trace.back().candidate_roads.size()<<std::endl;

    if(USE_DELAY && Pback[0].second.first<CONFIDENCE_THRESHOLD)
    {
        mm_output=last_mm_standard;
    }
    else if(trace.back().candidate_roads[Pback[0].first].start_node.size()>0 &&USE_LANE_MARKER)
    {
        mm_output=Pback[0].first;

        double p_max=trace.back().distanceEmisPsb[Pback[0].first]*trace.back().angleEmisPsb[Pback[0].first];//0;//trace.back().projection_distance[Pback[0].first];
        for(auto prob:trace.back().sorted_joint_prob)
        {
            //if(prob.second.first>p_max &&trace.back().projection_distance[prob.first]<trace.back().projection_distance[Pback[0].first])
            double p_current=trace.back().distanceEmisPsb[prob.first]*trace.back().angleEmisPsb[prob.first];
            if(p_current>p_max)
            {
                // for(auto& start:trace.back().candidate_roads[Pback[0].first].start_node)
                // {
                //     if(prob.first==start)
                if(trace.back().hpEmisPsb.size()>0&&trace.back().hpEmisPsb[prob.first]/trace.back().hpEmisPsb[mm_output]<0.7)
                {
                    continue;
                }
                if(global_tranPsb[prob.first][Pback[0].first]>0)
                {
                    // p_max=prob.second.first;
                    p_max=p_current;
                    mm_output=prob.first;
                }
                // }
                

            }
        }
    }
    else if((!(COMPARE_P2C||COMPARE_HMM))&&trace.back().candidate_roads[Pback[0].first].end_node.size()>0){
        //change to biggest prob fork road if not nearst
        mm_output=Pback[0].first;
        
        double p_max=trace.back().distanceEmisPsb[Pback[0].first]*trace.back().angleEmisPsb[Pback[0].first];//0;//trace.back().projection_distance[Pback[0].first];
        for(auto prob:trace.back().sorted_joint_prob)
        {
            double p_current=trace.back().distanceEmisPsb[prob.first]*trace.back().angleEmisPsb[prob.first];
            if(p_current>p_max)
            {
                if(trace.back().hpEmisPsb.size()>0&&trace.back().hpEmisPsb[prob.first]/trace.back().hpEmisPsb[mm_output]<0.7)
                {
                    continue;
                }
                if(global_tranPsb[Pback[0].first][prob.first]>0)
                {
                    p_max=p_current;
                    mm_output=prob.first;
                }
                

            }
        }
        
        
        // std::cout<<trace.back().candidate_roads.size()<<std::endl;
 
    }
    else{
        mm_output=Pback[0].first;

    }
    // for(auto& prob:Pback)
    // {
    if(!hypothesis_roads.count(mm_output))
    {
        hypothesis_roads[mm_output]=trace.back().candidate_roads[mm_output];
    }
    // }
    // for(int i=0;i<trace.back().sorted_joint_prob.size();i++)
    // {
    //     std::cout<<"hypothesis "<<i<<", "<< trace.back().sorted_joint_prob[i].first.getPart1()<<"-"<<trace.back().sorted_joint_prob[i].first.getPart2()<<", "<<trace.back().sorted_joint_prob[i].second.first<<std::endl;
    // }
    // findJumps(jumps,trace,last_mm_hp,mm_output,"Holistic path");
    findJumps(jumps,trace,last_mm_standard,mm_output,"standard",hypothesis_roads);
    std::cout<<trace.back().candidate_roads.size()<<std::endl;
    mm_result_collection.insert(mm_output);
    projection_points.push_back(std::make_pair(mm_output,trace.back().trace_projection[mm_output]));



    // last_mm_hp=mm_output;
    gt_mm_out<<std::setprecision(11);
    new_gt_mm_out<<std::setprecision(11);
    projection_out<<std::setprecision(11);
    if(global_tranPsb[last_mm_standard][mm_output]<1 && global_tranPsb[last_mm_standard][mm_output]>0)
    {
        for(auto& trans:global_tranPsb[last_mm_standard])
        {
            if(trans.first!=last_mm_standard && trans.first!=mm_output&&trans.second==1 &&global_tranPsb[trans.first][mm_output]==1)
            {
                new_gt_mm_out<<trans.first.getPart1()<<","<<trans.first.getPart2()<<","<<trace.back().time<<","<<trace.back().speed<<","<<trace.back().candidate_roads[trans.first].road_class<<","<<trace.back().candidate_roads[trans.first].link_type<<","<<trace.back().trace_projection[trans.first].lat<<","<<trace.back().trace_projection[trans.first].lon<<","<<trace.back().projection_distance[trans.first]<<","<<jumps.size()<<","<<reset_num;
                travel_length=0;
                if(jumps.size()>0)
                {
                    new_gt_mm_out<<","<<jumps.back().wrong_road.getPart1()<<","<<jumps.back().wrong_road.getPart2()<<'\n';
                }
                else{
                new_gt_mm_out <<","<<0<<","<<0<<'\n';
                }
            }
        }
        
    }
    new_gt_mm_out<<mm_output.getPart1()<<","<<mm_output.getPart2()<<","<<trace.back().time<<","<<trace.back().speed<<","<<trace.back().candidate_roads[mm_output].road_class<<","<<trace.back().candidate_roads[mm_output].link_type<<","<<trace.back().trace_projection[mm_output].lat<<","<<trace.back().trace_projection[mm_output].lon<<","<<trace.back().projection_distance[mm_output]<<","<<jumps.size()<<","<<reset_num;
    travel_length=0;
    if(jumps.size()>0)
    {
        new_gt_mm_out<<","<<jumps.back().wrong_road.getPart1()<<","<<jumps.back().wrong_road.getPart2()<<'\n';
    }
    else{
    new_gt_mm_out <<","<<0<<","<<0<<'\n';
    }
    for(auto dis:trace.back().projection_distance)
    {
        if(dis.second<10)
        {
            projection_out<<dis.first.getPart1()<<","<<dis.first.getPart2()<<","<<trace.back().time<<","<<trace.back().speed<<","<<trace.back().candidate_roads[dis.first].road_class<<","<<trace.back().candidate_roads[dis.first].link_type<<","<<trace.back().trace_projection[dis.first].lat<<","<<trace.back().trace_projection[dis.first].lon<<'\n';

        }
    }
    if(last_mm_standard!=mm_output)
    {
        // gt0_mm.insert(Pback[0].first);
        //补充间隔道路
        if(global_tranPsb[last_mm_standard][mm_output]<1 && global_tranPsb[last_mm_standard][mm_output]>0)
        {
            for(auto& trans:global_tranPsb[last_mm_standard])
            {
                if(trans.first!=last_mm_standard && trans.first!=mm_output&&trans.second==1 &&global_tranPsb[trans.first][mm_output]==1)
                {
                    gt_mm_out<<trans.first.getPart1()<<","<<trans.first.getPart2()<<","<<trace.back().time<<","<<trace.back().speed<<","<<trace.back().candidate_roads[trans.first].road_class<<","<<trace.back().candidate_roads[trans.first].link_type<<","<<0<<","<<trace.back().candidate_roads[trans.first].length<<","<<trace.back().projection_distance[trans.first]<<","<<jumps.size()<<","<<reset_num;
                    travel_length=0;
                    if(jumps.size()>0)
                    {
                        gt_mm_out<<","<<jumps.back().wrong_road.getPart1()<<","<<jumps.back().wrong_road.getPart2()<<'\n';
                    }
                    else{
                    gt_mm_out <<","<<0<<","<<0<<'\n';
                    }
                }
            }
            
        }
        gt_mm_out<<mm_output.getPart1()<<","<<mm_output.getPart2()<<","<<trace.back().time<<","<<trace.back().speed<<","<<trace.back().candidate_roads[mm_output].road_class<<","<<trace.back().candidate_roads[mm_output].link_type<<","<<travel_length<<","<<trace.back().candidate_roads[mm_output].length<<","<<trace.back().projection_distance[mm_output]<<","<<jumps.size()<<","<<reset_num;
        travel_length=0;
        if(jumps.size()>0)
        {
            gt_mm_out<<","<<jumps.back().wrong_road.getPart1()<<","<<jumps.back().wrong_road.getPart2()<<'\n';
        }
        else{
        gt_mm_out <<","<<0<<","<<0<<'\n';
        }

    }
    last_mm_standard=mm_output;
    std::cout<<trace.back().candidate_roads.size()<<std::endl;

    //Debug
    if(history_mm.size()==0 || history_mm.back().id!=mm_output)
    {
        history_mm.push_back(trace.back().candidate_roads[mm_output]);
        // if(history_mm.size()>20)
        // {
        //     history_mm.pop_front();
        // }        
    }

    
    mm_out_standard<<mm_output.getPart1()+ mm_output.getPart2()<<'\n';

    fileOut(trace.back());
    std::cout<<trace.back().candidate_roads.size()<<std::endl;

    


}




/*Map matching main process*/
void HMM_MADMAP::mapMatch(mad::MadMapStandard &madmapSD,std::deque<TraceNode>& trace,std::unique_ptr< KalmanFilter>& kf,double QUERY_DISTANCE,double PD_LATERAL_STD)
{
    // GCJ_to_WG(trace.back().lon,trace.back().lat);

    std::string time_str=std::to_string(trace.back().time-10000);
    std::cout<<"time: "+time_str<<std::endl;

    for(auto& time:DEBUG_TIME)
    {
        if(time==trace.back().time)
        {
            printf(" ");
        }        
    }

    LaneMarkerInstance::devideInstance(trace.back().lane_markers,trace.back().lane_marker_instances);
    // LaneMarkerInstance::previewLaneMarkers(trace.back().lane_marker_instances,LANE_PREVIEW_DISTANCE,LANE_PREVIEW_THRESHOLD);
    // LaneMarkerInstance::previewLanes(trace.back().lane_marker_instances,trace.back().lanes,LANE_WIDTH,LANE_FITTING_DEFINITION,LANE_PREVIEW_DISTANCE,LANE_PREVIEW_THRESHOLD,LANE_WIDTH_ERROR_TOLERANCE_FACTOR,LANE_PREVIEW_MIN,LANE_PREVIEW_MAX);
    // if(ENABLE_PLOT)
    // {
    //     HMM_MADMAP::lanePlot(trace.back(),ENABLE_PLOT_SHOW);
    // }
    // clock_t time1=clock();
    auto candidate_history=trace.size()>1? trace[trace.size()-2].candidate_roads: std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId>();
    TraceNode::candidateRoads(madmapSD,trace.back(),candidate_history,QUERY_DISTANCE,true);
    // if(trace.back().candidate_roads.count(mad::MapObjectId(1344622144104068560,1344624124083994600))!=0)
    // {
    //     std::cout<<"test1: "<<"exist"<<std::endl;
    // }
    // else{
    //     std::cout<<"test1: "<<"not exist"<<std::endl;

    // }
    if(USE_LANE_ICP)
    {
        HMM_MADMAP::laneMarkerICP(trace,lane_marker_cloud_map,cloud_map_init,CLOUD_MAP_QUERY_DISTANCE,ICP_RADIUS,ICP_TYPE_FACTOR,ICP_MAX_ITER,ICP_EPSILON, ICP_CONVERGE  ,ICP_MIN_POINTS, ICP_EARLY_STOP,ICP_MIN_MATCH_RATIO,ICP_MATCH_DIS,ICP_MATCH_NUM, ICP_HEADING_LIMIT,ENABLE_PLOT);

    }
    // std::cout<<"candidate history:"<<std::endl;
    // for(auto& p:candidate_history)
    // {
    //     std::cout<<p.first.getPart1()<<"-"<<p.first.getPart2()<<std::endl;
    // }
    HMM_MADMAP::roadsUpdate(trace.back().candidate_roads);

    // clock_t time2=clock();
    // std::cout<<"candidate time cost:"<<double(time2-time1)/1000<<" ms"<<std::endl;
    // clock_t time3=clock();
    // std::cout<<"transpsb time cost:"<<double(time3-time2)/1000<<" ms"<<std::endl;
    // if(last_mm_standard!=mad::MapObjectId())
    // {
    //     trace.back().candidate_roads[last_mm_standard]=candidate_history[last_mm_standard];//copy 
    // }
    // if(trace.back().candidate_roads.count(mad::MapObjectId(1344622144104068560,1344624124083994600))!=0 &&trace.back().candidate_roads[mad::MapObjectId(1344622144104068560,1344624124083994600)].node.size()==0)
    // {
    //     std::cout<<"bug"<<std::endl;
    //     assert(0);
    // }
    TraceNode::projection(trace.back());
    TraceNode::calDistanceEmisPsb(trace.back(),PD_LATERAL_STD);
    TraceNode::calAngleEmisPsb(trace.back(), ANGLE_POWER_EXPONENT, ANGLE_POSSI_DEFAULT);
    if(COMPARE_OBDSC)
    {
        // int index=(trace.back().time-10000)+OBDSC_OFFSET; // 65shanghai
        int index=(trace.back().time-10000)*10+OBDSC_OFFSET; //sweden
        // int index=(trace.back().time-10000)*15.147+OBDSC_OFFSET;
        // int index=(trace.back().time-10000)*1+OBDSC_OFFSET; //sweden
        if(index<OBDSC_results.size())
        {
            for(auto& road:trace.back().candidate_roads)
            {
                if(road.second.road_class=="MOTORWAY"||road.second.road_class=="TRUNK"||road.second.isBridge)
                {
                    trace.back().OBDSC_EmisPsb[road.first]+=OBDSC_results[index].p_express;
                }
                else if(road.second.road_class=="PRIMARY")
                {
                    trace.back().OBDSC_EmisPsb[road.first]+=(OBDSC_results[index].p_express+OBDSC_results[index].p_normal)/2;

                }
                else if(road.second.isTunnel)
                {
                    trace.back().OBDSC_EmisPsb[road.first]+=OBDSC_results[index].p_tunnel;
                }
                else{
                    trace.back().OBDSC_EmisPsb[road.first]+=OBDSC_results[index].p_normal;
                }
                trace.back().OBDSC_EmisPsb[road.first]+=1.0e-5;
            }            
        }

    
    }
    // HMM_MADMAP::holisticPathEmiss(trace);
    double distance=trace.size()>1? TraceNode::RealDistance(trace[trace.size()-2].lat,trace[trace.size()-2].lon,trace.back().lat,trace.back().lon):0;
    TraceNode::calPriorTranPsb(trace.back(),global_tranPsb,MIN_PROBABILITY_THRESHOLD);//,distance);
    if(USE_LANE_MARKER )
    {
        TraceNode::laneProjection(trace.back(),lane_marker_trackings,lane_road_association);
        if(trace.back().isConverged)
        {
            TraceNode::calLaneEmisPsb(trace.back(),lane_marker_trackings,lane_road_association,LANE_EMIS_STD,QUERY_DISTANCE,ANGLE_POWER_EXPONENT,ANGLE_POSSI_DEFAULT,LANE_PROB_SUM_LIMIT, LANE_WIDTH);

        }
 
    }
    // clock_t time4=clock();
    // std::cout<<"projection time cost:"<<double(time4-time3)/1000<<" ms"<<std::endl;



    HMM_MADMAP::Viterbi(trace,USE_LANE_MARKER,COMPARE_HMM,COMPARE_P2C,COMPARE_OBDSC,CONFIDENCE_THRESHOLD);     //计算结果
    HMM_MADMAP::holisticPath(trace);
    if(trace.back().sorted_joint_prob.size()>0) trace.back().confidence=trace.back().sorted_joint_prob[0].second.first;

    TraceNode::validHypothesis(trace.back(), MIN_PROBABILITY_THRESHOLD,true);
    
    // TraceNode::hypothesisFilter(trace.back(),MIN_PROBABILITY_THRESHOLD);
    stateMachine(madmapSD,trace,kf);
    // clock_t time5=clock();
    // std::cout<<"viterbi time cost:"<<double(time5-time4)/1000<<" ms"<<std::endl;
    // if(trace.back().candidate_roads.count(mad::MapObjectId(1344622144104068560,1344624124083994600))!=0)
    // {
    //     std::cout<<"test1.5: "<<"exist"<<std::endl;
    // }
    // else{
    //     std::cout<<"test1.5: "<<"not exist"<<std::endl;

    // }
    if(ENABLE_PLOT)
    {
        HMM_MADMAP::localPlot(trace,lane_marker_trackings,ENABLE_PLOT_SHOW);

    }
    // if(trace.back().candidate_roads.count(mad::MapObjectId(1344622144104068560,1344624124083994600))!=0)
    // {
    //     std::cout<<"test2: "<<"exist"<<std::endl;
    // }
    // else{
    //     std::cout<<"test2: "<<"not exist"<<std::endl;

    // }
}
std::vector<std::string> split(const std::string& line, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(line);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}


int main()
{
    YAML::Node config=YAML::LoadFile("/research/tongji_project_ws/src/hdf_read/config/config.yaml");
    HMM_MADMAP hmm;
    //Load Madmap
    if(hmm.madmapSD.getSDInstance().open(config["SDmap_path"].as<std::string>()) !=mad::ReturnCode::SUCCESS)
    {
        printf("Failed to open provided SD map file.\n");
        exit(2);
    }

    //HMM config
    hmm.TRACE_HISTORY_NUM=config["TRACE_HISTORY_NUM"].as<int>();
    hmm.TRACE_INTERVAL=config["TRACE_INTERVAL"].as<int>();
    // hmm.TRACE_NUM=config["TRACE_NUM"].as<int>();
    hmm.PD_LATERAL_STD=config["PD_LATERAL_STD"].as<double>();
    // hmm.TRANS_POSSI_DEFAULT=config["TRANS_POSSI_DEFAULT"].as<double>();
    hmm.QUERY_DISTANCE=config["QUERY_DISTANCE"].as<double>();
    hmm.FORK_PREVIEW_DISTANCE=config["FORK_PREVIEW_DISTANCE"].as<double>();
    hmm.ANGLE_POSSI_DEFAULT=config["ANGLE_POSSI_DEFAULT"].as<double>();
    hmm.USE_HOLISTIC_PATH=config["USE_HOLISTIC_PATH"].as<bool>();
    hmm.USE_RAW_GNSS=config["USE_RAW_GNSS"].as<bool>();
    hmm.USE_DELAY=config["USE_DELAY"].as<bool>();
    hmm.USE_PD_CORRECTION=config["USE_PD_CORRECTION"].as<bool>();
    hmm.USE_ROAD_CORRECTION=config["USE_ROAD_CORRECTION"].as<bool>();
    hmm.USE_LANE_ICP=config["USE_LANE_ICP"].as<bool>();
    hmm.USE_LANE_MARKER=config["USE_LANE_MARKER"].as<bool>();
    hmm.ENABLE_PLOT=config["ENABLE_PLOT"].as<bool>();
    hmm.ENABLE_PLOT_SHOW=config["ENABLE_PLOT_SHOW"].as<bool>();

    hmm.ERROR_MEAS_COV=config["ERROR_MEAS_COV"].as<double>();
    hmm.ERROR_PROC_COV=config["ERROR_PROC_COV"].as<double>();
    hmm.INTERP_LENGTH=config["INTERP_LENGTH"].as<double>();
    hmm.INTERP_INTERVAL=config["INTERP_INTERVAL"].as<double>();
    hmm.INTERP_DELTA_ANGLE_CONDITION=config["INTERP_DELTA_ANGLE_CONDITION"].as<double>();
    hmm.MIN_PROBABILITY_THRESHOLD=config["MIN_PROBABILITY_THRESHOLD"].as<double>();
    hmm.ANGLE_POWER_EXPONENT=config["ANGLE_POWER_EXPONENT"].as<double>();
    hmm.MAX_DISTANCE_THRESHOLD=config["MAX_DISTANCE_THRESHOLD"].as<double>();
    hmm.CONFIDENCE_THRESHOLD=config["CONFIDENCE_THRESHOLD"].as<double>();
    hmm.LANE_PD_LATERAL_STD=config["LANE_PD_LATERAL_STD"].as<double>();
    hmm.LANE_QUERY_DISTANCE=config["LANE_QUERY_DISTANCE"].as<double>();
    hmm.LANE_WIDTH=config["LANE_WIDTH"].as<double>();
    hmm.LANE_FITTING_DEFINITION=config["LANE_FITTING_DEFINITION"].as<double>();
    hmm.LANE_PREVIEW_DISTANCE=config["LANE_PREVIEW_DISTANCE"].as<double>();
    hmm.LANE_PREVIEW_THRESHOLD=config["LANE_PREVIEW_THRESHOLD"].as<double>();
    hmm.LANE_PREVIEW_MIN=config["LANE_PREVIEW_MIN"].as<double>();
    hmm.LANE_PREVIEW_MAX=config["LANE_PREVIEW_MAX"].as<double>();
    hmm.LANE_WIDTH_ERROR_TOLERANCE_FACTOR=config["LANE_WIDTH_ERROR_TOLERANCE_FACTOR"].as<double>();
    hmm.LANE_SAMPLE_INTERP=config["LANE_SAMPLE_INTERP"].as<int>();
    hmm.LANE_PROB_SUM_LIMIT=config["LANE_PROB_SUM_LIMIT"].as<double>();
    hmm.LANE_MIN_POINTS=config["LANE_MIN_POINTS"].as<int>();
    hmm.LANE_EMIS_STD=config["LANE_EMIS_STD"].as<double>();
    hmm.CLOUD_MAP_QUERY_DISTANCE=config["CLOUD_MAP_QUERY_DISTANCE"].as<int>();
    hmm.ICP_RADIUS=config["ICP_RADIUS"].as<double>();
    hmm.ICP_TYPE_FACTOR=config["ICP_TYPE_FACTOR"].as<double>();
    hmm.ICP_MAX_ITER=config["ICP_MAX_ITER"].as<int>();
    hmm.ICP_EPSILON=config["ICP_EPSILON"].as<double>();
    hmm.ICP_CONVERGE=config["ICP_CONVERGE"].as<double>();
    hmm.ICP_MIN_POINTS=config["ICP_MIN_POINTS"].as<int>();
    hmm.ICP_EARLY_STOP=config["ICP_EARLY_STOP"].as<double>();
    hmm.ICP_MIN_MATCH_RATIO=config["ICP_MIN_MATCH_RATIO"].as<double>();
    hmm.ICP_MATCH_DIS=config["ICP_MATCH_DIS"].as<double>();
    hmm.ICP_MATCH_NUM=config["ICP_MATCH_NUM"].as<int>();
    hmm.VISION_TRANS_DISCOUT_FACTOR=config["VISION_TRANS_DISCOUT_FACTOR"].as<double>();
    hmm.ICP_HEADING_LIMIT=config["ICP_HEADING_LIMIT"].as<double>();
    hmm.ICP_CLOUD_PREVIEW=config["ICP_CLOUD_PREVIEW"].as<double>();
    hmm.ASSOCIATE_LANE_MODE=config["ASSOCIATE_LANE_MODE"].as<bool>();
    hmm.COMPARE_HMM=config["COMPARE_HMM"].as<bool>();
    hmm.COMPARE_P2C=config["COMPARE_P2C"].as<bool>();
    hmm.COMPARE_AMM=config["COMPARE_AMM"].as<bool>();
    hmm.COMPARE_OBDSC=config["COMPARE_OBDSC"].as<bool>();
    hmm.OBDSC_OFFSET=config["OBDSC_OFFSET"].as<int>();
    hmm.ZOD_ID=config["ZOD_ID"].as<std::string>();
    hmm.AMM_RESULT_PATH=config["AMM_RESULT_PATH"].as<std::string>();
    hmm.DEBUG_TIME=config["DEBUG_TIME"].as<std::vector<double>>();
    hmm.DECODE_GCJ02=config["DECODE_GCJ02"].as<bool>();
    hmm.RESULT_CONFIDENCE_TRHE=config["RESULT_CONFIDENCE_TRHE"].as<double>();

    double hdf_start=config["hdf_start"].as<double>();
    double hdf_end=config["hdf_end"].as<double>();

    H5File  sensorfusion_file(config["sensorfusion_file"].as<std::string>(), H5F_ACC_RDONLY);
    Cxx_psd cxx_psd;
    Cxx_satellite cxx_satellite;
    Cxx_hp cxx_hp;
    Cxx_clmd cxx_clmd;
    // hmm.loadHDF5File(sensorfusion_file,cxx_psd);
    if(config["seperate_file"].as<bool>())
    {
        H5_psd h5_psd;
        h5_psd.getDataSet(sensorfusion_file);
        h5_psd.parseTo(cxx_psd);

        H5_satellite h5_satellite;
        h5_satellite.getDataSet(sensorfusion_file);
        h5_satellite.parseTo(cxx_satellite);
        
        H5File  vision_file(config["vision_file"].as<std::string>(), H5F_ACC_RDONLY);
        H5_hp h5_hp;
        h5_hp.getDataSet(vision_file,true);
        h5_hp.parseTo(cxx_hp);

        H5_clmd h5_clmd;
        h5_clmd.getDataSet(vision_file,true);
        h5_clmd.parseTo(cxx_clmd);    
    }
    else{
        // hmm.loadHDF5File(sensorfusion_file,cxx_psd,cxx_hp);
        H5_psd h5_psd;
        h5_psd.getDataSet(sensorfusion_file);
        h5_psd.parseTo(cxx_psd);

        H5_satellite h5_satellite;
        h5_satellite.getDataSet(sensorfusion_file);
        h5_satellite.parseTo(cxx_satellite);

        H5_hp h5_hp;
        h5_hp.getDataSet(sensorfusion_file,false);
        h5_hp.parseTo(cxx_hp);

        // H5File  vision_file(config["vision_file"].as<std::string>(), H5F_ACC_RDONLY);
        H5_clmd h5_clmd;
        h5_clmd.getDataSet(sensorfusion_file,false);
        h5_clmd.parseTo(cxx_clmd);    
    }
    // if(hmm.USE_LANE_MARKER)
    // {
    
    // }
    if(hmm.COMPARE_OBDSC)
    {
        std::ifstream file("/research/tongji_project_ws/OBDSC_result/route"+hmm.ZOD_ID+".txt"); // sweden
        // std::ifstream file("/research/tongji_project_ws/src/OBDSC/code/result_shanghai/result_log"+hmm.ZOD_ID+".txt"); // 65shanghai
        // std::ifstream file("/research/tongji_project_ws/src/OBDSC/code/result_log_lsm1_15.147Hz.txt"); // 
        // std::ifstream file("/research/tongji_project_ws/src/OBDSC/code/result_log_lsm2_1Hz.txt"); //

        std::string line;
    
        if (file.is_open()) {
            while (std::getline(file, line)) {
                std::istringstream iss(line);
                std::string token;
                iss >> token;
                // while (iss >> token) {
                    OBDSC_result result;
                    iss >> token;
                    result.p_normal=std::stod(token);
                    iss >> token;
                    result.p_express=std::stod(token);
                    iss >> token;
                    result.p_tunnel=std::stod(token);


                    hmm.OBDSC_results.push_back(result);

            }
            file.close();
        } else {
            std::cerr << "无法打开文件" << std::endl;
            return 1;
        }
    }
    if(hmm.COMPARE_AMM)
    {
        hmm.ASSOCIATE_LANE_MODE=false;
        hmm.USE_LANE_MARKER=false;
        hmm.USE_LANE_ICP=false;
        hmm.USE_HOLISTIC_PATH=false;
        hmm.COMPARE_P2C=true;


        std::ifstream file(hmm.AMM_RESULT_PATH);
        std::string line;
        std::getline(file, line);
        bool first=true;
        double last_lat,last_lon;
        while (std::getline(file, line)) {
            std::vector<std::string> fields = split(line, ',');
            double lat = std::stod(fields[0]);
            double lon = std::stod(fields[1]);
            TraceNode node{lat,lon};
            node.time=10000+std::stod(fields[2]);
            node.corrected_pos.lat=node.lat;
            node.corrected_pos.lon=node.lon;
            hmm.trace.push_back(node);
            if(first)
            {
                first=false;
            }
            else{
                hmm.travel_length+=TraceNode::RealDistance(last_lat,last_lon,lat,lon);
            }
            last_lat=lat;
            last_lon=lon;
            hmm.mapMatch(hmm.madmapSD,hmm.trace,hmm.kf,hmm.QUERY_DISTANCE,hmm.PD_LATERAL_STD);
        }

        file.close();
        // for(size_t i=cxx_psd.getLength()*hdf_start;i<cxx_psd.getLength()*hdf_end;i+=hmm.TRACE_INTERVAL)
        // {
        //     bool valid;
        //     if(hmm.USE_RAW_GNSS)
        //     {
        //         valid=hmm.traceUpdateSatellite(cxx_satellite,hmm.trace,i,hmm.kf,hmm.PD_LATERAL_STD,hmm.USE_PD_CORRECTION);

        //     }
        //     else{
        //         valid=hmm.traceUpdatePSD(cxx_psd,hmm.trace,i,hmm.kf,hmm.PD_LATERAL_STD,hmm.USE_PD_CORRECTION);
        //     }
        //     if(valid){
                //HMM map matching with sliding window

                // std::cout<<"time cost:"<<double(end-start)/1000<<" ms"<<std::endl;
                // std::cout<<"log ratio: "<<i/double(cxx_psd.getLength())<<", time: "<<cxx_psd.getZeaderTimestampNs()[i]<<std::endl;
        //     }
            
        // }
        hmm.roadOut();
        std::cout<<"Total time"<<(cxx_psd.getZeaderTimestampNs().back()-cxx_psd.getZeaderTimestampNs()[0])/1e9/3600*(hdf_end-hdf_start)<<" Hours"<<std::endl;
        std::cout<<"error_avg"<<hmm.err_sum_origin/hmm.count<<", "<<hmm.err_sum_correct/hmm.count<<std::endl;
        std::cout<<"jump num"<<hmm.jumps.size()<<std::endl;
        for(size_t i=0;i<hmm.jumps.size();i++)
        {
            std::cout<<"Jump by "<<hmm.jumps[i].error_info<<": "<<std::setprecision(8)<<hmm.jumps[i].time<<", "<<hmm.jumps[i].lat<<","<<hmm.jumps[i].lon<<", "<<(hmm.jumps[i].time-10000)/(hmm.trace.back().time-10000)*hdf_end<<std::endl;
            std::cout<<"Wrong: "<<hmm.jumps[i].wrong_road.getPart1()<<"-"<<hmm.jumps[i].wrong_road.getPart2()<<std::endl;
            std::cout<<"Correct: "<<hmm.jumps[i].corrected_road.getPart1()<<"-"<<hmm.jumps[i].corrected_road.getPart2()<<std::endl;
        }
        if(hmm.history_mm.size()>2)
        {
            for(size_t i=0;i<hmm.history_mm.size()-2;i++)
            {
                for(size_t j=0;j<hmm.history_mm[i].end_node.size();j++)
                {
                    if(hmm.history_mm[i].end_node[j]==hmm.history_mm[i+2].id &&hmm.history_mm[i+1].direction==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
                    {
                        std::cout<<"Detour: "<<std::setprecision(8)<<hmm.history_mm[i+1].node[0].lat<<","<<hmm.history_mm[i+1].node[0].lon<<std::endl;
                        std::cout<<"Last: "<<hmm.history_mm[i].id.getPart1()<<"-"<<hmm.history_mm[i].id.getPart2()<<std::endl;
                        std::cout<<"Wrong: "<<hmm.history_mm[i+1].id.getPart1()<<"-"<<hmm.history_mm[i+1].id.getPart2()<<std::endl;
                        std::cout<<"Next: "<<hmm.history_mm[i+2].id.getPart1()<<"-"<<hmm.history_mm[i+2].id.getPart2()<<std::endl;
                    }
                }
                
            }        
        }
        std::cout<<"reset_num"<<hmm.reset_num<<std::endl;
        return 0;
    }


    hmm.kf.reset(new KalmanFilter(1,1,0)) ;
    hmm.kf->R<<hmm.ERROR_MEAS_COV;
    hmm.kf->Q<<hmm.ERROR_PROC_COV;


    if(hmm.ASSOCIATE_LANE_MODE)
    {
        hmm.USE_LANE_MARKER=false;
        hmm.USE_LANE_ICP=false;
        hmm.USE_HOLISTIC_PATH=false;
        hmm.ENABLE_PLOT=false;

    }
    else if(hmm.USE_LANE_MARKER)
    {
        // std::string oxts_file=config["oxts_file"].as<std::string>();
        // std::string file_id=oxts_file.substr(oxts_file.size()-36,31);
        std::string psd_file=config["sensorfusion_file"].as<std::string>();
        std::string file_id=psd_file.substr(psd_file.size()-67,31);//europe
        // std::string file_id=psd_file.substr(psd_file.size()-65,31);//shanghai65   

        std::string lane_marker_tracking_file=config["lane_marker_tracking_file"].as<std::string>()+file_id+".json";
        std::ifstream lane_marker_tracking_file_ifs(lane_marker_tracking_file);
        if(!lane_marker_tracking_file_ifs.is_open())
        {
            std::cout<<"lane_marker_tracking_file:"<<lane_marker_tracking_file<<std::endl;
            std::cout<<"find no prebuilt lane marker map, exit"<<std::endl;
            assert(0);
        }
        json lane_marker_tracking_json;
        lane_marker_tracking_file_ifs>>lane_marker_tracking_json;
        lane_marker_tracking_file_ifs.close();
        // std::cout<<lane_marker_tracking.dump();
        hmm.lane_marker_cloud_map.reset(new pcl::PointCloud<pcl::PointXYZL>);
        for(size_t i=0;i<lane_marker_tracking_json["data"]["interpolatedLane_lla"].size();i++)
        {
            // if(lane_marker_tracking_json["data"]["interpolatedLane_lla"][i][0].size()==0)
            // {
            //     break;
            // }
            if(lane_marker_tracking_json["data"]["interpolatedLane_lla"][i][0].size()/hmm.LANE_SAMPLE_INTERP<hmm.LANE_MIN_POINTS)
            {
                continue;
            }
            // for(auto& lla_point:lane_marker_tracking_json["data"]["interpolatedLane_lla"][i][0])
            for(size_t j=0;j<lane_marker_tracking_json["data"]["interpolatedLane_lla"][i][0].size();j++)
            {
                auto lla_point=lane_marker_tracking_json["data"]["interpolatedLane_lla"][i][0][j];
                if(hmm.DECODE_GCJ02)
                {
                    std::pair<double, double> new_crd = gcj02_to_wgs84(lla_point[0].get<double>(), lla_point[1].get<double>());
                    lla_point[0]=new_crd.first;
                    lla_point[1]=new_crd.second;
                }
                if(i==0&&j==0)
                {
                    hmm.cloud_map_init.lat=lla_point[0].get<double>();
                    hmm.cloud_map_init.lon=lla_point[1].get<double>();
                }
                if(j%hmm.LANE_SAMPLE_INTERP==0)
                {
                    LanePoint interp_point(lla_point[0].get<double>(),lla_point[1].get<double>());
                    interp_point.alt=lla_point[2].get<double>();
                    hmm.lane_marker_trackings[i].points.push_back(interp_point);
                    hmm.lane_marker_trackings[i].points.back().type=lane_marker_tracking_json["data"]["laneTypeLeft"][i][int(j/10)].get<int>();
                }
                pcl::PointXYZL tmp_point;
                tmp_point.x=(lla_point[1].get<double>()-hmm.cloud_map_init.lon)*LONLAT2METER*cos(lla_point[0].get<double>()*M_PI/180);
                tmp_point.y=(lla_point[0].get<double>()-hmm.cloud_map_init.lat)*LONLAT2METER;
                // tmp_point.z=lla_point[2].get<double>();
                tmp_point.z=0;
                tmp_point.label=lane_marker_tracking_json["data"]["laneTypeLeft"][i][int(j/10)].get<int>();
                tmp_point.data[3]=tmp_point.label;

        // node.lat=trace_node.corrected_pos.lat+(cxx_hp.getPoints()[index][i].x*cos(trace_node.heading)+cxx_hp.getPoints()[index][i].y*sin(trace_node.heading))/LONLAT2METER;
        // node.lon=trace_node.corrected_pos.lon+(cxx_hp.getPoints()[index][i].x*sin(trace_node.heading)-cxx_hp.getPoints()[index][i].y*cos(trace_node.heading))/LONLAT2METER/cos(trace_node.lat*M_PI/180);
                // std::cout<<"first point"<<(31.33266959-hmm.cloud_map_init.lat)*LONLAT2METER<<", "<<(121.3530909-hmm.cloud_map_init.lon)*LONLAT2METER*cos(31.33266959*M_PI/180)<<std::endl;
                hmm.lane_marker_cloud_map->points.push_back(tmp_point);
                // if(tmp_point.x>-20 &&tmp_point.x<60 &&tmp_point.y<-350 &&tmp_point.y>-440)
                // {
                //     printf(" ");
                // }
            }
            for(size_t j=0;j<lane_marker_tracking_json["data"]["roadAssociation"][i].size();j++)
            {
                auto& asso=lane_marker_tracking_json["data"]["roadAssociation"][i][j];
                hmm.lane_marker_trackings[i].road_probs[mad::MapObjectId(asso[0].get<int64_t>(),asso[1].get<int64_t>())]=asso[2].get<double>();
            }
            for(auto& prob:hmm.lane_marker_trackings[i].road_probs)
            {
                hmm.lane_road_association[prob.first][i]=prob.second;
            }


        }
        hmm.lane_marker_cloud_map->width= hmm.lane_marker_cloud_map->points.size();



    }


    for(size_t i=cxx_psd.getLength()*hdf_start;i<cxx_psd.getLength()*hdf_end;i+=hmm.TRACE_INTERVAL)
    {
        bool valid;
        if(hmm.USE_RAW_GNSS)
        {
            valid=hmm.traceUpdateSatellite(cxx_satellite,hmm.trace,i,hmm.kf,hmm.PD_LATERAL_STD,hmm.USE_PD_CORRECTION);

        }
        else{
            valid=hmm.traceUpdatePSD(cxx_psd,hmm.trace,i,hmm.kf,hmm.PD_LATERAL_STD,hmm.USE_PD_CORRECTION);
        }
        if(valid){
            hmm.trace.back().speed=cxx_satellite.getSpeed()[i];
            if(hmm.USE_HOLISTIC_PATH)
            {
                HMM_MADMAP::addHolisticPath(cxx_hp,hmm.trace.back(),i,hmm.PD_LATERAL_STD,cxx_psd.getZeaderTimestampNs()[i],hmm.USE_HOLISTIC_PATH);
            }
            if(hmm.USE_LANE_MARKER)
            {
                HMM_MADMAP::addLaneMarker(cxx_clmd,hmm.trace.back(),i,hmm.PD_LATERAL_STD,hmm.ICP_CLOUD_PREVIEW,cxx_psd.getZeaderTimestampNs()[i]);

            }

            //HMM map matching with sliding window
            clock_t start=clock();

            hmm.mapMatch(hmm.madmapSD,hmm.trace,hmm.kf,hmm.QUERY_DISTANCE,hmm.PD_LATERAL_STD);
            hmm.count++;
            clock_t end=clock();
            // std::cout<<"time cost:"<<double(end-start)/1000<<" ms"<<std::endl;
            // std::cout<<"log ratio: "<<i/double(cxx_psd.getLength())<<", time: "<<cxx_psd.getZeaderTimestampNs()[i]<<std::endl;
        }
        
    }
    hmm.roadOut();
    std::cout<<"Total time"<<(cxx_psd.getZeaderTimestampNs().back()-cxx_psd.getZeaderTimestampNs()[0])/1e9/3600*(hdf_end-hdf_start)<<" Hours"<<std::endl;
    std::cout<<"error_avg"<<hmm.err_sum_origin/hmm.count<<", "<<hmm.err_sum_correct/hmm.count<<std::endl;
    std::cout<<"jump num"<<hmm.jumps.size()<<std::endl;
    for(size_t i=0;i<hmm.jumps.size();i++)
    {
        std::cout<<"Jump by "<<hmm.jumps[i].error_info<<": "<<std::setprecision(8)<<hmm.jumps[i].time<<", "<<hmm.jumps[i].lat<<","<<hmm.jumps[i].lon<<", "<<(hmm.jumps[i].time-10000)/(hmm.trace.back().time-10000)*hdf_end<<std::endl;
        std::cout<<"Wrong: "<<hmm.jumps[i].wrong_road.getPart1()<<"-"<<hmm.jumps[i].wrong_road.getPart2()<<std::endl;
        std::cout<<"Correct: "<<hmm.jumps[i].corrected_road.getPart1()<<"-"<<hmm.jumps[i].corrected_road.getPart2()<<std::endl;
    }
    if(hmm.history_mm.size()>2)
    {
        for(size_t i=0;i<hmm.history_mm.size()-2;i++)
        {
            for(size_t j=0;j<hmm.history_mm[i].end_node.size();j++)
            {
                if(hmm.history_mm[i].end_node[j]==hmm.history_mm[i+2].id &&hmm.history_mm[i+1].direction==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
                {
                    std::cout<<"Detour: "<<std::setprecision(8)<<hmm.history_mm[i+1].node[0].lat<<","<<hmm.history_mm[i+1].node[0].lon<<std::endl;
                    std::cout<<"Last: "<<hmm.history_mm[i].id.getPart1()<<"-"<<hmm.history_mm[i].id.getPart2()<<std::endl;
                    std::cout<<"Wrong: "<<hmm.history_mm[i+1].id.getPart1()<<"-"<<hmm.history_mm[i+1].id.getPart2()<<std::endl;
                    std::cout<<"Next: "<<hmm.history_mm[i+2].id.getPart1()<<"-"<<hmm.history_mm[i+2].id.getPart2()<<std::endl;
                }
            }
            
        }        
    }
    std::cout<<"reset_num"<<hmm.reset_num<<std::endl;
    

    if(hmm.ASSOCIATE_LANE_MODE)
    {
        // std::string oxts_file=config["oxts_file"].as<std::string>();
        // std::string file_id=oxts_file.substr(oxts_file.size()-36,31);
        std::string psd_file=config["sensorfusion_file"].as<std::string>();
        // std::string file_id=psd_file.substr(psd_file.size()-65,31);//shanghai
        std::string file_id=psd_file.substr(psd_file.size()-67,31);//europe
        std::string lane_marker_tracking_file_output=config["lane_marker_tracking_file"].as<std::string>()+file_id+".json";//lane_marker_tracking_file_input;
        std::vector<std::string> filenames;
        GetFileNames(config["lane_marker_tracking_file"].as<std::string>(),file_id,filenames);


            // std::string oxts_file=config["oxts_file"].as<std::string>();
            // std::string file_id=oxts_file.substr(oxts_file.size()-36,31);
            // std::string lane_marker_tracking_file=config["lane_marker_tracking_file"].as<std::string>()+file_id+"_"+std::to_string(hdf_start)+"_"+std::to_string(hdf_end)+".json";
        int lane_id=0;
        json out_json;
        for(auto& file:filenames)
        {
            if (file.find(lane_marker_tracking_file_output)!= std::string::npos)
            {
                continue;
            }
            std::ifstream lane_marker_tracking_file_ifs(file);

            // if(!lane_marker_tracking_file_ifs.is_open())
            // {
            //     std::cout<<"lane_marker_tracking_file:"<<lane_marker_tracking_file<<std::endl;
            //     std::cout<<"find no prebuilt lane marker map, exit"<<std::endl;
            //     assert(0);
            // }
            json lane_marker_tracking_json;
            lane_marker_tracking_file_ifs>>lane_marker_tracking_json;
            lane_marker_tracking_file_ifs.close();
            // std::cout<<lane_marker_tracking.dump();

            for(size_t i=0;i<lane_marker_tracking_json["data"]["interpolatedLane_lla"].size();i++)
            {
                // if(lane_marker_tracking_json["data"]["interpolatedLane_lla"][i][0].size()==0)
                // {
                //     break;
                // }
                if(lane_marker_tracking_json["data"]["interpolatedLane_lla"][i][0].size()/hmm.LANE_SAMPLE_INTERP<hmm.LANE_MIN_POINTS)
                {
                    continue;
                }
                // for(auto& lla_point:lane_marker_tracking_json["data"]["interpolatedLane_lla"][i][0])
                for(size_t j=0;j<lane_marker_tracking_json["data"]["interpolatedLane_lla"][i][0].size();j++)
                {
                    auto& lla_point=lane_marker_tracking_json["data"]["interpolatedLane_lla"][i][0][j];
                    if(j%hmm.LANE_SAMPLE_INTERP==0)
                    {

                        LanePoint interp_point(lla_point[0].get<double>(),lla_point[1].get<double>());
                        if(hmm.DECODE_GCJ02)
                        {
                            std::pair<double, double> new_crd = gcj02_to_wgs84(interp_point.lat, interp_point.lon);
                            interp_point.lat=new_crd.first;
                            interp_point.lon=new_crd.second;
                        }
                        interp_point.alt=lla_point[2].get<double>();
                        hmm.lane_marker_trackings[lane_id].points.push_back(interp_point);
                        hmm.lane_marker_trackings[lane_id].points.back().type=lane_marker_tracking_json["data"]["laneTypeLeft"][i][int(j/10)].get<int>();
                    }



                }
                //associate
                
                LaneMarkerSDAssociation::candidateRoads(hmm.madmapSD,hmm.hypothesis_roads,hmm.lane_marker_trackings[lane_id],hmm.LANE_QUERY_DISTANCE);
                
                LaneMarkerSDAssociation::projection(hmm.lane_marker_trackings[lane_id]);
                LaneMarkerSDAssociation::calDistanceEmisPsb(hmm.lane_marker_trackings[lane_id],hmm.LANE_PD_LATERAL_STD);
                LaneMarkerSDAssociation::calAngleEmisPsb(hmm.lane_marker_trackings[lane_id], hmm.ANGLE_POWER_EXPONENT, hmm.ANGLE_POSSI_DEFAULT);
                // clock_t time4=clock();
                // std::cout<<"projection time cost:"<<double(time4-time3)/1000<<" ms"<<std::endl;
                LaneMarkerSDAssociation::calPriorTranPsb(hmm.lane_marker_trackings[lane_id]);

                LaneMarkerSDAssociation::Viterbi(hmm.lane_marker_trackings[lane_id],hmm.global_tranPsb,hmm.MIN_PROBABILITY_THRESHOLD);     //计算结果
                LaneMarkerSDAssociation::refine(hmm.madmapSD,hmm.lane_marker_trackings[lane_id]);
                // for(auto& prob:hmm.lane_marker_trackings[lane_id].road_probs)
                // {
                //     hmm.lane_road_association[prob.first][lane_id]=prob.second;
                //     if(prob.second<0.999)
                //     {
                //         // std::cout<<"road"<<prob.first.getPart1()<<prob.first.getPart2()<<std::endl;
                //         for(auto& point:hmm.lane_marker_trackings[lane_id].points)
                //         {
                //             hmm.lane_out<<std::setprecision(8)<<lane_id<<' '<<point.lat<<' '<<point.lon<<'\n';

                //         }

                //     }
                // }
                

                out_json["data"]["interpolatedLane_lla"][lane_id]=lane_marker_tracking_json["data"]["interpolatedLane_lla"][i];
                out_json["data"]["laneTypeLeft"][lane_id]=lane_marker_tracking_json["data"]["laneTypeLeft"][i];
                int count_prob=0;

                for(auto& prob:hmm.lane_marker_trackings[lane_id].road_probs)
                {
                    out_json["data"]["roadAssociation"][lane_id][count_prob]={prob.first.getPart1(),prob.first.getPart2(),prob.second};
                    count_prob++;
                }
                lane_id++;
            }




        }
        // hmm.laneMatch(lane_marker_tracking_file_output,hmm.madmapSD,hmm.hypothesis_roads,hmm.lane_marker_trackings,hmm.LANE_QUERY_DISTANCE,hmm.LANE_PD_LATERAL_STD);
        // for(auto& lane_marker_tracking:hmm.lane_marker_trackings)
        // {

        // }
        
        std::ofstream output(lane_marker_tracking_file_output);
        if(output.is_open())
        {
            output<<out_json.dump(4)<<std::endl;
        }
        output.close();
    }

    // hmm.err_sum=0;


    return 0;

}

// int main()//test
// {
//     YAML::Node config=YAML::LoadFile("/research/tongji_project_ws/src/hdf_read/config/config.yaml");
//     HMM_MADMAP hmm;
//     //Load Madmap
//     if(hmm.madmapSD.getSDInstance().open(config["SDmap_path"].as<std::string>()) !=mad::ReturnCode::SUCCESS)
//     {
//         printf("Failed to open provided SD map file.\n");
//         exit(2);
//     }

//     //HMM config
        // hmm.TRACE_HISTORY_NUM=config["TRACE_HISTORY_NUM"].as<int>();

//     hmm.TRACE_INTERVAL=config["TRACE_INTERVAL"].as<int>();
//     hmm.TRACE_NUM=config["TRACE_NUM"].as<int>();
//     hmm.PD_LATERAL_STD=config["PD_LATERAL_STD"].as<double>();
//     // hmm.TRANS_POSSI_DEFAULT=config["TRANS_POSSI_DEFAULT"].as<double>();
//     hmm.QUERY_DISTANCE=config["QUERY_DISTANCE"].as<double>();
//     // hmm.ANGLE_POSSI_DEFAULT=config["ANGLE_POSSI_DEFAULT"].as<double>();
//     hmm.USE_HOLISTIC_PATH=config["USE_HOLISTIC_PATH"].as<bool>();
//     hmm.ERROR_MEAS_COV=config["ERROR_MEAS_COV"].as<double>();
//     hmm.ERROR_PROC_COV=config["ERROR_PROC_COV"].as<double>();
//     hmm.MIN_PROBABILITY_THRESHOLD=config["MIN_PROBABILITY_THRESHOLD"].as<double>();
//     hmm.ANGLE_POWER_EXPONENT=config["ANGLE_POWER_EXPONENT"].as<double>();
//     hmm.COMPARE_P2C=config["COMPARE_P2C"].as<bool>();
//     hmm.ENABLE_PLOT=config["ENABLE_PLOT"].as<bool>();
//     hmm.DEBUG_TIME=config["DEBUG_TIME"].as<std::vector<double>>();
//     hmm.DECODE_GCJ02=config["DECODE_GCJ02"].as<bool>();

//     double hdf_start=config["hdf_start"].as<double>();
//     double hdf_end=config["hdf_end"].as<double>();

//     // H5File  sensorfusion_file(config["sensorfusion_file"].as<std::string>(), H5F_ACC_RDONLY);
//     // Cxx_psd cxx_psd;
//     // Cxx_hp cxx_hp;
//     // // hmm.loadHDF5File(sensorfusion_file,cxx_psd);
//     // hmm.loadHDF5File(sensorfusion_file,cxx_psd,cxx_hp);
//     H5File  oxts_file(config["oxts_file"].as<std::string>(), H5F_ACC_RDONLY);
//     // Cxx_oxts oxts=std::move(hmm.loadOXTSFile(oxts_file));
//     H5_oxts h5_oxts;
//     Cxx_oxts oxts;
//     h5_oxts.getDataSet(oxts_file);
//     h5_oxts.parseTo(oxts);


//     //Load raw trajectory 
//     // Eigen::MatrixXd R(1, 1);
//     hmm.kf.reset(new KalmanFilter(1,1,0)) ;
//     hmm.kf->R<<hmm.ERROR_MEAS_COV;
//     // Eigen::MatrixXd Q(1, 1);
//     hmm.kf->Q<<hmm.ERROR_PROC_COV;
//     for(int i=oxts.getLength()*hdf_start;i<oxts.getLength()*hdf_end;i+=hmm.TRACE_INTERVAL)
//     {
        
//         if(hmm.traceOXTSUpdate(oxts,hmm.trace,i,hmm.kf,hmm.PD_LATERAL_STD)==true){
//             // if(hmm.USE_HOLISTIC_PATH)
//             // {
//             //     HMM_MADMAP::addHolisticPath(cxx_hp,hmm.trace.back(),i,hmm.PD_LATERAL_STD);
//             // }

//             //HMM map matching with sliding window
//             clock_t start=clock();

//             // hmm.map_match(hmm.madmapSD,hmm.trace,hmm.kf,hmm.QUERY_DISTANCE,hmm.PD_LATERAL_STD);
//             hmm.mapMatch(hmm.madmapSD,hmm.trace,hmm.kf,hmm.QUERY_DISTANCE,hmm.PD_LATERAL_STD);

//             clock_t end=clock();
//             // std::cout<<"time cost:"<<double(end-start)/1000<<" ms"<<std::endl;
//         }
        
//     }
//     hmm.roadOut();
//     // std::cout<<"Total time"<<(cxx_psd.getZeaderTimestampNs().back()-cxx_psd.getZeaderTimestampNs()[0])/1e9/3600*(hdf_end-hdf_start)<<" Hours"<<std::endl;
//     std::cout<<"error_avg"<<hmm.err_sum_origin/hmm.count<<", "<<hmm.err_sum_correct/hmm.count<<std::endl;
//     std::cout<<"jump num"<<hmm.jumps.size()<<std::endl;
//     for(size_t i=0;i<hmm.jumps.size();i++)
//     {
//         std::cout<<"Jump by "<<hmm.jumps[i].error_info<<": "<<std::setprecision(8)<<hmm.jumps[i].time<<", "<<hmm.jumps[i].lat<<","<<hmm.jumps[i].lon<<std::endl;//<<", "<<(hmm.jumps[i].time-10000)/(hmm.trace.back().time-10000)*hdf_end<<std::endl;
//         std::cout<<"Wrong: "<<hmm.jumps[i].wrong_road.getPart1()<<"-"<<hmm.jumps[i].wrong_road.getPart2()<<std::endl;
//         std::cout<<"Correct: "<<hmm.jumps[i].corrected_road.getPart1()<<"-"<<hmm.jumps[i].corrected_road.getPart2()<<std::endl;
//     }
//     if(hmm.history_mm.size()>2)
//     {
//         for(size_t i=0;i<hmm.history_mm.size()-2;i++)
//         {
//             for(size_t j=0;j<hmm.history_mm[i].end_node.size();j++)
//             {
//                 if(hmm.history_mm[i].end_node[j]==hmm.history_mm[i+2].id &&hmm.history_mm[i+1].direction==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
//                 {
//                     std::cout<<"Detour: "<<std::setprecision(8)<<hmm.history_mm[i+1].node[0].lat<<","<<hmm.history_mm[i+1].node[0].lon<<std::endl;
//                     std::cout<<"Last: "<<hmm.history_mm[i].id.getPart1()<<"-"<<hmm.history_mm[i].id.getPart2()<<std::endl;
//                     std::cout<<"Wrong: "<<hmm.history_mm[i+1].id.getPart1()<<"-"<<hmm.history_mm[i+1].id.getPart2()<<std::endl;
//                     std::cout<<"Next: "<<hmm.history_mm[i+2].id.getPart1()<<"-"<<hmm.history_mm[i+2].id.getPart2()<<std::endl;
//                 }
//             }
            
//         }        
//     }

//     // hmm.err_sum=0;


//     return 0;

// }
