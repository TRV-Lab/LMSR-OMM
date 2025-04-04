#include "hmm_madmap.h"



/*Add some holistic path points*/
void HMM_MADMAP::addHolisticPath(Cxx_hp& cxx_hp,TraceNode&  trace_node, int index,double PD_LATERAL_STD,double time,bool USE_HOLISTIC_PATH)
{

    if(time!=cxx_hp.getZeaderTimestampNs()[index])
    {
        // not synced
        int k=0;
        for(;k<cxx_hp.getZeaderTimestampNs().size();k++)
        {
            if(cxx_hp.getZeaderTimestampNs()[k]<time && cxx_hp.getZeaderTimestampNs()[k+1]>time)
            {
                index=k;
                break;
            }
        }
        if(k==cxx_hp.getZeaderTimestampNs().size())
        {
            return;
        }
    }

    for(int i=0;i<cxx_hp.getNumOfValidPoints()[index];i++)
    {
        TraceNode node;

        // node.lat=trace_node.corrected_pos.lat+(cxx_hp.getPoints()[index][i].x*cos(trace_node.heading)+cxx_hp.getPoints()[index][i].y*sin(trace_node.heading))/LONLAT2METER;
        // node.lon=trace_node.corrected_pos.lon+(cxx_hp.getPoints()[index][i].x*sin(trace_node.heading)-cxx_hp.getPoints()[index][i].y*cos(trace_node.heading))/LONLAT2METER/cos(trace_node.lat*M_PI/180);
        node.holistic_path_increment.lat=(cxx_hp.getPoints()[index][i].x*cos(trace_node.heading)+cxx_hp.getPoints()[index][i].y*sin(trace_node.heading))/LONLAT2METER;
        node.holistic_path_increment.lon=(cxx_hp.getPoints()[index][i].x*sin(trace_node.heading)-cxx_hp.getPoints()[index][i].y*cos(trace_node.heading))/LONLAT2METER/cos(trace_node.lat*M_PI/180);
        // node.corrected_pos.lat=node.lat;
        // node.corrected_pos.lon=node.lon;
        // node.time=(cxx_hp.getZeaderTimestampNs()[index]-cxx_hp.getZeaderTimestampNs()[0])/1e9+10000;
        node.lateral_std=cxx_hp.getPoints()[index][i].y_deviation+PD_LATERAL_STD;
        PointWithDeviation last_point=i>0? cxx_hp.getPoints()[index][i-1] : PointWithDeviation();
        double heading_local=M_PI/2-atan2(cxx_hp.getPoints()[index][i].x-last_point.x,cxx_hp.getPoints()[index][i].y-last_point.y);
        node.heading=trace_node.heading-heading_local;
        // double distance=sqrt(pow(cxx_hp.getPoints()[index][i].x,2)+pow(cxx_hp.getPoints()[index][i].y,2));
        // double angle=M_PI/2-atan2(cxx_hp.getPoints()[index][i].x,cxx_hp.getPoints()[index][i].y);
        // LatLon crd=TraceNode::WGS84incre(trace_node.corrected_pos.lat,trace_node.corrected_pos.lon,trace_node.corrected_heading-angle,distance);
        trace_node.holistic_path.push_back(node);
        // std::cout<<trace_node.heading<<", "<<node.heading<<std::endl;

    }
    if(USE_HOLISTIC_PATH )//&&trace.size()>1)
    {
        for(int i=0;i<trace_node.holistic_path.size();i++)
        {
            trace_node.holistic_path[i].lat=trace_node.corrected_pos.lat+trace_node.holistic_path[i].holistic_path_increment.lat;
            trace_node.holistic_path[i].lon=trace_node.corrected_pos.lon+trace_node.holistic_path[i].holistic_path_increment.lon;
            trace_node.holistic_path[i].corrected_pos.lat=trace_node.holistic_path[i].lat;
            trace_node.holistic_path[i].corrected_pos.lon=trace_node.holistic_path[i].lon;
            trace_node.holistic_path[i].corrected_heading=trace_node.holistic_path[i].heading;
        }
        int num_valid_hp=0;
        return;
    }        
}
mad::MapObjectId HMM_MADMAP::findForkedRoad(TraceNode& trace_node,mad::MapObjectId last_fork)
{
    Road_MADMAP road=trace_node.candidate_roads[trace_node.sorted_joint_prob[0].first];
    while(true)
    {
        if(road.end_node.size()>1)
        {
            return road.id;
        }
        if(road.end_node.size()==0 || trace_node.candidate_roads.count(road.end_node[0])==0)
        {
            return last_fork;
        }
        road=trace_node.candidate_roads[road.end_node[0]];

    }

}
double HMM_MADMAP::distanceToFork(TraceNode& trace_node,mad::MapObjectId forked_road)
{

    Road_MADMAP road=trace_node.candidate_roads[forked_road];
    if(road.direction==mad::LinkDirection::FROM_NODE_0)
    {
        return TraceNode::RealDistance(trace_node.corrected_pos,road.node.back());
    }
    else if(road.direction==mad::LinkDirection::FROM_NODE_1)
    {
        return TraceNode::RealDistance(trace_node.corrected_pos,road.node[0]);
    }
    else{
    
        double road_bearing=TraceNode::calBearing(road.node[0].lat,road.node[0].lon,road.node.back().lat,road.node.back().lon);
        double delta_heading=TraceNode::dAngle(trace_node.corrected_heading,road_bearing);
        if(delta_heading<M_PI/2 )
        {
            return TraceNode::RealDistance(trace_node.corrected_pos,road.node.back());
        }
        else{
            return TraceNode::RealDistance(trace_node.corrected_pos,road.node[0]);
        }
    
    }
}
void HMM_MADMAP::holisticPath(std::deque<TraceNode> &trace)
{


    if(USE_HOLISTIC_PATH &&trace.size()>1)
    {

        // mad::MapObjectId last_fork=trace.size()>1? trace[trace.size()-2].fork_road_id:mad::MapObjectId();
        // trace.back().fork_road_id= HMM_MADMAP::findForkedRoad(trace.back(),last_fork);
        // if(trace.back().candidate_roads.count(trace.back().fork_road_id)!=0 &&trace.back().joint_prob.count(trace.back().fork_road_id)>0&&trace.back().joint_prob[trace.back().fork_road_id].first>0)
        // {
            double fork_distance=trace.back().fork_road_id==mad::MapObjectId() ? 1000:HMM_MADMAP::distanceToFork(trace.back(),trace.back().fork_road_id);
            if(USE_ROAD_CORRECTION )//&& fork_distance<FORK_PREVIEW_DISTANCE)
            {
                Road_MADMAP::roadsCorrection(trace.back().candidate_roads,trace.back().fork_road_id,INTERP_LENGTH,INTERP_INTERVAL,INTERP_DELTA_ANGLE_CONDITION);
            }
            if(trace[trace.size()-2].sorted_joint_prob.size()>1 ||(trace.back().candidate_roads.count(last_mm_standard) && trace.back().candidate_roads[last_mm_standard].end_node.size()>1))//fork_distance<FORK_PREVIEW_DISTANCE || 
            {
                TraceNode::calVisionEmissPsb(madmapSD,trace.back(),lane_marker_trackings,lane_road_association,trace.back().fork_road_id,fork_distance,hp_out,ANGLE_POWER_EXPONENT,ANGLE_POSSI_DEFAULT,MIN_PROBABILITY_THRESHOLD,MAX_DISTANCE_THRESHOLD,USE_LANE_MARKER,PD_LATERAL_STD,QUERY_DISTANCE,VISION_TRANS_DISCOUT_FACTOR,LANE_PROB_SUM_LIMIT, LANE_WIDTH);
            }              
        // }

    }
}

/*Calculate vision emission possibility*/
void TraceNode::calVisionEmissPsb(mad::MadMapStandard &madmapSD,TraceNode &trace_node,std::unordered_map<int,LaneMarkerSDAssociation>& lane_marker_trackings,const std::unordered_map<mad::MapObjectId,std::unordered_map<int,double>,hash_MapObjectId>& lane_road_association,mad::MapObjectId fork_road_id,double fork_distance,std::ofstream& hp_out, double ANGLE_POWER_EXPONENT,double ANGLE_POSSI_DEFAULT,double MIN_PROBABILITY_THRESHOLD,double MAX_DISTANCE_THRESHOLD,bool USE_LANE_MARKER,double PD_LATERAL_STD,double QUERY_DISTANCE,double VISION_TRANS_DISCOUT_FACTOR,double LANE_PROB_SUM_LIMIT,double LANE_WIDTH)
{
    int num_valid_hp=0;

    // TraceNode::candidateRoad(madmapSD,trace_node.holistic_path[0],trace_node.sorted_joint_prob[0].first);
    for(int i=0;i<trace_node.holistic_path.size();i++)
    {
        TraceNode& last_node=i>0?trace_node.holistic_path[i-1]:trace_node;

        if(i==0)
        {
            trace_node.holistic_path[i].candidate_roads=trace_node.candidate_roads;

        }else{
            // auto candidate_history=i>1? trace_node.holistic_path[i-1].candidate_roads: trace_node.candidate_roads;
            auto candidate_history= trace_node.holistic_path[i-1].candidate_roads;

            TraceNode::candidateRoads(madmapSD,trace_node.holistic_path[i],candidate_history,MAX_DISTANCE_THRESHOLD);

        }
        double distance=TraceNode::RealDistance(last_node.lat,last_node.lon,trace_node.holistic_path[i].lat,trace_node.holistic_path[i].lon);
        std::unordered_map<mad::MapObjectId,std::unordered_map<mad::MapObjectId,double,hash_MapObjectId>,hash_MapObjectId> tmp_tranPsb;
        TraceNode::calPriorTranPsb(trace_node.holistic_path[i],tmp_tranPsb,MIN_PROBABILITY_THRESHOLD);//,distance);

        TraceNode::projection(trace_node.holistic_path[i]);
        for(auto dis:trace_node.holistic_path[i].projection_distance)//it=trace_node.holistic_path[i].projection_distance.begin();it!=trace_node.holistic_path[i].projection_distance.end();it++)
        {
            if(dis.second>MAX_DISTANCE_THRESHOLD)
            {
                trace_node.holistic_path[i].candidate_roads.erase(dis.first);
            }
        }
        TraceNode::calDistanceEmisPsb(trace_node.holistic_path[i],trace_node.holistic_path[i].lateral_std);
        TraceNode::calAngleEmisPsb(trace_node.holistic_path[i], ANGLE_POWER_EXPONENT,ANGLE_POSSI_DEFAULT);
        // if(USE_LANE_MARKER)
        // {
        //     TraceNode::laneProjection(trace_node.holistic_path[i],lane_marker_trackings,lane_road_association);
        //     TraceNode::calLaneEmisPsb(trace_node.holistic_path[i],lane_marker_trackings,lane_road_association,PD_LATERAL_STD,QUERY_DISTANCE,ANGLE_POWER_EXPONENT,ANGLE_POSSI_DEFAULT,LANE_PROB_SUM_LIMIT,LANE_WIDTH);
    
        // }

        for(auto road_i:trace_node.holistic_path[i].candidate_roads)
        {
            // if(trace_node.candidate_roads[fork_road_id].tranPsb[hypothesis_id.first]==0)
            // {
            //     continue;
            // }
            std::vector<std::pair<mad::MapObjectId,double> > possibilities;

            for(auto& joint_prob:last_node.joint_prob)
            {
                Road_MADMAP& road_j=last_node.candidate_roads[joint_prob.first];
                double possibility=0;

                possibility =(1+joint_prob.second.first*1.0e-5)*road_j.tranPsb[road_i.first] * trace_node.holistic_path[i].distanceEmisPsb[road_i.first]*trace_node.holistic_path[i].angleEmisPsb[road_i.first];
                if(i>0)
                {
                    possibility *= (0.15+joint_prob.second.first);
                }
                // if(possibility<1e-8)
                // {
                //     continue;
                // }

                // if(USE_LANE_MARKER&&trace_node.holistic_path[i].isLaneAvail)
                // {
                //     possibility*=trace_node.holistic_path[i].laneEmisPsb[road_i.first];
                // }
                possibilities.push_back(std::make_pair(road_j.id,possibility));

            }
            if(possibilities.size()>0)
            {
                auto compare=[](const std::pair<mad::MapObjectId,double> &p1,const std::pair<mad::MapObjectId,double> &p2){return p1.second < p2.second;};
                auto max_element=std::max_element(possibilities.begin(), possibilities.end(),compare);

                trace_node.holistic_path[i].joint_prob.insert(std::make_pair(road_i.first, std::make_pair(max_element->second, max_element->first) ));
            }


        }


        auto compare_joint_prob=[](const std::pair<mad::MapObjectId, std::pair<double, mad::MapObjectId>> &p1,const std::pair<mad::MapObjectId, std::pair<double, mad::MapObjectId>> &p2){return p1.second.first > p2.second.first ;};

        auto& Pback=trace_node.holistic_path[i].sorted_joint_prob;
        Pback.assign(trace_node.holistic_path[i].joint_prob.begin(), trace_node.holistic_path[i].joint_prob.end());
        std::sort(Pback.begin(),Pback.end(),compare_joint_prob);
        TraceNode::validHypothesis(trace_node.holistic_path[i], MIN_PROBABILITY_THRESHOLD,true);

        if(Pback.size()==0 || trace_node.holistic_path[i].candidate_roads.size()==0 )//|| trace_node.holistic_path[i].projection_distance[Pback[0].first] >MAX_DISTANCE_THRESHOLD)
        {
            break;
        }
        num_valid_hp++;   

    }

    //after 20241206 emission probability
    if(num_valid_hp==0)
    {
        return;
    }


    for(int i=num_valid_hp-1;i>=0;i--)
    {
        for(auto& prob:trace_node.holistic_path[i].sorted_joint_prob)
        {
            trace_node.holistic_path[i].hpEmisPsb[prob.first]=prob.second.first;
        }
    
        for(int j=i;j>=0;j--)
        {
            bool update=true;
            while(update)
            {
                update=false;
                for(auto& prob:trace_node.holistic_path[j].joint_prob)
                {
                    if(!trace_node.holistic_path[i].hpEmisPsb.count(prob.second.second) || trace_node.holistic_path[i].hpEmisPsb[prob.second.second]<trace_node.holistic_path[i].hpEmisPsb[prob.first])
                    {
                        trace_node.holistic_path[i].hpEmisPsb[prob.second.second]=trace_node.holistic_path[i].hpEmisPsb[prob.first];//trace_node.holistic_path[i].joint_prob[prob.second.second].first;
                        update=true;
                        // std::cout<<"hp no."<<j<<"find last prob "<<prob.first.getPart1()/100%100<<prob.first.getPart2()/100%100<<" is from "<<prob.second.second.getPart1()/100%100<<prob.second.second.getPart2()/100%100<<", so add"<<trace_node.holistic_path[i].hpEmisPsb[prob.first]<<" to the source, not "<<prob.second.first<<" or "<<trace_node.holistic_path[j].joint_prob[prob.second.second].first<<std::endl;
                    }
                    // trace_node.hpEmisPsb[prob.first]+=prob.second.first;
                }                
            }

        }
        for(auto& psb:trace_node.holistic_path[i].hpEmisPsb)
        {
            if(psb.second>0)
            {
                trace_node.hpEmisPsb[psb.first]+=psb.second;

            }
        }
        
    }
    auto compare=[](const std::pair<mad::MapObjectId,double> &p1,const std::pair<mad::MapObjectId,double> &p2){return p1.second < p2.second;};
    double max_hpEmisPsb=std::max_element(trace_node.hpEmisPsb.begin(), trace_node.hpEmisPsb.end(),compare)->second;


    for(auto& road_i:trace_node.candidate_roads)
    {
        if(trace_node.hpEmisPsb.count(road_i.first))
        {
            trace_node.hpEmisPsb[road_i.first]/=max_hpEmisPsb;
            trace_node.joint_prob[road_i.first].first*=trace_node.hpEmisPsb[road_i.first];
        }
    }
    

}
