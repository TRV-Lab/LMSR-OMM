#include "hmm_madmap.h"

void TraceNode::candidateRoad(mad::MadMapStandard &madmapSD,TraceNode &trace_node,mad::MapObjectId road_id)
{
    mad::MapObjects objs;
    madmapSD.getSDObject(road_id,objs);

    if(objs.size()==1)
    {
        auto & link=static_cast< mad::Link&>(objs[0]);
        // if(trace_node.candidate_roads.count(objs[0].getGeoId()) ==1)//had this road info
        // {
        //     trace_node.candidate_roads[objs[0].getGeoId()]=candidate_history[obj.getGeoId()];//copy 
        //     continue;
        // }
        if(link.isValid())
        {
            Road_MADMAP road;
            road.id=link.getGeoId();
            std::vector<mad::Link> predecessors,successors;
            road.direction=link.getTrafficDirection();
            road.length=link.getLength()/100.0;
            road.isTunnel=link.isTunnel();
            road.isBridge=link.isBridge();

            if(road.direction==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
            {
                
                if(madmapSD.getLinkSuccessors(link,mad::Direction::BACKWARD,predecessors))
                {
                    for(int i=0;i<predecessors.size();i++)
                    {
                        // road.end_node.push_back(predecessors[i].getGeoId());
                        if(predecessors[i].getTrafficDirection()==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
                        {
                            road.end_node.push_back(predecessors[i].getGeoId());
                            road.start_node.push_back(predecessors[i].getGeoId());
                        }
                        else{
                            std::vector<mad::Link> predecessors_successors;
                            madmapSD.getLinkSuccessors(predecessors[i],mad::Direction::FORWARD,predecessors_successors);
                            for(int j=0;j<predecessors_successors.size();j++)
                            {
                                if(predecessors_successors[j].getGeoId()==road.id)
                                {
                                    // road.end_node.push_back(predecessors[i].getGeoId());
                                    road.start_node.push_back(predecessors[i].getGeoId());

                                    break;
                                }
                            }     
                            std::vector<mad::Link> predecessors_predecessors;
                            madmapSD.getLinkSuccessors(predecessors[i],mad::Direction::BACKWARD,predecessors_predecessors);
                            for(int j=0;j<predecessors_predecessors.size();j++)
                            {
                                if(predecessors_predecessors[j].getGeoId()==road.id)
                                {
                                    road.end_node.push_back(predecessors[i].getGeoId());
                                    // road.start_node.push_back(predecessors[i].getGeoId());

                                    break;
                                }
                            }                            
                                              
                        }



                    }
                    
                }
                if(madmapSD.getLinkSuccessors(link,mad::Direction::FORWARD,successors))
                {
                    for(int i=0;i<successors.size();i++)
                    {
                        // road.end_node.push_back(successors[i].getGeoId());
                        if(successors[i].getTrafficDirection()==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
                        {
                            road.end_node.push_back(successors[i].getGeoId());
                            road.start_node.push_back(successors[i].getGeoId());

                        }
                        else{
                            std::vector<mad::Link> successors_predecessors;
                            madmapSD.getLinkSuccessors(successors[i],mad::Direction::BACKWARD,successors_predecessors);
                            for(int j=0;j<successors_predecessors.size();j++)
                            {
                                if(successors_predecessors[j].getGeoId()==road.id)
                                {
                                    road.end_node.push_back(successors[i].getGeoId());
                                    // road.start_node.push_back(successors[i].getGeoId());
                                    break;
                                }
                            }   
                            std::vector<mad::Link> successors_successors;
                            madmapSD.getLinkSuccessors(successors[i],mad::Direction::FORWARD,successors_successors);
                            for(int j=0;j<successors_successors.size();j++)
                            {
                                if(successors_successors[j].getGeoId()==road.id)
                                {
                                    // road.end_node.push_back(successors[i].getGeoId());
                                    road.start_node.push_back(successors[i].getGeoId());
                                    break;
                                }
                            }                            
                        }

                    }
                }
            }
            else{
                // if(madmapSD.getLinkSuccessors(link,mad::Direction::BACKWARD,predecessors))
                // {
                //     for(int i=0;i<predecessors.size();i++)
                //     {
                //         road.start_node.push_back(predecessors[i].getGeoId());
                //     }
                    
                // }
                if(madmapSD.getLinkSuccessors(link,mad::Direction::BACKWARD,predecessors))
                {
                    for(int i=0;i<predecessors.size();i++)
                    {
                        // road.end_node.push_back(predecessors[i].getGeoId());
                        if(predecessors[i].getTrafficDirection()==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
                        {
                            road.start_node.push_back(predecessors[i].getGeoId());
                        }
                        else{
                            std::vector<mad::Link> predecessors_successors;
                            madmapSD.getLinkSuccessors(predecessors[i],mad::Direction::FORWARD,predecessors_successors);
                            for(int j=0;j<predecessors_successors.size();j++)
                            {
                                if(predecessors_successors[j].getGeoId()==road.id)
                                {
                                    road.start_node.push_back(predecessors[i].getGeoId());

                                    break;
                                }
                            }                            
                        }

                    }
                    
                }
                if(madmapSD.getLinkSuccessors(link,mad::Direction::FORWARD,successors))
                {
                    for(int i=0;i<successors.size();i++)
                    {
                        // road.end_node.push_back(successors[i].getGeoId());
                        if(successors[i].getTrafficDirection()==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
                        {
                            road.end_node.push_back(successors[i].getGeoId());
                        }
                        else{
                            std::vector<mad::Link> successors_predecessors;
                            madmapSD.getLinkSuccessors(successors[i],mad::Direction::BACKWARD,successors_predecessors);
                            for(int j=0;j<successors_predecessors.size();j++)
                            {
                                if(successors_predecessors[j].getGeoId()==road.id)
                                {
                                    road.end_node.push_back(successors[i].getGeoId());
                                    break;
                                }
                            }                            
                        }

                    }
                }
                
            }
                        mad::RoadClass road_class=link.getRoadClass();
            switch (road_class)
            {
            case mad::RoadClass::UNCLASSIFIED:
                road.road_class="UNCLASSIFIED";
                break;
            case mad::RoadClass::MOTORWAY:
                road.road_class="MOTORWAY";
                break;
            case mad::RoadClass::TRUNK:
                road.road_class="TRUNK";
                break;
            case mad::RoadClass::PRIMARY:
                road.road_class="PRIMARY";
                break;
            case mad::RoadClass::SECONDARY:
                road.road_class="SECONDARY";
                break;
            case mad::RoadClass::TERTIARY:
                road.road_class="TERTIARY";
                break;
            case mad::RoadClass::IGNORED:
                road.road_class="IGNORED";
                break;
            case mad::RoadClass::BUILDING:
                road.road_class="BUILDING";
                break;
            case mad::RoadClass::NUM_ROADCLASS:
                road.road_class="NUM_ROADCLASS";
                break;

            default:
                break;
            }
            mad::LinkType link_type=link.getLinkType();
            switch (link_type)
            {
            case mad::LinkType::NONE:
                road.link_type="NONE";
                break;
            case mad::LinkType::RAMP:
                road.link_type="RAMP";
                break;
            case mad::LinkType::ROUNDABOUT:
                road.link_type="ROUNDABOUT";
                break;
            case mad::LinkType::INTERSECTION_OTHER:
                road.link_type="INTERSECTION_OTHER";
                break;
            case mad::LinkType::ON_RAMP:
                road.link_type="ON_RAMP";
                break;
            case mad::LinkType::OFF_RAMP:
                road.link_type="OFF_RAMP";
                break;
            case mad::LinkType::INTERCHANGE:
                road.link_type="INTERCHANGE";
                break;
            case mad::LinkType::DRIVABLE_OTHER:
                road.link_type="DRIVABLE_OTHER";
                break;


            default:
                break;
            }

            int32_t shapepointCount=link.getShapepointCount();
            int32_t index=0;
            for(int i=0;i<shapepointCount;i++)
            {
                double lon=link.getShapepoint(index).mLocation.getXLonDegree().toFloat();
                double lat=link.getShapepoint(index).mLocation.getYLatDegree().toFloat();
                road.node.push_back(LatLon(lat, lon) );
                index++;

            }
            
            trace_node.candidate_roads.insert(std::make_pair(road.id,road));
        }
    }
    else{
        assert(0);
    } 
}

/*Find candidate roads by multi sliding window*/
void TraceNode::candidateRoads(mad::MadMapStandard &madmapSD,TraceNode &trace_node,std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId>& candidate_history,double query_distance,bool use_bias)
{

    
    double sideLengthDeg=query_distance /LONLAT2METER;
    double window_offset_x=sideLengthDeg*2/3*sin(trace_node.heading);
    double window_offset_y=sideLengthDeg*2/3*cos(trace_node.heading);
    if(!use_bias)
    {
        window_offset_x=0;
        window_offset_y=0;
    }

    mad::MapObjects objects;
    mad::GeoPosition lowerLeft(mad::Longitude(mad::Degree(trace_node.lon-sideLengthDeg-window_offset_x)),mad::Latitude(mad::Degree(trace_node.lat-sideLengthDeg-window_offset_y)));
    mad::GeoPosition upperRight(mad::Longitude(mad::Degree(trace_node.lon+sideLengthDeg-window_offset_x)),mad::Latitude(mad::Degree(trace_node.lat+sideLengthDeg-window_offset_y)));
    mad::GeoRectangle area(lowerLeft,upperRight);
    madmapSD.getSDObjects(area,objects);

    for ( auto& obj:objects)
    {
        if(candidate_history.count(obj.getGeoId()) ==1)//had this road info
        {
            trace_node.candidate_roads[obj.getGeoId()]=candidate_history[obj.getGeoId()];//copy 
            continue;
        }
        auto & link=static_cast< mad::Link&>(obj);
        if(link.isValid())
        {
            Road_MADMAP road;
            road.id=link.getGeoId();
            std::vector<mad::Link> predecessors,successors;
            road.direction=link.getTrafficDirection();
            road.length=link.getLength()/100.0;
            road.isTunnel=link.isTunnel();
            road.isBridge=link.isBridge();

            if(road.direction==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
            {
                
                if(madmapSD.getLinkSuccessors(link,mad::Direction::BACKWARD,predecessors))
                {
                    for(int i=0;i<predecessors.size();i++)
                    {
                        // road.end_node.push_back(predecessors[i].getGeoId());
                        if(predecessors[i].getTrafficDirection()==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
                        {
                            road.end_node.push_back(predecessors[i].getGeoId());
                            road.start_node.push_back(predecessors[i].getGeoId());
                        }
                        else{
                            std::vector<mad::Link> predecessors_successors;
                            madmapSD.getLinkSuccessors(predecessors[i],mad::Direction::FORWARD,predecessors_successors);
                            for(int j=0;j<predecessors_successors.size();j++)
                            {
                                if(predecessors_successors[j].getGeoId()==road.id)
                                {
                                    // road.end_node.push_back(predecessors[i].getGeoId());
                                    road.start_node.push_back(predecessors[i].getGeoId());

                                    break;
                                }
                            }         
                            std::vector<mad::Link> predecessors_predecessors;
                            madmapSD.getLinkSuccessors(predecessors[i],mad::Direction::BACKWARD,predecessors_predecessors);
                            for(int j=0;j<predecessors_predecessors.size();j++)
                            {
                                if(predecessors_predecessors[j].getGeoId()==road.id)
                                {
                                    road.end_node.push_back(predecessors[i].getGeoId());
                                    // road.start_node.push_back(predecessors[i].getGeoId());

                                    break;
                                }
                            }                          
                        }



                    }
                    
                }
                if(madmapSD.getLinkSuccessors(link,mad::Direction::FORWARD,successors))
                {
                    for(int i=0;i<successors.size();i++)
                    {
                        // road.end_node.push_back(successors[i].getGeoId());
                        if(successors[i].getTrafficDirection()==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
                        {
                            road.end_node.push_back(successors[i].getGeoId());
                            road.start_node.push_back(successors[i].getGeoId());

                        }
                        else{
                            std::vector<mad::Link> successors_predecessors;
                            madmapSD.getLinkSuccessors(successors[i],mad::Direction::BACKWARD,successors_predecessors);
                            for(int j=0;j<successors_predecessors.size();j++)
                            {
                                if(successors_predecessors[j].getGeoId()==road.id)
                                {
                                    road.end_node.push_back(successors[i].getGeoId());
                                    // road.start_node.push_back(successors[i].getGeoId());
                                    break;
                                }
                            }  
                            std::vector<mad::Link> successors_successors;
                            madmapSD.getLinkSuccessors(successors[i],mad::Direction::FORWARD,successors_successors);
                            for(int j=0;j<successors_successors.size();j++)
                            {
                                if(successors_successors[j].getGeoId()==road.id)
                                {
                                    // road.end_node.push_back(successors[i].getGeoId());
                                    road.start_node.push_back(successors[i].getGeoId());
                                    break;
                                }
                            }                                
                        }

                    }
                }
            }
            else{
                // if(madmapSD.getLinkSuccessors(link,mad::Direction::BACKWARD,predecessors))
                // {
                //     for(int i=0;i<predecessors.size();i++)
                //     {
                //         road.start_node.push_back(predecessors[i].getGeoId());
                //     }
                    
                // }
                if(madmapSD.getLinkSuccessors(link,mad::Direction::BACKWARD,predecessors))
                {
                    for(int i=0;i<predecessors.size();i++)
                    {
                        // road.end_node.push_back(predecessors[i].getGeoId());
                        if(predecessors[i].getTrafficDirection()==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
                        {
                            road.start_node.push_back(predecessors[i].getGeoId());
                        }
                        else{
                            std::vector<mad::Link> predecessors_successors;
                            madmapSD.getLinkSuccessors(predecessors[i],mad::Direction::FORWARD,predecessors_successors);
                            for(int j=0;j<predecessors_successors.size();j++)
                            {
                                if(predecessors_successors[j].getGeoId()==road.id)
                                {
                                    road.start_node.push_back(predecessors[i].getGeoId());

                                    break;
                                }
                            }                            
                        }

                    }
                    
                }
                if(madmapSD.getLinkSuccessors(link,mad::Direction::FORWARD,successors))
                {
                    for(int i=0;i<successors.size();i++)
                    {
                        // road.end_node.push_back(successors[i].getGeoId());
                        if(successors[i].getTrafficDirection()==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
                        {
                            road.end_node.push_back(successors[i].getGeoId());
                        }
                        else{
                            std::vector<mad::Link> successors_predecessors;
                            madmapSD.getLinkSuccessors(successors[i],mad::Direction::BACKWARD,successors_predecessors);
                            for(int j=0;j<successors_predecessors.size();j++)
                            {
                                if(successors_predecessors[j].getGeoId()==road.id)
                                {
                                    road.end_node.push_back(successors[i].getGeoId());
                                    break;
                                }
                            }                            
                        }

                    }
                }
                
            }
            // road.type=7;
            mad::RoadClass road_class=link.getRoadClass();
            switch (road_class)
            {
            case mad::RoadClass::UNCLASSIFIED:
                road.road_class="UNCLASSIFIED";
                break;
            case mad::RoadClass::MOTORWAY:
                road.road_class="MOTORWAY";
                break;
            case mad::RoadClass::TRUNK:
                road.road_class="TRUNK";
                break;
            case mad::RoadClass::PRIMARY:
                road.road_class="PRIMARY";
                break;
            case mad::RoadClass::SECONDARY:
                road.road_class="SECONDARY";
                break;
            case mad::RoadClass::TERTIARY:
                road.road_class="TERTIARY";
                break;
            case mad::RoadClass::IGNORED:
                road.road_class="IGNORED";
                break;
            case mad::RoadClass::BUILDING:
                road.road_class="BUILDING";
                break;
            case mad::RoadClass::NUM_ROADCLASS:
                road.road_class="NUM_ROADCLASS";
                break;

            default:
                break;
            }
            mad::LinkType link_type=link.getLinkType();
            switch (link_type)
            {
            case mad::LinkType::NONE:
                road.link_type="NONE";
                break;
            case mad::LinkType::RAMP:
                road.link_type="RAMP";
                break;
            case mad::LinkType::ROUNDABOUT:
                road.link_type="ROUNDABOUT";
                break;
            case mad::LinkType::INTERSECTION_OTHER:
                road.link_type="INTERSECTION_OTHER";
                break;
            case mad::LinkType::ON_RAMP:
                road.link_type="ON_RAMP";
                break;
            case mad::LinkType::OFF_RAMP:
                road.link_type="OFF_RAMP";
                break;
            case mad::LinkType::INTERCHANGE:
                road.link_type="INTERCHANGE";
                break;
            case mad::LinkType::DRIVABLE_OTHER:
                road.link_type="DRIVABLE_OTHER";
                break;


            default:
                break;
            }
            int32_t shapepointCount=link.getShapepointCount();
            int32_t index=0;
            for(int i=0;i<shapepointCount;i++)
            {
                double lon=link.getShapepoint(index).mLocation.getXLonDegree().toFloat();
                double lat=link.getShapepoint(index).mLocation.getYLatDegree().toFloat();
                road.node.push_back(LatLon(lat, lon) );
                index++;

            }
            
            trace_node.candidate_roads.insert(std::make_pair(road.id,road));
            // if(road.id.getPart1()==1296417600131160364 &&road.id.getPart2()==1296254163740663294)
            // {
            //     printf(" ");
            // }
        }

    }
}



/*Calculate transition possibility*/
void TraceNode::calPriorTranPsb(TraceNode &trace_node,std::unordered_map<mad::MapObjectId,std::unordered_map<mad::MapObjectId,double,hash_MapObjectId>,hash_MapObjectId>& global_tranPsb,double MIN_PROBABILITY_THRESHOLD)//,double distance)
{
    // tranPsb matrix initialization
    // tranPsb.resize(roads.size());
    // for(auto& row: tranPsb)
    // {
    //     row.resize(roads.size(),TRANS_POSSI_DEFAULT);
    // }
    std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId>& roads=trace_node.candidate_roads;
    for(auto& road_i:roads)
    {
        // for(auto& road_j:roads)
        // {
        //     if(Road_MADMAP::isNextLink(road_i.second,road_j.second))
        //     {
        //         road_i.second.tranPsb[road_j.first]=1;
        //     }
        // }
        road_i.second.tranPsb[road_i.first]=1;
        global_tranPsb[road_i.first][road_i.first]=1;
        for(auto& end_node:road_i.second.end_node)
        {
            road_i.second.tranPsb[end_node]=1;
            global_tranPsb[road_i.first][end_node]=1;
        }
    }
    bool loop_search=true;
    while(loop_search)
    {
        loop_search=false;
        for(auto& road_i:roads)
        {
            for(auto& road_j:roads)
            {
                if(road_i.second.tranPsb[road_j.first]>0)
                {
                    for(auto& road_k:roads)
                    {
                        if(road_j.second.tranPsb[road_k.first]>0 && road_i.second.tranPsb[road_k.first]==0)
                        {
                            road_i.second.tranPsb[road_k.first]=exp(-1*road_j.second.length/10)*road_i.second.tranPsb[road_j.first]*road_j.second.tranPsb[road_k.first];
                            global_tranPsb[road_i.first][road_k.first]=exp(-1*road_j.second.length/10)*road_i.second.tranPsb[road_j.first]*road_j.second.tranPsb[road_k.first];

                            if(road_i.second.tranPsb[road_k.first]< MIN_PROBABILITY_THRESHOLD)
                            {
                                road_i.second.tranPsb[road_k.first]=0;
                                global_tranPsb[road_i.first][road_k.first]=0;

                            }
                            else
                            {
                                loop_search=true;
                            }
                        }
                    }
                }
            }
        }
    }
                


    
    //test
    // for(auto& road:tranPsb)
    // {
    //     std::cout<<road.first.getPart1()<<", "<<road.first.getPart2()<<"to "<<std::endl;
    //     for(auto& nd:road.second)
    //     {
    //         std::cout<<nd.first.getPart1()<<", "<<nd.first.getPart2()<<" "<<nd.second<<std::endl;
    //     }
    // }    


}



/*
TODO:speed model/post prior prob model/route length emission model
*/
void TraceNode::calDistanceEmisPsb(TraceNode &trace_node,double lateral_std)
{
    for(auto& road:trace_node.candidate_roads)
    {

        // double ans = 1. / ( lateral_std * sqrt( 2 * M_PI ) ) * exp( -0.5 * pow(trace_node.projection_distance[road.first] / lateral_std ,2) );
        double ans=exp( -0.5 * pow(trace_node.projection_distance[road.first] / lateral_std ,2) );
        trace_node.distanceEmisPsb[road.first]=ans;
    }
    

    
}




void TraceNode::calAngleEmisPsb(TraceNode &trace_node,double ANGLE_POWER_EXPONENT,double ANGLE_POSSI_DEFAULT)
{
    // std::vector<Road_MADMAP>& roads=trace_node.candidate_roads;
    // std::unordered_map<mad::MapObjectId,double,hash_MapObjectId>& ep=trace_node.angleEmisPsb;
    for(auto& road:trace_node.candidate_roads)
    {
        // double ans=cos(fabs(trace_node.heading-trace_node.road_heading[road.first]));
        double ans;
        double delta_heading=fabs(trace_node.corrected_heading-trace_node.road_heading[road.first]);
        if(delta_heading<M_PI/2 || fabs(delta_heading-2*M_PI)<M_PI/2)
        {
            ans=(cos(2*delta_heading)+1)/2;
        }
        else if(road.second.direction==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
        {
            ans=std::max((cos(2*delta_heading)+1)/2,(cos(2*(M_PI-delta_heading))+1)/2);
        }
        else{
            ans=0;
        }

        ans=pow(ans,ANGLE_POWER_EXPONENT)+ANGLE_POSSI_DEFAULT;

        // ep.insert(std::make_pair(road.first, ans ));
        trace_node.angleEmisPsb[road.first]=ans;
    }


    
}

/*TODO:Keep last matching road if not enough confident*/
void HMM_MADMAP::Viterbi(std::deque<TraceNode> &trace,bool USE_LANE_MARKER,bool COMPARE_P2C,bool COMPARE_OBDSC,double CONFIDENCE_THRESHOLD)
{



    for(auto& road_i:trace.back().candidate_roads)
    {
        std::vector<std::pair<mad::MapObjectId,double> > possibilities;

        if(trace.size()==1 ||trace[trace.size()-2].confidence<CONFIDENCE_THRESHOLD)
        {
            for(auto& road_j:trace.back().candidate_roads)
            {
                double possibility = road_j.second.tranPsb[road_i.first] * trace.back().distanceEmisPsb[road_i.first]*trace.back().angleEmisPsb[road_i.first];
                if(USE_LANE_MARKER&&trace.back().isLaneAvail)
                {
                    possibility*=trace.back().laneEmisPsb[road_i.first];
                }
                // if(trace.back().hpEmisPsb.size()>0)
                // {
                //     possibility*=trace.back().hpEmisPsb[road_i.first];
                // }

                if(COMPARE_P2C)
                {
                    possibility=trace.back().distanceEmisPsb[road_i.first];
                }
                if(COMPARE_OBDSC && trace.back().OBDSC_EmisPsb.size()>0)
                {
                    possibility*=trace.back().OBDSC_EmisPsb[road_i.first];
                }
                possibilities.push_back(std::make_pair(road_j.first,possibility));
            }
        }
        else {

            for(int i=0;i<trace[trace.size()-2].sorted_joint_prob.size();i++)
            {
                double possibility=0;
                Road_MADMAP& road_j=trace[trace.size()-2].candidate_roads[trace[trace.size()-2].sorted_joint_prob[i].first];
                if(trace[trace.size()-2].sorted_joint_prob[i].second.first==0)
                {
                    continue;
                }
                possibility = trace[trace.size()-2].sorted_joint_prob[i].second.first*road_j.tranPsb[road_i.first] * trace.back().distanceEmisPsb[road_i.first]*trace.back().angleEmisPsb[road_i.first];
                if(USE_LANE_MARKER&&trace.back().isLaneAvail)
                {
                    // ;//demo
                    // if(trace.back().laneEmisPsb[road_i.first]<1e-4)
                    // {
                    //     trace.back().laneEmisPsb[road_i.first]=1e-4;
                    // }
                    possibility*=trace.back().laneEmisPsb[road_i.first];
                }
                // if(road_j.preview_tranPsb.count(road_i.first)!=0)
                // {
                //     auto compare=[](const std::pair<mad::MapObjectId,double> &p1,const std::pair<mad::MapObjectId,double> &p2){return p1.second < p2.second;};
                //     double max_trans=std::max_element(road_j.preview_tranPsb.begin(),road_j.preview_tranPsb.end(),compare)->second;
                //     possibility*=road_j.preview_tranPsb[road_i.first]/max_trans;//external
                //     std::cout<<"holistic: "<<road_j.id.getPart1()/100%100<<road_j.id.getPart2()/100%100<<"->"<<road_i.first.getPart1()/100%100<<road_i.first.getPart2()/100%100<<": "<<road_j.preview_tranPsb[road_i.first]/max_trans<<std::endl;
                // }
                // if(trace.back().hpEmisPsb.size()>0)
                // {
                //     possibility*=trace.back().hpEmisPsb[road_i.first];
                // }

                if(COMPARE_P2C)
                {
                    possibility=trace.back().distanceEmisPsb[road_i.first];
                }
                if(COMPARE_OBDSC && trace.back().OBDSC_EmisPsb.size()>0)
                {
                    possibility*=trace.back().OBDSC_EmisPsb[road_i.first];
                }


                if(possibility>0)
                {
                    possibilities.push_back(std::make_pair(road_j.id,possibility));

                }
            }
        }
        if(possibilities.size()>0)
        {
            std::vector<std::pair<mad::MapObjectId,double> >::iterator max_element;
            auto compare=[](const std::pair<mad::MapObjectId,double> &p1,const std::pair<mad::MapObjectId,double> &p2){return p1.second < p2.second;};
            max_element=std::max_element(possibilities.begin(), possibilities.end(),compare);

            // if(max_element->second>0)
            // {
            trace.back().joint_prob[road_i.first]=std::make_pair(max_element->second, max_element->first) ;

        }
        // }//test
        // std::cout<< std::setprecision(8)<<"debug"<<trace.back().lat<<" "<<trace.back().lon<<"projection"<<trace.back().trace_projection[roads[j].id].lat<<", "<<trace.back().trace_projection[roads[j].id].lon<<", "<<roads[j].id.getPart1()<<" "<< roads[j].id.getPart2()<<" "<<max_element->second<<" "<< distanceEmisPsb[roads[j].id]<<" "<<angleEmisPsb[roads[j].id]<<std::endl;
        // std::cout<< std::setprecision(8)<<"debug"<<max_element->first.getPart1()<<" "<< max_element->first.getPart2()<<"to "<<road_i.first.getPart1()<<" "<< road_i.first.getPart2()<<" "<<max_element->second<<" "<< trace.back().distanceEmisPsb[road_i.first]<<" "<<trace.back().angleEmisPsb[road_i.first]<<std::endl;

    }


    // if(Pback.size()>0)
    // {
    //     std::cout<<"Probability: "<<Pback[0].first.getPart1()<<"-"<<Pback[0].first.getPart2()<<", "<<trace.back().distanceEmisPsb[Pback[0].first]<<", "<<trace.back().angleEmisPsb[Pback[0].first]<<", "<<trace.back().laneEmisPsb[Pback[0].first]<<", "<<std::endl;
    // }
}
// void HMM_MADMAP::Viterbi(TraceNode&trace_node,TraceNode& last_trace_node,bool USE_LANE_MARKER)
// {


//     for(auto& road_i:trace_node.candidate_roads)
//     {
//         std::vector<std::pair<mad::MapObjectId,double> > possibilities;
//         if(trace_node.sorted_joint_prob.size()==0)
//         {
//             for(auto& road_j:trace_node.candidate_roads)
//             {
//                 double possibility = road_j.second.tranPsb[road_i.first] * trace_node.distanceEmisPsb[road_i.first]*trace_node.angleEmisPsb[road_i.first];
//                 if(USE_LANE_MARKER&&trace_node.isLaneAvail)
//                 {
//                     possibility*=trace_node.laneEmisPsb[road_i.first];
//                 }
//                 possibilities.push_back(std::make_pair(road_j.first,possibility));
//             }
//         }
//         else{
//             for(int i=0;i<last_trace_node.sorted_joint_prob.size();i++)
//             {
//                 double possibility=0;
//                 Road_MADMAP& road_j=last_trace_node.candidate_roads[last_trace_node.sorted_joint_prob[i].first];
//                 if(last_trace_node.sorted_joint_prob[i].second.first==0)
//                 {
//                     continue;
//                 }
//                 possibility = last_trace_node.sorted_joint_prob[i].second.first*road_j.tranPsb[road_i.first] * trace_node.distanceEmisPsb[road_i.first]*trace_node.angleEmisPsb[road_i.first];
//                 if(USE_LANE_MARKER&&trace_node.isLaneAvail)
//                 {
//                     // ;//demo
//                     if(trace_node.laneEmisPsb[road_i.first]<1e-4)
//                     {
//                         trace_node.laneEmisPsb[road_i.first]=1e-4;
//                     }
//                     possibility*=trace_node.laneEmisPsb[road_i.first];
//                 }
//                 if(road_j.preview_tranPsb.count(road_i.first)!=0)
//                 {
//                     auto compare=[](const std::pair<mad::MapObjectId,double> &p1,const std::pair<mad::MapObjectId,double> &p2){return p1.second < p2.second;};
//                     double max_trans=std::max_element(road_j.preview_tranPsb.begin(),road_j.preview_tranPsb.end(),compare)->second;
//                     possibility*=road_j.preview_tranPsb[road_i.first]/max_trans;//external
//                     std::cout<<"holistic: "<<road_j.id.getPart1()/100%100<<road_j.id.getPart2()/100%100<<"->"<<road_i.first.getPart1()/100%100<<road_i.first.getPart2()/100%100<<": "<<road_j.preview_tranPsb[road_i.first]/max_trans<<std::endl;
//                 }


//                 if(possibility>0)
//                 {
//                     possibilities.push_back(std::make_pair(road_j.id,possibility));

//                 }
//             }
//         }

        
//         if(possibilities.size()>0)
//         {
//             std::vector<std::pair<mad::MapObjectId,double> >::iterator max_element;
//             auto compare=[](const std::pair<mad::MapObjectId,double> &p1,const std::pair<mad::MapObjectId,double> &p2){return p1.second < p2.second;};
//             max_element=std::max_element(possibilities.begin(), possibilities.end(),compare);

//             // if(max_element->second>0)
//             // {
//             trace_node.joint_prob[road_i.first]=std::make_pair(max_element->second, max_element->first) ;

//         }
//         // }//test
//         // std::cout<< std::setprecision(8)<<"debug"<<trace_node.lat<<" "<<trace_node.lon<<"projection"<<trace_node.trace_projection[roads[j].id].lat<<", "<<trace_node.trace_projection[roads[j].id].lon<<", "<<roads[j].id.getPart1()<<" "<< roads[j].id.getPart2()<<" "<<max_element->second<<" "<< distanceEmisPsb[roads[j].id]<<" "<<angleEmisPsb[roads[j].id]<<std::endl;
//         // std::cout<< std::setprecision(8)<<"debug"<<max_element->first.getPart1()<<" "<< max_element->first.getPart2()<<"to "<<road_i.first.getPart1()<<" "<< road_i.first.getPart2()<<" "<<max_element->second<<" "<< trace_node.distanceEmisPsb[road_i.first]<<" "<<trace_node.angleEmisPsb[road_i.first]<<std::endl;

//     }
//     auto& Pback=trace_node.sorted_joint_prob;
//     if(trace_node.joint_prob.size()>0)
//     {
//         auto compare_joint_prob=[](const std::pair<mad::MapObjectId, std::pair<double, mad::MapObjectId>> &p1,const std::pair<mad::MapObjectId, std::pair<double, mad::MapObjectId>> &p2){return p1.second.first > p2.second.first ;};
//         Pback.assign(trace_node.joint_prob.begin(), trace_node.joint_prob.end());
//         std::sort(Pback.begin(),Pback.end(),compare_joint_prob);

//     }

//     // if(Pback.size()>0)
//     // {
//     //     std::cout<<"Probability: "<<Pback[0].first.getPart1()<<"-"<<Pback[0].first.getPart2()<<", "<<trace_node.distanceEmisPsb[Pback[0].first]<<", "<<trace_node.angleEmisPsb[Pback[0].first]<<", "<<trace_node.laneEmisPsb[Pback[0].first]<<", "<<std::endl;
//     // }    
// }
