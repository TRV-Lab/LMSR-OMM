#include "hmm_madmap.h"
#include "hmm_lane.h"

/*Lane association main process*/
// void HMM_MADMAP::laneMatch(std::string lane_marker_tracking_file,mad::MadMapStandard &madmapSD,std::unordered_map<mad::MapObjectId,Road_MADMAP,hash_MapObjectId>& hypothesis_roads,std::unordered_map<int,LaneMarkerSDAssociation>& lane_marker_trackings,double LANE_QUERY_DISTANCE,double LANE_PD_LATERAL_STD)
// {
//     for(auto& lane_marker_tracking:lane_marker_trackings)
//     {
//         // if(lane_marker_tracking.first==18)
//         // {
//         //     printf(" ");
//         // }
//         if(lane_marker_tracking.second.points.size()>=5)
//         {
//             LaneMarkerSDAssociation::candidateRoads(madmapSD,hypothesis_roads,lane_marker_tracking.second,LANE_QUERY_DISTANCE);
            
//             LaneMarkerSDAssociation::projection(lane_marker_tracking.second);
//             LaneMarkerSDAssociation::calDistanceEmisPsb(lane_marker_tracking.second,LANE_PD_LATERAL_STD);
//             LaneMarkerSDAssociation::calAngleEmisPsb(lane_marker_tracking.second, ANGLE_POWER_EXPONENT, ANGLE_POSSI_DEFAULT);
//             // clock_t time4=clock();
//             // std::cout<<"projection time cost:"<<double(time4-time3)/1000<<" ms"<<std::endl;
//             LaneMarkerSDAssociation::calPriorTranPsb(lane_marker_tracking.second);

//             LaneMarkerSDAssociation::Viterbi(lane_marker_tracking.second,MIN_PROBABILITY_THRESHOLD);     
//             LaneMarkerSDAssociation::refine(madmapSD,lane_marker_tracking.second);
//             for(auto& prob:lane_marker_tracking.second.road_probs)
//             {
//                 lane_road_association[prob.first][lane_marker_tracking.first]=prob.second;
//                 if(prob.second<0.999)
//                 {
//                     // std::cout<<"road"<<prob.first.getPart1()<<prob.first.getPart2()<<std::endl;
//                     for(auto& point:lane_marker_tracking.second.points)
//                     {
//                         lane_out<<std::setprecision(8)<<lane_marker_tracking.first<<' '<<point.lat<<' '<<point.lon<<'\n';

//                     }

//                 }
//             }
//         }

//     }


// }

void LaneMarkerSDAssociation::candidateRoads(mad::MadMapStandard &madmapSD,std::unordered_map<mad::MapObjectId,Road_MADMAP,hash_MapObjectId>& hypothesis_roads,LaneMarkerSDAssociation &lane_marker,double query_distance)
{
   
    double sideLengthDeg=query_distance /LONLAT2METER;

    // std::unordered_map<mad::MapObjectId,mad::MapObject,hash_MapObjectId> object_map;
    // for(size_t i=0;i<lane_marker.points.size();i++)
    // {
    //     mad::MapObjects objects;
    //     mad::GeoPosition lowerLeft(mad::Longitude(mad::Degree(lane_marker.points[i].lon-sideLengthDeg)),mad::Latitude(mad::Degree(lane_marker.points[i].lat-sideLengthDeg)));
    //     mad::GeoPosition upperRight(mad::Longitude(mad::Degree(lane_marker.points[i].lon+sideLengthDeg)),mad::Latitude(mad::Degree(lane_marker.points[i].lat+sideLengthDeg)));
    //     mad::GeoRectangle area(lowerLeft,upperRight);
    //     madmapSD.getSDObjects(area,objects);
    //     for(auto& obj:objects)
    //     {
    //         object_map[obj.getGeoId()]=obj;
    //     }        
    // }

    for(size_t pi=0;pi<lane_marker.points.size();pi++)
    {
        auto candidate_history=pi>0? lane_marker.points[pi-1].candidate_roads: std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId>();
        mad::MapObjects objects;
        mad::GeoPosition lowerLeft(mad::Longitude(mad::Degree(lane_marker.points[pi].lon-sideLengthDeg)),mad::Latitude(mad::Degree(lane_marker.points[pi].lat-sideLengthDeg)));
        mad::GeoPosition upperRight(mad::Longitude(mad::Degree(lane_marker.points[pi].lon+sideLengthDeg)),mad::Latitude(mad::Degree(lane_marker.points[pi].lat+sideLengthDeg)));
        mad::GeoRectangle area(lowerLeft,upperRight);
        madmapSD.getSDObjects(area,objects);
        for ( auto& obj:objects)
        {
            if(!hypothesis_roads.count(obj.getGeoId()))
            {
                continue;
            }
            if(candidate_history.count(obj.getGeoId()) ==1)//had this road info
            {
                lane_marker.points[pi].candidate_roads[obj.getGeoId()]=candidate_history[obj.getGeoId()];//copy 
                continue;
            }
            auto & link=static_cast< mad::Link&>(obj);
            if(link.isValid())
            {
                Road_MADMAP road;
                road.id=obj.getGeoId();
                std::vector<mad::Link> predecessors,successors;
                road.direction=link.getTrafficDirection();
                // if(obj.getGeoId().getPart1()==1300759249425629010 && obj.getGeoId().getPart2()==1300756723984860889)
                // {
                //     printf(" ");
                // }
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
                int32_t shapepointCount=link.getShapepointCount();
                int32_t index=0;
                for(int i=0;i<shapepointCount;i++)
                {
                    double lon=link.getShapepoint(index).mLocation.getXLonDegree().toFloat();
                    double lat=link.getShapepoint(index).mLocation.getYLatDegree().toFloat();
                    road.node.push_back(LatLon(lat, lon) );
                    index++;

                }
                
                lane_marker.points[pi].candidate_roads.insert(std::make_pair(road.id,road));
                // if(road.id.getPart1()==1296417600131160364 &&road.id.getPart2()==1296254163740663294)
                // {
                //     printf(" ");
                // }
            }

        }
    }

}

/*Calculate trace projection point*/
void LaneMarkerSDAssociation::projection(LaneMarkerSDAssociation &lane_marker)
{


    // std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId>& candidate_roads=trace_node.candidate_roads;
    for(size_t i=0;i<lane_marker.points.size();i++)
    {
        for(auto& road:lane_marker.points[i].candidate_roads)
        {
            LatLon x(lane_marker.points[i].lat, lane_marker.points[i].lon);
            double ans=1000;
            int min_index=0;
            for(int k=0;k< road.second.node.size()-1;k++)
            {
                double tmp= LatLon::distance( x, road.second.node[k], road.second.node[k+1]);
                if(tmp < ans)
                {
                    min_index=k;
                    ans=tmp;
                }
            }
            lane_marker.points[i].projection_distance[road.first]=ans;
            lane_marker.points[i].trace_projection[road.first]=LatLon::projection_point(x,road.second.node[min_index],road.second.node[min_index+1]);

            // double heading_1=atan2((road.second.node[min_index+1].lon-road.second.node[min_index].lon)*cos(trace_node.lat),(road.second.node[min_index+1].lat-road.second.node[min_index].lat));
            double heading=TraceNode::calBearing(road.second.node[min_index].lat,road.second.node[min_index].lon,road.second.node[min_index+1].lat,road.second.node[min_index+1].lon);
            
            if(road.second.direction==mad::LinkDirection::FROM_NODE_0 || road.second.direction==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
            {
                lane_marker.points[i].road_heading[road.first]=heading;
            }
            else if(road.second.direction==mad::LinkDirection::FROM_NODE_1)
            {
                lane_marker.points[i].road_heading[road.first]=heading+M_PI;
            }
        }
    }




}


void LaneMarkerSDAssociation::calDistanceEmisPsb(LaneMarkerSDAssociation &lane_marker,double PD_LATERAL_STD)
{
    for(size_t i=0;i<lane_marker.points.size();i++)
    {
        for(auto& road:lane_marker.points[i].candidate_roads)
        {

            // double ans = 1. / ( PD_LATERAL_STD * sqrt( 2 * M_PI ) ) * exp( -0.5 * pow(lane_marker.points[i].projection_distance[road.first] / PD_LATERAL_STD ,2));
            double ans = exp( -0.5 * pow(lane_marker.points[i].projection_distance[road.first] / PD_LATERAL_STD ,2));

            // ep.insert(std::make_pair(road.first,ans)  );
            lane_marker.points[i].distanceEmisPsb[road.first]=ans;
        }
    }
}
void LaneMarkerSDAssociation::calAngleEmisPsb(LaneMarkerSDAssociation &lane_marker,double ANGLE_POWER_EXPONENT,double ANGLE_POSSI_DEFAULT)
{
    for(size_t i=0;i<lane_marker.points.size();i++)
    {
        lane_marker.points[i].heading=i<lane_marker.points.size()-1? TraceNode::calBearing(lane_marker.points[i].lat,lane_marker.points[i].lon,lane_marker.points[i+1].lat,lane_marker.points[i+1].lon):TraceNode::calBearing(lane_marker.points[i-1].lat,lane_marker.points[i-1].lon,lane_marker.points[i].lat,lane_marker.points[i].lon);
        for(auto& road:lane_marker.points[i].candidate_roads)
        {
            // double ans=cos(fabs(trace_node.heading-trace_node.road_heading[road.first]));
            double ans;

            double delta_heading=fabs(lane_marker.points[i].heading-lane_marker.points[i].road_heading[road.first]);
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
            lane_marker.points[i].angleEmisPsb[road.first]=ans;
        }
    }
}

/*Calculate transition possibility*/
void LaneMarkerSDAssociation::calPriorTranPsb(LaneMarkerSDAssociation &lane_marker)
{
    // tranPsb matrix initialization
    // tranPsb.resize(roads.size());
    // for(auto& row: tranPsb)
    // {
    //     row.resize(roads.size(),TRANS_POSSI_DEFAULT);
    // }
    for(size_t i=0;i<lane_marker.points.size();i++)
    {
        std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId>& roads=lane_marker.points[i].candidate_roads;
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
            for(auto& end_node:road_i.second.end_node)
            {
                road_i.second.tranPsb[end_node]=1;
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
                                road_i.second.tranPsb[road_k.first]=exp(-1*road_j.second.length/20)*road_i.second.tranPsb[road_j.first]*road_j.second.tranPsb[road_k.first];
                                if(road_i.second.tranPsb[road_k.first]>0)
                                {
                                    loop_search=true;
                                }
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


/*TODO:Keep last matching road if not enough confident*/
void LaneMarkerSDAssociation::Viterbi(LaneMarkerSDAssociation &lane_marker,std::unordered_map<mad::MapObjectId,std::unordered_map<mad::MapObjectId,double,hash_MapObjectId>,hash_MapObjectId>& global_tranPsb,double MIN_PROBABILITY_THRESHOLD)
{

    for(size_t p=0;p<lane_marker.points.size();p++)
    {
        for(auto& road_i:lane_marker.points[p].candidate_roads)
        {
            std::vector<std::pair<mad::MapObjectId,double> > possibilities;
            if(p==0)
            {
                for(auto& road_j:lane_marker.points[p].candidate_roads)
                {
                    double possibility = global_tranPsb[road_j.first][road_i.first] * lane_marker.points[p].distanceEmisPsb[road_i.first]*lane_marker.points[p].angleEmisPsb[road_i.first];
                    possibilities.push_back(std::make_pair(road_j.first,possibility));
                }
            }
            else {

                for(int i=0;i<lane_marker.points[p-1].sorted_joint_prob.size();i++)
                {
                    double possibility=0;
                    Road_MADMAP& road_j=lane_marker.points[p-1].candidate_roads[lane_marker.points[p-1].sorted_joint_prob[i].first];
                    if(lane_marker.points[p-1].sorted_joint_prob[i].second.first==0)
                    {
                        continue;
                    }
                    possibility = lane_marker.points[p-1].sorted_joint_prob[i].second.first*global_tranPsb[road_j.id][road_i.first]* lane_marker.points[p].distanceEmisPsb[road_i.first]*lane_marker.points[p].angleEmisPsb[road_i.first];

                    possibilities.push_back(std::make_pair(road_j.id,possibility));
                }
                if(possibilities.size()==0)
                {
                    for(auto& road_j:lane_marker.points[p].candidate_roads)
                    {
                        double possibility = global_tranPsb[road_j.first][road_i.first] * lane_marker.points[p].distanceEmisPsb[road_i.first]*lane_marker.points[p].angleEmisPsb[road_i.first];
                        possibilities.push_back(std::make_pair(road_j.first,possibility));
                    }
                }
            }
            std::vector<std::pair<mad::MapObjectId,double> >::iterator max_element;

            auto compare=[](const std::pair<mad::MapObjectId,double> &p1,const std::pair<mad::MapObjectId,double> &p2){return p1.second < p2.second;};
            max_element=std::max_element(possibilities.begin(), possibilities.end(),compare);

            if(max_element->second>0)
            {
                lane_marker.points[p].joint_prob.insert(std::make_pair(road_i.first, std::make_pair(max_element->second, max_element->first) ));
            }//test
            // std::cout<< std::setprecision(8)<<"debug"<<max_element->first.getPart1()<<" "<< max_element->first.getPart2()<<"to "<<road_i.first.getPart1()<<" "<< road_i.first.getPart2()<<" "<<max_element->second<<" "<< lane_marker.points[p].distanceEmisPsb[road_i.first]<<" "<<lane_marker.points[p].angleEmisPsb[road_i.first]<<std::endl;

        }
        auto compare_joint_prob=[](const std::pair<mad::MapObjectId, std::pair<double, mad::MapObjectId>> &p1,const std::pair<mad::MapObjectId, std::pair<double, mad::MapObjectId>> &p2){return p1.second.first > p2.second.first ;};

        auto& Pback=lane_marker.points[p].sorted_joint_prob;
        Pback.assign(lane_marker.points[p].joint_prob.begin(), lane_marker.points[p].joint_prob.end());
        std::sort(Pback.begin(),Pback.end(),compare_joint_prob);
        LaneMarkerSDAssociation::validHypothesis(lane_marker.points[p], MIN_PROBABILITY_THRESHOLD,true);

    }



}


/*Hypothesis filter and Normalization*/
void LaneMarkerSDAssociation::validHypothesis(LanePoint& lp,double MIN_PROBABILITY_THRESHOLD,bool removeLowProb)
{
    // for(size_t p=0;p<lane_marker.points.size();p++)
    // {
        auto& Pback=lp.sorted_joint_prob;
        double sum=0;
        int valid_hypothesis=0;
        for(int i=0; i<Pback.size(); i++)
        {
            // double& prob=Pback[i].second.first;

            sum+=Pback[i].second.first;

        }
        if(sum==0)
        {
            lp.num_hypothesis=valid_hypothesis;
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

        for(auto& prob:lp.joint_prob)
        {
            if(prob.second.first>0)
            {
                prob.second.first/=sum;
            }
        }
        lp.num_hypothesis=valid_hypothesis;
    // }

}
bool LaneMarkerSDAssociation::isSameSource(LanePoint& lane_point)
{
    mad::MapObjectId& first_source_road=lane_point.sorted_joint_prob[0].second.second;
    for(int i=1;i<lane_point.sorted_joint_prob.size();i++)
    {
        if(lane_point.sorted_joint_prob[i].second.second!=first_source_road
            &&lane_point.candidate_roads.count(lane_point.sorted_joint_prob[i].second.second)!=0 && lane_point.candidate_roads[lane_point.sorted_joint_prob[i].second.second].tranPsb[first_source_road]==0
            &&lane_point.candidate_roads.count(first_source_road)!=0 && lane_point.candidate_roads[first_source_road].tranPsb[lane_point.sorted_joint_prob[i].second.second]==0 )
        {
            
            return false;
        }
    }
    return true;
}
/*Different process when facing different number of hypothesis*/
void LaneMarkerSDAssociation::refine(mad::MadMapStandard &madmapSD,LaneMarkerSDAssociation &lane_marker)
{

    // std::cout<<"trace size"<<trace.size()<<std::endl;
    auto& Pback=lane_marker.points.back().sorted_joint_prob;
    // int pi=0;
    // while(pi<lane_marker.points.back().num_hypothesis)
    // {
    //     for(auto it=Pback.begin()+1;it!=Pback.end();)
    //     {
    //         if((lane_marker.points.back().candidate_roads[Pback[0].first].tranPsb[it->first]==1 && lane_marker.points.back().candidate_roads[Pback[0].first].end_node.size()==1)
    //         || (lane_marker.points.back().candidate_roads[it->first].tranPsb[Pback[0].first]==1 && lane_marker.points.back().candidate_roads[it->first].end_node.size()==1))
    //         {
    //             // std::cout<<"Merge probability "<<": "<<Pback[0].first.getPart1()<<"-"<<Pback[0].first.getPart2()<<","<<it->first.getPart1()<<"-"<<it->first.getPart2()<<std::endl;
    //             Pback[0].second.first+=it->second.first;
    //             it=Pback.erase(it);
    //             lane_marker.points.back().num_hypothesis-=1;
    //         }      
    //         else{
    //             it++;
    //         }  
    //     }
    //     pi++;    
    // }

    for(int i=0;i<lane_marker.points.back().sorted_joint_prob.size();i++)
    {
        // std::cout<<"hypothesis "<<i<<" probability: "<<lane_marker.points.back().sorted_joint_prob[i].second.first<<std::endl<<"Route:";
        mad::MapObjectId last_mm=lane_marker.points.back().sorted_joint_prob[i].first;

        // std::cout<<last_mm.getPart1()<<"-"<<last_mm.getPart2();
        lane_marker.road_probs[last_mm]=lane_marker.points.back().sorted_joint_prob[i].second.first;
        for(int j=lane_marker.points.size()-1;j>=0;j--)
        {
            if(lane_marker.points[j].joint_prob[last_mm].second!=last_mm)
            {
                last_mm=lane_marker.points[j].joint_prob[last_mm].second;
                // std::cout<<", "<<last_mm.getPart1()<<"-"<<last_mm.getPart2();
                if(lane_marker.road_probs.count(last_mm)!=0)
                {
                    lane_marker.road_probs[last_mm]+=lane_marker.points.back().sorted_joint_prob[i].second.first;
                }
                else{
                    lane_marker.road_probs[last_mm]=lane_marker.points.back().sorted_joint_prob[i].second.first;

                }
            }
        }
        if(lane_marker.road_probs[last_mm]>1)
        {
            printf(" ");
        }
        // std::cout<<std::endl;
    }



    
    printf(" ");
    // fileOut(trace.back());
    // mm_out_standard<<Pback[0].first.getPart1()+ Pback[0].first.getPart2()<<'\n';
    


}

// void TraceNode::localLaneMap(TraceNode &trace_node,std::unordered_map<mad::MapObjectId,std::vector<std::pair<int,double>>,hash_MapObjectId>& lane_road_association,std::unordered_set<int>& local_lane_index)
// {
//     local_lane_index.clear();
//     for(auto& road:trace_node.candidate_roads)
//     {
//         if(lane_road_association.count(road.first)!=0)
//         {
//             for(auto& lane:lane_road_association[road.first])
//             {
//                 local_lane_index.insert(lane.first);
//             }
//         }



//     }    
// }
/*Calculate trace projection point*/
void TraceNode::laneProjection(TraceNode &trace_node,std::unordered_map<int,LaneMarkerSDAssociation>& lane_marker_trackings,const std::unordered_map<mad::MapObjectId,std::unordered_map<int,double>,hash_MapObjectId>& lane_road_association)
{

    // std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId>& candidate_roads=trace_node.candidate_roads;
    // for(auto& road:trace_node.candidate_roads)
    // {
    //     if(lane_road_association.count(road.first)!=0)
    //     {
    //         for(auto& lane:lane_road_association[road.first])
    //         {
    //             LatLon x(trace_node.lat, trace_node.lon);
    //             double ans=1000;
    //             int min_index=0;
    //             for(int k=0;k< lane_marker_trackings[lane.first].points.size()-1;k++)
    //             {
    //                 double tmp= distance( x, lane_marker_trackings[lane.first].points[k], lane_marker_trackings[lane.first].points[k+1]);
    //                 if(tmp < ans)
    //                 {
    //                     min_index=k;
    //                     ans=tmp;
    //                 }
    //             }
    //             trace_node.lane_projection_distance[lane.first]=ans;
    //             trace_node.lane_projection_point[lane.first]=projection_point(x,lane_marker_trackings[lane.first].points[min_index],lane_marker_trackings[lane.first].points[min_index+1]);
                
    //         }
    //     }



    // }
    // trace_node.local_lane_index.clear();
    // for(auto&lr_ass:lane_road_association)
    // {
    //     for(auto& lane:lr_ass.second)
    //     {
    //         if(lane.first<0||lane.first>10000)
    //         {
    //             printf(" ");
    //         }
    //     }
    // }
    // mad::MapObjectId id(1346441754242272100,1346414266451555700);
    // std::cout<<"key fault size"<<lane_road_association.at(id).size()<<std::endl;
    for(auto& road:trace_node.candidate_roads)
    {
        if(lane_road_association.count(road.first)!=0)
        {
            // for(auto& lane:lane_road_association[road.first])
            // {
            //     if(lane.first<0 || lane.first>10000)
            //     {
            //         printf(" ");
            //     }
            // }
            for(auto& lane:lane_road_association.at(road.first))
            // for(auto it=lane_road_association[road.first].begin();it!=lane_road_association[road.first].end();it++)
            {
                // std::cout<<"size: "<<lane_road_association.at(road.first).size()<<std::endl;
                trace_node.local_lane_index.insert(lane.first);
                // if(lane.first<0||lane.first>10000)
                // {
                //     printf(" ");
                // }
            }
        }
    // std::cout<<"key fault size"<<lane_road_association.at(id).size()<<std::endl;



    }  
    // LatLon x(trace_node.lat, trace_node.lon);
    LatLon x(trace_node.corrected_pos.lat, trace_node.corrected_pos.lon);
    
    for(auto& lane_id:trace_node.local_lane_index)
    {
        double ans=1000;
        int min_index=0;
        for(int k=0;k< lane_marker_trackings[lane_id].points.size()-1;k++)��
        {
            double tmp= distance( x, lane_marker_trackings[lane_id].points[k], lane_marker_trackings[lane_id].points[k+1]);
            if(tmp < ans)
            {
                min_index=k;
                ans=tmp;
            }
        }
        trace_node.lane_projection_distance[lane_id]=ans;
        LatLon projection_point=LatLon::projection_point(x,lane_marker_trackings[lane_id].points[min_index],lane_marker_trackings[lane_id].points[min_index+1]);
        trace_node.lane_projection_point[lane_id]=projection_point;
        trace_node.lane_projection_heading[lane_id]=TraceNode::calBearing(lane_marker_trackings[lane_id].points[min_index].lat,lane_marker_trackings[lane_id].points[min_index].lon,lane_marker_trackings[lane_id].points[min_index+1].lat,lane_marker_trackings[lane_id].points[min_index+1].lon);

        double projection_direction=TraceNode::calBearing(trace_node.lat, trace_node.lon,projection_point.lat,projection_point.lon)-trace_node.corrected_heading;
        while (projection_direction > 2*M_PI) projection_direction -= 2*M_PI;
        while (projection_direction < 0) projection_direction += 2*M_PI;

        trace_node.lane_projection_direction[lane_id]=projection_direction>M_PI ?ProjectDirection::Left:ProjectDirection::Right;

    }


}
double TraceNode::calAngleEmisPsb(double heading1,double heading2,double ANGLE_POWER_EXPONENT,double ANGLE_POSSI_DEFAULT)
{
    // std::vector<Road_MADMAP>& roads=trace_node.candidate_roads;
    // std::unordered_map<mad::MapObjectId,double,hash_MapObjectId>& ep=trace_node.angleEmisPsb;
    // for(auto& road:trace_node.candidate_roads)
    // {
        // double ans=cos(fabs(trace_node.heading-trace_node.road_heading[road.first]));
    double ans;
    double delta_heading=fabs(heading1-heading2);
    if(delta_heading<M_PI/2 || fabs(delta_heading-2*M_PI)<M_PI/2)
    {
        ans=(cos(2*delta_heading)+1)/2;
    }
    else{
        ans=0;
    }

    ans=pow(ans,ANGLE_POWER_EXPONENT)+ANGLE_POSSI_DEFAULT;

        // ep.insert(std::make_pair(road.first, ans ));
    return ans;
    // }
}

/*lane emis psb befor 20241217*/
/*
void TraceNode::calLaneEmisPsb(TraceNode &trace_node,std::unordered_map<int,LaneMarkerSDAssociation>& lane_marker_trackings,const std::unordered_map<mad::MapObjectId,std::unordered_map<int,double>,hash_MapObjectId>& lane_road_association,double PD_LATERAL_STD,double QUERY_DISTANCE,double ANGLE_POWER_EXPONENT,double ANGLE_POSSI_DEFAULT,double LANE_PROB_SUM_LIMIT,double LANE_WIDTH) 
{
    // if(trace_node.time==11885.15)
    // {
    //     printf(" ");
    // }
    std::vector<std::pair<int,double> > distances_to_left, distances_to_right;
    for(auto& lane_id:trace_node.local_lane_index)
    {
        double distance=trace_node.lane_projection_distance[lane_id];
        if(distance>QUERY_DISTANCE)
        {
            continue;
        }
        else if(trace_node.lane_projection_direction[lane_id]==ProjectDirection::Left)
        {
            distances_to_left.push_back(std::make_pair(lane_id,distance));
        }
        else if(trace_node.lane_projection_direction[lane_id]==ProjectDirection::Right)
        {
            distances_to_right.push_back(std::make_pair(lane_id,distance));
        }
        else{
            assert(0);
        }
    }
    std::sort(distances_to_left.begin(),distances_to_left.end(),[](std::pair<int,double> a,std::pair<int,double> b){return a.second<b.second;});  
    std::sort(distances_to_right.begin(),distances_to_right.end(),[](std::pair<int,double> a,std::pair<int,double> b){return a.second<b.second;});  
    if(distances_to_left.size()>0&&distances_to_left[0].second>LANE_WIDTH)
    {
        distances_to_left.clear();
    }
    if(distances_to_right.size()>0&&distances_to_right[0].second>LANE_WIDTH)
    {
        distances_to_right.clear();
    }

    double laneEmisPsb_sum=0;
    for(auto& road:trace_node.candidate_roads)
    {
        trace_node.laneEmisPsb[road.first]=0;
        if(lane_road_association.count(road.first)==0)
        {
            // trace_node.laneEmisPsb[road.first]=0;
            continue;
        }
        if(distances_to_left.size()>1 && distances_to_right.size()>1)
        {
            double p_l,p_r;
            if(lane_road_association.at(road.first).count(distances_to_left[0].first)!=0)
            {
                p_l=lane_road_association.at(road.first).at(distances_to_left[0].first);
            }
            else{
                p_l=0;
            }
            if(lane_road_association.at(road.first).count(distances_to_right[0].first)!=0)
            {
                p_r=lane_road_association.at(road.first).at(distances_to_right[0].first);
            }
            else{
                p_r=0;
            }
            trace_node.laneEmisPsb[road.first]+=(p_l*distances_to_right[0].second+p_r*distances_to_left[0].second)/(distances_to_right[0].second+distances_to_left[0].second);
            // trace_node.laneEmisPsb[road.first]+= 1. / ( PD_LATERAL_STD * sqrt( 2 * M_PI ) ) * exp( -0.5 * pow(distances_to_left[0].second / PD_LATERAL_STD ,2) )*p_l;
            // trace_node.laneEmisPsb[road.first]+= 1. / ( PD_LATERAL_STD * sqrt( 2 * M_PI ) ) * exp( -0.5 * pow(distances_to_right[0].second / PD_LATERAL_STD ,2) )*p_r;
        }
        else if(distances_to_left.size()>1)
        {
            double p_l;
            if(lane_road_association.at(road.first).count(distances_to_left[0].first)!=0)
            {
                p_l=lane_road_association.at(road.first).at(distances_to_left[0].first);
            }
            else{
                p_l=0;
            }
            trace_node.laneEmisPsb[road.first]+=p_l;
            // trace_node.laneEmisPsb[road.first]+= 1. / ( PD_LATERAL_STD * sqrt( 2 * M_PI ) ) * exp( -0.5 * pow(distances_to_left[0].second / PD_LATERAL_STD ,2) )*p_l;
        }
        else if(distances_to_right.size()>1)
        {
            double p_r;
            if(lane_road_association.at(road.first).count(distances_to_right[0].first)!=0)
            {
                p_r=lane_road_association.at(road.first).at(distances_to_right[0].first);
            }
            else{
                p_r=0;
            }
            trace_node.laneEmisPsb[road.first]+=p_r;
            // trace_node.laneEmisPsb[road.first]+= 1. / ( PD_LATERAL_STD * sqrt( 2 * M_PI ) ) * exp( -0.5 * pow(distances_to_right[0].second / PD_LATERAL_STD ,2) )*p_r;
        }
        // else{
        //     trace_node.laneEmisPsb[road.first]=0;
        // }
        laneEmisPsb_sum+=trace_node.laneEmisPsb[road.first];

   }
    if(laneEmisPsb_sum>LANE_PROB_SUM_LIMIT)//Means no lane map for reference
    {
        // for(auto& laneEmisPsb:trace_node.laneEmisPsb)
        // {
        //     laneEmisPsb.second=1;
        // }
        trace_node.isLaneAvail=true;
    }
    

    // Stage 2
    for(auto& road:trace_node.candidate_roads)
    {
        // std::vector<std::pair<int,double> > distances;
        if(lane_road_association.count(road.first)!=0)
        {
            // for(auto lane:lane_road_association.at(road.first))
            // {
            //     if(trace_node.lane_projection_distance[lane.first]<QUERY_DISTANCE)
            //     {
            //         distances.push_back(std::make_pair(lane.first,trace_node.lane_projection_distance[lane.first]));

            //     }
            // }
            // if(distances_to_left.size()>1)
            for(auto& dis:distances_to_left)
            {
                if(lane_road_association.at(road.first).count(dis.first)!=0)
                {
                    trace_node.laneEmisPsb[road.first]+= 1. / ( PD_LATERAL_STD * sqrt( 2 * M_PI ) ) * exp( -0.5 * pow(dis.second / PD_LATERAL_STD ,2) )*lane_road_association.at(road.first).at(dis.first);
                    break;
                }
            }
            // if(distances_to_right.size()<1)
            for(auto& dis:distances_to_right)          
            {
                // std::sort(distances_to_right.begin(),distances_to_right.end(),[](std::pair<int,double> a,std::pair<int,double> b){return a.second<b.second;});  
                // double marker_distance=distances[0].second;
                // trace_node.laneEmisPsb[road.first]=lane_road_association.at(road.first).at(distances[0].first)*(marker_distance-distances[0].second)/marker_distance;
                if(lane_road_association.at(road.first).count(dis.first)!=0)
                {
                    trace_node.laneEmisPsb[road.first]+= 1. / ( PD_LATERAL_STD * sqrt( 2 * M_PI ) ) * exp( -0.5 * pow(dis.second / PD_LATERAL_STD ,2) )*lane_road_association.at(road.first).at(dis.first);
                    break;
                }
            }
            for(auto& dis:distances_to_left)
            {
                if(lane_road_association.at(road.first).count(dis.first)!=0)
                {
                    trace_node.laneEmisPsb[road.first]*= TraceNode::calAngleEmisPsb(trace_node.lane_projection_heading[dis.first],trace_node.corrected_heading,ANGLE_POWER_EXPONENT,ANGLE_POSSI_DEFAULT);
                    break;
                }
            }
            for(auto& dis:distances_to_right)
            {
                if(lane_road_association.at(road.first).count(dis.first)!=0)
                {
                    trace_node.laneEmisPsb[road.first]*= TraceNode::calAngleEmisPsb(trace_node.lane_projection_heading[dis.first],trace_node.corrected_heading,ANGLE_POWER_EXPONENT,ANGLE_POSSI_DEFAULT);
                    break;
                }
            }
            // if(distances_to_left.size()>1)
            // {            
            //     trace_node.laneEmisPsb[road.first]*= TraceNode::calAngleEmisPsb(trace_node.lane_projection_heading[distances_to_left[0].first],trace_node.heading,ANGLE_POWER_EXPONENT,ANGLE_POSSI_DEFAULT);
            // }
            // if(distances_to_right.size()>1)
            // {            
            //     trace_node.laneEmisPsb[road.first]*= TraceNode::calAngleEmisPsb(trace_node.lane_projection_heading[distances_to_right[0].first],trace_node.heading,ANGLE_POWER_EXPONENT,ANGLE_POSSI_DEFAULT);
            // }
        }
    }


}
*/

void TraceNode::calLaneEmisPsb(TraceNode &trace_node,std::unordered_map<int,LaneMarkerSDAssociation>& lane_marker_trackings,const std::unordered_map<mad::MapObjectId,std::unordered_map<int,double>,hash_MapObjectId>& lane_road_association,double LANE_EMIS_STD,double QUERY_DISTANCE,double ANGLE_POWER_EXPONENT,double ANGLE_POSSI_DEFAULT,double LANE_PROB_SUM_LIMIT,double LANE_WIDTH) 
{

    std::vector<std::pair<int,double> > distances_to_left, distances_to_right;
    for(auto& lane_id:trace_node.local_lane_index)
    {
        double distance=trace_node.lane_projection_distance[lane_id];
        if(distance>QUERY_DISTANCE)
        {
            continue;
        }
        else if(trace_node.lane_projection_direction[lane_id]==ProjectDirection::Left)
        {
            distances_to_left.push_back(std::make_pair(lane_id,distance));
        }
        else if(trace_node.lane_projection_direction[lane_id]==ProjectDirection::Right)
        {
            distances_to_right.push_back(std::make_pair(lane_id,distance));
        }
        else{
            assert(0);
        }
    }
    std::sort(distances_to_left.begin(),distances_to_left.end(),[](std::pair<int,double> a,std::pair<int,double> b){return a.second<b.second;});  
    std::sort(distances_to_right.begin(),distances_to_right.end(),[](std::pair<int,double> a,std::pair<int,double> b){return a.second<b.second;});  
    // if(distances_to_left.size()>0&&distances_to_left[0].second>LANE_WIDTH)
    // {
    //     distances_to_left.clear();
    // }
    // if(distances_to_right.size()>0&&distances_to_right[0].second>LANE_WIDTH)
    // {
    //     distances_to_right.clear();
    // }

    double laneEmisPsb_sum=0;
    

    /* Stage 2*/
    for(auto& road:trace_node.candidate_roads)
    {
        if(lane_road_association.count(road.first)!=0)
        {
            for(auto& dis:distances_to_left)
            {
                if(lane_road_association.at(road.first).count(dis.first)!=0)
                {
                    double disEmisPsb=1. / ( LANE_EMIS_STD * sqrt( 2 * M_PI ) ) * exp( -0.5 * pow(dis.second / LANE_EMIS_STD ,2) )*lane_road_association.at(road.first).at(dis.first);
                    trace_node.laneEmisPsb[road.first]+=disEmisPsb* TraceNode::calAngleEmisPsb(trace_node.lane_projection_heading[dis.first],trace_node.corrected_heading,ANGLE_POWER_EXPONENT,ANGLE_POSSI_DEFAULT);
                    // break;
                }
            }
            for(auto& dis:distances_to_right)          
            {
                if(lane_road_association.at(road.first).count(dis.first)!=0)
                {
                    double disEmisPsb=1. / ( LANE_EMIS_STD * sqrt( 2 * M_PI ) ) * exp( -0.5 * pow(dis.second / LANE_EMIS_STD ,2) )*lane_road_association.at(road.first).at(dis.first);
                    trace_node.laneEmisPsb[road.first]+=disEmisPsb* TraceNode::calAngleEmisPsb(trace_node.lane_projection_heading[dis.first],trace_node.corrected_heading,ANGLE_POWER_EXPONENT,ANGLE_POSSI_DEFAULT);

                    // trace_node.laneEmisPsb[road.first]+= 1. / ( LANE_EMIS_STD * sqrt( 2 * M_PI ) ) * exp( -0.5 * pow(dis.second / LANE_EMIS_STD ,2) )*lane_road_association.at(road.first).at(dis.first);
                    // trace_node.laneEmisPsb[road.first]*= TraceNode::calAngleEmisPsb(trace_node.lane_projection_heading[dis.first],trace_node.corrected_heading,ANGLE_POWER_EXPONENT,ANGLE_POSSI_DEFAULT);
                    // break;
                }
            }
            // for(auto& dis:distances_to_left)
            // {
            //     if(lane_road_association.at(road.first).count(dis.first)!=0)
            //     {
            //         break;
            //     }
            // }
            // for(auto& dis:distances_to_right)
            // {
            //     if(lane_road_association.at(road.first).count(dis.first)!=0)
            //     {
            //         break;
            //     }
            // }
        }
        // double disEmisPsb=1. / ( LANE_EMIS_STD * sqrt( 2 * M_PI ) ) * exp( -0.5 * pow(trace_node.projection_distance[road.first] / LANE_EMIS_STD ,2) );
        // trace_node.laneEmisPsb[road.first]+=disEmisPsb*trace_node.angleEmisPsb[road.first];
        laneEmisPsb_sum+=trace_node.laneEmisPsb[road.first];
    }
    if(laneEmisPsb_sum>LANE_PROB_SUM_LIMIT)//Means no lane map for reference
    {

        trace_node.isLaneAvail=true;
        auto compare=[](const std::pair<mad::MapObjectId,double> &p1,const std::pair<mad::MapObjectId,double> &p2){return p1.second < p2.second;};
        double max_psb=std::max_element(trace_node.laneEmisPsb.begin(),trace_node.laneEmisPsb.end(),compare)->second;
        for(auto& psb:trace_node.laneEmisPsb)
        {
            psb.second/=max_psb;
        }
    }


}














































void LaneMarkerInstance::devideInstance(std::vector<Edge_point>& edge_points,std::unordered_map<int,LaneMarkerInstance>& lane_marker_instances)
{
    for(auto& p:edge_points)
    {
        if(lane_marker_instances.count(p.instance_id)==0)
        {
            lane_marker_instances[p.instance_id].color=p.color;
            lane_marker_instances[p.instance_id].instance_id=p.instance_id;
            lane_marker_instances[p.instance_id].type=p.type;
        }
        lane_marker_instances[p.instance_id].x.push_back(p.x);
        lane_marker_instances[p.instance_id].y.push_back(p.y);
        lane_marker_instances[p.instance_id].std_dev.push_back(p.std_dev);
    }
    for(auto& l:lane_marker_instances)
    {
        auto minmax_x=std::minmax_element(l.second.x.begin(),l.second.x.end());
        l.second.min_x=*minmax_x.first;
        l.second.max_x=*minmax_x.second;
        auto minmax_y=std::minmax_element(l.second.y.begin(),l.second.y.end());
        l.second.min_y=*minmax_y.first;
        l.second.max_y=*minmax_y.second;
    }
}

void LaneMarkerInstance::previewLaneMarkers(std::unordered_map<int,LaneMarkerInstance>& lane_marker_instances,double LANE_PREVIEW_DISTANCE,double LANE_PREVIEW_THRESHOLD)
{
    // LaneDetection lane_detection;
    for(auto& l:lane_marker_instances)
    {
        if(l.second.min_x>LANE_PREVIEW_DISTANCE+LANE_PREVIEW_THRESHOLD || l.second.max_x<LANE_PREVIEW_DISTANCE-LANE_PREVIEW_THRESHOLD)
        {
            continue;
        }
        else{
            // lane_detection.preview_lane_markers_id.push_back(l.first);
            double sum_y=0;
            int count=0;
            for(int i=0;i<l.second.x.size();i++)
            {
                if(l.second.x[i]>LANE_PREVIEW_DISTANCE-LANE_PREVIEW_THRESHOLD &&l.second.x[i]<LANE_PREVIEW_DISTANCE+LANE_PREVIEW_THRESHOLD)
                {
                    sum_y+=l.second.y[i];
                    count++;
                }
            }
            l.second.track=true;
            l.second.preview_avg_y=sum_y/count;
        }
    }
}
void LaneMarkerInstance::previewLanes(std::unordered_map<int,LaneMarkerInstance>& lane_marker_instances,std::unordered_map<int,LaneInstance>& lanes,double LANE_WIDTH,double LANE_FITTING_DEFINITION,double LANE_PREVIEW_DISTANCE,double LANE_PREVIEW_THRESHOLD,double LANE_WIDTH_ERROR_TOLERANCE_FACTOR,double LANE_PREVIEW_MIN,double LANE_PREVIEW_MAX)
{
    // LaneDetection lane_detection;
    LaneCenter p=LaneMarkerInstance::laneCenterFitting(lane_marker_instances,LANE_PREVIEW_DISTANCE,0,LANE_WIDTH,LANE_WIDTH_ERROR_TOLERANCE_FACTOR,LANE_FITTING_DEFINITION);
    int lane_instance_id=0;
    //middle init
    if(p!=LaneCenter())
    {
        lanes[lane_instance_id].lane_centers.push_back(p);  
        lanes[lane_instance_id].instance_id=lane_instance_id;
        lane_instance_id+=2;   
    }
    else{
        return;
    }
    //left init
    lane_instance_id=2;
    while(lanes[lane_instance_id-2].lane_centers[0]!=LaneCenter())
    {
        p=LaneMarkerInstance::laneCenterFitting(lane_marker_instances,lanes[lane_instance_id-2].lane_centers[0].x,lanes[lane_instance_id-2].lane_centers[0].y+LANE_WIDTH,LANE_WIDTH,LANE_WIDTH_ERROR_TOLERANCE_FACTOR,LANE_FITTING_DEFINITION);
        if((p!=LaneCenter() && p.left_marker_type!=0))// || p.right_marker_type==2)
        {
            lanes[lane_instance_id].lane_centers.push_back(p); 
            lanes[lane_instance_id].instance_id=lane_instance_id;
            lane_instance_id+=2;   
    
        }
        else{
            break;
        }
    }
    lane_instance_id=-2;
    //right lane init
    while(lanes[lane_instance_id+2].lane_centers[0]!=LaneCenter())
    {
        p=LaneMarkerInstance::laneCenterFitting(lane_marker_instances,lanes[lane_instance_id+2].lane_centers[0].x,lanes[lane_instance_id+2].lane_centers[0].y-LANE_WIDTH,LANE_WIDTH,LANE_WIDTH_ERROR_TOLERANCE_FACTOR,LANE_FITTING_DEFINITION);
        if((p!=LaneCenter() && p.right_marker_type!=0))// || p.left_marker_type==2)
        {
            lanes[lane_instance_id].lane_centers.push_back(p);    
            lanes[lane_instance_id].instance_id=lane_instance_id;
            lane_instance_id-=2;   
 
        }
        else{
            break;
        }
    }    
    for(auto&lane:lanes)
    {
        // int point_id=0;
        if(lane.second.lane_centers.size()==0)
        {
            continue;
        }
        while(lane.second.lane_centers[0]!=LaneCenter())
        {
            p=LaneMarkerInstance::laneCenterFitting(lane_marker_instances,lane.second.lane_centers[0].x-LANE_FITTING_DEFINITION,lane.second.lane_centers[0].y,LANE_WIDTH,LANE_WIDTH_ERROR_TOLERANCE_FACTOR,LANE_FITTING_DEFINITION);
            if(p!=LaneCenter() && p.x>LANE_PREVIEW_MIN)
            {
                lane.second.lane_centers.push_front(p);     
                // lane.second.instance_id=lane_instance_id;
            }
            else{
                break;
            }
        }
        while(lane.second.lane_centers.back()!=LaneCenter())
        {
            p=LaneMarkerInstance::laneCenterFitting(lane_marker_instances,lane.second.lane_centers.back().x+LANE_FITTING_DEFINITION,lane.second.lane_centers.back().y,LANE_WIDTH,LANE_WIDTH_ERROR_TOLERANCE_FACTOR,LANE_FITTING_DEFINITION);
            if(p!=LaneCenter() && p.x<LANE_PREVIEW_MAX)
            {
                lane.second.lane_centers.push_back(p);     
                // lane.second.instance_id=lane_instance_id;
            }
            else{
                break;
            }
        }
    }
    for(auto&lane:lanes)
    {
        std::unordered_map<int,int> left_type_counts{std::make_pair(1,0),std::make_pair(2,0),std::make_pair(3,0)},right_type_counts{std::make_pair(1,0),std::make_pair(2,0),std::make_pair(3,0)};
        std::unordered_map<int,int> left_id_counts,right_id_counts;
        // std::cout<<"lane width";
        for(auto& p:lane.second.lane_centers)
        {
            if(p.left_marker_type!=0)
            {
                left_type_counts[p.left_marker_type]++;
                left_id_counts[p.left_instance_id]++;
            }
            if(p.right_marker_type!=0)
            {
                right_type_counts[p.right_marker_type]++;
                right_id_counts[p.right_instance_id]++;
            }
            // std::cout<<p.width<<" ";
        }
        // std::cout<<std::endl;
        if(left_type_counts[1]>left_type_counts[2]&&left_type_counts[1]>left_type_counts[3])
        {
            lane.second.left_marker_type=1;

        }
        else if(left_type_counts[2]>left_type_counts[3])
        {
            lane.second.left_marker_type=2;
        }
        else{
            lane.second.left_marker_type=3;
        }

        if(right_type_counts[1]>right_type_counts[2]&&right_type_counts[1]>right_type_counts[3])
        {
            lane.second.right_marker_type=1;
        }
        else if(right_type_counts[2]>right_type_counts[3])
        {
            lane.second.right_marker_type=2;
        }
        else{
            lane.second.right_marker_type=3;
        }
        if(left_id_counts.size()>0)
        {
            lane.second.left_marker_id=std::max_element(left_id_counts.begin(),left_id_counts.end(),[](std::pair<int,int> a,std::pair<int,int> b){return a.second>b.second;})->first;
        }
        if(right_id_counts.size()>0)
        {
            lane.second.right_marker_id=std::max_element(right_id_counts.begin(),right_id_counts.end(),[](std::pair<int,int> a,std::pair<int,int> b){return a.second>b.second;})->first;
        }

        std::cout<<"Lane "<<lane.first<<" : left id "<<lane.second.left_marker_id<<", right id "<<lane.second.right_marker_id<<" : left type "<<lane.second.left_marker_type<<", right type "<<lane.second.right_marker_type<<", point "<<lane.second.lane_centers[0].x<<", "<<lane.second.lane_centers[0].y<<", end "<<lane.second.lane_centers.back().x<<", "<<lane.second.lane_centers.back().y<<std::endl;
    }

}
LaneCenter LaneMarkerInstance::laneCenterFitting(std::unordered_map<int,LaneMarkerInstance>& lane_marker_instances,double x,double y,double LANE_WIDTH,double LANE_WIDTH_ERROR_TOLERANCE_FACTOR,double LANE_FITTING_DEFINITION)
{
    std::vector<std::pair<int,double>> left_min_distances,right_min_distances;
    for(auto& l:lane_marker_instances)
    {
        if(l.second.min_x-x>(1+LANE_WIDTH_ERROR_TOLERANCE_FACTOR)*LANE_WIDTH
            || x-l.second.max_x>(1+LANE_WIDTH_ERROR_TOLERANCE_FACTOR)*LANE_WIDTH
            || l.second.min_y-y>(1+LANE_WIDTH_ERROR_TOLERANCE_FACTOR)*LANE_WIDTH
            || y-l.second.max_y>(1+LANE_WIDTH_ERROR_TOLERANCE_FACTOR)*LANE_WIDTH)
        {
            continue;
        }
        else{
            //求点与点的最小距离
            std::vector<double> distance_square;
            for(int i=0;i<l.second.x.size();i++)
            {
                // if(fabs(x-l.second.x[i])>LANE_FITTING_DEFINITION)continue;
                distance_square.push_back((x-l.second.x[i])*(x-l.second.x[i])+(y-l.second.y[i])*(y-l.second.y[i]));
            }
            auto min_ele=std::min_element(distance_square.begin(),distance_square.end());
            int sign=y-l.second.y[min_ele-distance_square.begin()]>0? 1:-1;
            if(sign>0)
            {
                right_min_distances.push_back(std::make_pair(l.first,y-l.second.y[min_ele-distance_square.begin()]));
            }
            else{
                left_min_distances.push_back(std::make_pair(l.first,l.second.y[min_ele-distance_square.begin()]-y));
            }
        }
    }
    if(left_min_distances.size()!=0 &&right_min_distances.size()!=0 )
    {
        auto left_min=std::min_element(left_min_distances.begin(),left_min_distances.end(),[](std::pair<int,double>a,std::pair<int,double>b){return a.second<b.second;});
        auto right_min=std::min_element(right_min_distances.begin(),right_min_distances.end(),[](std::pair<int,double>a,std::pair<int,double>b){return a.second<b.second;});
        // for(auto& ld:left_min_distances)
        // {
        //     std::cout<<ld.first<<" "<<ld.second<<" ";
        // }
        // std::cout<<std::endl<<"left min"<<left_min->second<<std::endl;
        // for(auto& rd:right_min_distances)
        // {
        //     std::cout<<rd.first<<" "<<rd.second<<" ";
        // }
        // std::cout<<std::endl<<"right min"<<right_min->second<<std::endl;
        LaneCenter lc;
        lc.left_instance_id=left_min->first;
        lc.right_instance_id=right_min->first;
        lc.left_marker_type=lane_marker_instances[left_min->first].type;
        lc.right_marker_type=lane_marker_instances[right_min->first].type;
        lc.x=x;
        lc.y=y+(left_min->second-right_min->second)/2;
        if(left_min->second+right_min->second<(1+LANE_WIDTH_ERROR_TOLERANCE_FACTOR)*LANE_WIDTH && left_min->second+right_min->second>(1-LANE_WIDTH_ERROR_TOLERANCE_FACTOR)*LANE_WIDTH)
        {

            lc.center_info=Normal;
            lc.width=left_min->second+right_min->second;
            return lc;
        }
        else if(left_min->second+right_min->second<(1-LANE_WIDTH_ERROR_TOLERANCE_FACTOR)*LANE_WIDTH){
            lc.center_info=Narrow;
            lc.width=left_min->second+right_min->second;
            return lc;
        }
        else if(lane_marker_instances[left_min->first].type==1 && lane_marker_instances[right_min->first].type==1){
            lc.center_info=Broad;
            lc.width=left_min->second+right_min->second;
            return lc;
        }
        else if(left_min->second+right_min->second>(1+LANE_WIDTH_ERROR_TOLERANCE_FACTOR)*LANE_WIDTH)
        {
            lc.width=left_min->second+right_min->second;
            if(lane_marker_instances[left_min->first].type==1 && lane_marker_instances[right_min->first].type!=1)
            {
                lc.y=y+(left_min->second-LANE_WIDTH/2);
                lc.center_info=FollowLeft;
                return lc;                 
            }
            else if(lane_marker_instances[left_min->first].type!=1 && lane_marker_instances[right_min->first].type==1)
            {
                lc.y=y-(right_min->second-LANE_WIDTH/2);
                lc.center_info=FollowRight;
                return lc;
            }
            else if(left_min>right_min && right_min->second<(1+LANE_WIDTH_ERROR_TOLERANCE_FACTOR)*LANE_WIDTH)
            {
                // lc.left_instance_id=-1;
                // lc.left_marker_type=0;
                lc.y=y-(right_min->second-LANE_WIDTH/2);
                lc.center_info=FollowRight;
                return lc;
            }
            else if(left_min<=right_min && left_min->second<(1+LANE_WIDTH_ERROR_TOLERANCE_FACTOR)*LANE_WIDTH)
            {
                // lc.right_instance_id=-1;
                // lc.right_marker_type=0;
                lc.y=y+(left_min->second-LANE_WIDTH/2);
                lc.center_info=FollowLeft;
                return lc; 
            }
        }
        // else{
        //     assert(0);         
        // }

    }
    else if(left_min_distances.size()!=0)
    {
        auto left_min=std::min_element(left_min_distances.begin(),left_min_distances.end(),[](std::pair<int,double>a,std::pair<int,double>b){return a.second<b.second;});
        if(lane_marker_instances[left_min->first].type==2  && left_min->second<(1+LANE_WIDTH_ERROR_TOLERANCE_FACTOR)*LANE_WIDTH)
        {
            LaneCenter lc;
            lc.width=LANE_WIDTH;
            lc.left_instance_id=left_min->first;
            lc.right_instance_id=-1;
            lc.left_marker_type=lane_marker_instances[left_min->first].type;
            lc.right_marker_type=0;
            lc.x=x;
            lc.y=y+(left_min->second-LANE_WIDTH/2);
            lc.center_info=NoRight;
            return lc;
        }
    }
    else if(right_min_distances.size()!=0)
    {
        auto right_min=std::min_element(right_min_distances.begin(),right_min_distances.end(),[](std::pair<int,double>a,std::pair<int,double>b){return a.second<b.second;});
        if(lane_marker_instances[right_min->first].type==2 && right_min->second<(1+LANE_WIDTH_ERROR_TOLERANCE_FACTOR)*LANE_WIDTH)
        {
            LaneCenter lc;
            lc.width=LANE_WIDTH;
            lc.left_instance_id=-1;
            lc.right_instance_id=right_min->first;
            lc.left_marker_type=0;
            lc.right_marker_type=lane_marker_instances[right_min->first].type;
            lc.x=x;
            lc.y=y-(right_min->second-LANE_WIDTH/2);
            lc.center_info=NoLeft;
            return lc;
        }
    }
    return LaneCenter();

}

