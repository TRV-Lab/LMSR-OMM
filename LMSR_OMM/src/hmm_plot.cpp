
#include "hmm_madmap.h"

void HMM_MADMAP::localPlot(std::deque<TraceNode>& trace,std::unordered_map<int,LaneMarkerSDAssociation>& lane_marker_trackings,bool ENABLE_PLOT_SHOW)
{
    static bool first=true;
    static std::string folderPath;
    if(first)
    {
        time_t now = time(0);
        char timeString[20];
        strftime(timeString, sizeof(timeString), "%Y-%m-%d-%H:%M:%S", localtime(&now));

        folderPath = "/research/tongji_project_ws/build/hdf_read/lane_detection"+std::string(timeString);  // 要创建的文件夹路径
        mode_t mode = 0777;  
        first=false;
    }

    if(trace.size()==0)
    {
        return;
    }
    plt::figure_size(700,700);
    double range=QUERY_DISTANCE/LONLAT2METER;
    double window_offset_x=range*2/3*sin(trace.back().heading);
    double window_offset_y=range*2/3*cos(trace.back().heading);

    std::map<std::string,std::string> keywords;

    /*plot detected lane marker in original and corrected vehicle pos */
    // if(trace.back().isConverged)
    // {
        for(auto& lm:trace.back().lane_marker_instances)
        {
            std::map<std::string,std::string> keywords;
            keywords["marker"]=".";
            keywords["linestyle"]="";
            std::vector<double> tmp_x,tmp_y;
            std::vector<double> correct_x,correct_y;

            for(int i=0;i<lm.second.y.size();i++)
            {
                // tmp_x.push_back(trace.back().lon+lm.second.x[i]*sin(trace.back().corrected_heading)/LONLAT2METER/cos(TraceNode::rad(trace.back().corrected_pos.lat))-lm.second.y[i]*cos(trace.back().corrected_heading)/LONLAT2METER/cos(TraceNode::rad(trace.back().corrected_pos.lat)));
                // tmp_y.push_back(trace.back().lat+lm.second.y[i]*sin(trace.back().corrected_heading)/LONLAT2METER+lm.second.x[i]*cos(trace.back().corrected_heading)/LONLAT2METER);
                
                correct_x.push_back(trace.back().corrected_pos.lon+lm.second.x[i]*sin(trace.back().corrected_heading)/LONLAT2METER/cos(TraceNode::rad(trace.back().corrected_pos.lat))-lm.second.y[i]*cos(trace.back().corrected_heading)/LONLAT2METER/cos(TraceNode::rad(trace.back().corrected_pos.lat)));
                correct_y.push_back(trace.back().corrected_pos.lat+lm.second.y[i]*sin(trace.back().corrected_heading)/LONLAT2METER+lm.second.x[i]*cos(trace.back().corrected_heading)/LONLAT2METER);

            }
            if(lm.second.color==2)
            {
                keywords["c"]="y";
            }
            else if(lm.second.type==1)
            {
                keywords["c"]="k";
            }
            else if(lm.second.type==2)
            {
                keywords["c"]="gray";
            }
            else
            {
                keywords["c"]="r";
            }        
            plt::plot(correct_x,correct_y,keywords);
            // plt::plot(tmp_x,tmp_y,keywords);

        }
    // }


    /*plot lane marker map*/
    for(auto& lane_id:trace.back().local_lane_index)
    {

        std::vector<double> tmp_x,tmp_y;
        int last_type=0;
        for(auto& p:lane_marker_trackings[lane_id].points)
        {
            if(last_type!=p.type && tmp_x.size()>0)
            {
                last_type=p.type;
                if(last_type==2)
                {
                    keywords["marker"]=".";
                    keywords["linestyle"]="--";
                    keywords["c"]="gray";
                }
                else if(last_type==1)
                {
                    keywords["marker"]=".";
                    keywords["linestyle"]="-";
                    keywords["c"]="k";
                }
                else{
                    keywords["marker"]=".";
                    keywords["linestyle"]="-";
                    keywords["c"]="y";
                }
                plt::plot(tmp_x,tmp_y,keywords);
                tmp_x.clear();
                tmp_y.clear();
            }
            else{
                last_type=p.type;
                tmp_x.push_back(p.lon);
                tmp_y.push_back(p.lat);                  
            }

          
        }
        if(tmp_x.size()>0)
        {
            if(last_type==2)
            {
                keywords["marker"]=".";
                keywords["linestyle"]="--";
                keywords["c"]="gray";
            }
            else if(last_type==1)
            {
                keywords["marker"]=".";
                keywords["linestyle"]="-";
                keywords["c"]="k";
            }
            else{
                keywords["marker"]=".";
                keywords["linestyle"]="-";
                keywords["c"]="y";
                std::cout<<"lane type"<<last_type<<std::endl;
            }
            plt::plot(tmp_x,tmp_y,keywords);
        }

        /*plot lane association infomation*/
        // std::stringstream ss;
        // ss<<"Lane"<<lane_id<<std::endl;
        // for(auto& prob:lane_marker_trackings[lane_id].road_probs)
        // {
        //     if(trace.back().candidate_roads.count(prob.first)!=0 &&prob.second>0.01 && trace.back().joint_prob[prob.first].first!=0)
        //     {
        //         ss<<prob.first.getPart1()/100%100<<prob.first.getPart2()/100%100<<","<<std::setprecision(3)<< prob.second*100<<"%"<<std::endl;

        //     }
        // }
        // size_t middle=lane_marker_trackings[lane_id].points.size()/2;
        // if(lane_marker_trackings[lane_id].points.size()>=5)
        // {
        //     plt::text(lane_marker_trackings[lane_id].points[middle].lon,lane_marker_trackings[lane_id].points[middle].lat-0.00002,ss.str());
            
        // }
        // keywords["marker"]="^";
        // keywords["linestyle"]="";
        // keywords["c"]="g";
        // plt::plot({lane_marker_trackings[lane_id].points[middle].lon},{lane_marker_trackings[lane_id].points[middle].lat},keywords);


    }

    /*plot projection points*/
    for(auto& p:projection_points)
    {
        keywords["marker"]="o";
        keywords["linestyle"]="";
        keywords["c"]="g";
        // for(auto jump:jumps)
        // {
        //     if(jump.wrong_road==p.first)
        //     {
        //         keywords["c"]="r";
        //     }
        // }
        // if(global_tranPsb.count(p.first) &&global_tranPsb[p.first][trace.back().sorted_joint_prob[0].first]==0)
        // {
        //     keywords["c"]="r";
        // }
        plt::plot({p.second.lon},{p.second.lat},keywords);
      

        
    }


    /*plot SD map*/
    for(auto& candidate_road:trace.back().candidate_roads)
    {
        keywords["marker"]=".";
        keywords["linestyle"]="-";
        // if(candidate_road.first==mm_output)//trace.back().sorted_joint_prob[0].first)
        if(mm_result_collection.count(candidate_road.first))
        {
            keywords["c"]="g";
            // if(global_tranPsb.count(candidate_road.first) &&global_tranPsb[candidate_road.first][trace.back().sorted_joint_prob[0].first]==0)
            // {
            //     keywords["c"]="r";
            // }
        }
        else{
            keywords["c"]="gray";

        }
        // for(auto jump:jumps)
        // {
        //     if(jump.wrong_road==candidate_road.first)
        //     {
        //         keywords["c"]="r";
        //     }
        // }
        
        
        std::vector<double> tmp_x,tmp_y;
        for(auto& p:candidate_road.second.node)
        {
            tmp_x.push_back(p.lon);
            tmp_y.push_back(p.lat);             
        }
        plt::plot(tmp_x,tmp_y,keywords);
        // std::stringstream ss;
        // ss<<"R"<<candidate_road.first.getPart1()/100%100<<candidate_road.first.getPart2()/100%100<<std::endl;
        // if(candidate_road.second.node.size()==0)
        // {
        //     continue;
        // }
        // for(int i=0;i<candidate_road.second.node.size()-1;i++)
        // {

        //     plt::text((candidate_road.second.node[i].lon+candidate_road.second.node[i+1].lon)/2,(candidate_road.second.node[i].lat+candidate_road.second.node[i+1].lat)/2-0.00002,ss.str());
        // }
  
    }

    /*plot trajectory (original and corrected)*/
    std::vector<double> trace_x,trace_y,corrected_trace_x,corrected_trace_y;
    for(auto& tp:trace)
    {
        trace_x.push_back(tp.lon);
        trace_y.push_back(tp.lat);
        if(tp.isConverged)
        {
            corrected_trace_x.push_back(tp.corrected_pos.lon);
            corrected_trace_y.push_back(tp.corrected_pos.lat);

        }
    }
    keywords["marker"]="*";
    keywords["linestyle"]="-";
    keywords["c"]="g";
    plt::plot(corrected_trace_x,corrected_trace_y,keywords);
    keywords["c"]="b";
    plt::plot(trace_x,trace_y,keywords);

    /*plot probability info*/
    std::stringstream ss_psb;
    ss_psb<<"id, disPsb, anglePsb, lanePsb, hpPsb,typePsb, jointPsb; confidence: "<<trace.back().confidence<<std::endl;

    for(auto& sorted_prob:trace.back().sorted_joint_prob)
    {

            ss_psb<<sorted_prob.first.getPart1()/100%100<<sorted_prob.first.getPart2()/100%100<<","<<std::setiosflags(std::ios::fixed)<<std::setprecision(4)<<trace.back().distanceEmisPsb[sorted_prob.first]<<","<<trace.back().angleEmisPsb[sorted_prob.first]<<", "<<trace.back().laneEmisPsb[sorted_prob.first]<<","<<trace.back().hpEmisPsb[sorted_prob.first]<<","<<trace.back().OBDSC_EmisPsb[sorted_prob.first]<<","<<sorted_prob.second.first<<std::endl;

    }


    plt::text(trace.back().lon-range-window_offset_x,trace.back().lat-range-window_offset_y,ss_psb.str());

    /*plot transition probability info*/
    // std::stringstream ss_trans_psb;
    // ss_trans_psb<<"SD candidate transition:"<<std::endl;
    // for(auto& prob:trace.back().joint_prob)
    // {
    //     if(prob.second.first>1e-4)
    //     {
    //         for(auto& end:trace.back().candidate_roads[prob.first].end_node)
    //         {
    //             if((trace.back().candidate_roads[trace.back().sorted_joint_prob[0].first].tranPsb[prob.first]==1 || trace.back().candidate_roads[prob.first].tranPsb[trace.back().sorted_joint_prob[0].first]==1) && trace.back().sorted_joint_prob[0].first!=prob.first )
    //             {
    //                 continue;
    //             }
    //             ss_trans_psb<<prob.first.getPart1()/100%100<<prob.first.getPart2()/100%100<<"->"<<end.getPart1()/100%100<<end.getPart2()/100%100<<std::endl;
                

    //         }
    //     }

    // }
    // plt::text(trace.back().lon-range+window_offset_x,trace.back().lat-range+window_offset_y+0.00015,ss_trans_psb.str());

    /*plot vision probability info*/
    // if(trace.back().candidate_roads.count(trace.back().fork_road_id)>0 && trace.back().candidate_roads[trace.back().fork_road_id].preview_tranPsb.size()>0)
    // {
    //     std::stringstream ss_vision_psb;
    //     ss_vision_psb<<"Holistic path transition probability:"<<std::endl;
    //     double sum=0;
    //     for(auto& preview_tranPsb:trace.back().candidate_roads[trace.back().fork_road_id].preview_tranPsb)
    //     {
    //         sum+=preview_tranPsb.second;
    //     }
    //     for(auto& preview_tranPsb:trace.back().candidate_roads[trace.back().fork_road_id].preview_tranPsb)
    //     {
    //         ss_vision_psb<<trace.back().fork_road_id.getPart1()/100%100<<trace.back().fork_road_id.getPart2()/100%100<<"->"<<std::setiosflags(std::ios::fixed)<<std::setprecision(4)<<preview_tranPsb.first.getPart1()/100%100<<preview_tranPsb.first.getPart2()/100%100<<": "<<preview_tranPsb.second/sum<<"(raw value: "<<preview_tranPsb.second<<")"<<std::endl;
    //     }
    //     plt::text(trace.back().lon-range+window_offset_x,trace.back().lat-range+window_offset_y+0.00025,ss_vision_psb.str());

    // }

    /*plot holistic path*/   
    std::vector<double> hp_x,hp_y;
    for(auto& hp:trace.back().holistic_path)
    {
        hp_x.push_back(hp.lon);
        hp_y.push_back(hp.lat);
    }
    keywords["marker"]="+";
    keywords["linestyle"]="--";
    keywords["c"]="royalblue";
    plt::plot(hp_x,hp_y,keywords);



    /*plot status*/
    // if(trace.back().hpEmisPsb.size()>0)//(trace.back().candidate_roads.count(trace.back().fork_road_id)!=0 && trace.back().candidate_roads[trace.back().fork_road_id].preview_tranPsb.size()>0)
    // {
    //     plt::text(trace.back().lon+range+window_offset_x-0.00065,trace.back().lat+range+window_offset_y-0.00003,"Holistic path transition prediction: ON");
    // }
    // else{
    //     plt::text(trace.back().lon+range+window_offset_x-0.00065,trace.back().lat+range+window_offset_y-0.00003,"Holistic path transition prediction: OFF");
    // }
    // if(trace.back().isLaneAvail)
    // {
    //     plt::text(trace.back().lon+range+window_offset_x-0.00065,trace.back().lat+range+window_offset_y-0.00006,"Lane marker emission status: ON");
    // }
    // else{
    //     plt::text(trace.back().lon+range+window_offset_x-0.00065,trace.back().lat+range+window_offset_y-0.00006,"Lane marker emission status: OFF");
    // }
    // if(trace.back().isConverged&&trace.back().lane_marker_pointcloud->size()>=ICP_MIN_POINTS)
    // {
    //     plt::text(trace.back().lon+range+window_offset_x-0.00065,trace.back().lat+range+window_offset_y-0.00009,"lane marker ICP: ON");
    // }
    // else if(trace.back().icp_residual>0&&trace.back().lane_marker_pointcloud->size()>=ICP_MIN_POINTS)
    // {
    //     plt::text(trace.back().lon+range+window_offset_x-0.00065,trace.back().lat+range+window_offset_y-0.00009,"lane marker ICP: OFF(not converged)");

    // }
    // else{
    //     plt::text(trace.back().lon+range+window_offset_x-0.00065,trace.back().lat+range+window_offset_y-0.00009,"lane marker ICP: OFF(not enough points)");
    // }
    // if(trace.back().icp_residual>0)
    // {
    //     plt::text(trace.back().lon+range+window_offset_x-0.00065,trace.back().lat+range+window_offset_y-0.00012,"ICP residual: "+std::to_string(trace.back().icp_residual)+", iteration: "+std::to_string(trace.back().icp_iter));

    // }
    // if(trace.back().icp_residual>0)
    // {
    //     plt::text(trace.back().lon+range+window_offset_x-0.00065,trace.back().lat+range+window_offset_y-0.00015,"ICP match ratio: "+std::to_string((double)trace.back().icp_match_num/trace.back().lane_marker_pointcloud->size())+", match num: "+std::to_string(trace.back().icp_match_num));

    // }

    // plt::text(trace.back().lon+range+window_offset_x-0.00065,trace.back().lat+range+window_offset_y-0.00018,"Num of jumps: "+std::to_string(jumps.size())+", Num of reset: "+std::to_string(reset_num));




    /*plot settings*/
    plt::xlim(trace.back().lon-range-window_offset_x, trace.back().lon+range-window_offset_x);
    plt::ylim(trace.back().lat-range-window_offset_y, trace.back().lat+range-window_offset_y);
    plt::xlabel("lon/degree");
    plt::ylabel("lat/degree");
    std::string time_str=std::to_string(trace.back().time-10000);
    plt::title("time: "+time_str);
    // std::cout<<"time: "+time_str<<std::endl;

    // plt::save("/research/tongji_project_ws/build/hdf_read/lane_detection/"+time_str+".png");
    plt::save(folderPath+"/"+time_str+".png");

    if(ENABLE_PLOT_SHOW)
    {
        plt::show();
    }
    plt::close();

}