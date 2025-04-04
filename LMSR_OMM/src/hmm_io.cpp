#include "hmm_madmap.h"

void HMM_MADMAP::roadsUpdate(std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId>& candidate_roads)
{
    for(auto& road:candidate_roads)
    {
        // roads.insert(road);
        // if(roads.count(road.first)==0 ||road.second.interpolated==true)
        // {

            // assert(roads[road.first].interpolated==false);
            roads[road.first]=road.second;

        // }
    }
    
}
void HMM_MADMAP::roadOut()
{
    
    for(auto& road:roads)
    {
        if(road.second.start_node.size()>0 && road.second.end_node.size()>0)
        {
            for(int j=0;j<road.second.start_node.size();j++)
            {
                for(int k=0;k<road.second.end_node.size();k++)
                {
                    
                    road_out<<road.second.id.getPart1()+road.second.id.getPart2() <<' '<<road.second.start_node[j].getPart1()+road.second.start_node[j].getPart2()<<' '<<road.second.end_node[k].getPart1()+road.second.end_node[k].getPart2()<<" test "<<road.second.road_class<<' '<<road.second.node.size()<<' ';
                    for(int l=0;l<road.second.node.size();l++)
                    {
                        road_out<<road.second.node[l].lat<<' '<<road.second.node[l].lon<<' ';
                    }
                    road_out<<std::endl;
                }
            }
        }
        else{
            road_out<<road.second.id.getPart1()+road.second.id.getPart2() <<' '<<-1<<' '<<-1<<" test "<<road.second.road_class<<' '<<road.second.node.size()<<' ';
            for(int l=0;l<road.second.node.size();l++)
            {
                road_out<<road.second.node[l].lat<<' '<<road.second.node[l].lon<<' ';
            }
            road_out<<std::endl;
        }
        
    }


}

// void HMM_MADMAP::loadHDF5File(H5File sensorfusion_file,Cxx_psd& cxx_psd,Cxx_hp& cxx_hp)
// {
//     H5_psd h5_psd;
//     h5_psd.getDataSet(sensorfusion_file);
//     h5_psd.parseTo(cxx_psd);

//     H5_hp h5_hp;
//     h5_hp.getDataSet(sensorfusion_file,false);
//     h5_hp.parseTo(cxx_hp);
// }
// Cxx_oxts HMM_MADMAP::loadOXTSFile(H5File oxts_file)
// {
//     H5_oxts h5_oxts;
//     Cxx_oxts oxts;
//     h5_oxts.getDataSet(oxts_file);
//     h5_oxts.parseTo(oxts);
//     return oxts;
// }
// void HMM_MADMAP::traceOut(std::deque<TraceNode> &trace)
// {
//         // std::cout<<"lat,lon"<< std::setprecision(8)<<trace.back().lat<<' '<<trace.back().lon<<std::endl;
//     // trace_out<<count<<'\n';
//     // for(int k=0;k<trace.size();k++)
//     // {
//         trace_out<<trace.back().time<<' '<<trace.back().lat<<' '<<trace.back().lon<<'\n';
//     // }
// }

void HMM_MADMAP::fileOut(TraceNode& trace_node)
{
    //Test
    auto& Pback=trace_node.sorted_joint_prob;
    trace_out<<trace_node.time<<' '<<trace_node.lat<<' '<<trace_node.lon<<'\n';
    trace_out_corrected<<trace_node.time<<' '<<trace_node.corrected_pos.lat<<' '<<trace_node.corrected_pos.lon<<'\n';
    // mm_out_hp<<mm_output.getPart1()+ mm_output.getPart2()<<'\n';
    double err_lat=trace_node.trace_projection[mm_output].lat- trace_node.corrected_pos.lat;
    double err_lon=trace_node.trace_projection[mm_output].lon-trace_node.corrected_pos.lon;
    double s_correct=TraceNode::RealDistance(trace_node.trace_projection[mm_output].lat, trace_node.trace_projection[mm_output].lon, trace_node.corrected_pos.lat, trace_node.corrected_pos.lon);
    // std::cout<<"s_correct"<<s_correct<<std::endl;
    err_sum_correct+=s_correct;
    double s_origin=TraceNode::RealDistance(trace_node.trace_projection[mm_output].lat, trace_node.trace_projection[mm_output].lon, trace_node.lat, trace_node.lon);
    // std::cout<<"s_origin"<<s_origin<<std::endl;
    // std::cout<<"mm_output"<<mm_output.getPart1()<<"-"<<mm_output.getPart2()<<std::endl;
    err_sum_origin+=s_origin;   
    count++;


    // assert(s_origin<100);
    
    //TODO: cout whether corrected
    // projection_out<<trace_node.time<<' '<<err_lat<<' '<<err_lon<<' '<<trace_node.heading<<' '<<err_sum_correct<<'\n';
    
}
