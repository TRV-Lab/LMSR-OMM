#include "hmm_madmap.h"


void HMM_MADMAP::errorFilterInit(std::unique_ptr< KalmanFilter>& kf,double init_x)
{
    Eigen::MatrixXd P(1, 1);
    P.setIdentity();

    Eigen::VectorXd x(1);
    x << init_x;
    kf->init(x, P);
    kf->isKalmanInit=true;
}
/*TODO: use python to optimize kalman filter params*/
/*Update kalman filter*/
void HMM_MADMAP::errorFilterUpdate(std::unique_ptr< KalmanFilter>& kf,double projection_error)
{

    //use kalman filter to smooth the volume
    Eigen::MatrixXd H(1, 1);
    H << 1;

    Eigen::VectorXd z(1);
    z << projection_error;
    kf->update(H,z);

}
/*Batch update kalman filter and clear history*/
void HMM_MADMAP::filterBatchUpdate(std::unique_ptr< KalmanFilter>& kf,std::deque<TraceNode> &trace)
{
    // for(auto& trace_node:trace)
    for(int i=0;i<trace.size();i++)
    {
        /*TODO: better kalman init policy*/
        if(i!=0 && trace[i].mm_result!=mad::MapObjectId()  && trace[i].mm_result!=trace[i-1].mm_result &&trace[i-1].candidate_roads[trace[i-1].mm_result].end_node.size()>1)// match to new road 
            kf->isKalmanInit=false;
            std::cout<<"Kalman filter init"<<std::endl;
        }
        else if(i!=0)
        {
            std::cout<<"Debug"<<trace[i].mm_result.getPart1()<<"-"<<trace[i].mm_result.getPart2()<<", "<<trace[i-1].mm_result.getPart1()<<"-"<<trace[i-1].mm_result.getPart2()<<", "<<trace[i-1].candidate_roads[trace[i-1].mm_result].end_node.size()<<std::endl;
        }

        if(trace[i].isFiltered==false && trace[i].num_hypothesis==1)
        {
            double projection_error=TraceNode::RealDistance(trace[i].lat,trace[i].lon,trace[i].trace_projection[trace[i].mm_result].lat,trace[i].trace_projection[trace[i].mm_result].lon);
            // std::cout<<"projection error"<<std::setprecision(8)<<trace[i].trace_projection[trace[i].mm_result].lat<<", "<<trace[i].trace_projection[trace[i].mm_result].lon<<", "<<trace[i].lat<<", "<<trace[i].lon<<", "<<cos(trace[i].heading)<<std::endl;
            projection_error=(trace[i].trace_projection[trace[i].mm_result].lon-trace[i].lon)/cos(trace[i].heading)>0? projection_error:-projection_error;//pos in road left + : right -

            if (!kf->isKalmanInit)
            {
                HMM_MADMAP::errorFilterInit(kf,projection_error);
            }
            // std::cout<<(trace[i].trace_projection[trace[i].mm_result].lon-trace[i].lon)<<", "<<cos(trace[i].heading)<<", "<<cos(trace[i].lat*M_PI/180)<<std::endl;
            HMM_MADMAP::errorFilterUpdate(kf,projection_error);
            trace[i].isFiltered=true;
            kf->last_filtered_heading=trace[i].heading;
        }
    }

}
/*Predict projection distance by kalman filter*/
double HMM_MADMAP::errorFilterPredict(std::unique_ptr< KalmanFilter>& kf,double heading)
{
    Eigen::MatrixXd A(1, 1);
    A << 1;
    Eigen::VectorXd error_predict(1);
    error_predict<< kf->predict(A);
    // volume_kalman=error_predict(0);
    std::cout<< "error estimate: " << error_predict(0) << std::endl;
    return error_predict(0);
    // return TraceNode::WGS84incre(trace_node.lat,trace_node.lon,trace_node.heading,projection_error);//LatLon(error_predict(0)*sin(heading),-error_predict(0)*cos(heading));
    
}
