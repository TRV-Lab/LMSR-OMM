#include "sensorfusion.h"
using Eigen::Vector3d;
using Eigen::AngleAxisd;
using Eigen::Quaterniond;
using std::cout;
using std::endl;


void SensorFusion::fout_result_mre(std::string fout_path,GT_TYPE type,std::vector<uint8_t>&  valid_output)
{
    std::string save_path;
    std::string gt_string;
    if(type==GT_TYPE::OXTS)
    {
        gt_string="oxts/";
    }
    else if(type==GT_TYPE::Nibbler)    
    {
        gt_string="nibbler/";
    }
    else{
        printf("DATA_TYPE INVALID");
    }
    save_path=fout_path+gt_string+time_string+"_mre.txt";
    std::ofstream fout(save_path);
    fout.setf(std::ios::fixed);
    fout.precision(3);
    int count=0;
    for(int i=0;i<length;i++)
    {
        if(!valid_output[i])
        {
            count++;
            continue;
        }
        AngleAxisd roll(AngleAxisd(0,Vector3d::UnitX()));
        AngleAxisd pitch(AngleAxisd(0,Vector3d::UnitY()));
        AngleAxisd yaw(AngleAxisd(localization.getBestHypothesis().heading[i]*M_PI/180,Vector3d::UnitZ()));
        Quaterniond q;
        q=yaw*pitch*roll;

        fout<<(timestamps[i]-timestamps[0])<<' '<<(localization.getBestHypothesis().lon[i]-lonlat_init[0])*LONLAT2METER<<' '<<(localization.getBestHypothesis().lat[i]-lonlat_init[1])*LONLAT2METER<<' '<<0<<' '<<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<std::endl;
    }
    cout<<"count MRE invalid="<<count<<endl;
    fout.close();

}