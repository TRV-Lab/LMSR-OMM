#include "sensorfusion.h"
using Eigen::Vector3d;
using Eigen::AngleAxisd;
using Eigen::Quaterniond;
using std::cout;
using std::endl;


// void Output::fromSatellite(Cxx_satellite satellite)
// {
//     length=satellite.getLength();
//     lon.resize(length);
//     lat.resize(length);
//     heading.resize(length);
//     timestamps.resize(length);
//     for(int i=0;i<length;i++){
//         timestamps[i]=double(satellite.getZeaderTimestampNs()[i])/1e9;
//         lon[i]=satellite.getLongPosN()[i]/1e9;
//         lat[i]=satellite.getLatPosN()[i]/1e9;
//         heading[i]=satellite.getHeading()[i];
//     }

// }

void SensorFusion::fout_result_satellite(std::string fout_path,GT_TYPE type,std::vector<uint8_t>&  valid_output)
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
    save_path=fout_path+gt_string+time_string+"_satellite.txt";

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
        AngleAxisd yaw(AngleAxisd(satellite.getHeading()[i]*M_PI/180,Vector3d::UnitZ()));
        Quaterniond q;
        q=yaw*pitch*roll;

        fout<<(timestamps[i]-timestamps[0])<<' '<<(double(satellite.getLongPosN()[i])/1e9-lonlat_init[0])*LONLAT2METER<<' '<<(double(satellite.getLatPosN()[i])/1e9-lonlat_init[1])*LONLAT2METER<<' '<<0<<' '<<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<std::endl;
    }
    cout<<"count satellite invalid="<<count<<endl;
    fout.close();

}