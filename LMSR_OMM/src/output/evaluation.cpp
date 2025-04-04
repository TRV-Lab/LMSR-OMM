#include "sensorfusion.h"
using Eigen::Vector3d;
using Eigen::AngleAxisd;
using Eigen::Quaterniond;
using std::cout;
using std::endl;




void SensorFusion::calValidOutput(DATA_TYPE t1,GT_TYPE t2,std::vector<uint8_t>& valid_out)
{
    std::vector<uint8_t> *valid_data, *valid_gt;
    switch (t1)
    {
    case DATA_TYPE::PSD:
        valid_data=&psd.getValid();
        break;
    case DATA_TYPE::Satellite:
        valid_data=&satellite.getValid();
        break;
    case DATA_TYPE::MRE:
        valid_data=&localization.getBestHypothesis().valid;
        break;
    case DATA_TYPE::MRE2:
        valid_data=&localization2.getBestHypothesis().valid;
        break;
    default:
        printf("DATA_TYPE INVALID");
        break;
    } 

    if(t2==GT_TYPE::OXTS)
    {
        valid_gt=&oxts_valid;
    }
    else if(t2==GT_TYPE::Nibbler)    
    {
        valid_gt=&nibbler_valid;
    }
    else{
        printf("DATA_TYPE INVALID");
    }

    valid_out.clear();
    valid_out.reserve(length);
    valid_out.assign(length,1);
    for(int i=0;i<length;i++)
    {
        
        if(t1== DATA_TYPE::Satellite)
        {
            valid_out[i]=(*valid_data)[i] && satellite.getIsUpdate()[i] && (*valid_gt)[i];
        }
        else{
            valid_out[i]=(*valid_data)[i] && (*valid_gt)[i];
        }
    }

    
}
void SensorFusion::checkValid(std::string save_path)
{
    std::vector<uint8_t> valid_out;
    
    nibbler_valid.reserve(length);
    if(gt.nibbler_flag)
    {
        nibbler_valid.assign(length,1);
        for(int i=0;i<length;i++)
        {
            Cxx_nibbler& nibbler=gt.getNibbler();
            std::vector<double>::iterator it_nibbler_next=upper_bound(nibbler.getTimestamps().begin(),nibbler.getTimestamps().end(),timestamps[i]);
            size_t idx_nibbler_next=it_nibbler_next-nibbler.getTimestamps().begin();
            if(gt.getNibbler().getValid()[idx_nibbler_next]==0 ||
                gt.getNibbler().getValid()[idx_nibbler_next-1]==0){
                nibbler_valid[i]=0;
            }
        }
    }
    else{
        nibbler_valid.assign(length,-1);
    }
    
    oxts_valid.reserve(length);
    if(gt.oxts_flag)   
    {
        oxts_valid.assign(length,1);
        for(int i=0;i<length;i++)
        {
        
            Cxx_oxts& oxts=gt.getOxts();
            std::vector<double>::iterator it_oxts_next=upper_bound(oxts.getTimestamp().begin(),oxts.getTimestamp().end(),timestamps[i]);
            size_t idx_oxts_next=it_oxts_next-oxts.getTimestamp().begin();
            if(gt.getOxts().getIsValidLatLong()[idx_oxts_next]==0 ||
                gt.getOxts().getIsValidHeading()[idx_oxts_next]==0 ||
                gt.getOxts().getIsValidLatLong()[idx_oxts_next-1]==0 ||
                gt.getOxts().getIsValidHeading()[idx_oxts_next-1]==0){
                oxts_valid[i]=0;
            }
        }
    }
    else{
        oxts_valid.assign(length,-1);
    }
    

    
    std::ofstream fout(save_path);
    fout.setf(std::ios::fixed);
    fout.precision(3);
    for(int i=0;i<length;i++)
    {
        fout<<(timestamps[i]-timestamps[0])<<' '<<(int)nibbler_valid[i]<<' '<<(int)oxts_valid[i]<<' '<<(int)psd.getValid()[i]<<' '<<(int)(satellite.getValid()[i]&&satellite.getIsUpdate()[i])<<' '<<(int)localization.getBestHypothesis().valid[i]<<endl;
    }
    fout.close();
}




double calBearing(double lng_a, double lat_a, double lng_b, double lat_b) 
{
    double y = sin(lng_b - lng_a) * cos(lat_b);
    double x = cos(lat_a) * sin(lat_b) - sin(lat_a) * cos(lat_b) * cos(lng_b - lng_a);
    double bearing = atan2(y, x);
    if (bearing < 0) {
        bearing = bearing + 2*M_PI;
    }
    return bearing;
}
void SensorFusion::fout_nibbler(std::string fout_path,std::vector<uint8_t>& valid_out)
{
    if(!gt.nibbler_flag)return;
    std::string save_path=fout_path+"nibbler/"+time_string+"_nibbler.txt";
    std::ofstream fout(save_path);
    fout.setf(std::ios::fixed);
    fout.precision(3);


    Cxx_nibbler& nibbler=gt.getNibbler();
    int count=0;
    for(int i=0;i<length;i++)
    {
        if(!valid_out[i])
        {
            count++;
            continue;
        }
        std::vector<double>::iterator it_next=upper_bound(nibbler.getTimestamps().begin(),nibbler.getTimestamps().end(),timestamps[i]);
        assert(it_next!=nibbler.getTimestamps().end() ||it_next==nibbler.getTimestamps().begin());
        size_t idx_next=it_next-nibbler.getTimestamps().begin();
        float fractile=(timestamps[i]-*(it_next-1))/(*(it_next)-*(it_next-1));
        double lon_interp=fractile*nibbler.getPoseWgs84()[idx_next].y+(1-fractile)*nibbler.getPoseWgs84()[idx_next-1].y;
        double lat_interp=fractile*nibbler.getPoseWgs84()[idx_next].x+(1-fractile)*nibbler.getPoseWgs84()[idx_next-1].x;
        // double alt_interp=fractile*nibbler.getPoseWgs84()[idx_next].x+(1-fractile)*nibbler.getPoseWgs84()[idx_next-1].x;
        double heading=calBearing(nibbler.getPoseWgs84()[idx_next-1].y,nibbler.getPoseWgs84()[idx_next-1].x,nibbler.getPoseWgs84()[idx_next].y,nibbler.getPoseWgs84()[idx_next].x);
        AngleAxisd roll(AngleAxisd(0,Vector3d::UnitX()));
        AngleAxisd pitch(AngleAxisd(0,Vector3d::UnitY()));
        AngleAxisd yaw(AngleAxisd(heading,Vector3d::UnitZ()));
        Quaterniond q;
        q=yaw*pitch*roll;

        fout<<(timestamps[i]-timestamps[0])<<' '<<(lon_interp-lonlat_init[0])*LONLAT2METER<<' '<<(lat_interp-lonlat_init[1])*LONLAT2METER<<' '<<0<<' '<<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<std::endl;
    }
    cout<<"count nibbler invalid="<<count<<endl;
    fout.close();


}

void SensorFusion::fout_oxts(std::string fout_path,std::vector<uint8_t>& valid_out)
{
    if(!gt.oxts_flag)return;
    std::string save_path=fout_path+"oxts/"+time_string+"_oxts.txt";
    std::ofstream fout(save_path);
    fout.setf(std::ios::fixed);
    fout.precision(3);

    Cxx_oxts& oxts=gt.getOxts();
    int count=0;
    for(int i=0;i<length;i++)
    {
        if(!valid_out[i])
        {
            count++;
            continue;
        }
        std::vector<double>::iterator it_next=upper_bound(oxts.getTimestamp().begin(),oxts.getTimestamp().end(),timestamps[i]);
        assert(it_next!=oxts.getTimestamp().end() && it_next!=oxts.getTimestamp().begin());
        size_t idx_next=it_next-oxts.getTimestamp().begin();
        float fractile=(timestamps[i]-*(it_next-1))/(*(it_next)-*(it_next-1));
        double lon_interp=fractile*oxts.getPosLon()[idx_next]+(1-fractile)*oxts.getPosLon()[idx_next-1];
        double lat_interp=fractile*oxts.getPosLat()[idx_next]+(1-fractile)*oxts.getPosLat()[idx_next-1];
        double alt_interp=fractile*oxts.getPosAlt()[idx_next]+(1-fractile)*oxts.getPosAlt()[idx_next-1];
        
        double heading_deg=fractile*oxts.getHeading()[idx_next]+(1-fractile)*oxts.getHeading()[idx_next-1];
        double heading=heading_deg*M_PI/180;
        if (heading < 0) {
            heading = heading + 2*M_PI;
        }
        /*Method substitude*/
        // double heading=calBearing(oxts.getPosLon()[idx_next-1],oxts.getPosLat()[idx_next-1],oxts.getPosLon()[idx_next],oxts.getPosLat()[idx_next]);
        
        AngleAxisd roll(AngleAxisd(0,Vector3d::UnitX()));
        AngleAxisd pitch(AngleAxisd(0,Vector3d::UnitY()));
        AngleAxisd yaw(AngleAxisd(heading,Vector3d::UnitZ()));
        Quaterniond q;
        q=yaw*pitch*roll;

        fout<<(timestamps[i]-timestamps[0])<<' '<<(lon_interp-lonlat_init[0])*LONLAT2METER<<' '<<(lat_interp-lonlat_init[1])*LONLAT2METER<<' '<<0<<' '<<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<std::endl;
    }
    cout<<"count oxts invalid="<<count<<endl;
    fout.close();

}

// void SensorFusion::fout_diff(Output output,GT_TYPE gt_type,std::string save_path)
// {
//     if(!gt.oxts_flag)return;
//     std::ofstream fout(save_path);
//     fout.setf(std::ios::fixed);
//     fout.precision(3);
//     if(gt_type==OXTS)
//     {
//         Cxx_oxts& oxts=gt.getOxts();
    
//         for(int i=0;i<length;i++)
//         {
//             if(!valid[i])
//             {
//                 continue;
//             }
//         }
//     }
//     fout.close();

// }