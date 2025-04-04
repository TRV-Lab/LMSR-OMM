#include <iostream>
#include <cstdlib>
#include "mclmcr.h"
#include "matrix.h"
#include "mclcppclass.h"
#include "libmultiLaneTracking.h"
#include "mclmcrrt.h"
 #include "yaml-cpp/yaml.h"

using namespace std;
 
int main() {
 
    // initialize lib，这里必须做初始化！
    if( !libmultiLaneTrackingInitialize())
    {
        std::cout << "Could not initialize libmultiLaneTracking!" << std::endl;
        return -1;
    }
 

    YAML::Node config=YAML::LoadFile("/research/tongji_project_ws/src/hdf_read/config/config.yaml");
    //std::string oxts_file=config["oxts_file"].as<std::string>();
    //std::string file_id=oxts_file.substr(oxts_file.size()-36,31);
    std::string psd_file=config["sensorfusion_file"].as<std::string>();
    //std::string file_id=psd_file.substr(psd_file.size()-65,31);//shanghai
    std::string file_id=psd_file.substr(psd_file.size()-67,31);//europe
    std::string mat_file="/research/tongji_project_ws/src/libdateLoaderFeature/build/SLAM_data_000"+file_id+".mat";
    double hdf_start=config["hdf_start"].as<double>();
    double hdf_end=config["hdf_end"].as<double>();
    mwArray mwA=mwArray(mat_file.c_str());
    mwArray mwB(hdf_start);
    mwArray mwC(hdf_end);
    // 调用类里面的SetData函数给类赋值
    // mwA.SetData(&a, 1);
    // mwB.SetData(&b, 1);
 
    // 调用自己的函数，求和。
    multiLaneTracking(mwA,mwB,mwC);
 
    // c = mwC.Get(1, 1);
 
    cout<<"Finish "<<endl;
    std::string cmd="mv multiLaneTracking.json "+file_id+"_"+std::to_string(hdf_start)+"_"+std::to_string(hdf_end)+".json";
    std::system(cmd.c_str());
    std::string cmd1="mv multiLaneTracking.png "+file_id+"_"+std::to_string(hdf_start)+"_"+std::to_string(hdf_end)+".png";
    std::system(cmd1.c_str());
    // 后面是一些终止调用的程序
    // terminate the lib
    libmultiLaneTrackingTerminate();
 
    // terminate MCR
    mclTerminateApplication();
 
 
    return EXIT_SUCCESS;
}
