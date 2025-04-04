#include <iostream>
#include "mclmcr.h"
#include "matrix.h"
#include "mclcppclass.h"
#include "libdateLoaderFeature.h"
#include "mclmcrrt.h"
#include "yaml-cpp/yaml.h"
// #include <cstring>
 
using namespace std;
 
int main() {
 
    // initialize lib，这里必须做初始化！
    if( !libdateLoaderFeatureInitialize())
    {
        std::cout << "Could not initialize libdateLoaderFeature!" << std::endl;
        return -1;
    }
    YAML::Node config=YAML::LoadFile("/research/tongji_project_ws/src/hdf_read/config/config.yaml");
    std::string sensorfusion_file=config["sensorfusion_file"].as<std::string>();
    //std::string oxts_file=config["oxts_file"].as<std::string>();
    std::string vision_file=config["vision_file"].as<std::string>();
    std::vector<int> log_time_start=config["log_time_start"].as<std::vector<int>>();
    std::vector<int> log_time_end=config["log_time_end"].as<std::vector<int>>();
 
    //mwArray mwA_oxtsDataFolder=mwArray(oxts_file.c_str());
    mwArray mwA_sensorfusionDataFolder=mwArray(sensorfusion_file.c_str());
    mwArray mwA_visionDataFileName=mwArray(vision_file.c_str());

    mwArray mwA_datetime_enter(1,7,mxINT32_CLASS,mxREAL);
    mwA_datetime_enter.SetData(log_time_start.data(),7);

    mwArray mwA_datetime_leave(1,7,mxINT32_CLASS,mxREAL);
    mwA_datetime_leave.SetData(log_time_end.data(),7);

    //std::string file_id=oxts_file.substr(oxts_file.size()-36,31);
    std::string file_id=sensorfusion_file.substr(sensorfusion_file.size()-65,31);//shanghai
        //std::string file_id=sensorfusion_file.substr(sensorfusion_file.size()-67,31);//Europe
    mwArray mwA_dataID=mwArray(file_id.c_str());

    // // 调用类里面的SetData函数给类赋值
    // mwA.SetData(&a, 1);
    // mwB.SetData(&b, 1);
 
    // 调用自己的函数，求和。
    dateLoaderFeature(mwA_sensorfusionDataFolder,mwA_visionDataFileName,mwA_datetime_enter,mwA_datetime_leave,mwA_dataID);
 

 
    cout<<"Finish "<<endl;
 
    // 后面是一些终止调用的程序
    // terminate the lib
    libdateLoaderFeatureTerminate();
 
    // terminate MCR
    mclTerminateApplication();
 
 
    return EXIT_SUCCESS;
}
