#include "parse/satellite.h"
#include "parse/positioning_satellite_debug.h"
#include "parse/motion_data.h"
#include "parse/lane_markers.h"
#include "parse/vision_traffic_signs.h"
#include "parse/ego_vehicle_motion_trail_state.h"
#include "parse/visual_odometry_verification.h"
#include "parse/localization_data.h"

using std::cout;
using std::endl;
using namespace H5;


YAML::Node config;
void print_type(const DataSet dataset)
{
    H5T_class_t type_class = dataset.getTypeClass();        
    if (type_class == H5T_INTEGER) 
    {
        cout << "Dataset has INTEGER type" << endl;
        IntType intype = dataset.getIntType();
        size_t size = intype.getSize();
        cout << "Data size is " << size << endl;
    }
    else if (type_class == H5T_FLOAT) 
    {
        cout << "Dataset has FLOAT type" << endl;
        FloatType floatype = dataset.getFloatType();
        size_t size = floatype.getSize();
        cout << "Data size is " << size << endl;
    }
    else{
        cout << "Data type is " << type_class << endl;
    }
}

void setStackSize(int bytes)
{
    const rlim_t kStackSize = bytes;   
    struct rlimit rl;
    int result;

    result = getrlimit(RLIMIT_STACK, &rl);
    if (result == 0)
    {
        if (rl.rlim_cur < kStackSize)
        {
            rl.rlim_cur = kStackSize;
            result = setrlimit(RLIMIT_STACK, &rl);
            if (result != 0)
            {
                fprintf(stderr, "setrlimit returned result = %d\n", result);
            }
        }
    }
}
void getInput(H5File  sensorfusion_file,H5File  vision_file)
{
    // H5_satellite h5_satellite;
    // Cxx_satellite cxx_satellite;
    // h5_satellite.getDataSet(sensorfusion_file);
    // h5_satellite.parseTo(cxx_satellite);
    // cxx_satellite.print();

    // H5_psd h5_psd;
    // Cxx_psd cxx_psd;
    // h5_psd.getDataSet(sensorfusion_file);
    // h5_psd.parseTo(cxx_psd);
    // cxx_psd.print();
    
    // H5_motion h5_motion;
    // Cxx_motion cxx_motion;
    // h5_motion.getDataSet(sensorfusion_file);
    // h5_motion.parseTo(cxx_motion);
    // cxx_motion.print();
    
    // H5_lane_markers h5_lane_markers;
    // Cxx_lane_markers cxx_lane_markers;
    // h5_lane_markers.getDataSet(sensorfusion_file);
    // h5_lane_markers.parseTo(cxx_lane_markers);    
    // cxx_lane_markers.print();

    // H5_vts h5_vts;
    // Cxx_vts cxx_vts;
    // h5_vts.getDataSet(sensorfusion_file);
    // h5_vts.parseTo(cxx_vts);    
    // cxx_vts.print();

    // H5_motion_trail h5_motion_trail;
    // Cxx_motion_trail cxx_motion_trail;
    // h5_motion_trail.getDataSet(sensorfusion_file);
    // h5_motion_trail.parseTo(cxx_motion_trail);    
    // cxx_motion_trail.print();

    
    // H5_vov h5_vov;
    // Cxx_vov cxx_vov;
    // h5_vov.getDataSet(vision_file);
    // h5_vov.parseTo(cxx_vov);    
    // cxx_vov.print();

    H5_localization h5_localization;
    Cxx_localization cxx_localization;
    h5_localization.getDataSet(sensorfusion_file);
    h5_localization.parseTo(cxx_localization,true);    
    cxx_localization.print();
}

int main(void)
{   
    setStackSize(100 * 1024 * 1024);// stack size = 100 MB
    config=YAML::LoadFile("/research/tongji_project_ws/src/hdf_read/config/config.yaml");

    H5File  sensorfusion_file(config["sensorfusion_file"].as<std::string>(), H5F_ACC_RDONLY);
    H5File  vision_file(config["vision_file"].as<std::string>(), H5F_ACC_RDONLY);
    
    getInput(sensorfusion_file,vision_file);
    // mapMatch(config["SDmap_path"].as<std::string>());
    return 0; 
}
