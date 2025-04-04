#include "sensorfusion.h"

using std::cout;
using std::endl;
using namespace H5;

YAML::Node config;

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

GroundTruth::GroundTruth(std::string save_name)
{


    std::string nibbler_path="../../nibbler_dataset/"+save_name+".hdf5";
    if((access(nibbler_path.c_str(),F_OK))!=-1)   
    {   
        H5_nibbler h5_nibbler;
        H5File  nibbler_file(nibbler_path, H5F_ACC_RDONLY);
        h5_nibbler.getDataSet(nibbler_file);
        h5_nibbler.parseTo(nibbler);
        // nibbler.print();
        nibbler_flag=true;
    }   
    else{   
        cout<<nibbler_path<<" not find"<<endl;   
    } 
    

    std::string oxts_path="../../oxts_dataset/"+save_name+".hdf5";
    if((access(oxts_path.c_str(),F_OK))!=-1)   
    {   
        H5_oxts h5_oxts;
        H5File  oxts_file(oxts_path, H5F_ACC_RDONLY);
        h5_oxts.getDataSet(oxts_file);
        h5_oxts.parseTo(oxts);
        // oxts.print();        
        oxts_flag=true;
    }   
    else{   

        cout<<oxts_path<<" not find"<<endl;   
    } 

}

SensorFusion::SensorFusion(std::string sf_path)
{
    
    // std::string sf_path=file_dir+"zen_qm_sensorfusion_a.hdf5";
    
    H5File  sf_file(sf_path, H5F_ACC_RDONLY);

    
    int pos1 = sf_path.rfind("/",sf_path.length());
    time_string=sf_path.substr(0,pos1);
    int pos2 = time_string.rfind("/",time_string.length());
    time_string=time_string.substr(0,pos2);
    int pos3 = time_string.rfind("/",time_string.length());
    time_string=time_string.substr(pos3+1,time_string.length());
    std::string newpath="../../results/"+time_string;
    fout_path="../../results/"+time_string+"/data";


    // int isCreate;
    mkdir(newpath.c_str(),S_IRWXU);
    mkdir(fout_path.c_str(),S_IRWXU);

    // if(!isCreate)
    // {
    //     cout<<"create path:"<<newpath <<endl;
    // }

    gt=std::move(GroundTruth(time_string));

    H5_satellite h5_satellite;
    h5_satellite.getDataSet(sf_file);
    h5_satellite.parseTo(satellite);
    // satellite.print();

    H5_psd h5_psd;
    h5_psd.getDataSet(sf_file);
    h5_psd.parseTo(psd);
    // psd.print();

    H5_localization h5_localization;
    h5_localization.getDataSet(sf_file);
    h5_localization.parseTo(localization,true);   
    h5_localization.parseTo(localization2,false);   
    // cxx_localization.print();
    /*
    H5_motion h5_motion;
    h5_motion.getDataSet(sf_file);
    h5_motion.parseTo(motion);
    // motion.print();
    
    H5_lane_markers h5_lane_markers;
    h5_lane_markers.getDataSet(sf_file);
    h5_lane_markers.parseTo(lane_markers);    
    // lane_markers.print();

    H5_vts h5_vts;
    h5_vts.getDataSet(sf_file);
    h5_vts.parseTo(vts);    
    // vts.print();

    H5_vts h5_motion_trail;
    h5_motion_trail.getDataSet(sf_file);
    h5_motion_trail.parseTo(motion_trail);    
    // motion_trail.print();

    std::string vision_path=file_dir+"zen_qm_vision.hdf5";
    H5File  vision_file(vision_path, H5F_ACC_RDONLY);
    H5_vov h5_vov;
    h5_vov.getDataSet(vision_file);
    h5_vov.parseTo(vov);    
    // vov.print();
    
    */

    length=psd.getLength();
    
    timestamps.resize(length);
    for(int i=0;i<length;i++){
        timestamps[i]=double(psd.getZeaderTimestampNs()[i])/1e9;
    }
    
    checkValid(newpath+"/"+"valid_t-nibbler-oxts.txt");
    lonlat_init[0]=double(psd.getLon()[0])/1e9;
    lonlat_init[1]=double(psd.getLat()[0])/1e9;
    
    
    // psd_output.fromPSD(psd);
    // satellite_output.fromSatellite(satellite);

    
    
}
void fileRetrieveTo(std::vector<std::string>& sensorfusion_files)
{
    std::vector<std::string> filenames;
    GetFileNames(config["sensorfusion_dataset"].as<std::string>(),filenames);
    for(auto file:filenames)
    {
        if(file.find("zen_qm_sensorfusion_a.hdf5")<file.length())
        {
            cout<<file<<endl;
            sensorfusion_files.push_back(file);
        }
    }

}

int main(void)
{   
    setStackSize(100 * 1024 * 1024);// stack size = 100 MB
    config=YAML::LoadFile("/research/tongji_project_ws/src/hdf_read/config/config.yaml");
    // std::string file_dir("/research/fz1a/vccz/output/ZIS_CONFIG1/src-1.49.master.420/victor_agg2681/20221116T022933_20221116T031426/10489/");
    std::vector<std::string>  sensorfusion_files;
    fileRetrieveTo( sensorfusion_files);
    for(auto sensorfusion_file_path:sensorfusion_files)
    {
        SensorFusion sf(sensorfusion_file_path);
        std::vector<uint8_t> valid_out;

        // std::string newpath_data_psd=sf.fout_path+"/psd/";
        // mkdir(newpath_data_psd.c_str(),S_IRWXU);
        // std::string newpath_data_psd_oxts=newpath_data_psd+"/oxts/";
        // mkdir(newpath_data_psd_oxts.c_str(),S_IRWXU);
        // std::string newpath_data_psd_nibbler=newpath_data_psd+"/nibbler/";
        // mkdir(newpath_data_psd_nibbler.c_str(),S_IRWXU);
        // sf.calValidOutput(DATA_TYPE::PSD,GT_TYPE::OXTS,valid_out);
        // sf.fout_result_psd(sf.fout_path+"/psd/",GT_TYPE::OXTS,valid_out);
        // sf.fout_oxts(sf.fout_path+"/psd/",valid_out);
        // sf.calValidOutput(DATA_TYPE::PSD,GT_TYPE::Nibbler,valid_out);
        // sf.fout_result_psd(sf.fout_path+"/psd/",GT_TYPE::Nibbler,valid_out);
        // sf.fout_nibbler(sf.fout_path+"/psd/",valid_out);

        // std::string newpath_data_satellite=sf.fout_path+"/satellite";
        // mkdir(newpath_data_satellite.c_str(),S_IRWXU);
        // std::string newpath_data_satellite_oxts=newpath_data_satellite+"/oxts";
        // mkdir(newpath_data_satellite_oxts.c_str(),S_IRWXU);
        // std::string newpath_data_satellite_nibbler=newpath_data_satellite+"/nibbler";
        // mkdir(newpath_data_satellite_nibbler.c_str(),S_IRWXU);
        // sf.calValidOutput(DATA_TYPE::Satellite,GT_TYPE::OXTS,valid_out);
        // sf.fout_result_satellite(sf.fout_path+"/satellite/",GT_TYPE::OXTS,valid_out);
        // sf.fout_oxts(sf.fout_path+"/satellite/",valid_out);
        // sf.calValidOutput(DATA_TYPE::Satellite,GT_TYPE::Nibbler,valid_out);
        // sf.fout_result_satellite(sf.fout_path+"/satellite/",GT_TYPE::Nibbler,valid_out);
        // sf.fout_nibbler(sf.fout_path+"/satellite/",valid_out);

        // std::string newpath_data_mre=sf.fout_path+"/mre";
        // mkdir(newpath_data_mre.c_str(),S_IRWXU);
        // std::string newpath_data_mre_oxts=newpath_data_mre+"/oxts";
        // mkdir(newpath_data_mre_oxts.c_str(),S_IRWXU);
        // std::string newpath_data_mre_nibbler=newpath_data_mre+"/nibbler";
        // mkdir(newpath_data_mre_nibbler.c_str(),S_IRWXU);
        // sf.calValidOutput(DATA_TYPE::MRE,GT_TYPE::OXTS,valid_out);
        // sf.fout_result_mre(sf.fout_path+"/mre/",GT_TYPE::OXTS,valid_out);
        // sf.fout_oxts(sf.fout_path+"/mre/",valid_out);
        // sf.calValidOutput(DATA_TYPE::MRE,GT_TYPE::Nibbler,valid_out);
        // sf.fout_result_mre(sf.fout_path+"/mre/",GT_TYPE::Nibbler,valid_out);
        // sf.fout_nibbler(sf.fout_path+"/mre/",valid_out);

        std::string newpath_data_mre2=sf.fout_path+"/mre2";
        mkdir(newpath_data_mre2.c_str(),S_IRWXU);
        std::string newpath_data_mre2_oxts=newpath_data_mre2+"/oxts";
        mkdir(newpath_data_mre2_oxts.c_str(),S_IRWXU);
        std::string newpath_data_mre2_nibbler=newpath_data_mre2+"/nibbler";
        mkdir(newpath_data_mre2_nibbler.c_str(),S_IRWXU);
        sf.calValidOutput(DATA_TYPE::MRE2,GT_TYPE::OXTS,valid_out);
        sf.fout_result_mre(sf.fout_path+"/mre2/",GT_TYPE::OXTS,valid_out);
        sf.fout_oxts(sf.fout_path+"/mre2/",valid_out);
        sf.calValidOutput(DATA_TYPE::MRE2,GT_TYPE::Nibbler,valid_out);
        sf.fout_result_mre(sf.fout_path+"/mre2/",GT_TYPE::Nibbler,valid_out);
        sf.fout_nibbler(sf.fout_path+"/mre2/",valid_out);
    }
    
    return 0; 
}
