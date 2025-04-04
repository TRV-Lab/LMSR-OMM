#include "parse/oxts.h"
#include "parse/satellite.h"
#include <algorithm>

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
void getZeaderTime(H5std_string sensorfusion_file_path,std::vector<uint64_t>& zeader_timestamp_ns)
{
    H5File  sensorfusion_file(sensorfusion_file_path, H5F_ACC_RDONLY);
    H5_satellite h5_satellite;
    h5_satellite.getDataSet(sensorfusion_file);
    DataSpace dataspace_zeader_timestamp_ns = h5_satellite.getZeaderTimestampNs().getSpace();
    hsize_t dims_zeader_timestamp_ns[1];
    dataspace_zeader_timestamp_ns.getSimpleExtentDims(dims_zeader_timestamp_ns, NULL);
    const int length=dims_zeader_timestamp_ns[0];
    uint64_t array_zeader_timestamp_ns[length];
    h5_satellite.getZeaderTimestampNs().read(array_zeader_timestamp_ns, PredType::NATIVE_UINT64,H5::DataSpace::ALL, dataspace_zeader_timestamp_ns);
    zeader_timestamp_ns.reserve(length);
    zeader_timestamp_ns.assign(&array_zeader_timestamp_ns[0],&array_zeader_timestamp_ns[length]);
}

void fileRetrieveTo(std::vector<std::string>& oxts_files,std::vector<std::string>& sensorfusion_files)
{
    std::vector<std::string> filenames;
    GetFileNames(config["nibbler_dataset_source"].as<std::string>(),filenames);
    for(auto file:filenames)
    {
        if(file.find("zebra_victor_agg2681_")<file.length() && file.find(".hdf5")<file.length())
        {
            cout<<file<<endl;
            oxts_files.push_back(file);
        }
        if(file.find("zen_qm_sensorfusion_a.hdf5")<file.length())
        {
            cout<<file<<endl;
            sensorfusion_files.push_back(file);
        }
    }

}
void fileSplit(H5_oxts& h5_oxts,std::vector<double>& oxts_timestamp,const std::pair<std::string,std::vector<uint64_t>>& file_with_timestamp)
{
    size_t idx_start=0,idx_end=0;
    std::vector<double>::iterator oxts_timestamp_begin=oxts_timestamp.begin();
    std::vector<double>::iterator oxts_timestamp_back=oxts_timestamp.end()-1;
    std::vector<double>::reverse_iterator oxts_timestamp_rbegin=oxts_timestamp.rbegin();
    std::vector<double>::reverse_iterator oxts_timestamp_rback=oxts_timestamp.rend()-1;
    while(*oxts_timestamp_begin<=0)oxts_timestamp_begin++;
    while(*oxts_timestamp_back<=0)oxts_timestamp_back--;
    while(*oxts_timestamp_rbegin<=0)oxts_timestamp_rbegin++;
    while(*oxts_timestamp_rback<=0)oxts_timestamp_rback--;

    // std::vector<double>::iterator it_start=std::find_if(oxts_timestamp_begin,oxts_timestamp_back+1,std::bind2nd(std::greater<double>(),*file_with_timestamp.second.begin()/1.0e9));
    // if(it_start!=oxts_timestamp_back+1){
    //     idx_start=std::max<int>(it_start-oxts_timestamp.begin()-1,0);
    // }
    // else{
    //     // cout<<"Sensorfusion data begin not covered"<<endl;
    //     return;
    // }
    // std::vector<double>::reverse_iterator rit_end=std::find_if(oxts_timestamp_rbegin,oxts_timestamp_rback+1,std::bind2nd(std::less<double>(),*file_with_timestamp.second.rbegin()/1.0e9));
    // if(rit_end!=oxts_timestamp_rback+1){
    //     idx_end=std::min<int>(oxts_timestamp.rend()-rit_end+1,oxts_timestamp.size());
    // }
    // else{
    //     // cout<<"Sensorfusion data end not covered"<<endl;
    //     return;
    // }
    std::vector<double>::reverse_iterator rit_start=upper_bound(oxts_timestamp_rbegin,oxts_timestamp_rback+1,*file_with_timestamp.second.begin()/1.0e9,std::greater<double>());
    if(rit_start!=oxts_timestamp_rback+1){
        idx_start=oxts_timestamp.rend()-rit_start-1;
        
    }
    else{
        // cout<<"Sensorfusion data begin not covered"<<endl;
        return;
    }
    std::vector<double>::iterator it_end=upper_bound(oxts_timestamp_begin,oxts_timestamp_back+1,*file_with_timestamp.second.rbegin()/1.0e9);
    if(it_end!=oxts_timestamp_back+1){
        idx_end=it_end-oxts_timestamp.begin();
    }
    else{
        // cout<<"Sensorfusion data end not covered"<<endl;
        return;
    }

    cout<<uint64_t(oxts_timestamp[idx_start]*1e9)<<"< "<<file_with_timestamp.second[0]<<endl;
    cout<<uint64_t(oxts_timestamp[idx_end]*1e9)<<"> "<<file_with_timestamp.second.back()<<endl;
    h5_oxts.split(file_with_timestamp.first,config["oxts_dataset"].as<std::string>(),idx_start,idx_end);
}
int main(void)
{   

    setStackSize(100 * 1024 * 1024);// stack size = 100 MB
    config=YAML::LoadFile("/research/tongji_project_ws/src/hdf_read/config/config.yaml");
    std::vector<std::string>  oxts_files, sensorfusion_files;
    fileRetrieveTo( oxts_files, sensorfusion_files);

    std::map<std::string,std::vector<uint64_t>> sensorfusion_files_with_timestamps;
    
    for(auto sensorfusion_file_path:sensorfusion_files)
    {
        std::vector<uint64_t> zeader_timestamp_ns;
        getZeaderTime(sensorfusion_file_path,zeader_timestamp_ns);
        sensorfusion_files_with_timestamps.insert(std::pair<std::string,std::vector<uint64_t>>(sensorfusion_file_path,std::move(zeader_timestamp_ns)));
    }
    for(auto oxts_file_path:oxts_files)
    {
        H5File  oxts_file(oxts_file_path, H5F_ACC_RDONLY);
        H5_oxts h5_oxts;
        Cxx_oxts cxx_oxts;
        h5_oxts.getDataSet(oxts_file);
        h5_oxts.parseTo(cxx_oxts);
        std::vector<double>& oxts_timestamp=cxx_oxts.getTimestamp();
        for(auto& file_with_timestamp:sensorfusion_files_with_timestamps)
        {
            fileSplit(h5_oxts,oxts_timestamp,file_with_timestamp);
        }
    }

    return 0; 
}