#include "parse/positioning_satellite_debug.h"
using std::cout;
using std::endl;
using namespace H5;

void H5_psd::getDataSet(H5File& file)
{
    Group psd_group=file.openGroup("zen_qm_sensorfusion_a").openGroup("zen_qm_sensorfusion_a_positioning_satellite_debug_data");
    Group psd_data=psd_group.openGroup("data");
    zeader_timestamp_ns=psd_group.openGroup("zeader").openDataSet("timestamp_ns");
    lon=psd_data.openGroup("geodetic_pose_wgs84").openGroup("lon").openGroup("nanodegrees").openDataSet("value");
    lat=psd_data.openGroup("geodetic_pose_wgs84").openGroup("lat").openGroup("nanodegrees").openDataSet("value");
    heading=psd_data.openGroup("geodetic_pose_wgs84").openGroup("heading").openGroup("degrees").openDataSet("value");
    // lon=psd_data.openGroup("geodetic_pose_wgs84").openGroup("longitude").openGroup("degrees").openDataSet("value");
    // lat=psd_data.openGroup("geodetic_pose_wgs84").openGroup("latitude").openGroup("degrees").openDataSet("value");
    // heading=psd_data.openGroup("geodetic_pose_wgs84").openGroup("heading").openGroup("degrees").openDataSet("value");

    lon_variance=psd_data.openGroup("ego_frame_variance").openGroup("longitudinal").openGroup("meters2").openDataSet("value");
    lat_variance=psd_data.openGroup("ego_frame_variance").openGroup("lateral").openGroup("meters2").openDataSet("value");
    yaw_variance=psd_data.openGroup("ego_frame_variance").openGroup("yaw").openGroup("radians2").openDataSet("value");
    filter_error_code=psd_data.openDataSet("filter_error_code");
    state=psd_data.openDataSet("state");

}

void H5_psd::parseTo(Cxx_psd &cxx_psd) const
{
    /*Parse zeader timestamp*/
    parseZeaderTimestampNsTo(cxx_psd);
    /*Debug
    for(auto timestamp:cxx_psd.getZeaderTimestampNs())
    {
        cout<<timestamp<<" ";
    }
    cout<<endl;*/


    /*Parse lon*/
    parseLonTo(cxx_psd);
    /*Debug
    for(auto lon:cxx_psd.getLon())
    {
        cout<<lon<<" ";
    }
    cout<<endl;*/


    /*Parse lat*/
    parseLatTo(cxx_psd);
    /*Debug
    for(auto lat:cxx_psd.getLat())
    {
        cout<<lat<<" ";
    }
    cout<<endl;*/

    /*Parse heading*/
    parseHeadingTo(cxx_psd);
    /*Debug
    for(auto heading:cxx_psd.getHeading())
    {
        cout<<heading<<" ";
    }
    cout<<endl;*/

    /*Parse lon_variance*/
    parseLonVarianceTo(cxx_psd);
    /*Debug
    for(auto lon_variance:cxx_psd.getLonVariance())
    {
        cout<<lon_variance<<" ";
    }
    cout<<endl;*/

    /*Parse lat_variance*/
    parseLatVarianceTo(cxx_psd);
    /*Debug
    for(auto lat_variance:cxx_psd.getLatVariance())
    {
        cout<<lat_variance<<" ";
    }
    cout<<endl;*/

    /*Parse yaw_variance*/
    parseYawVarianceTo(cxx_psd);
    /*Debug
    for(auto yaw_variance:cxx_psd.getYawVariance())
    {
        cout<<yaw_variance<<" ";
    }
    cout<<endl;*/

    /*Parse filter_error_code*/
    parseFilterErrorCodeTo(cxx_psd);
    /*Debug
    for(auto filter_error_code:cxx_psd.getFilterErrorCode())
    {
        cout<<filter_error_code<<" ";
    }
    cout<<endl;*/

    /*Parse state*/
    parseStateTo(cxx_psd);
    /*Debug
    for(auto state:cxx_psd.getState())
    {
        cout<<state<<" ";
    }
    cout<<endl;*/
    cxx_psd.checkValid();

}

void H5_psd::parseZeaderTimestampNsTo(Cxx_psd &cxx_psd) const
{
    DataSpace dataspace_zeader_timestamp_ns = zeader_timestamp_ns.getSpace();
    const int rank_zeader_timestamp_ns = dataspace_zeader_timestamp_ns.getSimpleExtentNdims();
    assert(rank_zeader_timestamp_ns==1);
    hsize_t dims_zeader_timestamp_ns[rank_zeader_timestamp_ns];
    dataspace_zeader_timestamp_ns.getSimpleExtentDims(dims_zeader_timestamp_ns, NULL);
    const int length=dims_zeader_timestamp_ns[0];
    cxx_psd.setLength(length);
    uint64_t array_zeader_timestamp_ns[length];
    zeader_timestamp_ns.read(array_zeader_timestamp_ns, PredType::NATIVE_UINT64,H5::DataSpace::ALL, dataspace_zeader_timestamp_ns);
    cxx_psd.getZeaderTimestampNs().reserve(length);
    cxx_psd.getZeaderTimestampNs().assign(&array_zeader_timestamp_ns[0],&array_zeader_timestamp_ns[length]);

}
void H5_psd::parseLonTo(Cxx_psd &cxx_psd) const
{
    // DataSpace dataspace_lon = lon.getSpace();
    // const int rank_lon = dataspace_lon.getSimpleExtentNdims();
    // assert(rank_lon==1);
    // hsize_t dims_lon[rank_lon];
    // dataspace_lon.getSimpleExtentDims(dims_lon, NULL);
    // const int length=cxx_psd.getLength();
    // assert(length==dims_lon[0]);
    // int64_t array_lon[length];
    // lon.read(array_lon, PredType::NATIVE_INT64,H5::DataSpace::ALL, dataspace_lon);
    // cxx_psd.getLon().reserve(length);
    // cxx_psd.getLon().assign(&array_lon[0],&array_lon[length]);

    DataSpace dataspace_lon = lon.getSpace();
    const int rank_lon = dataspace_lon.getSimpleExtentNdims();
    assert(rank_lon==1);
    hsize_t dims_lon[rank_lon];
    dataspace_lon.getSimpleExtentDims(dims_lon, NULL);
    const int length=cxx_psd.getLength();
    assert(length==dims_lon[0]);
    int64_t array_lon[length];
    lon.read(array_lon, PredType::NATIVE_INT64,H5::DataSpace::ALL, dataspace_lon);
    cxx_psd.getLon().reserve(length);
    cxx_psd.getLon().assign(&array_lon[0],&array_lon[length]);

}
void H5_psd::parseLatTo(Cxx_psd &cxx_psd) const
{
    DataSpace dataspace_lat = lat.getSpace();
    const int rank_lat = dataspace_lat.getSimpleExtentNdims();
    assert(rank_lat==1);
    hsize_t dims_lat[rank_lat];
    dataspace_lat.getSimpleExtentDims(dims_lat, NULL);
    const int length=cxx_psd.getLength();
    assert(length==dims_lat[0]);
    int64_t array_lat[length];
    lat.read(array_lat, PredType::NATIVE_INT64,H5::DataSpace::ALL, dataspace_lat);
    cxx_psd.getLat().reserve(length);
    cxx_psd.getLat().assign(&array_lat[0],&array_lat[length]);

}
void H5_psd::parseHeadingTo(Cxx_psd &cxx_psd) const
{
    DataSpace dataspace_heading = heading.getSpace();
    const int rank_heading = dataspace_heading.getSimpleExtentNdims();
    assert(rank_heading==1);
    hsize_t dims_heading[rank_heading];
    dataspace_heading.getSimpleExtentDims(dims_heading, NULL);
    const int length=cxx_psd.getLength();
    assert(length==dims_heading[0]);
    float array_heading[length];
    heading.read(array_heading, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_heading);
    cxx_psd.getHeading().reserve(length);
    cxx_psd.getHeading().assign(&array_heading[0],&array_heading[length]);

}
void H5_psd::parseLonVarianceTo(Cxx_psd &cxx_psd) const
{
    DataSpace dataspace_lon_variance = lon_variance.getSpace();
    const int rank_lon_variance = dataspace_lon_variance.getSimpleExtentNdims();
    assert(rank_lon_variance==1);
    hsize_t dims_lon_variance[rank_lon_variance];
    dataspace_lon_variance.getSimpleExtentDims(dims_lon_variance, NULL);
    const int length=cxx_psd.getLength();
    assert(length==dims_lon_variance[0]);
    float array_lon_variance[length];
    lon_variance.read(array_lon_variance, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_lon_variance);
    cxx_psd.getLonVariance().reserve(length);
    cxx_psd.getLonVariance().assign(&array_lon_variance[0],&array_lon_variance[length]);

}
void H5_psd::parseLatVarianceTo(Cxx_psd &cxx_psd) const
{
    DataSpace dataspace_lat_variance = lat_variance.getSpace();
    const int rank_lat_variance = dataspace_lat_variance.getSimpleExtentNdims();
    assert(rank_lat_variance==1);
    hsize_t dims_lat_variance[rank_lat_variance];
    dataspace_lat_variance.getSimpleExtentDims(dims_lat_variance, NULL);
    const int length=cxx_psd.getLength();
    assert(length==dims_lat_variance[0]);
    float array_lat_variance[length];
    lat_variance.read(array_lat_variance, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_lat_variance);
    cxx_psd.getLatVariance().reserve(length);
    cxx_psd.getLatVariance().assign(&array_lat_variance[0],&array_lat_variance[length]);

}
void H5_psd::parseYawVarianceTo(Cxx_psd &cxx_psd) const
{
    DataSpace dataspace_yaw_variance = yaw_variance.getSpace();
    const int rank_yaw_variance = dataspace_yaw_variance.getSimpleExtentNdims();
    assert(rank_yaw_variance==1);
    hsize_t dims_yaw_variance[rank_yaw_variance];
    dataspace_yaw_variance.getSimpleExtentDims(dims_yaw_variance, NULL);
    const int length=cxx_psd.getLength();
    assert(length==dims_yaw_variance[0]);
    float array_yaw_variance[length];
    yaw_variance.read(array_yaw_variance, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_yaw_variance);
    cxx_psd.getYawVariance().reserve(length);
    cxx_psd.getYawVariance().assign(&array_yaw_variance[0],&array_yaw_variance[length]);

}
void H5_psd::parseFilterErrorCodeTo(Cxx_psd &cxx_psd) const
{
    DataSpace dataspace_filter_error_code = filter_error_code.getSpace();
    const int rank_filter_error_code = dataspace_filter_error_code.getSimpleExtentNdims();
    assert(rank_filter_error_code==1);
    hsize_t dims_filter_error_code[rank_filter_error_code];
    dataspace_filter_error_code.getSimpleExtentDims(dims_filter_error_code, NULL);
    const int length=cxx_psd.getLength();
    assert(length==dims_filter_error_code[0]);
    int32_t array_filter_error_code[length];
    filter_error_code.read(array_filter_error_code, PredType::NATIVE_UINT32,H5::DataSpace::ALL, dataspace_filter_error_code);
    cxx_psd.getFilterErrorCode().reserve(length);
    cxx_psd.getFilterErrorCode().assign(&array_filter_error_code[0],&array_filter_error_code[length]);

}
void H5_psd::parseStateTo(Cxx_psd &cxx_psd) const
{
    DataSpace dataspace_state = state.getSpace();
    const int rank_state = dataspace_state.getSimpleExtentNdims();
    assert(rank_state==1);
    hsize_t dims_state[rank_state];
    dataspace_state.getSimpleExtentDims(dims_state, NULL);
    const int length=cxx_psd.getLength();
    assert(length==dims_state[0]);
    int32_t array_state[length];
    state.read(array_state, PredType::NATIVE_UINT32,H5::DataSpace::ALL, dataspace_state);
    cxx_psd.getState().reserve(length);
    cxx_psd.getState().assign(&array_state[0],&array_state[length]);

}
void Cxx_psd::checkValid()
{
    valid.reserve(length);
    valid.assign(length,1);
    for(int i=0;i<length;i++)
    {
        if(lon[i]==0 || lat[i]==0 || heading[i]==0 || filter_error_code[i]==1){
            valid[i]=0;
        }

    }
}

void Cxx_psd::print() const
{
    for(int i=0;i<length;i++)
    {
        if(i%20==0)
        {
            cout<<"zeader_time"<<"\t"<<"lon"<<"\t"<<"lat"<<"\t"<<"heading"<<"\t"<<"lon_variance"<<"\t"<<"lat_variance"<<"\t"<<"yaw_variance"<<"\t"<<"filter_error_code"<<"\t"<<"state"<<endl;
        }
        cout<<zeader_timestamp_ns[i]<<"\t"<<lon[i]<<"\t"<<lat[i]<<"\t"<<heading[i]<<"\t"<<lon_variance[i]<<"\t"<<lat_variance[i]<<"\t"<<yaw_variance[i]<<"\t"<<filter_error_code[i]<<"\t"<<state[i]<<endl;
    }
}