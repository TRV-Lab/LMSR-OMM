#include "parse/satellite.h"
using std::cout;
using std::endl;
using namespace H5;


void H5_satellite::getDataSet(H5File& file)
{
    Group satellite_group=file.openGroup("zen_qm_sensorfusion_a").openGroup("zen_qm_sensorfusion_a_satellite_data");
    Group satellite_data=satellite_group.openGroup("data");
    zeader_timestamp_ns=satellite_group.openGroup("zeader").openDataSet("timestamp_ns");
    valid=satellite_data.openGroup("valid").openGroup("unitless").openDataSet("value");
    satellite_timestamp=satellite_data.openGroup("timestamp").openGroup("nanoseconds").openDataSet("value");
    longposn=satellite_data.openGroup("longposn").openGroup("nanodegrees").openDataSet("value");
    latposn=satellite_data.openGroup("latposn").openGroup("nanodegrees").openDataSet("value");
    altitude=satellite_data.openGroup("altitude").openGroup("meters").openDataSet("value");
    heading=satellite_data.openGroup("heading").openGroup("degrees").openDataSet("value");
    speed=satellite_data.openGroup("speed").openGroup("meters_per_second").openDataSet("value");
    horizontal_standard_deviation=satellite_data.openGroup("horizontal_standard_deviation").openGroup("meters").openDataSet("value");
    heading_standard_deviation=satellite_data.openGroup("heading_standard_deviation").openGroup("degrees").openDataSet("value");
    speed_standard_deviation=satellite_data.openGroup("speed_standard_deviation").openGroup("meters_per_second").openDataSet("value");

}

void H5_satellite::parseTo(Cxx_satellite &cxx_satellite) const
{
    /*Parse zeader timestamp*/
    parseZeaderTimestampNsTo(cxx_satellite);
    /*Debug
    for(auto timestamp:cxx_satellite.getZeaderTimestampNs())
    {
        cout<<timestamp<<" ";
    }
    cout<<endl;*/


    /*Parse valid*/
    parseValidTo(cxx_satellite);
    /*Debug
    for(auto valid:cxx_satellite.getValid())
    {
        cout<<valid<<" ";
    }
    cout<<endl;*/

    /*Parse satellite timestamp*/
    parseSatelliteTimestampTo(cxx_satellite);
    /*Debug
    for(auto satellite_timestamp:cxx_satellite.getSatelliteTimestamp())
    {
        cout<<satellite_timestamp<<" ";
    }
    cout<<endl;*/

    /*Parse longposn*/
    parseLongPosNTo(cxx_satellite);
    /*Debug
    for(auto longposn:cxx_satellite.getLongPosN())
    {
        cout<<longposn<<" ";
    }
    cout<<endl;*/

    /*Parse latposn*/
    parseLatPosNTo(cxx_satellite);
    /*Debug
    for(auto latposn:cxx_satellite.getLatPosN())
    {
        cout<<latposn<<" ";
    }
    cout<<endl;*/

    /*Parse altitude*/
    parseAltitudeTo(cxx_satellite);
    /*Debug
    for(auto altitude:cxx_satellite.getAltitude())
    {
        cout<<altitude<<" ";
    }
    cout<<endl;*/

    /*Parse heading*/
    parseHeadingTo(cxx_satellite);
    /*Debug
    for(auto heading:cxx_satellite.getHeading())
    {
        cout<<heading<<" ";
    }
    cout<<endl;*/

    /*Parse speed*/
    parseSpeedTo(cxx_satellite);
    /*Debug
    for(auto speed:cxx_satellite.getSpeed())
    {
        cout<<speed<<" ";
    }
    cout<<endl;*/

    /*Parse horizontal_standard_deviation*/
    parseHorizontalStdTo(cxx_satellite);
    /*Debug
    for(auto horizontal_std:cxx_satellite.getHorizontalStd())
    {
        cout<<horizontal_std<<" ";
    }
    cout<<endl;*/

    /*Parse heading_standard_deviation*/
    parseHeadingStdTo(cxx_satellite);
    /*Debug
    for(auto heading_std:cxx_satellite.getHeadingStd())
    {
        cout<<heading_std<<" ";
    }
    cout<<endl;*/

    /*Parse speed_standard_deviation*/
    parseSpeedStdTo(cxx_satellite);
    /*Debug
    for(auto speed_std:cxx_satellite.getSpeedStd())
    {
        cout<<speed_std<<" ";
    }
    cout<<endl;*/
    checkUpdate(cxx_satellite);
    

    
}

void H5_satellite::parseZeaderTimestampNsTo(Cxx_satellite &cxx_satellite) const
{
    DataSpace dataspace_zeader_timestamp_ns = zeader_timestamp_ns.getSpace();
    const int rank_zeader_timestamp_ns = dataspace_zeader_timestamp_ns.getSimpleExtentNdims();
    assert(rank_zeader_timestamp_ns==1);
    hsize_t dims_zeader_timestamp_ns[rank_zeader_timestamp_ns];
    dataspace_zeader_timestamp_ns.getSimpleExtentDims(dims_zeader_timestamp_ns, NULL);
    const int length=dims_zeader_timestamp_ns[0];
    cxx_satellite.setLength(length);
    uint64_t array_zeader_timestamp_ns[length];
    zeader_timestamp_ns.read(array_zeader_timestamp_ns, PredType::NATIVE_UINT64,H5::DataSpace::ALL, dataspace_zeader_timestamp_ns);
    cxx_satellite.getZeaderTimestampNs().reserve(length);
    cxx_satellite.getZeaderTimestampNs().assign(&array_zeader_timestamp_ns[0],&array_zeader_timestamp_ns[length]);
    
}

void H5_satellite::parseValidTo(Cxx_satellite &cxx_satellite) const
{
    DataSpace dataspace_valid = valid.getSpace();
    const int rank_valid = dataspace_valid.getSimpleExtentNdims();
    assert(rank_valid==1);
    hsize_t dims_valid[rank_valid];
    dataspace_valid.getSimpleExtentDims(dims_valid, NULL);
    const int length=cxx_satellite.getLength();
    assert(length==dims_valid[0]);
    uint8_t array_valid[length];
    valid.read(array_valid, PredType::NATIVE_UINT8,H5::DataSpace::ALL, dataspace_valid);
    cxx_satellite.getValid().reserve(length);
    cxx_satellite.getValid().assign(&array_valid[0],&array_valid[length]);

}
void H5_satellite::parseSatelliteTimestampTo(Cxx_satellite &cxx_satellite) const
{
    DataSpace dataspace_satellite_timestamp = satellite_timestamp.getSpace();
    const int rank_satellite_timestamp = dataspace_satellite_timestamp.getSimpleExtentNdims();
    assert(rank_satellite_timestamp==1);
    hsize_t dims_satellite_timestamp[rank_satellite_timestamp];
    dataspace_satellite_timestamp.getSimpleExtentDims(dims_satellite_timestamp, NULL);
    const int length=cxx_satellite.getLength();
    assert(length==dims_satellite_timestamp[0]);
    int64_t array_satellite_timestamp[length];
    satellite_timestamp.read(array_satellite_timestamp, PredType::NATIVE_INT64,H5::DataSpace::ALL, dataspace_satellite_timestamp);
    cxx_satellite.getSatelliteTimestamp().reserve(length);
    cxx_satellite.getSatelliteTimestamp().assign(&array_satellite_timestamp[0],&array_satellite_timestamp[length]);

}
void H5_satellite::parseLongPosNTo(Cxx_satellite &cxx_satellite) const
{
    DataSpace dataspace_longposn = longposn.getSpace();
    const int rank_longposn = dataspace_longposn.getSimpleExtentNdims();
    assert(rank_longposn==1);
    hsize_t dims_longposn[rank_longposn];
    dataspace_longposn.getSimpleExtentDims(dims_longposn, NULL);
    const int length=cxx_satellite.getLength();
    assert(length==dims_longposn[0]);
    int64_t array_longposn[length];
    longposn.read(array_longposn, PredType::NATIVE_INT64,H5::DataSpace::ALL, dataspace_longposn);
    cxx_satellite.getLongPosN().reserve(length);
    cxx_satellite.getLongPosN().assign(&array_longposn[0],&array_longposn[length]);

}
void H5_satellite::parseLatPosNTo(Cxx_satellite &cxx_satellite) const
{
    DataSpace dataspace_latposn = latposn.getSpace();
    const int rank_latposn = dataspace_latposn.getSimpleExtentNdims();
    assert(rank_latposn==1);
    hsize_t dims_latposn[rank_latposn];
    dataspace_latposn.getSimpleExtentDims(dims_latposn, NULL);
    const int length=cxx_satellite.getLength();
    assert(length==dims_latposn[0]);
    int64_t array_latposn[length];
    latposn.read(array_latposn, PredType::NATIVE_INT64,H5::DataSpace::ALL, dataspace_latposn);
    cxx_satellite.getLatPosN().reserve(length);
    cxx_satellite.getLatPosN().assign(&array_latposn[0],&array_latposn[length]);

}
void H5_satellite::parseAltitudeTo(Cxx_satellite &cxx_satellite) const
{
    DataSpace dataspace_altitude = altitude.getSpace();
    const int rank_altitude = dataspace_altitude.getSimpleExtentNdims();
    assert(rank_altitude==1);
    hsize_t dims_altitude[rank_altitude];
    dataspace_altitude.getSimpleExtentDims(dims_altitude, NULL);
    const int length=cxx_satellite.getLength();
    assert(length==dims_altitude[0]);
    float array_altitude[length];
    altitude.read(array_altitude, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_altitude);
    cxx_satellite.getAltitude().reserve(length);
    cxx_satellite.getAltitude().assign(&array_altitude[0],&array_altitude[length]);

}
void H5_satellite::parseHeadingTo(Cxx_satellite &cxx_satellite) const
{
    DataSpace dataspace_heading = heading.getSpace();
    const int rank_heading = dataspace_heading.getSimpleExtentNdims();
    assert(rank_heading==1);
    hsize_t dims_heading[rank_heading];
    dataspace_heading.getSimpleExtentDims(dims_heading, NULL);
    const int length=cxx_satellite.getLength();
    assert(length==dims_heading[0]);
    float array_heading[length];
    heading.read(array_heading, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_heading);
    cxx_satellite.getHeading().reserve(length);
    cxx_satellite.getHeading().assign(&array_heading[0],&array_heading[length]);

}
void H5_satellite::parseSpeedTo(Cxx_satellite &cxx_satellite) const
{
    DataSpace dataspace_speed = speed.getSpace();
    const int rank_speed = dataspace_speed.getSimpleExtentNdims();
    assert(rank_speed==1);
    hsize_t dims_speed[rank_speed];
    dataspace_speed.getSimpleExtentDims(dims_speed, NULL);
    const int length=cxx_satellite.getLength();
    assert(length==dims_speed[0]);
    float array_speed[length];
    speed.read(array_speed, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_speed);
    cxx_satellite.getSpeed().reserve(length);
    cxx_satellite.getSpeed().assign(&array_speed[0],&array_speed[length]);

}
void H5_satellite::parseHorizontalStdTo(Cxx_satellite &cxx_satellite) const
{
    DataSpace dataspace_horizontal_std = horizontal_standard_deviation.getSpace();
    const int rank_horizontal_std = dataspace_horizontal_std.getSimpleExtentNdims();
    assert(rank_horizontal_std==1);
    hsize_t dims_horizontal_std[rank_horizontal_std];
    dataspace_horizontal_std.getSimpleExtentDims(dims_horizontal_std, NULL);
    const int length=cxx_satellite.getLength();
    assert(length==dims_horizontal_std[0]);
    float array_horizontal_std[length];
    horizontal_standard_deviation.read(array_horizontal_std, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_horizontal_std);
    cxx_satellite.getHorizontalStd().reserve(length);
    cxx_satellite.getHorizontalStd().assign(&array_horizontal_std[0],&array_horizontal_std[length]);

}
void H5_satellite::parseHeadingStdTo(Cxx_satellite &cxx_satellite) const
{
    DataSpace dataspace_heading_std = heading_standard_deviation.getSpace();
    const int rank_heading_std = dataspace_heading_std.getSimpleExtentNdims();
    assert(rank_heading_std==1);
    hsize_t dims_heading_std[rank_heading_std];
    dataspace_heading_std.getSimpleExtentDims(dims_heading_std, NULL);
    const int length=cxx_satellite.getLength();
    assert(length==dims_heading_std[0]);
    float array_heading_std[length];
    heading_standard_deviation.read(array_heading_std, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_heading_std);
    cxx_satellite.getHeadingStd().reserve(length);
    cxx_satellite.getHeadingStd().assign(&array_heading_std[0],&array_heading_std[length]);

}
void H5_satellite::parseSpeedStdTo(Cxx_satellite &cxx_satellite) const
{
    DataSpace dataspace_speed_std = speed_standard_deviation.getSpace();
    const int rank_speed_std = dataspace_speed_std.getSimpleExtentNdims();
    assert(rank_speed_std==1);
    hsize_t dims_speed_std[rank_speed_std];
    dataspace_speed_std.getSimpleExtentDims(dims_speed_std, NULL);
    const int length=cxx_satellite.getLength();
    assert(length==dims_speed_std[0]);
    float array_speed_std[length];
    speed_standard_deviation.read(array_speed_std, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_speed_std);
    cxx_satellite.getSpeedStd().reserve(length);
    cxx_satellite.getSpeedStd().assign(&array_speed_std[0],&array_speed_std[length]);

}
void H5_satellite::checkUpdate(Cxx_satellite &cxx_satellite) const
{
    int length=cxx_satellite.getLength();
    auto& isUpdate=cxx_satellite.getIsUpdate();
    auto& satellite_timestamp=cxx_satellite.getSatelliteTimestamp();
    isUpdate.resize(length,0);
    int64_t last_timestamp=satellite_timestamp[0];
    for(int i=0;i<length;i++)
    {
        if(satellite_timestamp[i]>0)
        {
            if(satellite_timestamp[i]!=last_timestamp && cxx_satellite.getValid()[i]==1)
            {
                isUpdate[i]=1;
                last_timestamp=satellite_timestamp[i];
            }
        }
    }
}
void Cxx_satellite::print() const
{
    for(int i=0;i<length;i++)
    {
        if(i%20==0)
        {
            cout<<"zeader_time"<<"\t"<<"satellite_timestamp"<<"\t"<<"isUpdate"<<"\t"<<"valid"<<"\t"<<"longposn"<<"\t"<<"latposn"<<"\t"<<"altitude"<<"\t"<<"heading"<<"\t"<<"speed"<<"\t"<<"horizontal_std"<<"\t"<<"heading_std"<<"\t"<<"speed_std"<<endl;
        }
        cout<<zeader_timestamp_ns[i]<<"\t"<<satellite_timestamp[i]<<"\t"<<(int)isUpdate[i]<<"\t"<<(int)valid[i]<<"\t"<<longposn[i]<<"\t"<<latposn[i]<<"\t"<<altitude[i]<<"\t"<<heading[i]<<"\t"<<speed[i]<<"\t"<<horizontal_std[i]<<"\t"<<heading_std[i]<<"\t"<<speed_std[i]<<"\t"<<endl;
    }
}