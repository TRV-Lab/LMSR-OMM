#include "parse/oxts.h"
using std::cout;
using std::endl;
using namespace H5;

void H5_oxts::getDataSet(H5File& file)
{
    Group Refnav=file.openGroup("Refnav");
    // H5File Refnav=file;
    timestamp=Refnav.openDataSet("timestamp");
    isValidLatLong=Refnav.openDataSet("isValidLatLong");
    isValidAltitude=Refnav.openDataSet("isValidAltitude");
    isValidHeading=Refnav.openDataSet("isValidHeading");
    posLon=Refnav.openDataSet("posLon");
    posLat=Refnav.openDataSet("posLat");
    posAlt=Refnav.openDataSet("posAlt");
    heading=Refnav.openDataSet("heading");
    speed=Refnav.openDataSet("speed");
    stdDevHeading=Refnav.openDataSet("stdDevHeading");
    stdDevPosDown=Refnav.openDataSet("stdDevPosDown");
    stdDevPosEast=Refnav.openDataSet("stdDevPosEast");
    stdDevPosNorth=Refnav.openDataSet("stdDevPosNorth");

}

void H5_oxts::parseTo(Cxx_oxts &cxx_oxts) const
{
    /*Parse timestamp*/
    parseTimestampTo(cxx_oxts);
    /*Debug
    for(auto timestamp:cxx_oxts.getTimestamp())
    {
        cout<<timestamp<<" ";
    }
    cout<<endl;*/

    /*Parse isValidLatLong*/
    parseIsValidLatLongTo(cxx_oxts);
    /*Debug
    for(auto isValidLatLong:cxx_oxts.getisValidLatLong())
    {
        cout<<isValidLatLong<<" ";
    }
    cout<<endl;*/

    /*Parse isValidAltitude*/
    parseIsValidAltitudeTo(cxx_oxts);
    /*Debug
    for(auto isValidAltitude:cxx_oxts.getisValidAltitude())
    {
        cout<<isValidAltitude<<" ";
    }
    cout<<endl;*/

    /*Parse isValidHeading*/
    parseIsValidHeadingTo(cxx_oxts);
    /*Debug
    for(auto isValidHeading:cxx_oxts.getisValidHeading())
    {
        cout<<isValidHeading<<" ";
    }
    cout<<endl;*/

    /*Parse posLon*/
    parsePosLonTo(cxx_oxts);
    /*Debug
    for(auto posLon:cxx_oxts.getposLon())
    {
        cout<<posLon<<" ";
    }
    cout<<endl;*/


    /*Parse posLat*/
    parsePosLatTo(cxx_oxts);
    /*Debug
    for(auto posLat:cxx_oxts.getposLat())
    {
        cout<<posLat<<" ";
    }
    cout<<endl;*/

    /*Parse posAlt*/
    parsePosAltTo(cxx_oxts);
    /*Debug
    for(auto posAlt:cxx_oxts.getposAlt())
    {
        cout<<posAlt<<" ";
    }
    cout<<endl;*/

    /*Parse heading*/
    parseHeadingTo(cxx_oxts);
    /*Debug
    for(auto heading:cxx_oxts.getheading())
    {
        cout<<heading<<" ";
    }
    cout<<endl;*/

    /*Parse speed*/
    parseSpeedTo(cxx_oxts);
    /*Debug
    for(auto speed:cxx_oxts.getspeed())
    {
        cout<<speed<<" ";
    }
    cout<<endl;*/

    /*Parse stdDevHeading*/
    parseStdDevHeadingTo(cxx_oxts);
    /*Debug
    for(auto stdDevHeading:cxx_oxts.getstdDevHeading())
    {
        cout<<stdDevHeading<<" ";
    }
    cout<<endl;*/

    /*Parse stdDevPosDown*/
    parseStdDevPosDownTo(cxx_oxts);
    /*Debug
    for(auto stdDevPosDown:cxx_oxts.getstdDevPosDown())
    {
        cout<<stdDevPosDown<<" ";
    }
    cout<<endl;*/

    /*Parse stdDevPosEast*/
    parseStdDevPosEastTo(cxx_oxts);
    /*Debug
    for(auto stdDevPosEast:cxx_oxts.getstdDevPosEast())
    {
        cout<<stdDevPosEast<<" ";
    }
    cout<<endl;*/

    /*Parse stdDevPosNorth*/
    parseStdDevPosNorthTo(cxx_oxts);
    /*Debug
    for(auto stdDevPosNorth:cxx_oxts.getstdDevPosNorth())
    {
        cout<<stdDevPosNorth<<" ";
    }
    cout<<endl;*/
}

void H5_oxts::parseTimestampTo(Cxx_oxts &cxx_oxts) const
{
    DataSpace dataspace_timestamp = timestamp.getSpace();
    const int rank_timestamp = dataspace_timestamp.getSimpleExtentNdims();
    assert(rank_timestamp==1);
    hsize_t dims_timestamp[rank_timestamp];
    dataspace_timestamp.getSimpleExtentDims(dims_timestamp, NULL);
    const int length=dims_timestamp[0];
    cxx_oxts.setLength(length);
    double array_timestamp[length];
    timestamp.read(array_timestamp, PredType::NATIVE_DOUBLE,H5::DataSpace::ALL, dataspace_timestamp);
    // transfer from UTC time to Unix time
    for(auto&time:array_timestamp){
        time+=315964800-18;
    }
    cxx_oxts.getTimestamp().reserve(length);
    cxx_oxts.getTimestamp().assign(&array_timestamp[0],&array_timestamp[length]);


    

}
void H5_oxts::parseIsValidLatLongTo(Cxx_oxts &cxx_oxts) const
{
    DataSpace dataspace_isValidLatLong = isValidLatLong.getSpace();
    const int rank_isValidLatLong = dataspace_isValidLatLong.getSimpleExtentNdims();
    assert(rank_isValidLatLong==1);
    hsize_t dims_isValidLatLong[rank_isValidLatLong];
    dataspace_isValidLatLong.getSimpleExtentDims(dims_isValidLatLong, NULL);
    const int length=cxx_oxts.getLength();
    assert(length==dims_isValidLatLong[0]);
    uint8_t array_isValidLatLong[length];
    isValidLatLong.read(array_isValidLatLong, PredType::NATIVE_UINT8,H5::DataSpace::ALL, dataspace_isValidLatLong);
    cxx_oxts.getIsValidLatLong().reserve(length);
    cxx_oxts.getIsValidLatLong().assign(&array_isValidLatLong[0],&array_isValidLatLong[length]);

}
void H5_oxts::parseIsValidAltitudeTo(Cxx_oxts &cxx_oxts) const
{
    DataSpace dataspace_isValidAltitude = isValidAltitude.getSpace();
    const int rank_isValidAltitude = dataspace_isValidAltitude.getSimpleExtentNdims();
    assert(rank_isValidAltitude==1);
    hsize_t dims_isValidAltitude[rank_isValidAltitude];
    dataspace_isValidAltitude.getSimpleExtentDims(dims_isValidAltitude, NULL);
    const int length=cxx_oxts.getLength();
    assert(length==dims_isValidAltitude[0]);
    uint8_t array_isValidAltitude[length];
    isValidAltitude.read(array_isValidAltitude, PredType::NATIVE_UINT8,H5::DataSpace::ALL, dataspace_isValidAltitude);
    cxx_oxts.getIsValidAltitude().reserve(length);
    cxx_oxts.getIsValidAltitude().assign(&array_isValidAltitude[0],&array_isValidAltitude[length]);

}
void H5_oxts::parseIsValidHeadingTo(Cxx_oxts &cxx_oxts) const
{
    DataSpace dataspace_isValidHeading = isValidHeading.getSpace();
    const int rank_isValidHeading = dataspace_isValidHeading.getSimpleExtentNdims();
    assert(rank_isValidHeading==1);
    hsize_t dims_isValidHeading[rank_isValidHeading];
    dataspace_isValidHeading.getSimpleExtentDims(dims_isValidHeading, NULL);
    const int length=cxx_oxts.getLength();
    assert(length==dims_isValidHeading[0]);
    uint8_t array_isValidHeading[length];
    isValidHeading.read(array_isValidHeading, PredType::NATIVE_UINT8,H5::DataSpace::ALL, dataspace_isValidHeading);
    cxx_oxts.getIsValidHeading().reserve(length);
    cxx_oxts.getIsValidHeading().assign(&array_isValidHeading[0],&array_isValidHeading[length]);

}
void H5_oxts::parsePosLonTo(Cxx_oxts &cxx_oxts) const
{
    DataSpace dataspace_posLon = posLon.getSpace();
    const int rank_posLon = dataspace_posLon.getSimpleExtentNdims();
    assert(rank_posLon==1);
    hsize_t dims_posLon[rank_posLon];
    dataspace_posLon.getSimpleExtentDims(dims_posLon, NULL);
    const int length=cxx_oxts.getLength();
    assert(length==dims_posLon[0]);
    double array_posLon[length];
    posLon.read(array_posLon, PredType::NATIVE_DOUBLE,H5::DataSpace::ALL, dataspace_posLon);
    cxx_oxts.getPosLon().reserve(length);
    cxx_oxts.getPosLon().assign(&array_posLon[0],&array_posLon[length]);

}
void H5_oxts::parsePosLatTo(Cxx_oxts &cxx_oxts) const
{
    DataSpace dataspace_posLat = posLat.getSpace();
    const int rank_posLat = dataspace_posLat.getSimpleExtentNdims();
    assert(rank_posLat==1);
    hsize_t dims_posLat[rank_posLat];
    dataspace_posLat.getSimpleExtentDims(dims_posLat, NULL);
    const int length=cxx_oxts.getLength();
    assert(length==dims_posLat[0]);
    double array_posLat[length];
    posLat.read(array_posLat, PredType::NATIVE_DOUBLE,H5::DataSpace::ALL, dataspace_posLat);
    cxx_oxts.getPosLat().reserve(length);
    cxx_oxts.getPosLat().assign(&array_posLat[0],&array_posLat[length]);

}
void H5_oxts::parsePosAltTo(Cxx_oxts &cxx_oxts) const
{
    DataSpace dataspace_posAlt = posAlt.getSpace();
    const int rank_posAlt = dataspace_posAlt.getSimpleExtentNdims();
    assert(rank_posAlt==1);
    hsize_t dims_posAlt[rank_posAlt];
    dataspace_posAlt.getSimpleExtentDims(dims_posAlt, NULL);
    const int length=cxx_oxts.getLength();
    assert(length==dims_posAlt[0]);
    double array_posAlt[length];
    posAlt.read(array_posAlt, PredType::NATIVE_DOUBLE,H5::DataSpace::ALL, dataspace_posAlt);
    cxx_oxts.getPosAlt().reserve(length);
    cxx_oxts.getPosAlt().assign(&array_posAlt[0],&array_posAlt[length]);

}
void H5_oxts::parseHeadingTo(Cxx_oxts &cxx_oxts) const
{
    DataSpace dataspace_heading = heading.getSpace();
    const int rank_heading = dataspace_heading.getSimpleExtentNdims();
    assert(rank_heading==1);
    hsize_t dims_heading[rank_heading];
    dataspace_heading.getSimpleExtentDims(dims_heading, NULL);
    const int length=cxx_oxts.getLength();
    assert(length==dims_heading[0]);
    double array_heading[length];
    heading.read(array_heading, PredType::NATIVE_DOUBLE,H5::DataSpace::ALL, dataspace_heading);
    cxx_oxts.getHeading().reserve(length);
    cxx_oxts.getHeading().assign(&array_heading[0],&array_heading[length]);

}
void H5_oxts::parseSpeedTo(Cxx_oxts &cxx_oxts) const
{
    DataSpace dataspace_speed = speed.getSpace();
    const int rank_speed = dataspace_speed.getSimpleExtentNdims();
    assert(rank_speed==1);
    hsize_t dims_speed[rank_speed];
    dataspace_speed.getSimpleExtentDims(dims_speed, NULL);
    const int length=cxx_oxts.getLength();
    assert(length==dims_speed[0]);
    double array_speed[length];
    speed.read(array_speed, PredType::NATIVE_DOUBLE,H5::DataSpace::ALL, dataspace_speed);
    cxx_oxts.getSpeed().reserve(length);
    cxx_oxts.getSpeed().assign(&array_speed[0],&array_speed[length]);

}
void H5_oxts::parseStdDevHeadingTo(Cxx_oxts &cxx_oxts) const
{
    DataSpace dataspace_stdDevHeading = stdDevHeading.getSpace();
    const int rank_stdDevHeading = dataspace_stdDevHeading.getSimpleExtentNdims();
    assert(rank_stdDevHeading==1);
    hsize_t dims_stdDevHeading[rank_stdDevHeading];
    dataspace_stdDevHeading.getSimpleExtentDims(dims_stdDevHeading, NULL);
    const int length=cxx_oxts.getLength();
    assert(length==dims_stdDevHeading[0]);
    float array_stdDevHeading[length];
    stdDevHeading.read(array_stdDevHeading, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_stdDevHeading);
    cxx_oxts.getStdDevHeading().reserve(length);
    cxx_oxts.getStdDevHeading().assign(&array_stdDevHeading[0],&array_stdDevHeading[length]);

}
void H5_oxts::parseStdDevPosDownTo(Cxx_oxts &cxx_oxts) const
{
    DataSpace dataspace_stdDevPosDown = stdDevPosDown.getSpace();
    const int rank_stdDevPosDown = dataspace_stdDevPosDown.getSimpleExtentNdims();
    assert(rank_stdDevPosDown==1);
    hsize_t dims_stdDevPosDown[rank_stdDevPosDown];
    dataspace_stdDevPosDown.getSimpleExtentDims(dims_stdDevPosDown, NULL);
    const int length=cxx_oxts.getLength();
    assert(length==dims_stdDevPosDown[0]);
    float array_stdDevPosDown[length];
    stdDevPosDown.read(array_stdDevPosDown, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_stdDevPosDown);
    cxx_oxts.getStdDevPosDown().reserve(length);
    cxx_oxts.getStdDevPosDown().assign(&array_stdDevPosDown[0],&array_stdDevPosDown[length]);

}
void H5_oxts::parseStdDevPosEastTo(Cxx_oxts &cxx_oxts) const
{
    DataSpace dataspace_stdDevPosEast = stdDevPosEast.getSpace();
    const int rank_stdDevPosEast = dataspace_stdDevPosEast.getSimpleExtentNdims();
    assert(rank_stdDevPosEast==1);
    hsize_t dims_stdDevPosEast[rank_stdDevPosEast];
    dataspace_stdDevPosEast.getSimpleExtentDims(dims_stdDevPosEast, NULL);
    const int length=cxx_oxts.getLength();
    assert(length==dims_stdDevPosEast[0]);
    float array_stdDevPosEast[length];
    stdDevPosEast.read(array_stdDevPosEast, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_stdDevPosEast);
    cxx_oxts.getStdDevPosEast().reserve(length);
    cxx_oxts.getStdDevPosEast().assign(&array_stdDevPosEast[0],&array_stdDevPosEast[length]);

}
void H5_oxts::parseStdDevPosNorthTo(Cxx_oxts &cxx_oxts) const
{
    DataSpace dataspace_stdDevPosNorth = stdDevPosNorth.getSpace();
    const int rank_stdDevPosNorth = dataspace_stdDevPosNorth.getSimpleExtentNdims();
    assert(rank_stdDevPosNorth==1);
    hsize_t dims_stdDevPosNorth[rank_stdDevPosNorth];
    dataspace_stdDevPosNorth.getSimpleExtentDims(dims_stdDevPosNorth, NULL);
    const int length=cxx_oxts.getLength();
    assert(length==dims_stdDevPosNorth[0]);
    float array_stdDevPosNorth[length];
    stdDevPosNorth.read(array_stdDevPosNorth, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_stdDevPosNorth);
    cxx_oxts.getStdDevPosNorth().reserve(length);
    cxx_oxts.getStdDevPosNorth().assign(&array_stdDevPosNorth[0],&array_stdDevPosNorth[length]);

}


void Cxx_oxts::print() const
{
    for(int i=0;i<length;i++)
    {
        if(i%20==0)
        {
            cout<<"timestamp"<<"\t"<<"posLon"<<"\t"<<"stdDevPosEast"<<"\t"<<"posLat"<<"\t"<<"stdDevPosNorth"<<"\t"<<"isValidLatLong"<<"\t"<<"posAlt"<<"\t"<<"stdDevPosDown"<<"\t"<<"isValidAltitude"<<"\t"<<"heading"<<"\t"<<"stdDevHeading"<<"\t"<<"isValidHeading"<<"\t"<<"speed"<<endl;
        }
        cout<<timestamp[i]<<"\t"<<posLon[i]<<"\t"<<stdDevPosEast[i]<<"\t"<<posLat[i]<<"\t"<<stdDevPosNorth[i]<<"\t"<<(int)isValidLatLong[i]<<"\t"<<posAlt[i]<<"\t"<<stdDevPosDown[i]<<"\t"<<(int)isValidAltitude[i]<<"\t"<<heading[i]<<"\t"<<stdDevHeading[i]<<"\t"<<(int)isValidHeading[i]<<"\t"<<speed[i]<<endl;

    }
}

void H5_oxts::split(std::string file_with_timestamp,std::string output_path,size_t id_start,size_t id_end)const
{
    int pos1 = file_with_timestamp.rfind("/",file_with_timestamp.length());//从后往前寻找第一个/出现的位置
    std::string id=file_with_timestamp.substr(0,pos1);
    int pos2 = id.rfind("/",id.length());
    id=id.substr(0,pos2);
    int pos3 = id.rfind("/",id.length());
    id=id.substr(pos3+1,id.length());
    std::string path=output_path+id+".hdf5";

    H5File file(path, H5F_ACC_TRUNC);

    const hsize_t length=id_end-id_start+1;
    Group Refnav=file.createGroup("Refnav");

    DataSpace dataspace_timestamp = timestamp.getSpace();
    hsize_t offset_timestamp[1]={id_start}; // hyperslab offset in the file
    hsize_t count_timestamp[1]={length};  // size of the hyperslab in the file
    dataspace_timestamp.selectHyperslab(H5S_SELECT_SET, count_timestamp, offset_timestamp);
    double array_timestamp[length];
    DataSpace split_dataspace_timestamp(1,count_timestamp);
    timestamp.read(array_timestamp, PredType::NATIVE_DOUBLE,split_dataspace_timestamp, dataspace_timestamp);
    DataSet dataset_split_timestamp=Refnav.createDataSet("timestamp",PredType::NATIVE_DOUBLE,split_dataspace_timestamp);
    dataset_split_timestamp.write(array_timestamp,PredType::NATIVE_DOUBLE);
    

    DataSpace dataspace_isValidLatLong = isValidLatLong.getSpace();
    hsize_t offset_isValidLatLong[1]={id_start}; // hyperslab offset in the file
    hsize_t count_isValidLatLong[1]={length};  // size of the hyperslab in the file
    dataspace_isValidLatLong.selectHyperslab(H5S_SELECT_SET, count_isValidLatLong, offset_isValidLatLong);
    uint8_t array_isValidLatLong[length];
    DataSpace split_dataspace_isValidLatLong(1,count_isValidLatLong);
    isValidLatLong.read(array_isValidLatLong, PredType::NATIVE_UINT8,split_dataspace_isValidLatLong, dataspace_isValidLatLong);
    DataSet dataset_split_isValidLatLong=Refnav.createDataSet("isValidLatLong",PredType::NATIVE_UINT8,split_dataspace_isValidLatLong);
    dataset_split_isValidLatLong.write(array_isValidLatLong,PredType::NATIVE_UINT8);

    DataSpace dataspace_isValidAltitude = isValidAltitude.getSpace();
    hsize_t offset_isValidAltitude[1]={id_start}; // hyperslab offset in the file
    hsize_t count_isValidAltitude[1]={length};  // size of the hyperslab in the file
    dataspace_isValidAltitude.selectHyperslab(H5S_SELECT_SET, count_isValidAltitude, offset_isValidAltitude);
    uint8_t array_isValidAltitude[length];
    DataSpace split_dataspace_isValidAltitude(1,count_isValidAltitude);
    isValidAltitude.read(array_isValidAltitude, PredType::NATIVE_UINT8,split_dataspace_isValidAltitude, dataspace_isValidAltitude);
    DataSet dataset_split_isValidAltitude=Refnav.createDataSet("isValidAltitude",PredType::NATIVE_UINT8,split_dataspace_isValidAltitude);
    dataset_split_isValidAltitude.write(array_isValidAltitude,PredType::NATIVE_UINT8);

    DataSpace dataspace_isValidHeading = isValidHeading.getSpace();
    hsize_t offset_isValidHeading[1]={id_start}; // hyperslab offset in the file
    hsize_t count_isValidHeading[1]={length};  // size of the hyperslab in the file
    dataspace_isValidHeading.selectHyperslab(H5S_SELECT_SET, count_isValidHeading, offset_isValidHeading);
    uint8_t array_isValidHeading[length];
    DataSpace split_dataspace_isValidHeading(1,count_isValidHeading);
    isValidHeading.read(array_isValidHeading, PredType::NATIVE_UINT8,split_dataspace_isValidHeading, dataspace_isValidHeading);
    DataSet dataset_split_isValidHeading=Refnav.createDataSet("isValidHeading",PredType::NATIVE_UINT8,split_dataspace_isValidHeading);
    dataset_split_isValidHeading.write(array_isValidHeading,PredType::NATIVE_UINT8);

    DataSpace dataspace_posLon = posLon.getSpace();
    hsize_t offset_posLon[1]={id_start}; // hyperslab offset in the file
    hsize_t count_posLon[1]={length};  // size of the hyperslab in the file
    dataspace_posLon.selectHyperslab(H5S_SELECT_SET, count_posLon, offset_posLon);
    double array_posLon[length];
    DataSpace split_dataspace_posLon(1,count_posLon);
    posLon.read(array_posLon, PredType::NATIVE_DOUBLE,split_dataspace_posLon, dataspace_posLon);
    DataSet dataset_split_posLon=Refnav.createDataSet("posLon",PredType::NATIVE_DOUBLE,split_dataspace_posLon);
    dataset_split_posLon.write(array_posLon,PredType::NATIVE_DOUBLE);

    DataSpace dataspace_posLat = posLat.getSpace();
    hsize_t offset_posLat[1]={id_start}; // hyperslab offset in the file
    hsize_t count_posLat[1]={length};  // size of the hyperslab in the file
    dataspace_posLat.selectHyperslab(H5S_SELECT_SET, count_posLat, offset_posLat);
    double array_posLat[length];
    DataSpace split_dataspace_posLat(1,count_posLat);
    posLat.read(array_posLat, PredType::NATIVE_DOUBLE,split_dataspace_posLat, dataspace_posLat);
    DataSet dataset_split_posLat=Refnav.createDataSet("posLat",PredType::NATIVE_DOUBLE,split_dataspace_posLat);
    dataset_split_posLat.write(array_posLat,PredType::NATIVE_DOUBLE);

    DataSpace dataspace_posAlt = posAlt.getSpace();
    hsize_t offset_posAlt[1]={id_start}; // hyperslab offset in the file
    hsize_t count_posAlt[1]={length};  // size of the hyperslab in the file
    dataspace_posAlt.selectHyperslab(H5S_SELECT_SET, count_posAlt, offset_posAlt);
    double array_posAlt[length];
    DataSpace split_dataspace_posAlt(1,count_posAlt);
    posAlt.read(array_posAlt, PredType::NATIVE_DOUBLE,split_dataspace_posAlt, dataspace_posAlt);
    DataSet dataset_split_posAlt=Refnav.createDataSet("posAlt",PredType::NATIVE_DOUBLE,split_dataspace_posAlt);
    dataset_split_posAlt.write(array_posAlt,PredType::NATIVE_DOUBLE);

    DataSpace dataspace_heading = heading.getSpace();
    hsize_t offset_heading[1]={id_start}; // hyperslab offset in the file
    hsize_t count_heading[1]={length};  // size of the hyperslab in the file
    dataspace_heading.selectHyperslab(H5S_SELECT_SET, count_heading, offset_heading);
    double array_heading[length];
    DataSpace split_dataspace_heading(1,count_heading);
    heading.read(array_heading, PredType::NATIVE_DOUBLE,split_dataspace_heading, dataspace_heading);
    DataSet dataset_split_heading=Refnav.createDataSet("heading",PredType::NATIVE_DOUBLE,split_dataspace_heading);
    dataset_split_heading.write(array_heading,PredType::NATIVE_DOUBLE);

    DataSpace dataspace_speed = speed.getSpace();
    hsize_t offset_speed[1]={id_start}; // hyperslab offset in the file
    hsize_t count_speed[1]={length};  // size of the hyperslab in the file
    dataspace_speed.selectHyperslab(H5S_SELECT_SET, count_speed, offset_speed);
    double array_speed[length];
    DataSpace split_dataspace_speed(1,count_speed);
    speed.read(array_speed, PredType::NATIVE_DOUBLE,split_dataspace_speed, dataspace_speed);
    DataSet dataset_split_speed=Refnav.createDataSet("speed",PredType::NATIVE_DOUBLE,split_dataspace_speed);
    dataset_split_speed.write(array_speed,PredType::NATIVE_DOUBLE);


    DataSpace dataspace_stdDevHeading = stdDevHeading.getSpace();
    hsize_t offset_stdDevHeading[1]={id_start}; // hyperslab offset in the file
    hsize_t count_stdDevHeading[1]={length};  // size of the hyperslab in the file
    dataspace_stdDevHeading.selectHyperslab(H5S_SELECT_SET, count_stdDevHeading, offset_stdDevHeading);
    float array_stdDevHeading[length];
    DataSpace split_dataspace_stdDevHeading(1,count_stdDevHeading);
    stdDevHeading.read(array_stdDevHeading, PredType::NATIVE_FLOAT,split_dataspace_stdDevHeading, dataspace_stdDevHeading);
    DataSet dataset_split_stdDevHeading=Refnav.createDataSet("stdDevHeading",PredType::NATIVE_FLOAT,split_dataspace_stdDevHeading);
    dataset_split_stdDevHeading.write(array_stdDevHeading,PredType::NATIVE_FLOAT);

    DataSpace dataspace_stdDevPosDown = stdDevPosDown.getSpace();
    hsize_t offset_stdDevPosDown[1]={id_start}; // hyperslab offset in the file
    hsize_t count_stdDevPosDown[1]={length};  // size of the hyperslab in the file
    dataspace_stdDevPosDown.selectHyperslab(H5S_SELECT_SET, count_stdDevPosDown, offset_stdDevPosDown);
    float array_stdDevPosDown[length];
    DataSpace split_dataspace_stdDevPosDown(1,count_stdDevPosDown);
    stdDevPosDown.read(array_stdDevPosDown, PredType::NATIVE_FLOAT,split_dataspace_stdDevPosDown, dataspace_stdDevPosDown);
    DataSet dataset_split_stdDevPosDown=Refnav.createDataSet("stdDevPosDown",PredType::NATIVE_FLOAT,split_dataspace_stdDevPosDown);
    dataset_split_stdDevPosDown.write(array_stdDevPosDown,PredType::NATIVE_FLOAT);

    DataSpace dataspace_stdDevPosEast = stdDevPosEast.getSpace();
    hsize_t offset_stdDevPosEast[1]={id_start}; // hyperslab offset in the file
    hsize_t count_stdDevPosEast[1]={length};  // size of the hyperslab in the file
    dataspace_stdDevPosEast.selectHyperslab(H5S_SELECT_SET, count_stdDevPosEast, offset_stdDevPosEast);
    float array_stdDevPosEast[length];
    DataSpace split_dataspace_stdDevPosEast(1,count_stdDevPosEast);
    stdDevPosEast.read(array_stdDevPosEast, PredType::NATIVE_FLOAT,split_dataspace_stdDevPosEast, dataspace_stdDevPosEast);
    DataSet dataset_split_stdDevPosEast=Refnav.createDataSet("stdDevPosEast",PredType::NATIVE_FLOAT,split_dataspace_stdDevPosEast);
    dataset_split_stdDevPosEast.write(array_stdDevPosEast,PredType::NATIVE_FLOAT);

    DataSpace dataspace_stdDevPosNorth = stdDevPosNorth.getSpace();
    hsize_t offset_stdDevPosNorth[1]={id_start}; // hyperslab offset in the file
    hsize_t count_stdDevPosNorth[1]={length};  // size of the hyperslab in the file
    dataspace_stdDevPosNorth.selectHyperslab(H5S_SELECT_SET, count_stdDevPosNorth, offset_stdDevPosNorth);
    float array_stdDevPosNorth[length];
    DataSpace split_dataspace_stdDevPosNorth(1,count_stdDevPosNorth);
    stdDevPosNorth.read(array_stdDevPosNorth, PredType::NATIVE_FLOAT,split_dataspace_stdDevPosNorth, dataspace_stdDevPosNorth);
    DataSet dataset_split_stdDevPosNorth=Refnav.createDataSet("stdDevPosNorth",PredType::NATIVE_FLOAT,split_dataspace_stdDevPosNorth);
    dataset_split_stdDevPosNorth.write(array_stdDevPosNorth,PredType::NATIVE_FLOAT);

}
