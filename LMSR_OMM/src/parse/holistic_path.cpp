#include "parse/holistic_path.h"
using std::cout;
using std::endl;
using namespace H5;

void H5_hp::getDataSet(H5File& file,bool is_vision_file)
{
    Group hp_group;
    if(is_vision_file)
    {
        hp_group=file.openGroup("zen_qm_vision").openGroup("holistic_path");
   
    }
    else{
        hp_group=file.openGroup("zen_qm_sensorfusion_a").openGroup("zen_qm_sensorfusion_a_holistic_path");
    }
    Group hp_data=hp_group.openGroup("data");
    zeader_timestamp_ns=hp_group.openGroup("zeader").openDataSet("timestamp_ns");
    timestamp=hp_data.openGroup("timestamp").openGroup("nanoseconds").openDataSet("value");
    n_valid_points=hp_data.openGroup("n_valid_points").openGroup("unitless").openDataSet("value");
    points_x=hp_data.openGroup("points").openGroup("coordinate").openGroup("x").openGroup("meters").openDataSet("value");
    points_y=hp_data.openGroup("points").openGroup("coordinate").openGroup("y").openGroup("meters").openDataSet("value");
    points_z=hp_data.openGroup("points").openGroup("coordinate").openGroup("z").openGroup("meters").openDataSet("value");
    points_x_deviation=hp_data.openGroup("points").openGroup("uncertainty").openGroup("x_deviation").openGroup("meters").openDataSet("value");
    points_y_deviation=hp_data.openGroup("points").openGroup("uncertainty").openGroup("y_deviation").openGroup("meters").openDataSet("value");
    points_z_deviation=hp_data.openGroup("points").openGroup("uncertainty").openGroup("z_deviation").openGroup("meters").openDataSet("value");

    
}
void H5_hp::parseTo(Cxx_hp &Cxx_hp) const
{
    /*Parse zeader timestamp*/
    parseZeaderTimestampNsTo(Cxx_hp);
    /*Debug
    for(auto timestamp:Cxx_hp.getZeaderTimestampNs())
    {
        cout<<timestamp<<" ";
    }
    cout<<endl;*/

    /*Parse timestamp*/
    parseTimestampTo(Cxx_hp);
    /*Debug
    for(auto timestamp:Cxx_hp.getTimestamp())
    {
        cout<<timestamp<<" ";
    }
    cout<<endl;*/

    /*Parse number of valid points*/
    parseNumOfValidPoints(Cxx_hp);


    /*Parse points*/
    parsePoints(Cxx_hp);




}
void H5_hp::parseZeaderTimestampNsTo(Cxx_hp &Cxx_hp) const
{
    DataSpace dataspace_zeader_timestamp_ns = zeader_timestamp_ns.getSpace();
    const int rank_zeader_timestamp_ns = dataspace_zeader_timestamp_ns.getSimpleExtentNdims();
    assert(rank_zeader_timestamp_ns==1);
    hsize_t dims_zeader_timestamp_ns[rank_zeader_timestamp_ns];
    dataspace_zeader_timestamp_ns.getSimpleExtentDims(dims_zeader_timestamp_ns, NULL);
    const int length=dims_zeader_timestamp_ns[0];
    Cxx_hp.setLength(length);
    uint64_t array_zeader_timestamp_ns[length];
    zeader_timestamp_ns.read(array_zeader_timestamp_ns, PredType::NATIVE_UINT64,H5::DataSpace::ALL, dataspace_zeader_timestamp_ns);
    Cxx_hp.getZeaderTimestampNs().reserve(length);
    Cxx_hp.getZeaderTimestampNs().assign(&array_zeader_timestamp_ns[0],&array_zeader_timestamp_ns[length]);

}
void H5_hp::parseTimestampTo(Cxx_hp &Cxx_hp) const
{
    DataSpace dataspace_timestamp = timestamp.getSpace();
    const int rank_timestamp = dataspace_timestamp.getSimpleExtentNdims();
    assert(rank_timestamp==1);
    hsize_t dims_timestamp[rank_timestamp];
    dataspace_timestamp.getSimpleExtentDims(dims_timestamp, NULL);
    const int length=Cxx_hp.getLength();
    assert(length==dims_timestamp[0]);
    int64_t array_timestamp[length];
    timestamp.read(array_timestamp, PredType::NATIVE_INT64,H5::DataSpace::ALL, dataspace_timestamp);
    Cxx_hp.getTimestamp().reserve(length);
    Cxx_hp.getTimestamp().assign(&array_timestamp[0],&array_timestamp[length]);

}
void H5_hp::parseNumOfValidPoints(Cxx_hp &Cxx_hp) const
{
    DataSpace dataspace_n_valid_points = n_valid_points.getSpace();
    const int rank_n_valid_points = dataspace_n_valid_points.getSimpleExtentNdims();
    assert(rank_n_valid_points==1);
    hsize_t dims_n_valid_points[rank_n_valid_points];
    dataspace_n_valid_points.getSimpleExtentDims(dims_n_valid_points, NULL);
    const int length=Cxx_hp.getLength();
    assert(length==dims_n_valid_points[0]);
    uint8_t array_n_valid_points[length];
    n_valid_points.read(array_n_valid_points, PredType::NATIVE_UINT8,H5::DataSpace::ALL, dataspace_n_valid_points);
    Cxx_hp.getNumOfValidPoints().reserve(length);
    Cxx_hp.getNumOfValidPoints().assign(&array_n_valid_points[0],&array_n_valid_points[length]);

}
void H5_hp::parsePoints(Cxx_hp &Cxx_hp) const
{
    const int length=Cxx_hp.getLength();
    /*Parse x*/
    DataSpace dataspace_points_x = points_x.getSpace();
    const int rank_points_x = dataspace_points_x.getSimpleExtentNdims();
    assert(rank_points_x==2);
    hsize_t dims_points_x[rank_points_x];
    dataspace_points_x.getSimpleExtentDims(dims_points_x, NULL);
    assert(dims_points_x[1]==17);
    assert(length==dims_points_x[0]);
    auto array_points_x=new float[length][17];
    // float array_points_x[length];
    points_x.read(array_points_x, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_points_x);

    /*Parse y*/
    DataSpace dataspace_points_y = points_y.getSpace();
    const int rank_points_y = dataspace_points_y.getSimpleExtentNdims();
    assert(rank_points_y==2);
    hsize_t dims_points_y[rank_points_y];
    dataspace_points_y.getSimpleExtentDims(dims_points_y, NULL);
    assert(dims_points_y[1]==17);
    assert(length==dims_points_y[0]);
    auto array_points_y=new float[length][17];
    points_y.read(array_points_y, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_points_y);

    /*Parse z*/
    DataSpace dataspace_points_z = points_z.getSpace();
    const int rank_points_z = dataspace_points_z.getSimpleExtentNdims();
    assert(rank_points_z==2);
    hsize_t dims_points_z[rank_points_z];
    dataspace_points_z.getSimpleExtentDims(dims_points_z, NULL);
    assert(dims_points_x[1]==17);
    assert(length==dims_points_z[0]);
    auto array_points_z=new float[length][17];
    points_z.read(array_points_z, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_points_z);

    /*Parse x_deviation*/
    DataSpace dataspace_points_x_deviation = points_x_deviation.getSpace();
    const int rank_points_x_deviation = dataspace_points_x_deviation.getSimpleExtentNdims();
    assert(rank_points_x_deviation==2);
    hsize_t dims_points_x_deviation[rank_points_x_deviation];
    dataspace_points_x_deviation.getSimpleExtentDims(dims_points_x_deviation, NULL);
    assert(dims_points_x[1]==17);
    assert(length==dims_points_x_deviation[0]);
    auto array_points_x_deviation=new float[length][17];
    points_x_deviation.read(array_points_x_deviation, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_points_x_deviation);

    /*Parse y_deviation*/
    DataSpace dataspace_points_y_deviation = points_y_deviation.getSpace();
    const int rank_points_y_deviation = dataspace_points_y_deviation.getSimpleExtentNdims();
    assert(rank_points_y_deviation==2);
    hsize_t dims_points_y_deviation[rank_points_y_deviation];
    dataspace_points_y_deviation.getSimpleExtentDims(dims_points_y_deviation, NULL);
    assert(dims_points_x[1]==17);
    assert(length==dims_points_y_deviation[0]);
    auto array_points_y_deviation=new float[length][17];
    points_y_deviation.read(array_points_y_deviation, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_points_y_deviation);

    /*Parse z_deviation*/
    DataSpace dataspace_points_z_deviation = points_z_deviation.getSpace();
    const int rank_points_z_deviation = dataspace_points_z_deviation.getSimpleExtentNdims();
    assert(rank_points_z_deviation==2);
    hsize_t dims_points_z_deviation[rank_points_z_deviation];
    dataspace_points_z_deviation.getSimpleExtentDims(dims_points_z_deviation, NULL);
    assert(dims_points_x[1]==17);
    assert(length==dims_points_z_deviation[0]);
    auto array_points_z_deviation=new float[length][17];
    points_z_deviation.read(array_points_z_deviation, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_points_z_deviation);

    Cxx_hp.getPoints().resize(length);
    for(int i=0;i<length;i++)
    {
        Cxx_hp.getPoints()[i].resize(17);
        for(int j=0;j<17;j++)
        {
            Cxx_hp.getPoints()[i][j].x=array_points_x[i][j];
            Cxx_hp.getPoints()[i][j].y=array_points_y[i][j];
            Cxx_hp.getPoints()[i][j].z=array_points_z[i][j];
            Cxx_hp.getPoints()[i][j].x_deviation=array_points_x_deviation[i][j];
            Cxx_hp.getPoints()[i][j].y_deviation=array_points_y_deviation[i][j];
            Cxx_hp.getPoints()[i][j].z_deviation=array_points_z_deviation[i][j];
        }
    }
    delete[] array_points_x;
    array_points_x=nullptr;
    delete[] array_points_y;
    array_points_y=nullptr;
    delete[] array_points_z;
    array_points_z=nullptr;
    delete[] array_points_x_deviation;
    array_points_x_deviation=nullptr;
    delete[] array_points_y_deviation;
    array_points_y_deviation=nullptr;
    delete[] array_points_z_deviation;
    array_points_z_deviation=nullptr;
}


void Cxx_hp::print() const
{
    for(int i=0;i<length;i++)
    {
        if(i%20==0)
        {
            cout<<"zeader_time"<<"\t"<<"timestamp"<<"\t"<<"n_valid_points"<<"\t"<<"points"<<endl;
        }
        cout<<zeader_timestamp_ns[i]<<"\t"<<timestamp[i]<<"\t"<<n_valid_points[i]<<"\t"<<points[i][8].x<<","<<points[i][8].y<<","<<points[i][8].z<<"\t"<<points[i][8].x_deviation<<"\t"<<points[i][8].y_deviation<<"\t"<<points[i][8].z_deviation<<endl;
    }
}