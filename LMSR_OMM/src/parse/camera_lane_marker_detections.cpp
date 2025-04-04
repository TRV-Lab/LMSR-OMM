#include "parse/camera_lane_marker_detections.h"
using std::cout;
using std::endl;
using namespace H5;

void H5_clmd::getDataSet(H5File &file,bool is_vision_file)
{
    Group lane_markers_group;
    if(is_vision_file)
    {
        lane_markers_group=file.openGroup("zen_qm_vision").openGroup("camera_lane_marker_detections");

    }
    else{
        lane_markers_group=file.openGroup("zen_qm_sensorfusion_a").openGroup("zen_qm_sensorfusion_a_camera_lane_marker_detections");

    }
    Group lane_markers_data=lane_markers_group.openGroup("data");
    zeader_timestamp_ns=lane_markers_group.openGroup("zeader").openDataSet("timestamp_ns");
    vision_timestamp=lane_markers_data.openGroup("timestamp").openGroup("nanoseconds").openDataSet("value");
    n_valid_points=lane_markers_data.openGroup("n_valid_points").openGroup("unitless").openDataSet("value");
    points=lane_markers_data.openGroup("points");

    
}
void H5_clmd::parseTo(Cxx_camera_lane_marker_detections &cxx_clmd) const
{
    /*Parse zeader timestamp*/
    parseZeaderTimestampNsTo(cxx_clmd);
    /*Debug
    for(auto timestamp:Cxx_hp.getZeaderTimestampNs())
    {
        cout<<timestamp<<" ";
    }
    cout<<endl;*/

    /*Parse timestamp*/
    parseVisionTimestampTo(cxx_clmd);
    /*Debug
    for(auto timestamp:Cxx_hp.getTimestamp())
    {
        cout<<timestamp<<" ";
    }
    cout<<endl;*/
    /*Parse number of valid points*/
    parseNumOfValidPointsTo(cxx_clmd);


    /*Parse points*/
    parsePointsTo(cxx_clmd);



}
void H5_clmd::parseZeaderTimestampNsTo(Cxx_clmd &cxx_clmd) const
{
    DataSpace dataspace_zeader_timestamp_ns = zeader_timestamp_ns.getSpace();
    const int rank_zeader_timestamp_ns = dataspace_zeader_timestamp_ns.getSimpleExtentNdims();
    assert(rank_zeader_timestamp_ns==1);
    hsize_t dims_zeader_timestamp_ns[rank_zeader_timestamp_ns];
    dataspace_zeader_timestamp_ns.getSimpleExtentDims(dims_zeader_timestamp_ns, NULL);
    const int length=dims_zeader_timestamp_ns[0];
    cxx_clmd.setLength(length);
    uint64_t array_zeader_timestamp_ns[length];
    zeader_timestamp_ns.read(array_zeader_timestamp_ns, PredType::NATIVE_UINT64,H5::DataSpace::ALL, dataspace_zeader_timestamp_ns);
    cxx_clmd.getZeaderTimestampNs().reserve(length);
    cxx_clmd.getZeaderTimestampNs().assign(&array_zeader_timestamp_ns[0],&array_zeader_timestamp_ns[length]);

}


void H5_clmd::parseVisionTimestampTo(Cxx_clmd &cxx_clmd) const
{
    DataSpace dataspace_vision_timestamp = vision_timestamp.getSpace();
    const int rank_vision_timestamp = dataspace_vision_timestamp.getSimpleExtentNdims();
    assert(rank_vision_timestamp==1);
    hsize_t dims_vision_timestamp[rank_vision_timestamp];
    dataspace_vision_timestamp.getSimpleExtentDims(dims_vision_timestamp, NULL);
    const int length=cxx_clmd.getLength();
    assert(length==dims_vision_timestamp[0]);
    int64_t array_vision_timestamp[length];
    vision_timestamp.read(array_vision_timestamp, PredType::NATIVE_INT64,H5::DataSpace::ALL, dataspace_vision_timestamp);
    cxx_clmd.getVisionTimestamp().reserve(length);
    cxx_clmd.getVisionTimestamp().assign(&array_vision_timestamp[0],&array_vision_timestamp[length]);

}

void H5_clmd::parseNumOfValidPointsTo(Cxx_clmd &cxx_clmd) const
{
    DataSpace dataspace_n_valid_points = n_valid_points.getSpace();
    const int rank_n_valid_points = dataspace_n_valid_points.getSimpleExtentNdims();
    assert(rank_n_valid_points==1);
    hsize_t dims_n_valid_points[rank_n_valid_points];
    dataspace_n_valid_points.getSimpleExtentDims(dims_n_valid_points, NULL);
    const int length=cxx_clmd.getLength();
    assert(length==dims_n_valid_points[0]);
    uint8_t array_n_valid_points[length];
    n_valid_points.read(array_n_valid_points, PredType::NATIVE_UINT8,H5::DataSpace::ALL, dataspace_n_valid_points);
    cxx_clmd.getNumOfValidPoints().reserve(length);
    cxx_clmd.getNumOfValidPoints().assign(&array_n_valid_points[0],&array_n_valid_points[length]);

}

void H5_clmd::parsePointsTo(Cxx_clmd &cxx_clmd) const
{
    const int length=cxx_clmd.getLength();
    /*Parse left_edge*/
    DataSet left_edge_points_x=points.openGroup("left_edge").openGroup("x").openGroup("meters").openDataSet("value");
    DataSpace dataspace_left_edge_points_x = left_edge_points_x.getSpace();
    const int rank_left_edge_points_x = dataspace_left_edge_points_x.getSimpleExtentNdims();
    assert(rank_left_edge_points_x==2);
    hsize_t dims_left_edge_points_x[rank_left_edge_points_x];
    dataspace_left_edge_points_x.getSimpleExtentDims(dims_left_edge_points_x, NULL);
    assert(dims_left_edge_points_x[1]==1000);
    assert(length==dims_left_edge_points_x[0]);
    auto array_left_edge_points_x=new float[length][1000];
    left_edge_points_x.read(array_left_edge_points_x, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_left_edge_points_x);

    DataSet left_edge_points_y=points.openGroup("left_edge").openGroup("y").openGroup("meters").openDataSet("value");
    DataSpace dataspace_left_edge_points_y = left_edge_points_y.getSpace();
    const int rank_left_edge_points_y = dataspace_left_edge_points_y.getSimpleExtentNdims();
    assert(rank_left_edge_points_y==2);
    hsize_t dims_left_edge_points_y[rank_left_edge_points_y];
    dataspace_left_edge_points_y.getSimpleExtentDims(dims_left_edge_points_y, NULL);
    assert(dims_left_edge_points_y[1]==1000);
    assert(length==dims_left_edge_points_y[0]);
    auto array_left_edge_points_y=new float[length][1000];
    left_edge_points_y.read(array_left_edge_points_y, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_left_edge_points_y);

    DataSet left_edge_std_dev=points.openGroup("left_point_std_dev").openGroup("meters").openDataSet("value");
    DataSpace dataspace_left_edge_std_dev = left_edge_std_dev.getSpace();
    const int rank_left_edge_std_dev = dataspace_left_edge_std_dev.getSimpleExtentNdims();
    assert(rank_left_edge_std_dev==2);
    hsize_t dims_left_edge_std_dev[rank_left_edge_std_dev];
    dataspace_left_edge_std_dev.getSimpleExtentDims(dims_left_edge_std_dev, NULL);
    assert(dims_left_edge_std_dev[1]==1000);
    assert(length==dims_left_edge_std_dev[0]);
    auto array_left_edge_std_dev=new float[length][1000];
    left_edge_std_dev.read(array_left_edge_std_dev, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_left_edge_std_dev);

    /*Parse right_edge*/
    DataSet right_edge_points_x=points.openGroup("right_edge").openGroup("x").openGroup("meters").openDataSet("value");
    DataSpace dataspace_right_edge_points_x = right_edge_points_x.getSpace();
    const int rank_right_edge_points_x = dataspace_right_edge_points_x.getSimpleExtentNdims();
    assert(rank_right_edge_points_x==2);
    hsize_t dims_right_edge_points_x[rank_right_edge_points_x];
    dataspace_right_edge_points_x.getSimpleExtentDims(dims_right_edge_points_x, NULL);
    assert(dims_right_edge_points_x[1]==1000);
    assert(length==dims_right_edge_points_x[0]);
    auto array_right_edge_points_x=new float[length][1000];
    right_edge_points_x.read(array_right_edge_points_x, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_right_edge_points_x);

    DataSet right_edge_points_y=points.openGroup("right_edge").openGroup("y").openGroup("meters").openDataSet("value");
    DataSpace dataspace_right_edge_points_y = right_edge_points_y.getSpace();
    const int rank_right_edge_points_y = dataspace_right_edge_points_y.getSimpleExtentNdims();
    assert(rank_right_edge_points_y==2);
    hsize_t dims_right_edge_points_y[rank_right_edge_points_y];
    dataspace_right_edge_points_y.getSimpleExtentDims(dims_right_edge_points_y, NULL);
    assert(dims_right_edge_points_y[1]==1000);
    assert(length==dims_right_edge_points_y[0]);
    auto array_right_edge_points_y=new float[length][1000];
    right_edge_points_y.read(array_right_edge_points_y, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_right_edge_points_y);

    DataSet right_edge_std_dev=points.openGroup("right_point_std_dev").openGroup("meters").openDataSet("value");
    DataSpace dataspace_right_edge_std_dev = right_edge_std_dev.getSpace();
    const int rank_right_edge_std_dev = dataspace_right_edge_std_dev.getSimpleExtentNdims();
    assert(rank_right_edge_std_dev==2);
    hsize_t dims_right_edge_std_dev[rank_right_edge_std_dev];
    dataspace_right_edge_std_dev.getSimpleExtentDims(dims_right_edge_std_dev, NULL);
    assert(dims_right_edge_std_dev[1]==1000);
    assert(length==dims_right_edge_std_dev[0]);
    auto array_right_edge_std_dev=new float[length][1000];
    right_edge_std_dev.read(array_right_edge_std_dev, PredType::NATIVE_FLOAT,H5::DataSpace::ALL, dataspace_right_edge_std_dev);

    DataSet color=points.openDataSet("color");
    DataSpace dataspace_color = color.getSpace();
    const int rank_color = dataspace_color.getSimpleExtentNdims();
    assert(rank_color==2);
    hsize_t dims_color[rank_color];
    dataspace_color.getSimpleExtentDims(dims_color, NULL);
    assert(dims_color[1]==1000);
    assert(length==dims_color[0]);
    auto array_color=new int32_t[length][1000];
    color.read(array_color, PredType::NATIVE_INT32,H5::DataSpace::ALL, dataspace_color);

    DataSet instance_id=points.openGroup("instance_id").openGroup("unitless").openDataSet("value");
    DataSpace dataspace_instance_id = instance_id.getSpace();
    const int rank_instance_id = dataspace_instance_id.getSimpleExtentNdims();
    assert(rank_instance_id==2);
    hsize_t dims_instance_id[rank_instance_id];
    dataspace_instance_id.getSimpleExtentDims(dims_instance_id, NULL);
    assert(dims_instance_id[1]==1000);
    assert(length==dims_instance_id[0]);
    auto array_instance_id=new uint8_t[length][1000];
    instance_id.read(array_instance_id, PredType::NATIVE_UINT8,H5::DataSpace::ALL, dataspace_instance_id);

    DataSet type=points.openDataSet("type");
    DataSpace dataspace_type = type.getSpace();
    const int rank_type = dataspace_type.getSimpleExtentNdims();
    assert(rank_type==2);
    hsize_t dims_type[rank_type];
    dataspace_type.getSimpleExtentDims(dims_type, NULL);
    assert(dims_type[1]==1000);
    assert(length==dims_type[0]);
    auto array_type=new uint8_t[length][1000];
    type.read(array_type, PredType::NATIVE_UINT8,H5::DataSpace::ALL, dataspace_type);

    cxx_clmd.getEdgePoints().resize(length);
    for(int i=0;i<length;i++)
    {
        // cxx_clmd.getEdgePoints()[i].resize(cxx_clmd.getNumOfValidPoints());
        for(int j=0;j<cxx_clmd.getNumOfValidPoints()[i];j++)
        {
            Edge_point p;
            p.instance_id=array_instance_id[i][j];
            p.color=array_color[i][j];
            p.type=array_type[i][j];
            p.x=array_left_edge_points_x[i][j];
            p.y=array_left_edge_points_y[i][j];
            p.std_dev=array_left_edge_std_dev[i][j];
            cxx_clmd.getEdgePoints()[i].push_back(p);
            p.x=array_right_edge_points_x[i][j];
            p.y=array_right_edge_points_y[i][j];            
            p.std_dev=array_right_edge_std_dev[i][j];
            cxx_clmd.getEdgePoints()[i].push_back(p);
        }

    }
    delete[] array_left_edge_points_x;
    array_left_edge_points_x=nullptr;
    delete[] array_left_edge_points_y;
    array_left_edge_points_y=nullptr;
    delete[] array_left_edge_std_dev;
    array_left_edge_std_dev=nullptr;
    delete[] array_right_edge_points_x;
    array_right_edge_points_x=nullptr;
    delete[] array_right_edge_points_y;
    array_right_edge_points_y=nullptr;
    delete[] array_right_edge_std_dev;
    array_right_edge_std_dev=nullptr;
    delete[] array_color;
    array_color=nullptr;
    delete[] array_instance_id;
    array_instance_id=nullptr;
    delete[] array_type;
    array_type=nullptr;
        
}