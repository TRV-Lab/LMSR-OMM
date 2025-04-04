#ifndef _VISION_TRAFFIC_SIGNS_H_
#define _VISION_TRAFFIC_SIGNS_H_
#include "utils.h"

using namespace H5;

/*struct Bbox_2d
{

}*/
struct Bbox_3d
{
    float height;
    float length;
    float width;
    float x;
    float y;
    float z;
    float pitch;
    float roll;
    float yaw;
};
struct Object_base
{   
    // Bbox_2d bbox_2d;
    Bbox_3d bbox_3d;
    float class_score[16];
    float objectness;

};
struct Signs
{
    int64_t associated_track_id;
    float association_confidence;
    float detection_perpendicular_ratio;
    int64_t id;
    float max_objectness;
    int32_t max_sign_probabilities_index;
    uint8_t number_supplementary_sign_indices;
    Object_base object_base;
    float position_covariance[9];
    float sign_height_std;
    float sign_is_electronic_probability;
    float sign_probabilities[273];
    float sign_width_std;
    std::vector<uint8_t> supplementary_sign_indices;
    float yaw_angle_std;
};
class Cxx_vision_traffic_signs
{
private:
    int length;
    std::vector<uint64_t> zeader_timestamp_ns;
    std::vector<float> mount_pitch;
    std::vector<float> mount_roll;
    std::vector<float> mount_yaw;
    std::vector<float> mount_x;
    std::vector<float> mount_y;
    std::vector<float> mount_z;
    std::vector<uint8_t> number_of_valid_entries;
    std::vector<std::vector<Signs> > signs;
    std::vector<int64_t> vision_timestamp;
    std::vector<uint8_t> isUpdate;

public:
    void setLength(int l){length=l;}
    int getLength() const {return length;}
    std::vector<uint64_t>& getZeaderTimestampNs() {return zeader_timestamp_ns;}
    std::vector<float>& getMountPitch() {return mount_pitch;}
    std::vector<float>& getMountRoll() {return mount_roll;}
    std::vector<float>& getMountYaw() {return mount_yaw;}
    std::vector<float>& getMountX() {return mount_x;}
    std::vector<float>& getMountY() {return mount_y;}
    std::vector<float>& getMountZ() {return mount_z;}
    std::vector<uint8_t>& getNumOfValid() {return number_of_valid_entries;}
    std::vector<std::vector<Signs> >& getSigns() {return signs;}
    std::vector<int64_t>& getVisionTimestamp() {return vision_timestamp;}
    std::vector<uint8_t>& getIsUpdate() {return isUpdate;}
    void print()const;

};
using Cxx_vts=Cxx_vision_traffic_signs;
class H5_vision_traffic_signs
{
private:
    DataSet zeader_timestamp_ns;
    DataSet mount_pitch;
    DataSet mount_roll;
    DataSet mount_yaw;
    DataSet mount_x;
    DataSet mount_y;
    DataSet mount_z;
    DataSet number_of_valid_entries;
    Group signs;
    DataSet vision_timestamp;
    void parseZeaderTimestampNsTo(Cxx_vts &) const;
    void parseMountPitchTo(Cxx_vts &) const;
    void parseMountRollTo(Cxx_vts &) const;
    void parseMountYawTo(Cxx_vts &) const;
    void parseMountXTo(Cxx_vts &) const;
    void parseMountYTo(Cxx_vts &) const;
    void parseMountZTo(Cxx_vts &) const;
    void parseNumOfValidTo(Cxx_vts &) const;
    void parseSignsTo(Cxx_vts &) const;
    void parseVisionTimestampTo(Cxx_vts &) const;

public:
    void getDataSet(H5File&);
    void parseTo(Cxx_vts &cxx_vts) const;
    // void checkUpdate();
    DataSet getZeaderTimestampNs()const {return zeader_timestamp_ns;}
    DataSet getMountPitch()const {return mount_pitch;}
    DataSet getMountRoll()const {return mount_roll;}
    DataSet getMountYaw()const {return mount_yaw;}
    DataSet getMountX()const {return mount_x;}
    DataSet getMountY()const {return mount_y;}
    DataSet getMountZ()const {return mount_z;}
    DataSet getNumOfValid()const {return number_of_valid_entries;}
    Group getSigns()const {return signs;}
    DataSet getVisionTimestamp()const {return vision_timestamp;}

};
using H5_vts=H5_vision_traffic_signs;
#endif