#ifndef _NIBBLER_H_
#define _NIBBLER_H_
#include "utils.h"

using namespace H5;
struct Pose
{
    double x;
    double y;
    double z;
};

class Cxx_nibbler
{
private:
    int length;
    std::vector<double> timestamps;
    std::vector<Pose> aligned_pose_cart;
    std::vector<Pose> aligned_pose_std;
    std::vector<Pose> aligned_pose_wgs84;
    std::vector<uint8_t> nibbler_valid; 
    double origin[2];
    std::vector<double> speed;
public:
    void setLength(int l) {length=l;}
    int getLength() const {return length;}
    std::vector<double>& getTimestamps() {return timestamps;}
    std::vector<Pose>& getPoseCart(){return aligned_pose_cart;}
    std::vector<Pose>& getPoseStd() {return aligned_pose_std;}
    std::vector<Pose>& getPoseWgs84(){return aligned_pose_wgs84;}
    std::vector<uint8_t>& getValid(){return nibbler_valid; }
    double* getOrigin(){return origin;}
    std::vector<double>& getSpeed(){return speed;}
    void print() const;


};
class H5_nibbler
{
private:
    DataSet aligned_pose_cart;
    DataSet aligned_pose_std;
    DataSet aligned_pose_wgs84;
    DataSet nibbler_valid;
    DataSet timestamps;
    DataSet origin;
    DataSet speed;
    H5File nibbler;
    void parsePoseCartTo(Cxx_nibbler &) const;
    void parsePoseStdTo(Cxx_nibbler &) const;
    void parsePoseWgs84To(Cxx_nibbler &) const;
    void parseValidTo(Cxx_nibbler &) const;
    void parseTimestampsTo(Cxx_nibbler &) const;
    void parseOriginTo(Cxx_nibbler &) const;
    void parseSpeedTo(Cxx_nibbler &) const;

public:
    void getDataSet(H5File&);
    void parseTo(Cxx_nibbler &) const;
    DataSet getPoseCart()const {return aligned_pose_cart;}
    DataSet getPoseStd()const {return aligned_pose_std;}
    DataSet getPoseWgs84()const {return aligned_pose_wgs84;}
    DataSet getValid()const {return nibbler_valid;}
    DataSet getTimestamps()const {return timestamps;}
    DataSet getOrigin()const {return origin;}
    DataSet getSpeed()const {return speed;}
    void split(std::string,std::string,size_t,size_t)const;

};
#endif