#ifndef _HMM_H_
#define _HMM_H_
#include <bits/stdc++.h>
#include <sys/resource.h>

#define N 121380
#define pi 3.1415926535897932384626433832795
#define EARTH_RADIUS 6378.137 //地球半径 KM

struct Road
{
    int num, type;
    int start_node, end_node;
    std::vector<std::pair<double, double> > node;
};//路网
class HMM
{
public:
    double distance(std::pair<double, double> x, std::pair<double, double> a, std::pair<double, double> b );
    double road_min_distance(double x, double y, int num);
    std::vector<int> select_road(std::vector<std::pair<double, double> > &trace,int err[]);
    void get_projection(std::vector<std::pair<double, double> > &trace,std::vector<int> &chosen_road,std::vector<std::pair<double, double> > trace_projection[]);
    void calculate_A(std::vector<int> &chosen_road,std::vector<double> A[]);
    double calculate_sigma(std::vector<double> dst);
    double rad(double d);
    double RealDistance(double lat1,double lng1,double lat2,double lng2);//lat1第一个点纬度,lng1第一个点经度,lat2第二个点纬度,lng2第二个点经度
    double highest_speed(int p);
    double calculate_angle(std::vector<std::pair<double, double> > &trace,int p, int q);//第p条trace，第q条真路
    std::vector<double> calculate_emission_possibility(std::vector<std::pair<double, double> > &trace,std::vector<int> &chosen_road,std::vector<std::pair<double, double> > trace_projection[],std::vector<int> time_node,double x, double y, int p);
    void Viterbi(std::vector<std::pair<double, double> > &trace,std::vector<int> &chosen_road,std::vector<std::pair<double, double> > trace_projection[],std::vector<int> time_node,std::vector<double> A[]);
    HMM(){};
    static void load_road(std::string road_file,Road road[],std::vector<int> next_road[],std::vector<int> pre_road[]);
    void load_trace(std::ifstream& myfile, int trace_id);
    void map_match(std::vector<std::pair<double, double> > &trace,std::vector<int> &chosen_road,std::vector<std::pair<double, double> > trace_projection[],std::vector<int> time_node,std::vector<double> A[]);

private:
    int err[N];//路段是否被选择
    std::vector<int> chosen_road;
    std::vector<std::pair<double, double> > trace;//每条实时轨迹的坐标序列
    std::vector<int> time_node;
    std::vector<std::pair<double, double> > trace_projection[N];
    //D A[2000][2000];
    std::vector<double> A[N];//状态转移概率


};
#endif