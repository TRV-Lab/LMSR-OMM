#ifndef _UTILS_H
#define _UTILS_H
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <string.h>
#include <vector>
#include <time.h>
#include <H5Cpp.h>
#include <cassert>
#include <numeric>
#include <algorithm>
#include <sys/resource.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/crop_box.h>
#include "nlohmann/json.hpp"
using json=nlohmann::json;
// #include <fcntl.h>
// Earth radius in meter
constexpr double EARTH_RADIUS{6378137.0};
constexpr double LONLAT2METER=EARTH_RADIUS*M_PI/180;

void GetFileNames(std::string path,std::vector<std::string>& filenames);
void GetFileNames(std::string path,std::string sub_name,std::vector<std::string>& filenames);

void mapMatch(const std::string sdmap_path);

#endif