#ifndef _GCJ02_WGS84_HPP_
#define _GCJ02_WGS84_HPP_
#include <iostream>
#include <cmath>

// 定义常量
const double ellipse_a = 6378245.0;
const double eccentricity2 = 0.00669342162296594323;

// 转换经度的辅助函数
double transform_lon(double d_lat, double d_lon) {
    return 300.0 + d_lon + 2.0 * d_lat + 0.1 * d_lon * d_lon + 0.1 * d_lon * d_lat + 0.1 * std::sqrt(std::abs(d_lon)) +
           (2.0 / 3.0) * (
               20.0 * std::sin(6.0 * M_PI * d_lon) +
               20.0 * std::sin(2.0 * M_PI * d_lon) +
               20.0 * std::sin(M_PI * d_lon) +
               40.0 * std::sin(M_PI * d_lon / 3.0) +
               150.0 * std::sin(M_PI * d_lon / 12.0) +
               300.0 * std::sin(M_PI * d_lon / 30.0)
           );
}

// 转换纬度的辅助函数
double transform_lat(double d_lat, double d_lon) {
    return -100.0 + 2.0 * d_lon + 3.0 * d_lat + 0.2 * d_lat * d_lat + 0.1 * d_lon * d_lat + 0.2 * std::sqrt(std::abs(d_lon)) +
           (2.0 / 3.0) * (
               20.0 * std::sin(6.0 * M_PI * d_lon) +
               20.0 * std::sin(2.0 * M_PI * d_lon) +
               20.0 * std::sin(M_PI * d_lat) +
               40.0 * std::sin(M_PI * d_lat / 3.0) +
               160.0 * std::sin(M_PI * d_lat / 12.0) +
               320.0 * std::sin(M_PI * d_lat / 30.0)
           );
}

// 转换函数
std::pair<double, double> transform(double lat, double lon) {
    double d_lat_deg = transform_lat(lat - 35.0, lon - 105.0);
    double d_lon_deg = transform_lon(lat - 35.0, lon - 105.0);
    double lat_rad = lat / 180.0 * M_PI;

    double sin_lat = std::sin(lat_rad);
    double cos_lat = std::cos(lat_rad);

    double magic = 1.0 - (eccentricity2 * sin_lat * sin_lat);
    double sqrt_magic = std::sqrt(magic);

    lat += 180.0 * d_lat_deg * magic * sqrt_magic / (M_PI * ellipse_a * (1.0 - eccentricity2));
    lon += 180.0 * d_lon_deg * sqrt_magic / (M_PI * ellipse_a * cos_lat);

    return std::make_pair(lat, lon);
}

// 从 GCJ-02 转换为 WGS-84
std::pair<double, double> gcj02_to_wgs84(double lat, double lon) {
    double tmp_lat = lat;
    double tmp_lon = lon;
    std::pair<double, double> new_crd = transform(lat, lon);
    lat = tmp_lat + tmp_lat - new_crd.first;
    lon = tmp_lon + tmp_lon - new_crd.second;
    return std::make_pair(lat, lon);
}

// int main() {
//     double lat = 30.0;  // 示例纬度
//     double lon = 120.0; // 示例经度
//     auto [wgs84_lat, wgs84_lon] = gcj02_to_wgs84(lat, lon);
//     std::cout << "GCJ-02: (" << lat << ", " << lon << ")" << std::endl;
//     std::cout << "WGS-84: (" << wgs84_lat << ", " << wgs84_lon << ")" << std::endl;
//     return 0;
// }
#endif