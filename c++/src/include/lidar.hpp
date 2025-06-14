#ifndef PROCESSING_HPP
#define PROCESSING_HPP

#include <string>


const float LIDAR_GNSS_OFFSET_X = -0.036;
const float LIDAR_GNSS_OFFSET_Y = 0.0;
const float LIDAR_GNSS_OFFSET_Z = 0.69;
const int LIDAR_POINT_DIST_MAX = 4;

struct LidarPoint {
    double timestamp;
    double x, y, z;
    double intensity;
};

struct TransformedPoint {
    double x, y, z;
    double intensity;
};

void ProcessLidarPoints(const std::string& filepath, const std::string& gnss_data_dir, const std::string& imu_data_dir, const std::string& outfile);

#endif
