#ifndef IMU_HPP
#define IMU_HPP

#include <vector>
#include <string>

struct IMURead {
    double timestamp;
    double roll;
    double pitch;
    double yaw;
    double wx;
    double wy;
    double wz;
    double ax;
    double ay;
    double az;
};

std::vector<IMURead> ReadIMUData(const std::string& filepath);

#endif