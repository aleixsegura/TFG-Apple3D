#include "include/imu.hpp"
#include <cstdio>
#include <cstdlib>
#include <iostream>

std::vector<IMURead> ReadIMUData(const std::string& filepath) {
    std::vector<IMURead> imu_data;

    FILE* file = std::fopen(filepath.c_str(), "r");
    if (!file) {
        std::cerr << "Could not open the file" << "\n";
        return imu_data;
    }

    char buff[256];
    std::fgets(buff, sizeof(buff), file);

    double timestamp, roll, pitch, yaw, wx, wy, wz, ax, ay, az;

    while (std::fscanf(file, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &timestamp, &roll, &pitch, &yaw, &wx, &wy, &wz, &ax, &ay, &az) == 10) {
        IMURead p {timestamp, roll, pitch, yaw, wx, wy, wz, ax, ay, az};
        imu_data.push_back(p);
    }
    
    std::fclose(file);
    return imu_data;
}