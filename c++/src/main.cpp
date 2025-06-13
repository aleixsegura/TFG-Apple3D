#include "include/gnss.hpp"
#include "include/lidar.hpp"
#include <string>
#include <filesystem>
#include <iostream>
#include <chrono>

int main(int argc, char** argv) {
    auto start = std::chrono::high_resolution_clock::now();

    std::string lidar_model = argv[1];
    std::string track_type  = argv[2];

    const std::string gnss_data_path    = "../../" + lidar_model + "/" + track_type + "/gnss_" + track_type + ".txt";
    const std::string imu_data_path     = "../../" + lidar_model + "/" + track_type + "/imu_" + track_type + ".txt";
    const std::string lidar_points_path = "../../" + lidar_model + "/" + track_type + "/lidar_" + track_type + "txt";

    const std::string outfile_path = "../../results/c++/pointcloud/" + lidar_model + "_" + track_type + "_pointcloud.txt";
    
    ProcessLidarPoints(lidar_points_path, gnss_data_path, imu_data_path, outfile_path);
    
    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration = end - start;

    std::cout << "Duration of the program = " << duration.count() << "seconds\n";

    return EXIT_SUCCESS;
}