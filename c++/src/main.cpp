#include "include/gnss.hpp"
#include "include/lidar.hpp"
#include <string>
#include <filesystem>
#include <iostream>
#include <chrono>

int main(int argc, char** argv) {
    auto start = std::chrono::high_resolution_clock::now();
    
    if (argc != 3) {
        std::cerr << "Lidar model and track type are required as arguments.\nUsage: ./main <lidar_model> <track_type>" << std::endl;
        return EXIT_FAILURE;
    }

    std::string lidar_model = argv[1];
    std::string track_type  = argv[2];

    if (lidar_model != "ouster" && lidar_model != "livox") {
        std::cerr << "Invalid lidar model" << std::endl;
        return EXIT_FAILURE;
    }

    if (track_type != "go" &&
        track_type != "back" &&
        track_type != "loop") {
        std::cerr << "Invalid track type" << std::endl;
        return EXIT_FAILURE;
    }

    const std::string gnss_data_path    = "../../" + lidar_model + "/" + track_type + "/gnss_" + track_type + ".txt";
    const std::string imu_data_path     = "../../" + lidar_model + "/" + track_type + "/imu_" + track_type + ".txt";
    const std::string lidar_points_path = "../../" + lidar_model + "/" + track_type + "/lidar_" + track_type + ".txt";

    
    ProcessLidarPoints(lidar_points_path, gnss_data_path, imu_data_path);
    
    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration = end - start;

    std::cout << "Duration of the program = " << duration.count() << " seconds\n";

    return EXIT_SUCCESS;
}
