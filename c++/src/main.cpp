#include "include/gnss.hpp"
#include "include/lidar.hpp"
#include <string>
#include <vector>
#include <filesystem>
#include <iostream>
#include <chrono>

namespace {

struct Args {
    std::string lidar_model;
    std::string track_type;
    std::string data_dir = "../..";
    std::string out_dir  = "../../results/c++/pointcloud";
};

bool ParseArgs(int argc, char** argv, Args& args) {
    std::vector<std::string> positional;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--data-dir" || arg == "--out-dir") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << arg << std::endl;
                return false;
            }
            if (arg == "--data-dir") args.data_dir = argv[++i];
            else                     args.out_dir  = argv[++i];
        } else {
            positional.push_back(arg);
        }
    }

    if (positional.size() != 2) {
        std::cerr << "Lidar model and track type are required as arguments.\n"
                   << "Usage: ./main <lidar_model> <track_type> [--data-dir DIR] [--out-dir DIR]" << std::endl;
        return false;
    }

    args.lidar_model = positional[0];
    args.track_type  = positional[1];
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    auto start = std::chrono::high_resolution_clock::now();

    Args args;
    if (!ParseArgs(argc, argv, args)) {
        return EXIT_FAILURE;
    }

    const std::string& lidar_model = args.lidar_model;
    const std::string& track_type  = args.track_type;

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

    const std::string gnss_data_path    = args.data_dir + "/" + lidar_model + "/" + track_type + "/gnss_" + track_type + ".txt";
    const std::string imu_data_path     = args.data_dir + "/" + lidar_model + "/" + track_type + "/imu_" + track_type + ".txt";
    const std::string lidar_points_path = args.data_dir + "/" + lidar_model + "/" + track_type + "/lidar_" + track_type + ".txt";

    std::filesystem::create_directories(args.out_dir);

    ProcessLidarPoints(lidar_points_path, gnss_data_path, imu_data_path, args.out_dir);

    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration = end - start;

    std::cout << "Duration of the program = " << duration.count() << " seconds\n";

    return EXIT_SUCCESS;
}
