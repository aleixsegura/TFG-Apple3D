#include "include/gnss.hpp"
#include "include/imu.hpp"
#include "include/lidar.hpp"
#include <thread>
#include <string>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

struct Quatd {
    double x, y, z, w;
};

// Interpolates using SLERP and two quarternions, this function is executed at GPU device
__device__ void quat_slerp(Quatd& result, const Quatd& q1, const Quatd& q2, double t) {
    double dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
    
    Quatd q2_corr = q2;
    if (dot < 0.0) {
        dot = -dot;
        q2_corr.x = -q2.x; q2_corr.y = -q2.y; q2_corr.z = -q2.z; q2_corr.w = -q2.w;
    }

    const double DOT_THRESHOLD = 0.9995;
    if (dot > DOT_THRESHOLD) {
        result.x = q1.x + t * (q2_corr.x - q1.x);
        result.y = q1.y + t * (q2_corr.y - q1.y);
        result.z = q1.z + t * (q2_corr.z - q1.z);
        result.w = q1.w + t * (q2_corr.w - q1.w);
    } else {
        double theta_0 = acos(dot);
        double theta = theta_0 * t;
        double sin_theta = sin(theta);
        double sin_theta_0 = sin(theta_0);
        double s0 = cos(theta) - dot * sin_theta / sin_theta_0;
        double s1 = sin_theta / sin_theta_0;
        
        result.x = (s0 * q1.x) + (s1 * q2_corr.x);
        result.y = (s0 * q1.y) + (s1 * q2_corr.y);
        result.z = (s0 * q1.z) + (s1 * q2_corr.z);
        result.w = (s0 * q1.w) + (s1 * q2_corr.w);
    }

    double mag = rsqrt(result.x * result.x + result.y * result.y + result.z * result.z + result.w * result.w);
    result.x *= mag; result.y *= mag; result.z *= mag; result.w *= mag;
}

// Converts a quarternion to Euler angles 
__device__ void toEulerAngles(const Quatd& q, double& pitch, double& yaw) {
    double m20 = 2.0 * (q.x * q.z - q.w * q.y);
    
    if (fabs(m20) >= 1.0) {
        pitch = copysign(M_PI / 2.0, -m20);
    } else {
        pitch = asin(-m20);
    }
    double m10 = 2.0 * (q.x * q.y + q.w * q.z);
    double m00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw = atan2(m10, m00);
}


/* 
    Main unified CUDA kernel executed on the GPU, that interpolates, generates the transformation matrix and transforms the LiDAR 
    points.
*/
__global__ void generateAndTransformPoints_kernel(
    const LidarPoint* d_input_points,
    TransformedPoint* d_output_points,
    const GNSSRead* d_gnss_data,
    const IMURead* d_imu_data,
    const int* d_gnss_indices,
    const int* d_imu_indices,
    int num_points
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    const LidarPoint& p = d_input_points[idx];
    double lidar_timestamp = p.timestamp;

    int gnss_idx = d_gnss_indices[idx];
    const GNSSRead& gnss1 = d_gnss_data[gnss_idx];
    const GNSSRead& gnss2 = d_gnss_data[gnss_idx + 1];

    double t_gnss = (lidar_timestamp - gnss1.timestamp) / (gnss2.timestamp - gnss1.timestamp);
    double easting = fma(t_gnss, (gnss2.easting - gnss1.easting), gnss1.easting);
    double northing = fma(t_gnss, (gnss2.northing - gnss1.northing), gnss1.northing);
    double altitude = fma(t_gnss, (gnss2.altitude - gnss1.altitude), gnss1.altitude);

    int imu_idx = d_imu_indices[idx];
    const IMURead& imu1 = d_imu_data[imu_idx];
    const IMURead& imu2 = d_imu_data[imu_idx + 1];
    
    double t_imu = (lidar_timestamp - imu1.timestamp) / (imu2.timestamp - imu1.timestamp);

    double deg2rad = M_PI / 180.0;
    double yaw1_rad = imu1.yaw * deg2rad;
    double pitch1_rad = imu1.pitch * deg2rad;
    double yaw2_rad = imu2.yaw * deg2rad;
    double pitch2_rad = imu2.pitch * deg2rad;

    double cy1 = cos(yaw1_rad * 0.5), sy1 = sin(yaw1_rad * 0.5);
    double cp1 = cos(pitch1_rad * 0.5), sp1 = sin(pitch1_rad * 0.5);
    Quatd q1 = { -sy1 * sp1, cy1 * sp1, sy1 * cp1, cy1 * cp1 };

    double cy2 = cos(yaw2_rad * 0.5), sy2 = sin(yaw2_rad * 0.5);
    double cp2 = cos(pitch2_rad * 0.5), sp2 = sin(pitch2_rad * 0.5);
    Quatd q2 = { -sy2 * sp2, cy2 * sp2, sy2 * cp2, cy2 * cp2 };

    Quatd q_interp;
    quat_slerp(q_interp, q1, q2, t_imu);

    double pitch_interp_rad, yaw_interp_rad;
    toEulerAngles(q_interp, pitch_interp_rad, yaw_interp_rad);

    double T[12];
    double cos_pitch = cos(pitch_interp_rad);
    double sin_pitch = sin(pitch_interp_rad);
    double cos_yaw = cos(yaw_interp_rad);
    double sin_yaw = sin(yaw_interp_rad);
    
    T[0] = cos_yaw * cos_pitch; T[1] = -sin_yaw; T[2] = cos_yaw * sin_pitch;  T[3] = northing;
    T[4] = sin_yaw * cos_pitch; T[5] = cos_yaw;  T[6] = sin_yaw * sin_pitch;  T[7] = easting;
    T[8] = -sin_pitch;          T[9] = 0;       T[10] = cos_pitch;            T[11] = altitude;

    double x = p.x, y = p.y, z = p.z;
    d_output_points[idx].x = T[0] * x + T[1] * y + T[2] * z + T[3];
    d_output_points[idx].y = T[4] * x + T[5] * y + T[6] * z + T[7];
    d_output_points[idx].z = T[8] * x + T[9] * y + T[10] * z + T[11];
    d_output_points[idx].intensity = p.intensity;
}


double norm(const double& x, const double& y, const double& z) {
    return std::sqrt((x * x + y * y + z * z));
}


void ExportTransformedPointsParallel(const std::vector<TransformedPoint> points, const std::string& out_filepath) {
    std::ofstream outfile(out_filepath);
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not create output file: " << out_filepath << "\n";
        return;
    }

    std::ostringstream oss;
    oss << "x y z intensity\n";
    oss << std::fixed << std::setprecision(6);

    for (const auto& p: points) {
        oss << p.y << " " << p.x << " " << p.z << " " << p.intensity << "\n";
    }

    outfile << oss.str();
    outfile.close();
}


void ProcessLidarPoints(const std::string& filepath, const std::string& gnss_data_dir, const std::string& imu_data_dir) {
    
    std::vector<GNSSRead> gnss_data = ReadGNSSData(gnss_data_dir);
    std::vector<IMURead> imu_data = ReadIMUData(imu_data_dir); 

    if (gnss_data.size() < 2 || imu_data.size() < 2) {
        std::cerr << "No sufficient GNSS or IMU data to do interpolation.\n";
        return;
    }

    int fd = open(filepath.c_str(), O_RDONLY);
    if (fd == -1) {
        std::cerr << "Could not open LiDAR file: " << filepath << "\n";
        return;
    }

    struct stat sb;
    if (fstat(fd, &sb) == -1) {
        close(fd);
        return;
    }

    const char* file_data = static_cast<const char*>(mmap(nullptr, sb.st_size, PROT_READ, MAP_PRIVATE, fd, 0));
    if (file_data == MAP_FAILED) {
        close(fd);
        return;
    }

    const double start_gnss = gnss_data[0].timestamp;
    const double end_gnss = gnss_data.back().timestamp;


    std::vector<LidarPoint> lidar_points;
    lidar_points.reserve(75000000);

    auto start_read_lidar = std::chrono::high_resolution_clock::now();

    const char* ptr = file_data;
    const char* end = file_data + sb.st_size;
    
    while (ptr < end && *ptr != '\n') ptr++;
    if (ptr < end) ptr++;

    while (ptr < end) {
        double timestamp, x, y, z, intensity;
        
        char* next_ptr;
        timestamp = strtod(ptr, &next_ptr);
        if (next_ptr == ptr) break;
        ptr = next_ptr;
        
        x = strtod(ptr, &next_ptr);
        if (next_ptr == ptr) break;
        ptr = next_ptr;
        
        y = strtod(ptr, &next_ptr);
        if (next_ptr == ptr) break;
        ptr = next_ptr;
        
        z = strtod(ptr, &next_ptr);
        if (next_ptr == ptr) break;
        ptr = next_ptr;
        
        intensity = strtod(ptr, &next_ptr);
        if (next_ptr == ptr) break;
        ptr = next_ptr;
        
        while (ptr < end && *ptr != '\n') ptr++;
        if (ptr < end) ptr++;
        
        if (timestamp >= start_gnss && timestamp <= end_gnss && norm(x, y, z) < LIDAR_POINT_DIST_MAX) {
            lidar_points.push_back({timestamp, x + LIDAR_GNSS_OFFSET_X, y + LIDAR_GNSS_OFFSET_Y, z + LIDAR_GNSS_OFFSET_Z, intensity});
        }
    }
    
    auto end_read_lidar = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration_read_lidar = end_read_lidar - start_read_lidar;

    std::cout << "Duration of reading lidar points file = " << duration_read_lidar.count() << " seconds\n";
    
    std::vector<int> gnss_indices(lidar_points.size());
    std::vector<int> imu_indices(lidar_points.size());

    size_t gnss_search_idx = 0;
    size_t imu_search_idx = 0;

    for (size_t i = 0; i < lidar_points.size(); ++i) {
        double p_ts = lidar_points[i].timestamp;
        
        while (gnss_search_idx < gnss_data.size() - 2 && gnss_data[gnss_search_idx + 1].timestamp < p_ts) {
            gnss_search_idx++;
        }
        gnss_indices[i] = gnss_search_idx;

        while (imu_search_idx < imu_data.size() - 2 && imu_data[imu_search_idx + 1].timestamp < p_ts) {
            imu_search_idx++;
        }
        imu_indices[i] = imu_search_idx;
    }
    
    int num_points = lidar_points.size();
    
    LidarPoint* d_input_points;
    TransformedPoint* d_output_points;
    GNSSRead* d_gnss_data;
    IMURead* d_imu_data;
    int* d_gnss_indices;
    int* d_imu_indices;

    cudaMalloc(&d_input_points, num_points * sizeof(LidarPoint));
    cudaMalloc(&d_output_points, num_points * sizeof(TransformedPoint));
    cudaMalloc(&d_gnss_data, gnss_data.size() * sizeof(GNSSRead));
    cudaMalloc(&d_imu_data, imu_data.size() * sizeof(IMURead));
    cudaMalloc(&d_gnss_indices, num_points * sizeof(int));
    cudaMalloc(&d_imu_indices, num_points * sizeof(int));
    
    cudaMemcpy(d_input_points, lidar_points.data(), num_points * sizeof(LidarPoint), cudaMemcpyHostToDevice);
    cudaMemcpy(d_gnss_data, gnss_data.data(), gnss_data.size() * sizeof(GNSSRead), cudaMemcpyHostToDevice);
    cudaMemcpy(d_imu_data, imu_data.data(), imu_data.size() * sizeof(IMURead), cudaMemcpyHostToDevice);
    cudaMemcpy(d_gnss_indices, gnss_indices.data(), num_points * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_imu_indices, imu_indices.data(), num_points * sizeof(int), cudaMemcpyHostToDevice);
    
    int threadsPerBlock = 256;
    int blocks = (num_points + threadsPerBlock - 1) / threadsPerBlock;
    
    generateAndTransformPoints_kernel<<<blocks, threadsPerBlock>>>(
        d_input_points, d_output_points, d_gnss_data, d_imu_data,
        d_gnss_indices, d_imu_indices, num_points
    );
    
    cudaDeviceSynchronize();
    cudaError_t cuda_error = cudaGetLastError();
    if (cuda_error != cudaSuccess) {
        std::cerr << "Error in CUDA kernel: " << cudaGetErrorString(cuda_error) << "\n";
    }

    int total_threads = num_points / 1000000;
    int remainder = num_points % 1000000;
    
    if (remainder > 0) total_threads++;

    std::vector<std::thread> threads;

    auto start_write_output = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < total_threads; i++) {
        int start_idx = i * 1000000;

        int block_size = std::min(1000000, num_points - start_idx);

        std::vector<TransformedPoint> points(block_size);

        cudaMemcpy(points.data(), d_output_points + start_idx, block_size * sizeof(TransformedPoint), cudaMemcpyDeviceToHost);

        const std::string filename = "../../results/c++/pointcloud/pointcloud_" + std::to_string(i + 1) + ".txt";
        
        threads.emplace_back(ExportTransformedPointsParallel, std::move(points), filename);
    }

    for (auto& t: threads) {
        t.join();
    }

    auto end_write_output = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration = end_write_output - start_write_output;

    std::cout << "Duration of writing the transformed lidar points = " << duration.count() << "\n";

    cudaFree(d_input_points);
    cudaFree(d_output_points);
    cudaFree(d_gnss_data);
    cudaFree(d_imu_data);
    cudaFree(d_gnss_indices);
    cudaFree(d_imu_indices);
}
