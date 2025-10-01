# TFG (Treball de Fi de Grau) - 3D Point Cloud reconstruction of Apple trees using LiDAR, IMU and RTK-GNSS data

## Project Description

This project focuses on the reconstruction of 3D point clouds of apple tree rows using data acquired from a LiDAR sensor, an Inertial Measurement Unit (IMU), and a Real-Time Kinematic Global Navigation Satellite System (RTK-GNSS). The primary objective is to generate accurate and georeferenced 3D models of orchard environments to support applications in precision agriculture, such as canopy estimation and crop monitoring.

The system is designed for researchers, engineers, and practitioners working in agricultural technology and environmental monitoring. It addresses the challenge of fusing heterogeneous sensor data to produce precise spatial representations, especially in outdoor environments where traditional mapping methods are less effective.

By integrating LiDAR point clouds with orientation data from the IMU and positioning data from RTK-GNSS, the project implements a pipeline that the result is a georeferenced 3D reconstruction of apple trees.


## Data samples

The GNSS, IMU, and LiDAR data read by the main program were exported using the script [read_rosbag.py](python/src/read_rosbag.py).

To allow users to experiment with sample data, three files — `gnss_go.txt`, `imu_go.txt`, and `lidar_go.txt` — are available at the following link:

👉 [Download sample data (Dropbox)](https://www.dropbox.com/scl/fo/ybj3vl4eu7f0qm2v1rbvs/ALlHGTBi_BUeOnjpmA3d150?rlkey=yycn443kjopfqss8zi0wdi993&st=tejejo7t&dl=0)

While the GNSS and IMU files contain the full set of sensed data, the LiDAR file includes only a sample, representing approximately 20% of the total points captured.


## Cloning the project

To clone the project, run the following command:

```bash
$ git clone https://github.com/aleixsegura/TFG-Apple3D
```

## Python (main) version

The Python version of the program was implemented using Python 3.12.3 in a WSL Ubuntu 24.04.2 LTS environment. 

### Installing Dependencies

After cloning the project, the next step is to create a virtual environment. To do so, run:

```bash
$ python3 -m venv .env
```

After that activate the virtual environment and install the dependencies contained in [requirements.txt](python/requirements.txt):

```bash
$ source .env/bin/activate
$ pip install -r requirements.txt
```

### Running the Program

To run the main program, first create the following directory structure in the root of the project:

Place the downloaded sample data files (`gnss_go.txt`, `imu_go.txt`, `lidar_go.txt`) inside the `ouster/go/` directory.

Then, navigate to the `python/src/` directory and run:

```bash
$ python3 main.py ouster go
```

The point clouds will be saved as `.txt` files in the `results/python/pointcloud/ouster/go` directory, ready to be loaded into point cloud visualization and manipulation software such as CloudCompare.


## C++ Version
The C++ version of the program was implemented using C++17 in a WSL Ubuntu 24.04.2 LTS environment. It consists of CUDA-accelerated (and also CPU-accelerated) code to preprocess and transform LiDAR point clouds using GNSS and IMU data. It is built using CMake and requires a C++17-compatible compiler and a CUDA-capable GPU (tested on NVIDIA RTX 3070, Compute Capability 8.6).

### Installing Dependencies

To compile the C++ version, you must have the following installed:

- **CMake ≥ 3.10**
- **CUDA Toolkit** (compatible with your GPU)
- **A C++17 compiler** (e.g., `g++` ≥ 7 or `clang`)
- **PROJ library** (for coordinate transformations)

On Ubuntu/Debian, you can install the required system packages with:

```bash
$ sudo apt update
$ sudo apt install build-essential cmake pkg-config libproj-dev
```

To install CUDA, you can follow this tutorial:

👉 [CUDA Installation Tutorial by Aleksandar Haber, PhD](https://www.youtube.com/watch?v=sNhUK9IvR5E&ab_channel=AleksandarHaberPhD)

> **Note:** Depending on your GPU model, you may need to modify the CUDA architecture setting in `CMakeLists.txt` (the `CMAKE_CUDA_ARCHITECTURES` variable) and choose the appropriate CUDA Toolkit version to install.    
> Make sure to verify your GPU's Compute Capability and install the compatible CUDA version accordingly.


### Running the program

To run the main program, first create the following directory structure in the root of the project:

Place the downloaded sample data files (`gnss_go.txt`, `imu_go.txt`, `lidar_go.txt`) inside the `ouster/go/` directory.

Also, create the `results/c++/pointcloud/` directory.

Then, follow these steps from the root of the `c++` folder:

1. **Create a build directory and navigate into it:**

```bash
$ mkdir build
$ cd build
```

2. **Run CMake to configure the project:**

```bash
$ cmake ..
```

3. **Build the executable:**

```bash
$ make
```

4. **Run the program:**

```bash
$ ./main
```

The point clouds will be saved as `.txt` files in the `results/c++/pointcloud/` directory, ready to be loaded into point cloud visualization and manipulation software such as CloudCompare.

## Results

### Point clouds

After loading the point clouds in to CloudCompare the visualization should look like this:

<img src="https://www.dropbox.com/scl/fi/08ljtygfmabvccx10fxuj/ouster_go.png?rlkey=qf77oyrq3df9qw5pb5o3mia3p&st=d2b4azmr&raw=1" alt="OUSTGOPCL" width="400"/>


### Metrics

The tree's geometrical characteristics computed were:

- **AmpleMax:** Refers to the maximum width of the tree.
- **Ample95:** Refers to the width up to the 95th percentile of the tree. That is, points farther away (in width) from the main group of tree points are not considered.
- **AltMax:** Refers to the maximum height of the tree.
- **Alt95:** Refers to the height up to the 95th percentile of the tree. That is, points farther away (in height) from the main group of tree points are not considered.
- **S Chull:** Refers to the cross-sectional area of the convex hull of the points in the section.
- **S AmpleMax:** Refers to the cross-sectional area measured using the maximum height and maximum width.
- **S Ample95:** Refers to the cross-sectional area measured using the real distribution of points up to the 90th percentile of the tree. That is, points farther away (in width) from the main group of points are not considered.
- **S reticula005:** Refers to the cross-sectional area calculated using a 5cm × 5cm grid. It sums the area of those cells containing points.


The computed metrics with respect to the *ground truth* are: **MAE**, **MAPE**, **RMSE**, and **R²**.

<img src="https://www.dropbox.com/scl/fi/mxsaao2lxiybq0ythwxte/metriques_ouster.png?rlkey=522uf3hmdj0w64e7vdfc3lcry&st=s76umyim&raw=1" alt="GEOMCHARS" width="300"/>

### Python vs C++

After computing the execution times of the two versions, the results are the following:

<img src="https://www.dropbox.com/scl/fi/s74urbdywemv6je2rtgqv/tempstotal.png?rlkey=n86wk27ujbqz62p0dak7s8akf&st=jxk2hsoc&raw=1" alt="TOTALTIME" width="300"/>

<img src="https://www.dropbox.com/scl/fi/wejdwy56wxf9h4fzi6alp/tempsfases.png?rlkey=i39qy7hdydq10sbyo9t1wy9d7&st=a3zc58xl&raw=1" alt="PHASESTIME" width="300"/>

## Instituional Research

DIGIFRUIT is the research project TED2021-131871B-I00 funded by the Ministerio de Ciencia e Innovación within the 2021 call for strategic projects aimed at the Ecological and Digital Transition of the Plan Estatal de Investigación Científica, Técnica y de Innovación 2021-2023 (MCIN/AEI/ 10.13039/501100011033 and ERDF UE).

<img src="https://www.grap.udl.cat/export/sites/Grap/ca/.galleries/logos/LogoMICIN.png" alt="DIGIFRUIT" width="200"/>

