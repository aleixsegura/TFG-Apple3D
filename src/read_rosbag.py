import os
import sys
import rosbag
import numpy as np
from numpy.typing import NDArray
from dataclasses import dataclass
from scipy.spatial.transform import Rotation
from correct_imu_yaw import compute_offset
import sensor_msgs.point_cloud2 as pc2


LIDAR_TYPE = sys.argv[1]
TRACK_TYPE = sys.argv[2]
BAG_FILE   = f'{LIDAR_TYPE}_{TRACK_TYPE}.bag'

LIVOX_TOPIC   = '/livox/lidar'
OUSTER_TOPIC  = '/ouster/points'
YAW_OFFSET = compute_offset(BAG_FILE)


lidar_points = []

@dataclass
class IMURead:
    timestamp: np.float32
    orientation: NDArray[np.float32]
    angular_velocity: NDArray[np.float32]
    linear_acceleration: NDArray[np.float32]


@dataclass
class GNSSRead:
    timestamp: np.float32
    latitude: np.float32
    longitude: np.float32
    altitude: np.float32


def builddirs():
    dirs = ['./go/', './back/', './loop/']
    for dir in dirs:
        if not os.path.exists(dir):
            os.mkdir(dir)


def reset_files():
    reset_move_files(TRACK_TYPE)


def reset_move_files(move: str):
    with open(f'./{move}/imu_{move}.txt', 'w') as f:
        f.write(f'{"timestamp".center(15)}\t'
                f'{"Roll(degrees)".center(20)}\t{"Pitch(degrees)".center(20)}\t{"Yaw(degrees)".center(10)}\t'
                f'{"wx".center(20)}\t{"wy".center(20)}\t{"wz".center(20)}\t'
                f'{"ax".center(20)}\t{"ay".center(20)}\t{"az".center(20)}\n'
        )
    
    with open(f'./{move}/gnss_{move}.txt', 'w') as f:
        f.write(f'{"timestamp".center(15)}\t'
                f'{"latitude".center(25)}\t{"longitude".center(20)}\t{"altitude".center(10)}\n'
        )

    with open(f'./{move}/lidar_{move}.txt', 'w') as f:
        f.write(f'{"timestamp".center(20)}\t'
                f'{"x".center(16)}\t{"y".center(10)}\t{"z".center(10)}\t{"intensity"}\n'
        )


def process_lidar(msg, t, move):
    
    for p in pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True):
        x, y, z, intensity = p
        if is_valid_point([x, y, z, intensity]) and is_valid_coordinate([x, y, z]):
            seconds = t.secs + t.nsecs * 1e-9
            lidar_points.append([seconds, x, y, z, intensity])

    if len(lidar_points) >= 10000:
        write_lidar_batch(lidar_points, move)
        lidar_points.clear()


def is_valid_point(point):
    return all(np.isfinite(value) for value in point) and point[3] != 0


def is_valid_coordinate(coord):
    return any(component != 0 for component in coord)


def write_lidar_batch(points, move: str):
    lidar_points = np.array(points, dtype=np.float64)
    with open(f'./{move}/lidar_{move}.txt', 'a') as f:
        np.savetxt(f, lidar_points, fmt='%.10f', delimiter='\t')


def process_imu(msg, t, move, lidar_type):
    seconds = t.secs + t.nsecs * 1e-9
    
    quaternion = Rotation.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    rpy = quaternion.as_euler('xyz', degrees=True)

    # correct IMU
    print('computed offset: ', YAW_OFFSET)

    if lidar_type == 'livox' or lidar_type == 'ouster':
        rpy[2] = (-rpy[2] - YAW_OFFSET) % 360
    
    angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], 
                                dtype=np.float32)

    linear_acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                                    dtype=np.float32) 
    
    imu_read = IMURead(seconds, rpy, angular_velocity, linear_acceleration)
    write_imu(imu_read, move)
    

def write_imu(imu_read: IMURead, move: str):
    r, p, y = imu_read.orientation
    wx, wy, wz = imu_read.angular_velocity
    ax, ay, az = imu_read.linear_acceleration
    
    with open(f'./{move}/imu_{move}.txt', 'a') as f:
        f.write(
            f'{imu_read.timestamp:.10f}\t{r:.20f}\t{p:.20f}\t{y:.20f}\t'
            f'{wx:.20f}\t{wy:.20f}\t{wz:.20f}\t'
            f'{ax:.20f}\t{ay:.20f}\t{az:.20f}\n'
        )


def process_gnss(msg, t, move):
    seconds = t.secs + t.nsecs * 1e-9
    gnss_read = GNSSRead(seconds, msg.latitude, msg.longitude, msg.altitude)
    write_gnss(gnss_read, move)


def write_gnss(gnss_read: GNSSRead, move: str):
    with open(f'./{move}/gnss_{move}.txt', 'a') as f:
        f.write(f'{gnss_read.timestamp:.10f}\t'
                f'{gnss_read.latitude:.20f}\t{gnss_read.longitude:.20f}\t{gnss_read.altitude:.20f}\n')


def process_bag(lidar_type, track_type: str):
    bag = rosbag.Bag(BAG_FILE)
    
    for topic, msg, t in bag.read_messages():
        if topic == OUSTER_TOPIC or topic == LIVOX_TOPIC:
            process_lidar(msg, t, track_type)
        elif topic == '/imu_correct':
            process_imu(msg, t, track_type, lidar_type)
        elif topic == '/gps/fix':
            process_gnss(msg, t, track_type)
    bag.close()

    if lidar_points:
        write_lidar_batch(lidar_points, track_type)
    

def main():
    builddirs()
    reset_files()
    process_bag(LIDAR_TYPE, TRACK_TYPE)


if __name__ == '__main__':
    main()