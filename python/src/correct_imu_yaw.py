import rosbag
import numpy as np
from scipy.spatial.transform import Rotation as R
from math import atan2, degrees, radians, sin, cos
import matplotlib.pyplot as plt
import os
import rospy


def extract_imu_yaw(bag_path, imu_topic, start_rostime=None, end_time_limit=None):

    bag = rosbag.Bag(bag_path)
    yaws = []

    message_count = 0
    first_timestamp = None
    last_timestamp = None
    for _, msg, t in bag.read_messages(topics=[imu_topic], start_time=start_rostime, end_time=end_time_limit):
        message_count += 1
        if first_timestamp is None:
            first_timestamp = t
        last_timestamp = t
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler = R.from_quat(quat).as_euler('xyz', degrees=True)
        yaws.append(euler[2])
    bag.close()

    if not yaws:
        print('No IMU messages found in this time interval.')
        return 0

    if first_timestamp and last_timestamp:
        print(
            f'IMU: {message_count} read messages beetween {first_timestamp.to_sec():.2f}s and {last_timestamp.to_sec():.2f}s')

    yaw_mean = np.mean(yaws)
    print(f'Mean Yaw (inside time interval): {yaw_mean:.2f}°')
    return yaw_mean

def extract_gnss_coordinates(bag_path, gnss_topic):

    bag = rosbag.Bag(bag_path)
    coords = []

    for _, msg, _ in bag.read_messages(topics=[gnss_topic]):
        coords.append((msg.latitude, msg.longitude))
    bag.close()

    return coords[0], coords[-1]


def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    delta_lon = lon2 - lon1

    x = sin(delta_lon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon)

    bearing = (degrees(atan2(x, y)) + 360) % 360

    return bearing


def compute_offset(bag: str):

    imu_topic = '/imu_correct'
    gnss_topic = '/gps/fix'
    limit_duration_sec = 70.0

    start_rostime = None
    end_time_limit = None


    filename = os.path.basename(bag)

    if filename.endswith('loop.bag'):
        print(f"File '{filename}' detected. Time limit of {limit_duration_sec} seconds will be applied.")
        try:
            with rosbag.Bag(bag) as temp_bag:
                start_time_sec = temp_bag.get_start_time()
                start_rostime = rospy.Time.from_sec(start_time_sec)
                end_time_limit = start_rostime + rospy.Duration(limit_duration_sec)
        except Exception as e:
            print(f"ERROR: Could not open file: {bag} to obtain start time: {e}")
            print('Entire file will be read.')
            start_rostime = None
            end_time_limit = None
    else:
        print(f"File: '{filename}' is not a 'loop'. Default time interval will be used.")


    yaw_mean = extract_imu_yaw(bag, imu_topic, start_rostime=start_rostime, end_time_limit=end_time_limit)

    coord_inici, coord_final = extract_gnss_coordinates(bag, gnss_topic)
    lat1, lon1 = coord_inici
    lat2, lon2 = coord_final

    gnss_bearing = calculate_bearing(lat1, lon1, lat2, lon2)

    yaw_mean_corrected = -yaw_mean

    offset = yaw_mean_corrected - gnss_bearing

    if offset > 180:
        offset -= 360
    elif offset < -180:
        offset += 360

    print(f'IMU corrected heading (Changed sign to use clockwise convention): {yaw_mean_corrected:.2f}°')
    print(f'GNSS heading (Nord=0°, clockwise): {gnss_bearing:.2f}°')
    print(f'Offset calculat (IMU_corregida - GNSS): {offset:.2f}°')

    yaw_aligned = yaw_mean_corrected - offset
    print(f'Yaw angle of the IMU aligned with GNSS (verification): {yaw_aligned:.2f}°')

    return round(offset, 2)


if __name__ == '__main__':
    compute_offset('ouster_loop.bag')
