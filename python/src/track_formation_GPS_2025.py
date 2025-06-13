import os
import matplotlib
import numpy as np
import pandas as pd
from util import latlon_to_utm
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline
matplotlib.use('Agg')
from geopy.distance import geodesic

def transform_gnss_data():
    gnss_data = np.loadtxt('../ouster/go/gnss_go.txt', skiprows=1)
    result = np.apply_along_axis(lambda row: latlon_to_utm(row[1], row[2]), axis=1, arr=gnss_data)

    result = np.array(result)
    gnss_data[:, 1] = result[:, 1] # lat - northing
    gnss_data[:, 2] = result[:, 0] # lon - easting

    return gnss_data


original_gps_data = transform_gnss_data()




def track_gps(K, filt_size, filt_size_z, smoothing_param, smoothing_param_z):
    # Identification of Livox start
    lidar_points = np.loadtxt('../ouster/go/lidar_go.txt', skiprows=1, dtype=np.float64)

    timestamp_gnss = original_gps_data[:, 0] * 1e6
    timestamp_lidar = lidar_points[:, 0] * 1e6

    livox_start = 0
    while timestamp_gnss[livox_start] < timestamp_lidar[0]:
        livox_start += 1

    livox_end = livox_start
    try:
        while timestamp_gnss[livox_end] < timestamp_lidar[-1]:
            livox_end += 1
    except IndexError:
        livox_end = len(timestamp_gnss) - 1
    

    treated_data_gps = np.zeros((493, 13))
    treated_data_gps[:, 0] = timestamp_gnss[(livox_start - 1) - int(max(np.ceil(K / 2), np.ceil(filt_size / 2))): livox_end + int(max(np.ceil(K / 2), np.ceil(filt_size / 2))) + 1]


    data = original_gps_data[(livox_start - 1) - int(max(np.ceil(K / 2), np.ceil(filt_size / 2))): livox_end + int(max(np.ceil(K / 2), np.ceil(filt_size / 2))) + 1, :]
    data_size = data.shape[0]
    
    # Filter creation
    filt   = np.ones(filt_size) / filt_size
    filt_z = np.ones(filt_size_z) / filt_size_z

    gps_x_filt = np.convolve(data[:, 1], filt, 'same')
    gps_y_filt = np.convolve(data[:, 2], filt, 'same')
    gps_z_filt = np.convolve(data[:, 3], filt_z, 'same')

    if smoothing_param != 1:
        spline_x = UnivariateSpline(data[:, 0], gps_x_filt, s=smoothing_param)
        spline_y = UnivariateSpline(data[:, 0], gps_y_filt, s=smoothing_param)

        smoothed_x = spline_x(data[:, 0])
        smoothed_y = spline_y(data[:, 0])

        treated_data_gps[:, 1] = smoothed_x
        treated_data_gps[:, 2] = smoothed_y
    else:
        treated_data_gps[:, 1] = gps_x_filt
        treated_data_gps[:, 2] = gps_y_filt

    if smoothing_param_z != 1:
        spline_z = UnivariateSpline(data[:, 0], gps_z_filt, s=smoothing_param_z)
        
        smoothed_z = spline_z(data[:, 0])
        
        treated_data_gps[:, 3] = smoothed_z
    else:
        treated_data_gps[:, 3] = gps_z_filt

    # GPS curve representation (if it's not a straight track, this version is not valid)

    fig, axes = plt.subplots(1, 2, figsize=(10, 5))

    start = int(max(np.ceil(K / 2), np.ceil(filt_size / 2))) # Probar d'agafar el primer
    end = data.shape[0]

    axes[0].plot(
        data[start - 1: end - start, 1],
        data[start - 1: end - start, 2],
        '.'
    )

    axes[0].plot(
        treated_data_gps[start - 1: end - start, 1],
        treated_data_gps[start - 1: end - start, 2],
    )

    axes[0].set_title('(x, y) track')

    axes[1].plot(
        data[start - 1: end - start, 3]
    )
    
    axes[1].plot(
        treated_data_gps[start - 1: end - start, 3]
    )

    axes[1].set_title('z track')

    os.makedirs('./figures', exist_ok=True)
    plt.savefig('./figures/gps_tracks.png')

    ## Rotation angles computation (rotation matrix)

    # Canviar treated_data_gps, 1, 2 i 3 que no estigui a valor filtre

    start = 1

    a = original_gps_data[end - 1, 1] - original_gps_data[livox_start, 1]  # ΔY (Nort-Sur)
    b = original_gps_data[end - 1, 2] - original_gps_data[livox_start, 2]  # ΔX (Est-Oest)
    c = original_gps_data[end - 1, 3] - original_gps_data[livox_start, 3]  # ΔZ (altitud)


    horizontal_dist = np.hypot(b, a)  

    yaw = np.degrees(np.arctan2(b, a))
    yaw = (yaw + 360) % 360

    pitch = np.degrees(np.arctan2(c, horizontal_dist))

    treated_data_gps[:, 8] = yaw
    treated_data_gps[:, 9] = pitch

    treated_data_gps[data_size - 1, 4:10] = treated_data_gps[data_size - 2, 4:10]
    treated_data_gps[:, 10:] = data[:, 1:4]

    track = np.zeros((end - start * 2 + 1, 9))
    track = treated_data_gps[start - 1: end - start, [0, 1, 2, 3, 8, 9, 10, 11, 12]]

    out_file = f'gnss_livox_tractat.txt'
    with open(out_file, 'w') as file:
        file.write('Hora GPS\tXgps_filtrat\tYgps_filtrat\tZgps_filtrat\t\\theta\t\\phi\tXgps_original\tYgps_original\tZgps_original\n')
        
        for row in treated_data_gps[start - 1: end - start, [0, 1, 2, 3, 8, 9, 10, 11, 12]]:
            file.write(
                '{:.10f}\t{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\n'.format(
                    row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7], row[8]
                )
            )
    return track


track_gps(15, 15, 15, 1, 1)