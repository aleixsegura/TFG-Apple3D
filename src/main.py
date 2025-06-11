import sys
import time
import logging
import numpy as np
from util import latlon_to_utm
from pathlib import Path
from numpy.typing import NDArray
from scipy.interpolate import interp1d
from config import DIST_MAX, LIDAR_GNSS_OFFSETS, LIVOX_RAW_FRAME_SIZE, OUSTER_RAW_FRAME_SIZE


LIDAR_MODEL       = sys.argv[1].lower()
TRACK_TYPE        = sys.argv[2].lower()
FRAME_SIZE        = LIVOX_RAW_FRAME_SIZE if LIDAR_MODEL == 'livox' else OUSTER_RAW_FRAME_SIZE


logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    filename='lidar_processing.log')
logger = logging.getLogger(__name__)


def transform_gnss_data(lidar_model: str, track_type: str) -> NDArray[np.float64]:
    try:
        gnss_data = np.loadtxt(f'../{lidar_model}/{track_type}/gnss_{track_type}.txt', skiprows=1, dtype=np.float64)
        
        result = np.apply_along_axis(lambda row: latlon_to_utm(row[1], row[2]), axis=1, arr=gnss_data)  # (lat, lon) ===> easting northing
        result = np.array(result)

        gnss_data[:, 1] = result[:, 1]  # Latitude -> Northing
        gnss_data[:, 2] = result[:, 0]  # Longitude -> Easting

        if not is_strictly_increasing(gnss_data[:, 0]):
            logger.warning('GNSS timestamps are not strictly increasing')
            gnss_data = gnss_data[np.argsort(gnss_data[:, 0])]
            
        logger.info(f'Loaded {len(gnss_data)} GNSS points')
        return gnss_data
    except Exception as e:
        logger.error(f'Error transforming GNSS data: {e}')
        raise


def transform_lidar_points(lidar_model: str, track_type: str, gnss_data: NDArray[np.float64]) -> NDArray[np.float64]:
    try:
        lidar_points = np.loadtxt(f'../{lidar_model}/{track_type}/lidar_{track_type}.txt', skiprows=1, dtype=np.float64)
        lidar_points = lidar_points[(lidar_points[:, 0] <= gnss_data[-1, 0]) & 
                                   (lidar_points[:, 0] >= gnss_data[0, 0])]
        
        
        num_frames = len(lidar_points) // FRAME_SIZE
        remainder = len(lidar_points) % FRAME_SIZE
        
        lidar_frames = []
        
        for i in range(num_frames):
            frame = lidar_points[i * FRAME_SIZE : (i + 1) * FRAME_SIZE]
            lidar_frames.append(frame)
        
        if remainder > 0:
            frame = lidar_points[-remainder:]
            lidar_frames.append(frame)
        
        lidar_frames = np.array(lidar_frames, dtype=object)
        logger.info(f'Split LiDAR data into {len(lidar_frames)} frames')
        return lidar_frames
    except Exception as e:
        logger.error(f'Error transforming LiDAR points: {e}')
        raise


def is_strictly_increasing(arr: NDArray[np.float64]) -> bool:
    return np.all(np.diff(arr) > 0)


def transform_points(lidar_points: NDArray[np.float64], imu_data: NDArray[np.float64], timestamps_seconds: NDArray[np.float64], gnss_x, gnss_y, gnss_z):    
    imu_timestamps = imu_data[:, 0]
    
    pitch_interp = interp1d(imu_timestamps, imu_data[:, 2], 
                            bounds_error=False, fill_value='extrapolate')
    yaw_interp   = interp1d(imu_timestamps, imu_data[:, 3], 
                            bounds_error=False, fill_value='extrapolate')
    
    pitch = pitch_interp(timestamps_seconds)
    yaw   = yaw_interp(timestamps_seconds)

    cos_yaw = np.cos(np.radians(yaw))
    sin_yaw = np.sin(np.radians(yaw))
    cos_pitch = np.cos(np.radians(pitch))
    sin_pitch = np.sin(np.radians(pitch))

    x = lidar_points[:, 1]
    y = lidar_points[:, 2]
    z = lidar_points[:, 3]

    transformed_points = lidar_points.copy()
    
    transformed_points[:, 1] = (x * cos_yaw * cos_pitch - y * sin_yaw + z * cos_yaw * sin_pitch) + gnss_x
    transformed_points[:, 2] = (x * sin_yaw * cos_pitch + y * cos_yaw + z * sin_yaw * sin_pitch) + gnss_y
    transformed_points[:, 3] = (-x * sin_pitch + z * cos_pitch) + gnss_z
    
    return transformed_points


def get_pointcloud_with_imu(frame_idx: int, lidar_frame: NDArray[np.float64], imu_data: NDArray[np.float64], gnss_data: NDArray[np.float64]):
    start_time = time.time()
    try:

        valid_reads = np.linalg.norm(lidar_frame[:, 1:4], axis=1) < DIST_MAX

        filtered_lidar_frame_data = lidar_frame[valid_reads, :]
        logger.info(f'Frame {frame_idx}: {len(filtered_lidar_frame_data)}/{len(lidar_frame)} valid points')

        lidar_timestamps_seconds = filtered_lidar_frame_data[:, 0]
        gnss_timestamps_seconds  = gnss_data[:, 0]

        filtered_lidar_frame_data[:, 1:4] = filtered_lidar_frame_data[:, 1:4] + LIDAR_GNSS_OFFSETS[LIDAR_MODEL]


        interp_x = interp1d(gnss_timestamps_seconds, gnss_data[:, 1], bounds_error=False, fill_value='extrapolate')
        interp_y = interp1d(gnss_timestamps_seconds, gnss_data[:, 2], bounds_error=False, fill_value='extrapolate')
        interp_z = interp1d(gnss_timestamps_seconds, gnss_data[:, 3], bounds_error=False, fill_value='extrapolate')
        
        gnss_x = interp_x(lidar_timestamps_seconds)
        gnss_y = interp_y(lidar_timestamps_seconds)
        gnss_z = interp_z(lidar_timestamps_seconds)

        rotated_lidar_data = transform_points(filtered_lidar_frame_data, imu_data, lidar_timestamps_seconds, gnss_x, gnss_y, gnss_z)


        frame_data = np.column_stack((
            lidar_timestamps_seconds,
            np.full(len(lidar_timestamps_seconds), frame_idx), 
            rotated_lidar_data[:, 2], # La Y del lidar està a l'esquerra per tant easting
            rotated_lidar_data[:, 1], # La X del lidar va cap endavant per tant northing 
            rotated_lidar_data[:, 3],
            rotated_lidar_data[:, 4]
        ))

        with open(f'../results/pointcloud/{LIDAR_MODEL}/{TRACK_TYPE}/point_cloud_{frame_idx}.txt', 'w') as pcf:
            pcf.write(f'{"Timestamp (s)".center(10)}\t{"Frame num.".center(10)}\t{"Easting (X)".center(10)}\t{"Northing (Y)".center(30)}\t'
                    f'{"Altitude (Z)".center(10)}\t{"Intensity".center(15)}\r\n'
            )
            for row in frame_data:
                pcf.write(
                    f'{row[0]}\t{int(row[1])}\t{row[2]}\t{row[3]}\t{row[4]}\t{row[5]}\r\n'
                )
        
        processing_time = time.time() - start_time
        logger.info(f'Processed frame {frame_idx} in {processing_time:.2f} seconds')
        
        return frame_data
        
    except Exception as e:
        logger.error(f'Error processing frame {frame_idx}: {e}')
        return None


def main():
    try:
        np.set_printoptions(suppress=True)

        result_dir = Path(f'../results/pointcloud/{LIDAR_MODEL}/{TRACK_TYPE}')
        result_dir.mkdir(parents=True, exist_ok=True)
        
        logger.info(f'Created directory: {result_dir}')

        logger.info('Loading sensor data...')
        
        imu_data     = np.loadtxt(f'../{LIDAR_MODEL}/{TRACK_TYPE}/imu_{TRACK_TYPE}.txt', skiprows=1)
        gnss_data    = transform_gnss_data(LIDAR_MODEL, TRACK_TYPE)            
        lidar_frames = transform_lidar_points(LIDAR_MODEL, TRACK_TYPE, gnss_data)

        
        logger.info(f'IMU time range: {imu_data[0, 0]} to {imu_data[-1, 0]}')
        logger.info(f'LiDAR time range: {lidar_frames[0][0, 0]} to {lidar_frames[-1][-1, 0]}')
        logger.info(f'GNSS time range: {gnss_data[0, 0]} to {gnss_data[-1, 0]}')
        
        processed_frames = []
        
        for frame_idx, frame in enumerate(lidar_frames):
            logger.info(f'Processing frame {frame_idx + 1}/{len(lidar_frames)}')
            processed_frame = get_pointcloud_with_imu(frame_idx + 1, frame, imu_data, gnss_data)
            
            if processed_frame is not None:
                processed_frames.append(processed_frame)
        
        logger.info(f'Processed {len(processed_frames)}/{len(lidar_frames)} frames successfully')
        
    except Exception as e:
        logger.error(f'Error in main function: {e}')

if __name__ == '__main__':
    start = time.time()
    main()
    elapsed = time.time() - start
    logger.info(f'Total processing time: {elapsed:.2f} seconds')
