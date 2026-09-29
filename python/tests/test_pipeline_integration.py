from pathlib import Path

import numpy as np

import main

DATA_DIR = Path(__file__).resolve().parent / 'data'
EXPECTED_OUTPUT = DATA_DIR / 'expected_point_cloud_1.txt'


def test_full_pipeline_matches_reference_output(tmp_path):
    lidar_model = 'ouster'
    track_type = 'go'

    imu_data = np.loadtxt(DATA_DIR / f'imu_{track_type}.txt', skiprows=1)
    gnss_data = main.transform_gnss_data(DATA_DIR, track_type)
    frame_size = main.frame_size_for(lidar_model)
    lidar_frames = main.transform_lidar_points(gnss_data, DATA_DIR, track_type, frame_size)

    assert len(lidar_frames) == 1

    result = main.get_pointcloud_with_imu(1, lidar_frames[0], imu_data, gnss_data, lidar_model, tmp_path)

    assert result is not None

    produced_output = tmp_path / 'point_cloud_1.txt'
    assert produced_output.read_text() == EXPECTED_OUTPUT.read_text()
