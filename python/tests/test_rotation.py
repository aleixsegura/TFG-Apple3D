import numpy as np
import pytest

from main import transform_points


def _make_imu(pitch_deg: float, yaw_deg: float) -> np.ndarray:
    # timestamp, roll, pitch, yaw, wx, wy, wz, ax, ay, az
    return np.array([
        [0.0, 0.0, pitch_deg, yaw_deg, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [10.0, 0.0, pitch_deg, yaw_deg, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ])


def _make_lidar_points(xyz: np.ndarray) -> np.ndarray:
    # timestamp, x, y, z, intensity
    points = np.zeros((xyz.shape[0], 5))
    points[:, 0] = 1.0
    points[:, 1:4] = xyz
    points[:, 4] = 42.0
    return points


def test_null_angles_are_identity():
    imu_data = _make_imu(pitch_deg=0.0, yaw_deg=0.0)
    xyz = np.array([
        [1.0, 2.0, 3.0],
        [-4.0, 5.0, -6.0],
    ])
    lidar_points = _make_lidar_points(xyz)
    timestamps = lidar_points[:, 0]
    zeros = np.zeros(len(xyz))

    transformed = transform_points(lidar_points, imu_data, timestamps, zeros, zeros, zeros)

    np.testing.assert_allclose(transformed[:, 1:4], xyz, atol=1e-10)


def test_yaw_90_degrees_permutes_axes():
    imu_data = _make_imu(pitch_deg=0.0, yaw_deg=90.0)
    xyz = np.array([
        [1.0, 2.0, 3.0],
        [5.0, -1.0, 0.5],
    ])
    lidar_points = _make_lidar_points(xyz)
    timestamps = lidar_points[:, 0]
    zeros = np.zeros(len(xyz))

    transformed = transform_points(lidar_points, imu_data, timestamps, zeros, zeros, zeros)

    expected = np.column_stack((-xyz[:, 1], xyz[:, 0], xyz[:, 2]))
    np.testing.assert_allclose(transformed[:, 1:4], expected, atol=1e-10)


def test_gnss_offset_is_added():
    imu_data = _make_imu(pitch_deg=0.0, yaw_deg=0.0)
    xyz = np.array([[1.0, 2.0, 3.0]])
    lidar_points = _make_lidar_points(xyz)
    timestamps = lidar_points[:, 0]

    gnss_x = np.array([100.0])
    gnss_y = np.array([200.0])
    gnss_z = np.array([300.0])

    transformed = transform_points(lidar_points, imu_data, timestamps, gnss_x, gnss_y, gnss_z)

    # x_transformed uses gnss_y, y_transformed uses gnss_x (see main.transform_points).
    expected = np.array([[1.0 + 200.0, 2.0 + 100.0, 3.0 + 300.0]])
    np.testing.assert_allclose(transformed[:, 1:4], expected, atol=1e-10)
