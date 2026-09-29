import numpy as np
from scipy.interpolate import interp1d

# The pipeline (main.transform_points / main.get_pointcloud_with_imu) uses
# scipy.interpolate.interp1d with bounds_error=False, fill_value='extrapolate'
# to resample IMU/GNSS series onto LiDAR timestamps. These tests pin down
# that behaviour against a synthetic series with a known analytical result.


def test_linear_interpolation_matches_analytical_result():
    x = np.array([0.0, 1.0, 2.0, 3.0, 4.0])
    y = 2.0 * x + 1.0  # y = 2x + 1

    interp = interp1d(x, y, bounds_error=False, fill_value='extrapolate')

    query = np.array([0.5, 1.5, 2.5, 3.5])
    expected = 2.0 * query + 1.0

    np.testing.assert_allclose(interp(query), expected, atol=1e-10)


def test_extrapolation_beyond_bounds():
    x = np.array([0.0, 1.0, 2.0])
    y = 3.0 * x - 1.0  # y = 3x - 1

    interp = interp1d(x, y, bounds_error=False, fill_value='extrapolate')

    query = np.array([-1.0, 5.0])
    expected = 3.0 * query - 1.0

    np.testing.assert_allclose(interp(query), expected, atol=1e-10)


def test_interpolation_at_known_points_returns_exact_values():
    x = np.array([0.0, 10.0, 20.0, 30.0])
    y = np.array([100.0, 110.0, 90.0, 120.0])

    interp = interp1d(x, y, bounds_error=False, fill_value='extrapolate')

    np.testing.assert_allclose(interp(x), y, atol=1e-10)
