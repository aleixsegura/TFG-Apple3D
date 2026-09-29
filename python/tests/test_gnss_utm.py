import numpy as np
import pytest

from util import latlon_to_utm, latlon_array_to_utm

# Known dataset point (ouster/go/gnss_go.txt, first row) and its expected
# projection in EPSG:25831 (UTM 31N / ETRS89), computed with pyproj directly.
LAT = 41.6574
LON = 0.3938
EXPECTED_EASTING = 282997.9921876813
EXPECTED_NORTHING = 4615020.494502106


def test_latlon_to_utm_known_point():
    easting, northing = latlon_to_utm(LON, LAT)

    assert easting == pytest.approx(EXPECTED_EASTING, abs=1e-3)
    assert northing == pytest.approx(EXPECTED_NORTHING, abs=1e-3)


def test_latlon_array_to_utm_known_point():
    easting, northing = latlon_array_to_utm(np.array([LON]), np.array([LAT]))

    assert easting[0] == pytest.approx(EXPECTED_EASTING, abs=1e-3)
    assert northing[0] == pytest.approx(EXPECTED_NORTHING, abs=1e-3)


def test_latlon_array_to_utm_multiple_points():
    lons = np.array([LON, LON + 0.001])
    lats = np.array([LAT, LAT + 0.001])

    easting, northing = latlon_array_to_utm(lons, lats)

    assert easting.shape == (2,)
    assert northing.shape == (2,)
    assert easting[0] == pytest.approx(EXPECTED_EASTING, abs=1e-3)
    assert northing[0] == pytest.approx(EXPECTED_NORTHING, abs=1e-3)
    # Moving north-east should increase both easting and northing.
    assert easting[1] > easting[0]
    assert northing[1] > northing[0]
