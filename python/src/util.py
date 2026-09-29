from typing import Tuple
import numpy as np
from numpy import float64
from numpy.typing import NDArray
from pyproj import Transformer

_TRANSFORMER = Transformer.from_crs('epsg:4326', 'epsg:25831', always_xy=True)

def latlon_to_utm(longitude: float64, latitude: float64) -> Tuple[float64, float64]:
    easting, northing = _TRANSFORMER.transform(longitude, latitude)

    return float64(easting), float64(northing)

def latlon_array_to_utm(longitude: NDArray[np.float64], latitude: NDArray[np.float64]) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
    easting, northing = _TRANSFORMER.transform(longitude, latitude)

    return np.asarray(easting, dtype=np.float64), np.asarray(northing, dtype=np.float64)
