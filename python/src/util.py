from typing import Tuple
from numpy import float64
from pyproj import Transformer

def latlon_to_utm(longitude: float64, latitude: float64) -> Tuple[float64, float64]:
    transformer = Transformer.from_crs('epsg:4326', 'epsg:25831', always_xy=True)
    easting, northing = transformer.transform(longitude, latitude)
    
    return float64(easting), float64(northing)
