from numpy import array

DIST_MAX              = 4
LIVOX_RAW_FRAME_SIZE  = 100000
OUSTER_RAW_FRAME_SIZE = 1000000

LIDAR_GNSS_OFFSETS = {
    'livox' : array([-0.034, 0.020, 1.861]),
    'ouster': array([-0.036, 0.0, 0.69])
}

FRAME_SIZE = {
    'livox' : 100000,
    'ouster': 1000000
}