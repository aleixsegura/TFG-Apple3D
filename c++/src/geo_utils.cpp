#include "include/geo_utils.hpp"
#include <stdexcept>

std::pair<double, double> LonLatToUTM(double longitude, double latitude) {
    PJ_CONTEXT *context = proj_context_create();
    if (!context) throw std::runtime_error("Failed to create PROJ context");

    PJ* transformation = proj_create_crs_to_crs(context, 
        "EPSG:4326",
        "epsg:25831",
        nullptr);
    
    if (!transformation) throw std::runtime_error("Failed to create transformation");

    PJ* transform = proj_normalize_for_visualization(context, transformation);
    proj_destroy(transformation);
    if (!transform) throw std::runtime_error("Failed to normalize transformation");

    PJ_COORD input = proj_coord(longitude, latitude, 0, 0);
    PJ_COORD output = proj_trans(transform, PJ_FWD, input);

    proj_destroy(transform);

    return {output.xy.x, output.xy.y};
}