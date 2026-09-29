#include "include/geo_utils.hpp"
#include <stdexcept>

namespace {

// Built once and reused: proj_create_crs_to_crs() does a PROJ database lookup
// and pipeline construction, which is orders of magnitude more expensive than
// the coordinate transform itself.
struct GeoTransformer {
    PJ_CONTEXT* context = nullptr;
    PJ* transform = nullptr;

    GeoTransformer() {
        context = proj_context_create();
        if (!context) throw std::runtime_error("Failed to create PROJ context");

        PJ* transformation = proj_create_crs_to_crs(context,
            "EPSG:4326",
            "epsg:25831",
            nullptr);

        if (!transformation) {
            proj_context_destroy(context);
            throw std::runtime_error("Failed to create transformation");
        }

        transform = proj_normalize_for_visualization(context, transformation);
        proj_destroy(transformation);
        if (!transform) {
            proj_context_destroy(context);
            throw std::runtime_error("Failed to normalize transformation");
        }
    }

    ~GeoTransformer() {
        proj_destroy(transform);
        proj_context_destroy(context);
    }
};

GeoTransformer& GetTransformer() {
    static GeoTransformer instance;
    return instance;
}

}  // namespace

std::pair<double, double> LonLatToUTM(double longitude, double latitude) {
    PJ_COORD input = proj_coord(longitude, latitude, 0, 0);
    PJ_COORD output = proj_trans(GetTransformer().transform, PJ_FWD, input);

    return {output.xy.x, output.xy.y};
}