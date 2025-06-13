#ifndef GNSS_HPP
#define GNSS_HPP

#include <vector>
#include <string>

struct GNSSRead {
    double timestamp;
    double easting;
    double northing;
    double altitude;
};

std::vector<GNSSRead> ReadGNSSData(const std::string& filepath);

#endif