#include "include/gnss.hpp"
#include "include/geo_utils.hpp"
#include <cstdio>
#include <cstdlib>
#include <iostream>


std::vector<GNSSRead> ReadGNSSData(const std::string& filepath){
    std::vector<GNSSRead> gnss_data;

    FILE* file = std::fopen(filepath.c_str(), "r");
    if (!file) {
        std::cerr << "Could not open the file: " << filepath << "\n";
        return gnss_data;
    }

    char buff[256];
    std::fgets(buff, sizeof(buff), file);

    double timestamp, longitude, latitude, altitude;

    while (std::fscanf(file, "%lf %lf %lf %lf", &timestamp, &longitude, &latitude, &altitude) == 4) {
        auto [easting, northing] = LonLatToUTM(longitude, latitude);
        GNSSRead p {timestamp, easting, northing, altitude};
        gnss_data.push_back(p);
    }

    std::fclose(file);
    return gnss_data;
}

