#ifndef GEO_UTILS_HPP
#define GEO_UTILS_HPP

#include <proj.h>
#include <utility>

std::pair<double, double> LonLatToUTM(double longitude, double latitude);

#endif