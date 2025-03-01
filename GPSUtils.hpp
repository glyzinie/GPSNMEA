#ifndef GPSUTILS_HPP
#define GPSUTILS_HPP

#include "GPSNMEA.hpp" // RawDegrees を利用

int gpsFromHex(char a);
int32_t gpsParseDecimal(const char *term);
void gpsParseDegrees(const char *term, RawDegrees &deg);
double gpsDistanceBetween(double lat1, double lon1, double lat2, double lon2);
double gpsCourseTo(double lat1, double lon1, double lat2, double lon2);
const char* gpsCardinal(double course);

#endif // GPSUTILS_HPP
