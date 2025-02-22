#include "GPSUtils.hpp"
#include <stdlib.h>

int gpsFromHex(char a) {
	if (a >= 'A' && a <= 'F') {
		return a - 'A' + 10;
	} else if (a >= 'a' && a <= 'f') {
		return a - 'a' + 10;
	} else {
		return a - '0';
	}
}

int32_t gpsParseDecimal(const char *term) {
	bool negative = *term == '-';
	if (negative) {
		++term;
	}
	int32_t ret = 100 * (int32_t)atol(term);
	while (isdigit(*term)) {
		++term;
	}
	if (*term == '.' && isdigit(term[1])) {
		ret += 10 * (term[1] - '0');
		if (isdigit(term[2])) {
			ret += (term[2] - '0');
		}
	}
	return negative ? -ret : ret;
}

void gpsParseDegrees(const char *term, RawDegrees &deg) {
	uint32_t leftOfDecimal = (uint32_t)atol(term);
	uint16_t minutes = (uint16_t)(leftOfDecimal % 100);
	uint32_t multiplier = 10000000UL;
	uint32_t tenMillionths = minutes * multiplier;
	deg.deg = leftOfDecimal / 100;
	while (isdigit(*term)) {
		++term;
	}
	if (*term == '.') {
		while (isdigit(*++term)) {
			multiplier /= 10;
			tenMillionths += (*term - '0') * multiplier;
		}
	}
	// 換算処理（概ねの精度）
	deg.billionths = (5 * tenMillionths + 1) / 3;
	deg.negative = false;
}

double gpsDistanceBetween(double lat1, double lon1, double lat2, double lon2) {
	double delta = radians(lon1 - lon2);
	double sdlon = sin(delta);
	double cdlon = cos(delta);
	lat1 = radians(lat1);
	lat2 = radians(lat2);
	double slat1 = sin(lat1);
	double clat1 = cos(lat1);
	double slat2 = sin(lat2);
	double clat2 = cos(lat2);
	delta = (clat1 * slat2) - (slat1 * clat2 * cdlon);
	delta = delta * delta;
	delta += (clat2 * sdlon) * (clat2 * sdlon);
	delta = sqrt(delta);
	double denom = (slat1 * slat2) + (clat1 * clat2 * cdlon);
	delta = atan2(delta, denom);
	return delta * GPS_EARTH_MEAN_RADIUS;
}

double gpsCourseTo(double lat1, double lon1, double lat2, double lon2) {
	double dlon = radians(lon2 - lon1);
	lat1 = radians(lat1);
	lat2 = radians(lat2);
	double a1 = sin(dlon) * cos(lat2);
	double a2 = sin(lat1) * cos(lat2) * cos(dlon);
	a2 = cos(lat1) * sin(lat2) - a2;
	a2 = atan2(a1, a2);
	if (a2 < 0.0) {
		a2 += TWO_PI;
	}
	return degrees(a2);
}

const char* gpsCardinal(double course) {
	static const char* directions[] = {
		"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
		"S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
	};
	int index = (int)((course + 11.25) / 22.5);
	return directions[index % 16];
}
