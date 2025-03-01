#include "GPSUtils.hpp"
#include <Arduino.h>
#include <cstdlib>
#include <cctype>
#include <cmath>

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
	int32_t ret = 100 * static_cast<int32_t>(std::atol(term));
	while (std::isdigit(*term)) {
		++term;
	}
	if (*term == '.' && std::isdigit(term[1])) {
		ret += 10 * (term[1] - '0');
		if (std::isdigit(term[2])) {
			ret += (term[2] - '0');
		}
	}
	return negative ? -ret : ret;
}

void gpsParseDegrees(const char *term, RawDegrees &deg) {
	uint32_t leftOfDecimal = static_cast<uint32_t>(std::atol(term));
	uint16_t minutes = static_cast<uint16_t>(leftOfDecimal % 100);
	uint32_t multiplier = 10000000UL;
	uint32_t tenMillionths = minutes * multiplier;
	deg.deg = leftOfDecimal / 100;
	while (std::isdigit(*term)) {
		++term;
	}
	if (*term == '.') {
		while (std::isdigit(*++term)) {
			multiplier /= 10;
			tenMillionths += (*term - '0') * multiplier;
		}
	}
	// 換算処理（概ねの精度）
	deg.billionths = (5 * tenMillionths + 1) / 3;
	deg.negative = false;
}

double gpsDistanceBetween(double lat1, double lon1, double lat2, double lon2) {
	double delta = toRadians(lon1 - lon2);
	double sdlon = sin(delta);
	double cdlon = cos(delta);
	lat1 = toRadians(lat1);
	lat2 = toRadians(lat2);
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
	double dlon = toRadians(lon2 - lon1);
	lat1 = toRadians(lat1);
	lat2 = toRadians(lat2);
	double a1 = sin(dlon) * cos(lat2);
	double a2 = sin(lat1) * cos(lat2) * cos(dlon);
	a2 = cos(lat1) * sin(lat2) - a2;
	a2 = atan2(a1, a2);
	if (a2 < 0.0) {
		a2 += TWO_PI;
	}
	return toDegrees(a2);
}

const char* gpsCardinal(double course) {
	static const char* directions[] = {
		"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
		"S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
	};
	int index = static_cast<int>((course + 11.25) / 22.5);
	return directions[index % 16];
}
