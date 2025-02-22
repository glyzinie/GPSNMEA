#ifndef __GPSUTILS_HPP
#define __GPSUTILS_HPP

#include <Arduino.h>
#include <inttypes.h>
#include <ctype.h>
#include <math.h>

// 定数
#ifndef TWO_PI
	#define TWO_PI 6.283185307179586476925286766559
#endif
// https://en.wikipedia.org/wiki/Earth_radius
#define GPS_EARTH_MEAN_RADIUS 6371009

// RawDegrees：緯度／経度の内部表現
struct RawDegrees {
	uint16_t deg;         // 度部分
	uint32_t billionths;  // 小数部（10^-9 単位）
	bool negative;        // 負の場合 true
	RawDegrees() : deg(0), billionths(0), negative(false) {}
};

// 共通ユーティリティ関数
int gpsFromHex(char a);
int32_t gpsParseDecimal(const char *term);
void gpsParseDegrees(const char *term, RawDegrees &deg);

double gpsDistanceBetween(double lat1, double lon1, double lat2, double lon2);
double gpsCourseTo(double lat1, double lon1, double lat2, double lon2);
const char* gpsCardinal(double course);

#endif // __GPSUTILS_HPP
