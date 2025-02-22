#ifndef __GPSUTILS_H
#define __GPSUTILS_H

#include <Arduino.h>
#include <inttypes.h>
#include <ctype.h>
#include <math.h>

// 定数
#ifndef TWO_PI
	#define TWO_PI 6.283185307179586476925286766559
#endif

constexpr uint32_t GPS_EARTH_MEAN_RADIUS = 6371009UL;

// 速度・距離変換用定数
constexpr float GPS_MPH_PER_KNOT    = 1.15077945f;
constexpr float GPS_MPS_PER_KNOT    = 0.51444444f;
constexpr float GPS_KMPH_PER_KNOT   = 1.852f;
constexpr float GPS_MILES_PER_METER = 0.00062137112f;
constexpr float GPS_KM_PER_METER    = 0.001f;
constexpr float GPS_FEET_PER_METER  = 3.2808399f;

// NMEAパースで使用する最大フィールド長
constexpr int GPS_MAX_FIELD_SIZE = 15;

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

#endif // __GPSUTILS_H
