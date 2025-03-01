#ifndef GPSNMEA_HPP
#define GPSNMEA_HPP

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// 定数
#define GPS_EARTH_MEAN_RADIUS 6371000.0
#define TWO_PI 6.283185307179586476925286766559

// ※ Arduino.h の radians, degrees マクロと衝突しないよう、変換関数名を変更
inline double toRadians(double deg) { return deg * 3.14159265358979323846 / 180.0; }
inline double toDegrees(double rad) { return rad * 180.0 / 3.14159265358979323846; }

// RawDegrees 構造体（緯度・経度の生データ）
struct RawDegrees {
	int deg;
	uint32_t billionths;
	bool negative;
};

// GSA, GSV, VTG 用の構造体
struct GPSGSA {
	char mode;          // 'M' (Manual) or 'A' (Automatic)
	int fixType;        // 1 = No fix, 2 = 2D fix, 3 = 3D fix
	int satPrn[12];     // 使用中の衛星番号（未使用は 0）
	double pdop;
	double hdop;
	double vdop;
	bool valid;
};

struct GPSGSVSatellite {
	int prn;
	int elevation;
	int azimuth;
	int snr;
};

struct GPSGSV {
	int totalMessages;    // 全メッセージ数
	int messageNumber;    // この文の番号
	int satellitesInView; // 観測されている衛星数
	GPSGSVSatellite satellites[4]; // この文で報告される最大4衛星
	bool valid;
};

struct GPSVTG {
	double trueTrack;     // 真方位（度）
	double magneticTrack; // 磁気方位（度）
	double speedKnots;    // 速度（ノット）
	double speedKmph;     // 速度（km/h）
	bool valid;
};

// 前方宣言
class GPSNMEA;

// 各クラスの宣言

class GPSLocation {
public:
	RawDegrees rawNewLatData, rawNewLngData;
	RawDegrees rawLatData, rawLngData;
	unsigned long lastCommitTime;
	bool valid;
	bool updated;
	void setLatitude(const char *term);
	void setLongitude(const char *term);
	void commit();
	double lat();
	double lng();
	// ユーザ向け追加メソッド
	bool isValid() const { return valid; }
	const RawDegrees& rawLat() const { return rawLatData; }
	const RawDegrees& rawLng() const { return rawLngData; }
};

class GPSTime {
public:
	uint32_t newTime;
	uint32_t time;
	unsigned long lastCommitTime;
	bool valid;
	bool updated;
	void setTime(const char *term);
	void commit();
	uint8_t hour();
	uint8_t minute();
	uint8_t second();
	uint8_t centisecond();
	// ユーザ向け追加メソッド
	bool isValid() const { return valid; }
};

class GPSDate {
public:
	long newDate;
	long date;
	unsigned long lastCommitTime;
	bool valid;
	bool updated;
	void setDate(const char *term);
	void commit();
	uint16_t year();
	uint8_t month();
	uint8_t day();
	// ユーザ向け追加メソッド
	bool isValid() const { return valid; }
};

class GPSDecimal {
public:
	int newval;
	int val;
	unsigned long lastCommitTime;
	bool valid;
	bool updated;
	void set(const char *term);
	void commit();
	// ユーザ向け追加メソッド
	bool isValid() const { return valid; }
	// 例：高度や速度を表示するための値。ここでは生の値を返す例です。
	double meters() const { return static_cast<double>(val); }
	double kmph() const { return static_cast<double>(val); }
};

class GPSInteger {
public:
	long newval;
	long val;
	unsigned long lastCommitTime;
	bool valid;
	bool updated;
	void set(const char *term);
	void commit();
	// ユーザ向け追加メソッド
	bool isValid() const { return valid; }
	long value() const { return val; }
};

class GPSCustom {
public:
	const char *sentenceName;
	int termNumber;
	char stagingBuffer[20];
	char buffer[20];
	unsigned long lastCommitTime;
	bool valid;
	bool updated;
	GPSCustom *next;
	GPSCustom(GPSNMEA &gps, const char *sentenceName, int termNumber);
	void begin(GPSNMEA &gps, const char *sentenceName, int termNumber);
	void commit();
	void set(const char *term);
};

// NMEA 文の種類
enum SentenceType {
	SentenceType_Other,
	SentenceType_RMC,
	SentenceType_GGA,
	SentenceType_GSA,
	SentenceType_GSV,
	SentenceType_VTG
};

class GPSNMEA {
public:
	GPSNMEA();
	void reset();
	bool encode(char c);
	int fromHex(char a);
	bool endOfTermHandler();
	void insertCustom(GPSCustom *pElt, const char *sentenceName, int termNumber);

	// メンバ変数
	uint8_t parity;
	bool isChecksumTerm;
	SentenceType curSentenceType;
	int curTermNumber;
	int curTermOffset;
	bool sentenceHasFix;
	unsigned long encodedCharCount;
	unsigned long sentencesWithFixCount;
	unsigned long failedChecksumCount;
	unsigned long passedChecksumCount;
	char termBuffer[20];
	GPSCustom *customElts;
	GPSCustom *customCandidates;

	// GSA, GSV, VTG 用のメンバ変数
	GPSGSA gsa;
	GPSGSV gsv;
	GPSVTG vtg;

	// RMC, GGA の各フィールド
	GPSLocation location;
	GPSTime time;
	GPSDate date;
	GPSDecimal speed;
	GPSDecimal course;
	GPSInteger satellites;
	GPSDecimal hdop;
	GPSDecimal altitude;
};

#endif // GPSNMEA_HPP
