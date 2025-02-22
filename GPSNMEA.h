#ifndef __GPSNMEA_H
#define __GPSNMEA_H

#include <inttypes.h>
#include "Arduino.h"
#include <limits.h>
#include "GPSUtils.h"

// ライブラリバージョン
#define GPSNMEA_VERSION "0.0.1"

// 最大項目サイズ（NMEAフィールド用、余裕を持って15文字）
#define GPS_MAX_FIELD_SIZE 15

// カスタムフィールド機能の有効／無効（不要なら 0 に設定）
#ifndef ENABLE_CUSTOM_FIELDS
#define ENABLE_CUSTOM_FIELDS 0
#endif

// NMEA文種別（内部識別用）
enum {
	SentenceType_GGA,
	SentenceType_RMC,
	SentenceType_Other
};

//
// GPSLocation：位置情報
//
struct GPSLocation {
	friend class GPSNMEA;
public:
	enum Quality { Invalid = '0', GPS = '1', DGPS = '2', PPS = '3', RTK = '4', FloatRTK = '5', Estimated = '6', Manual = '7', Simulated = '8' };
	enum Mode { N = 'N', A = 'A', D = 'D', E = 'E' };

	bool isValid() const { return valid; }
	bool isUpdated() const { return updated; }
	uint32_t age() const { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
	const RawDegrees &rawLat() { updated = false; return rawLatData; }
	const RawDegrees &rawLng() { updated = false; return rawLngData; }
	double lat();
	double lng();
	Quality fixQuality() { updated = false; return quality; }
	Mode fixMode() { updated = false; return mode; }

	GPSLocation() : valid(false), updated(false), quality(Invalid), mode(N) {}

private:
	bool valid, updated;
	RawDegrees rawLatData, rawLngData, rawNewLatData, rawNewLngData;
	Quality quality, newQuality;
	Mode mode, newMode;
	uint32_t lastCommitTime;
	void commit();
	void setLatitude(const char *term);
	void setLongitude(const char *term);
};

//
// GPSDate：日付情報
//
struct GPSDate {
	friend class GPSNMEA;
public:
	bool isValid() const { return valid; }
	bool isUpdated() const { return updated; }
	uint32_t age() const { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
	uint32_t value() { updated = false; return date; }
	uint16_t year();
	uint8_t month();
	uint8_t day();

	GPSDate() : valid(false), updated(false), date(0) {}

private:
	bool valid, updated;
	uint32_t date, newDate;
	uint32_t lastCommitTime;
	void commit();
	void setDate(const char *term);
};

//
// GPSTime：時刻情報
//
struct GPSTime {
	friend class GPSNMEA;
public:
	bool isValid() const { return valid; }
	bool isUpdated() const { return updated; }
	uint32_t age() const { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
	uint32_t value() { updated = false; return time; }
	uint8_t hour();
	uint8_t minute();
	uint8_t second();
	uint8_t centisecond();

	GPSTime() : valid(false), updated(false), time(0) {}

private:
	bool valid, updated;
	uint32_t time, newTime;
	uint32_t lastCommitTime;
	void commit();
	void setTime(const char *term);
};

//
// GPSDecimal：小数値保持の基本クラス
//
struct GPSDecimal {
	friend class GPSNMEA;
public:
	bool isValid() const { return valid; }
	bool isUpdated() const { return updated; }
	uint32_t age() const { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
	int32_t value() { updated = false; return val; }

	GPSDecimal() : valid(false), updated(false), val(0) {}

private:
	bool valid, updated;
	uint32_t lastCommitTime;
	int32_t val, newval;
	void commit();
	void set(const char *term);
};

//
// GPSInteger：整数値保持の基本クラス
//
struct GPSInteger {
	friend class GPSNMEA;
public:
	bool isValid() const { return valid; }
	bool isUpdated() const { return updated; }
	uint32_t age() const { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
	uint32_t value() { updated = false; return val; }

	GPSInteger() : valid(false), updated(false), val(0) {}

private:
	bool valid, updated;
	uint32_t lastCommitTime;
	uint32_t val, newval;
	void commit();
	void set(const char *term);
};

//
// GPSSpeed, GPSCourse, GPSAltitude, GPSHDOP
//
struct GPSSpeed : GPSDecimal {
	double knots() { return value() / 100.0; }
	double mph() { return GPS_MPH_PER_KNOT * value() / 100.0; }
	double mps() { return GPS_MPS_PER_KNOT * value() / 100.0; }
	double kmph() { return GPS_KMPH_PER_KNOT * value() / 100.0; }
};

struct GPSCourse : public GPSDecimal {
	double deg() { return value() / 100.0; }
};

struct GPSAltitude : GPSDecimal {
	double meters() { return value() / 100.0; }
	double miles() { return GPS_MILES_PER_METER * value() / 100.0; }
	double kilometers() { return GPS_KM_PER_METER * value() / 100.0; }
	double feet() { return GPS_FEET_PER_METER * value() / 100.0; }
};

struct GPSHDOP : GPSDecimal {
	double hdop() { return value() / 100.0; }
};

//
// GPSCustom：カスタムフィールド対応
// カスタムフィールド機能は ENABLE_CUSTOM_FIELDS マクロで有効／無効を切り替え
#if ENABLE_CUSTOM_FIELDS
class GPSCustom {
public:
	GPSCustom() {}
	GPSCustom(GPSNMEA &gps, const char *sentenceName, int termNumber);
	void begin(GPSNMEA &gps, const char *sentenceName, int termNumber);

	bool isUpdated() const { return updated; }
	bool isValid() const { return valid; }
	uint32_t age() const { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
	const char *value() { updated = false; return buffer; }

private:
	void commit();
	void set(const char *term);

	char stagingBuffer[GPS_MAX_FIELD_SIZE + 1];
	char buffer[GPS_MAX_FIELD_SIZE + 1];
	unsigned long lastCommitTime;
	bool valid, updated;
	const char *sentenceName;
	int termNumber;
	friend class GPSNMEA;
	GPSCustom *next;
};
#endif

//
// GPSNMEA：NMEA文全体のパース
//
class GPSNMEA {
public:
	GPSNMEA();
	bool encode(char c);
	GPSNMEA &operator << (char c) { encode(c); return *this; }
	void reset();

	// 統計情報
	uint32_t charsProcessed() const { return encodedCharCount; }
	uint32_t sentencesWithFix() const { return sentencesWithFixCount; }
	uint32_t failedChecksum() const { return failedChecksumCount; }
	uint32_t passedChecksum() const { return passedChecksumCount; }

	// 解析結果
	GPSLocation location;
	GPSDate date;
	GPSTime time;
	GPSSpeed speed;
	GPSCourse course;
	GPSAltitude altitude;
	GPSInteger satellites;
	GPSHDOP hdop;

	static const char *libraryVersion() { return GPSNMEA_VERSION; }

	// ユーティリティ関数
	static double distanceBetween(double lat1, double lon1, double lat2, double lon2);
	static double courseTo(double lat1, double lon1, double lat2, double lon2);
	static const char *cardinal(double course);

	static int32_t parseDecimal(const char *term);
	static void parseDegrees(const char *term, RawDegrees &deg);

#if ENABLE_CUSTOM_FIELDS
	void insertCustom(GPSCustom *pElt, const char *sentenceName, int termNumber);
#endif

private:
	enum { SentenceType_GGA, SentenceType_RMC, SentenceType_Other };

	// 内部パース状態変数
	uint8_t parity;
	bool isChecksumTerm;
	char termBuffer[GPS_MAX_FIELD_SIZE];
	uint8_t curSentenceType;
	uint8_t curTermNumber;
	uint8_t curTermOffset;
	bool sentenceHasFix;

#if ENABLE_CUSTOM_FIELDS
	GPSCustom *customElts;
	GPSCustom *customCandidates;
#endif

	// 統計カウンタ
	uint32_t encodedCharCount;
	uint32_t sentencesWithFixCount;
	uint32_t failedChecksumCount;
	uint32_t passedChecksumCount;

	int fromHex(char a);
	bool endOfTermHandler();
};

#endif // __GPSNMEA_H
