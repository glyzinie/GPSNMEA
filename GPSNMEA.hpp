#ifndef GPSNMEA_HPP
#define GPSNMEA_HPP

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// 緯度・経度などの度数表示用
struct RawDegrees {
	uint8_t deg;            // 度の整数部分
	uint32_t billionths;    // 度の小数部分を 1e9 単位で保持
	bool negative;          // 南緯、または西経なら true
};

// 16進文字 -> 数値変換
int gpsFromHex(char a);

// 数値文字列 -> (100倍の整数値)として返す
//  例: "1234.56" => 123456
int32_t gpsParseDecimal(const char *term);

// 緯度・経度文字列を度数表現(RawDegrees)へ変換
void gpsParseDegrees(const char *term, RawDegrees &deg);

// 方位角(deg)を16方位(N, NNE, NEなど)の文字列として返す
const char* gpsCardinal(double course);

//=================================================================
// GPSNMEA クラス本体
//=================================================================
class GPSNMEA;  // 前方宣言

// ------------------------------
// 各種データサブクラス
// ------------------------------

// 緯度・経度 (Location)
class GPSLocation {
public:
	GPSLocation();

	void setLatitude(const char *term);
	void setLongitude(const char *term);
	void commit();
	double lat();
	double lng();

	bool isValid() const { return valid; }
	bool isUpdated() const { return updated; }

private:
	RawDegrees rawLatData, rawNewLatData;
	RawDegrees rawLngData, rawNewLngData;
	bool valid, updated;
	unsigned long lastCommitTime;

	friend class GPSNMEA; // GPSNMEAのprivate static関数から直接アクセス可
};

// 時刻情報 (RMCなどで使用)
class GPSTime {
public:
	GPSTime();

	void setTime(const char *term);
	void commit();
	uint8_t hour();
	uint8_t minute();
	uint8_t second();
	uint8_t centisecond();

	bool isValid() const { return valid; }
	bool isUpdated() const { return updated; }

private:
	uint32_t time, newTime;
	bool valid, updated;
	unsigned long lastCommitTime;

	friend class GPSNMEA;
};

// 日付情報 (RMCで使用)
class GPSDate {
public:
	GPSDate();

	void setDate(const char *term);
	void commit();
	uint16_t year();
	uint8_t month();
	uint8_t day();

	bool isValid() const { return valid; }
	bool isUpdated() const { return updated; }

private:
	long date, newDate;
	bool valid, updated;
	unsigned long lastCommitTime;

	friend class GPSNMEA;
};

// 小数値 (speed, course, hdop, altitudeなど)
class GPSDecimal {
public:
	GPSDecimal();

	void set(const char *term);
	void commit();
	int32_t value() { updated = false; return val; }

	bool isValid() const { return valid; }
	bool isUpdated() const { return updated; }

private:
	int32_t val, newval;
	bool valid, updated;
	unsigned long lastCommitTime;

	friend class GPSNMEA;
};

// 整数値 (衛星数など)
class GPSInteger {
public:
	GPSInteger();

	void set(const char *term);
	void commit();
	long value() { updated = false; return val; }

	bool isValid() const { return valid; }
	bool isUpdated() const { return updated; }

private:
	long val, newval;
	bool valid, updated;
	unsigned long lastCommitTime;

	friend class GPSNMEA;
};

// カスタムフィールド (特定のtermを取得したい場合に使用)
class GPSCustom {
public:
	GPSCustom();
	GPSCustom(GPSNMEA &gps, const char *sentenceName, int termNumber);

	void begin(GPSNMEA &gps, const char *sentenceName, int termNumber);

	const char *value() { updated = false; return buffer; }
	bool isValid() const { return valid; }
	bool isUpdated() const { return updated; }

private:
	void commit();
	void set(const char *term);

	const char *sentenceName;
	int termNumber;
	GPSCustom *next;

	char stagingBuffer[16];
	char buffer[16];

	bool valid, updated;
	unsigned long lastCommitTime;

	friend class GPSNMEA;  // GPSNMEA から commit/set呼び出し可
};

// ------------------------------
// メインクラス: GPSNMEA
// ------------------------------
class GPSNMEA {
public:
	static const int MAX_TERM_LENGTH = 20;

	GPSNMEA();
	void reset();

	// 受信バイトを1文字ずつ渡してデコード。trueが返れば文末(Checksumまで)が処理完了
	bool encode(char c);

	// --------------------
	// 取得データ
	// --------------------
	GPSLocation location;
	GPSTime time;
	GPSDate date;
	GPSDecimal speed;
	GPSDecimal course;
	GPSInteger satellites;
	GPSDecimal hdop;
	GPSDecimal altitude;

	// GSA情報
	struct {
		char mode;      // 'A'=Auto, 'M'=Manual
		int fixType;    // 1=NoFix,2=2D,3=3D
		int satPrn[12];
		double pdop;
		double hdop;
		double vdop;
		bool valid;
	} gsa;

	// GSV情報
	struct {
		struct Satellite {
			int prn;
			int elevation;
			int azimuth;
			int snr;
		};
		int totalMessages;
		int messageNumber;
		int satellitesInView;
		Satellite satellites[4];
		bool valid;
	} gsv;

	// VTG情報
	struct {
		double trueTrack;
		double magneticTrack;
		double speedKnots;
		double speedKmph;
		bool valid;
	} vtg;

	// 統計情報
	uint32_t encodedCharCount;
	uint32_t sentencesWithFixCount;
	uint32_t failedChecksumCount;
	uint32_t passedChecksumCount;

private:
	// パース中の状態
	uint8_t parity;
	bool isChecksumTerm;

	enum SentenceType {
		SentenceType_Other,
		SentenceType_RMC,
		SentenceType_GGA,
		SentenceType_GSA,
		SentenceType_GSV,
		SentenceType_VTG
	};
	SentenceType curSentenceType;

	uint8_t curTermNumber;
	uint8_t curTermOffset;
	bool sentenceHasFix;

	char termBuffer[MAX_TERM_LENGTH];

	// カスタム項目リスト
	GPSCustom *customElts;
	GPSCustom *customCandidates;
	void insertCustom(GPSCustom *pElt, const char *sentenceName, int index);

	// term切り出し終わりで呼ばれる内部処理
	bool endOfTermHandler();
	int fromHex(char a);

	static void parseRMCTerm(int termNumber, const char *term, GPSNMEA &gps);
	static void parseGGATerm(int termNumber, const char *term, GPSNMEA &gps);
	static void parseGSATerm(int termNumber, const char *term, GPSNMEA &gps);
	static void parseGSVTerm(int termNumber, const char *term, GPSNMEA &gps);
	static void parseVTGTerm(int termNumber, const char *term, GPSNMEA &gps);

	friend class GPSCustom; // カスタムフィールドがcommit/set等を呼ぶ場合
};

#endif // GPSNMEA_HPP
