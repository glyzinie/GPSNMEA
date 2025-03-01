#include "GPSNMEA.hpp"
#include <Arduino.h>
#include <cstdlib>
#include <cctype>
#include <cmath>

int gpsFromHex(char a) {
	if (a >= 'A' && a <= 'F')
		return a - 'A' + 10;
	if (a >= 'a' && a <= 'f')
		return a - 'a' + 10;
	return a - '0';
}

int32_t gpsParseDecimal(const char *term) {
	bool negative = (*term == '-');
	if (negative) ++term;
	int32_t ret = 100 * static_cast<int32_t>(atol(term));
	while (isdigit(*term)) ++term;
	if (*term == '.' && isdigit(term[1])) {
		ret += 10 * (term[1] - '0');
		if (isdigit(term[2])) {
			ret += (term[2] - '0');
		}
	}
	return negative ? -ret : ret;
}

void gpsParseDegrees(const char *term, RawDegrees &deg) {
	uint32_t leftOfDecimal = static_cast<uint32_t>(atol(term));
	uint16_t minutes = static_cast<uint16_t>(leftOfDecimal % 100);
	uint32_t multiplier = 10000000UL;
	uint32_t tenMillionths = minutes * multiplier;
	deg.deg = leftOfDecimal / 100;

	while (isdigit(*term)) ++term;
	if (*term == '.') {
		while (isdigit(*++term)) {
			multiplier /= 10;
			tenMillionths += (term[0] - '0') * multiplier;
		}
	}
	// TinyGPS++ の実装に準拠 (60分=1度 など計算)
	deg.billionths = (5 * tenMillionths + 1) / 3;
	deg.negative = false;
}

const char* gpsCardinal(double course) {
	static const char* directions[] = {
		"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
		"S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
	};
	int index = static_cast<int>((course + 11.25) / 22.5);
	return directions[index % 16];
}

//=================================================================
// GPSNMEA クラス実装
//=================================================================

GPSNMEA::GPSNMEA()
	: parity(0),
	isChecksumTerm(false),
	curSentenceType(SentenceType_Other),
	curTermNumber(0),
	curTermOffset(0),
	sentenceHasFix(false),
	encodedCharCount(0),
	sentencesWithFixCount(0),
	failedChecksumCount(0),
	passedChecksumCount(0),
	customElts(nullptr),
	customCandidates(nullptr)
{
	memset(termBuffer, 0, MAX_TERM_LENGTH);

	// GSA初期化
	gsa.mode = '\0';
	gsa.fixType = 0;
	memset(gsa.satPrn, 0, sizeof(gsa.satPrn));
	gsa.pdop = gsa.hdop = gsa.vdop = 0.0;
	gsa.valid = false;

	// GSV初期化
	gsv.totalMessages = 0;
	gsv.messageNumber = 0;
	gsv.satellitesInView = 0;
	memset(gsv.satellites, 0, sizeof(gsv.satellites));
	gsv.valid = false;

	// VTG初期化
	vtg.trueTrack = 0.0;
	vtg.magneticTrack = 0.0;
	vtg.speedKnots = 0.0;
	vtg.speedKmph = 0.0;
	vtg.valid = false;
}

void GPSNMEA::reset() {
	parity = 0;
	isChecksumTerm = false;
	curSentenceType = SentenceType_Other;
	curTermNumber = 0;
	curTermOffset = 0;
	sentenceHasFix = false;
	memset(termBuffer, 0, MAX_TERM_LENGTH);

	// 統計
	encodedCharCount = 0;
	sentencesWithFixCount = 0;
	failedChecksumCount = 0;
	passedChecksumCount = 0;

	// カスタムフィールド
	customElts = nullptr;
	customCandidates = nullptr;

	// GSA, GSV, VTGリセット
	gsa.valid = false;
	gsv.valid = false;
	vtg.valid = false;
}

bool GPSNMEA::encode(char c) {
	++encodedCharCount;

	switch(c) {
		case ',':
			if (!isChecksumTerm)
				parity ^= (uint8_t)c;
		// FALLTHROUGH
		case '\r':
		case '\n':
		case '*': {
			if (curTermOffset < MAX_TERM_LENGTH)
				termBuffer[curTermOffset] = '\0';
			bool validSentence = endOfTermHandler();
			curTermNumber++;
			curTermOffset = 0;
			isChecksumTerm = (c == '*');
			return validSentence;
		}
		case '$': {
			// 文頭初期化
			curTermNumber = 0;
			curTermOffset = 0;
			parity = 0;
			curSentenceType = SentenceType_Other;
			isChecksumTerm = false;
			sentenceHasFix = false;
			memset(termBuffer, 0, MAX_TERM_LENGTH);
			return false;
		}
		default:
			if (curTermOffset < (MAX_TERM_LENGTH - 1)) {
				termBuffer[curTermOffset++] = c;
			}
			if (!isChecksumTerm) {
				parity ^= (uint8_t)c;
			}
			return false;
	}
}

int GPSNMEA::fromHex(char a) {
	return gpsFromHex(a);
}

bool GPSNMEA::endOfTermHandler() {
	if (isChecksumTerm) {
		// チェックサム部を処理
		uint8_t chksum = (uint8_t)(16 * fromHex(termBuffer[0]) + fromHex(termBuffer[1]));
		if (chksum == parity) {
			passedChecksumCount++;
			if (sentenceHasFix)
				sentencesWithFixCount++;

			// センテンス種類ごとのcommit
			switch(curSentenceType) {
				case SentenceType_RMC:
					date.commit();
					time.commit();
					if (sentenceHasFix)
						location.commit();
					speed.commit();
					course.commit();
					break;
				case SentenceType_GGA:
					time.commit();
					if (sentenceHasFix)
						location.commit();
					satellites.commit();
					hdop.commit();
					altitude.commit();
					break;
				case SentenceType_GSA:
					gsa.valid = true;
					break;
				case SentenceType_GSV:
					gsv.valid = true;
					break;
				case SentenceType_VTG:
					vtg.valid = true;
					break;
				default:
					break;
			}

			// カスタムフィールドをcommit
			for (GPSCustom *p = customCandidates; p != nullptr &&
			strcmp(p->sentenceName, customCandidates->sentenceName) == 0;
			p = p->next)
			{
				p->commit();
			}
			return true;
		} else {
			failedChecksumCount++;
			return false;
		}
	}

	// センテンス名（termNumber=0）を解析
	if (curTermNumber == 0) {
		// 例: "GPRMC", "GPGGA", "GPGSA" など
		if ((termBuffer[0] == 'G' || termBuffer[0] == 'N') &&
			(termBuffer[1] == 'P' || termBuffer[1] == 'N'))
		{
			if (strcmp(termBuffer + 2, "RMC") == 0)
				curSentenceType = SentenceType_RMC;
			else if (strcmp(termBuffer + 2, "GGA") == 0)
				curSentenceType = SentenceType_GGA;
			else if (strcmp(termBuffer + 2, "GSA") == 0)
				curSentenceType = SentenceType_GSA;
			else if (strcmp(termBuffer + 2, "GSV") == 0)
				curSentenceType = SentenceType_GSV;
			else if (strcmp(termBuffer + 2, "VTG") == 0)
				curSentenceType = SentenceType_VTG;
			else
				curSentenceType = SentenceType_Other;
		}

		// カスタム候補のリスト頭出し
		for (customCandidates = customElts; customCandidates != nullptr &&
		strcmp(customCandidates->sentenceName, termBuffer) < 0;
		customCandidates = customCandidates->next)
		{
			/* no-op */
		}
		if (customCandidates != nullptr &&
			strcmp(customCandidates->sentenceName, termBuffer) > 0)
		{
			customCandidates = nullptr;
		}

		return false;
	}

	// 本文のパース
	if (curSentenceType != SentenceType_Other && termBuffer[0] != '\0') {
		switch(curSentenceType) {
			case SentenceType_RMC:
				parseRMCTerm(curTermNumber, termBuffer, *this);
				break;
			case SentenceType_GGA:
				parseGGATerm(curTermNumber, termBuffer, *this);
				break;
			case SentenceType_GSA:
				parseGSATerm(curTermNumber, termBuffer, *this);
				break;
			case SentenceType_GSV:
				parseGSVTerm(curTermNumber, termBuffer, *this);
				break;
			case SentenceType_VTG:
				parseVTGTerm(curTermNumber, termBuffer, *this);
				break;
			default:
				break;
		}
	}

	// カスタムフィールドの更新
	for (GPSCustom *p = customCandidates; 
	p != nullptr && strcmp(p->sentenceName, customCandidates->sentenceName) == 0
	&& p->termNumber <= curTermNumber;
	p = p->next)
	{
		if (p->termNumber == curTermNumber) {
			p->set(termBuffer);
		}
	}
	return false;
}

// ---------------------------
// private static parse関数群
// ---------------------------
void GPSNMEA::parseRMCTerm(int termNumber, const char *term, GPSNMEA &gps) {
	switch(termNumber) {
		case 1:
			gps.time.setTime(term);
			break;
		case 2:
			gps.sentenceHasFix = (term[0] == 'A');
			break;
		case 3:
			gps.location.setLatitude(term);
			break;
		case 4:
			gps.location.rawNewLatData.negative = (term[0] == 'S');
			break;
		case 5:
			gps.location.setLongitude(term);
			break;
		case 6:
			gps.location.rawNewLngData.negative = (term[0] == 'W');
			break;
		case 7:
			gps.speed.set(term);
			break;
		case 8:
			gps.course.set(term);
			break;
		case 9:
			gps.date.setDate(term);
			break;
		default:
			break;
	}
}

void GPSNMEA::parseGGATerm(int termNumber, const char *term, GPSNMEA &gps) {
	switch(termNumber) {
		case 1:
			gps.time.setTime(term);
			break;
		case 2:
			gps.location.setLatitude(term);
			break;
		case 3:
			gps.location.rawNewLatData.negative = (term[0] == 'S');
			break;
		case 4:
			gps.location.setLongitude(term);
			break;
		case 5:
			gps.location.rawNewLngData.negative = (term[0] == 'W');
			break;
		case 6:
			gps.sentenceHasFix = (term[0] > '0');
			break;
		case 7:
			gps.satellites.set(term);
			break;
		case 8:
			gps.hdop.set(term);
			break;
		case 9:
			gps.altitude.set(term);
			break;
		default:
			break;
	}
}

void GPSNMEA::parseGSATerm(int termNumber, const char *term, GPSNMEA &gps) {
	if (termNumber == 1) {
		if (term[0] != '\0') {
			gps.gsa.mode = term[0];
		}
	} else if (termNumber == 2) {
		gps.gsa.fixType = atoi(term);
	} else if (termNumber >= 3 && termNumber <= 14) {
		int index = termNumber - 3;
		if (index >= 0 && index < 12) {
			gps.gsa.satPrn[index] = atoi(term);
		}
	} else if (termNumber == 15) {
		gps.gsa.pdop = atof(term);
	} else if (termNumber == 16) {
		gps.gsa.hdop = atof(term);
	} else if (termNumber == 17) {
		gps.gsa.vdop = atof(term);
	}
}

void GPSNMEA::parseGSVTerm(int termNumber, const char *term, GPSNMEA &gps) {
	if (termNumber == 1) {
		gps.gsv.totalMessages = atoi(term);
	} else if (termNumber == 2) {
		gps.gsv.messageNumber = atoi(term);
	} else if (termNumber == 3) {
		gps.gsv.satellitesInView = atoi(term);
	} else if (termNumber >= 4) {
		int fieldIndex = termNumber - 4;
		int satIndex = fieldIndex / 4;
		int field = fieldIndex % 4;
		if (satIndex < 4) {
			switch(field) {
				case 0: gps.gsv.satellites[satIndex].prn       = atoi(term); break;
				case 1: gps.gsv.satellites[satIndex].elevation = atoi(term); break;
				case 2: gps.gsv.satellites[satIndex].azimuth   = atoi(term); break;
				case 3: gps.gsv.satellites[satIndex].snr       = atoi(term); break;
			}
		}
	}
}

void GPSNMEA::parseVTGTerm(int termNumber, const char *term, GPSNMEA &gps) {
	// VTG: True Track, T, Magnetic Track, M, Speed (knots), N, Speed (km/h), K
	if (termNumber == 1) {
		gps.vtg.trueTrack = atof(term);
	} else if (termNumber == 3) {
		gps.vtg.magneticTrack = atof(term);
	} else if (termNumber == 5) {
		gps.vtg.speedKnots = atof(term);
	} else if (termNumber == 7) {
		gps.vtg.speedKmph = atof(term);
	}
}

//=================================================================
// サブクラス実装
//=================================================================
GPSLocation::GPSLocation()
: lastCommitTime(0), valid(false), updated(false)
{
	rawLatData = rawNewLatData = {0, 0, false};
	rawLngData = rawNewLngData = {0, 0, false};
}

void GPSLocation::setLatitude(const char *term) {
	gpsParseDegrees(term, rawNewLatData);
}
void GPSLocation::setLongitude(const char *term) {
	gpsParseDegrees(term, rawNewLngData);
}
void GPSLocation::commit() {
	rawLatData = rawNewLatData;
	rawLngData = rawNewLngData;
	valid = true;
	updated = true;
	lastCommitTime = millis();
}
double GPSLocation::lat() {
	updated = false;
	double ret = rawLatData.deg + rawLatData.billionths / 1000000000.0;
	return rawLatData.negative ? -ret : ret;
}
double GPSLocation::lng() {
	updated = false;
	double ret = rawLngData.deg + rawLngData.billionths / 1000000000.0;
	return rawLngData.negative ? -ret : ret;
}

//-----------------------------------
GPSTime::GPSTime()
: time(0), newTime(0), lastCommitTime(0), valid(false), updated(false)
{}
void GPSTime::setTime(const char *term) {
	newTime = static_cast<uint32_t>(gpsParseDecimal(term));
}
void GPSTime::commit() {
	time = newTime;
	valid = true;
	updated = true;
	lastCommitTime = millis();
}
uint8_t GPSTime::hour() {
	updated = false;
	return time / 1000000;
}
uint8_t GPSTime::minute() {
	updated = false;
	return (time / 10000) % 100;
}
uint8_t GPSTime::second() {
	updated = false;
	return (time / 100) % 100;
}
uint8_t GPSTime::centisecond() {
	updated = false;
	return time % 100;
}

//-----------------------------------
GPSDate::GPSDate()
: date(0), newDate(0), lastCommitTime(0), valid(false), updated(false)
{}
void GPSDate::setDate(const char *term) {
	newDate = atol(term);
}
void GPSDate::commit() {
	date = newDate;
	valid = true;
	updated = true;
	lastCommitTime = millis();
}
uint16_t GPSDate::year() {
	updated = false;
	uint16_t yr = date % 100;
	return yr + 2000;
}
uint8_t GPSDate::month() {
	updated = false;
	return (date / 100) % 100;
}
uint8_t GPSDate::day() {
	updated = false;
	return date / 10000;
}

//-----------------------------------
GPSDecimal::GPSDecimal()
: val(0), newval(0), lastCommitTime(0), valid(false), updated(false)
{}
void GPSDecimal::set(const char *term) {
	newval = gpsParseDecimal(term);
}
void GPSDecimal::commit() {
	val = newval;
	valid = true;
	updated = true;
	lastCommitTime = millis();
}

//-----------------------------------
GPSInteger::GPSInteger()
: val(0), newval(0), lastCommitTime(0), valid(false), updated(false)
{}
void GPSInteger::set(const char *term) {
	newval = atol(term);
}
void GPSInteger::commit() {
	val = newval;
	valid = true;
	updated = true;
	lastCommitTime = millis();
}

//-----------------------------------
GPSCustom::GPSCustom()
	: sentenceName(nullptr), termNumber(0), next(nullptr),
	valid(false), updated(false), lastCommitTime(0)
{
	stagingBuffer[0] = '\0';
	buffer[0] = '\0';
}

GPSCustom::GPSCustom(GPSNMEA &gps, const char *sentenceName, int termNumber) {
	begin(gps, sentenceName, termNumber);
}

void GPSCustom::begin(GPSNMEA &gps, const char *sentenceName, int termNumber) {
	lastCommitTime = 0;
	valid = false;
	updated = false;
	this->sentenceName = sentenceName;
	this->termNumber = termNumber;
	stagingBuffer[0] = '\0';
	buffer[0] = '\0';
	gps.insertCustom(this, sentenceName, termNumber);
}

void GPSCustom::commit() {
	strcpy(buffer, stagingBuffer);
	valid = true;
	updated = true;
	lastCommitTime = millis();
}

void GPSCustom::set(const char *term) {
	strncpy(stagingBuffer, term, sizeof(stagingBuffer) - 1);
	stagingBuffer[sizeof(stagingBuffer) - 1] = '\0';
}

//=================================================================
// カスタムフィールド管理
//=================================================================
void GPSNMEA::insertCustom(GPSCustom *pElt, const char *sentenceName, int termNumber) {
	GPSCustom **pp = &customElts;
	while (*pp != nullptr) {
		int cmp = strcmp(sentenceName, (*pp)->sentenceName);
		if (cmp < 0 || (cmp == 0 && termNumber < (*pp)->termNumber))
			break;
		pp = &((*pp)->next);
	}
	pElt->next = *pp;
	*pp = pElt;
}
