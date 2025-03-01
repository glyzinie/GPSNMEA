#include "GPSNMEA.hpp"
#include "GPSUtils.hpp"
#include <Arduino.h>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <cmath>

// コンストラクタ
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
	termBuffer[0] = '\0';
	// 初期化：GSA
	gsa.mode = '\0';
	gsa.fixType = 0;
	for (int i = 0; i < 12; i++) {
		gsa.satPrn[i] = 0;
	}
	gsa.pdop = gsa.hdop = gsa.vdop = 0.0;
	gsa.valid = false;
	// 初期化：GSV
	gsv.totalMessages = 0;
	gsv.messageNumber = 0;
	gsv.satellitesInView = 0;
	for (int i = 0; i < 4; i++) {
		gsv.satellites[i].prn = 0;
		gsv.satellites[i].elevation = 0;
		gsv.satellites[i].azimuth = 0;
		gsv.satellites[i].snr = 0;
	}
	gsv.valid = false;
	// 初期化：VTG
	vtg.trueTrack = 0.0;
	vtg.magneticTrack = 0.0;
	vtg.speedKnots = 0.0;
	vtg.speedKmph = 0.0;
	vtg.valid = false;
}

void GPSNMEA::reset() {
	parity = 0;
	isChecksumTerm = false;
	curTermNumber = curTermOffset = 0;
	sentenceHasFix = false;
	encodedCharCount = sentencesWithFixCount = failedChecksumCount = passedChecksumCount = 0;
	termBuffer[0] = '\0';
	customElts = nullptr;
	customCandidates = nullptr;
	// リセット：GSA, GSV, VTG
	gsa.valid = false;
	gsv.valid = false;
	vtg.valid = false;
}

bool GPSNMEA::encode(char c) {
	++encodedCharCount;

	switch(c) {
		case ',':
			if (!isChecksumTerm) {
				parity ^= static_cast<uint8_t>(c);
			}
		// FALLTHROUGH
		case '\r':
		case '\n':
		case '*': {
			bool validSentence = false;
			if (curTermOffset < sizeof(termBuffer)) {
				termBuffer[curTermOffset] = '\0';
			}
			validSentence = endOfTermHandler();
			++curTermNumber;
			curTermOffset = 0;
			isChecksumTerm = (c == '*');
			return validSentence;
		}
		case '$': {  // 文頭
			curTermNumber = curTermOffset = 0;
			parity = 0;
			curSentenceType = SentenceType_Other;
			isChecksumTerm = false;
			sentenceHasFix = false;
			termBuffer[0] = '\0';
			return false;
		}
		default: {
			if (curTermOffset < (sizeof(termBuffer) - 1)) {
				termBuffer[curTermOffset++] = c;
			}
			if (!isChecksumTerm) {
				parity ^= static_cast<uint8_t>(c);
			}
			return false;
		}
	}
	return false;
}

int GPSNMEA::fromHex(char a) {
	return gpsFromHex(a);
}

bool GPSNMEA::endOfTermHandler() {
	if (isChecksumTerm) {
		// チェックサム計算: termBuffer の先頭2文字を使用
		uint8_t chksum = 16 * fromHex(termBuffer[0]) + fromHex(termBuffer[1]);
		if (chksum == parity) {
			passedChecksumCount++;
			if (sentenceHasFix) {
				sentencesWithFixCount++;
			}

			// チェックサム通過後、各フィールドの commit を行う
			switch(curSentenceType) {
				case SentenceType_RMC:
					date.commit();
					time.commit();
					if (sentenceHasFix) {
						location.commit();
					}
					speed.commit();
					course.commit();
					break;
				case SentenceType_GGA:
					time.commit();
					if (sentenceHasFix) {
						location.commit();
					}
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
			for (GPSCustom *p = customCandidates; p != nullptr && strcmp(p->sentenceName, customCandidates->sentenceName) == 0; p = p->next) {
				p->commit();
			}
			return true;
		} else {
			failedChecksumCount++;
		}
		return false;
	}

	if (curTermNumber == 0) {
		// 文識別子の判定（GSA, GSV, VTG を追加）
		if ((strchr("GB", termBuffer[0]) != nullptr) &&
			(strchr("PNABLD", termBuffer[1]) != nullptr)) {
			if (strcmp(termBuffer + 2, "RMC") == 0) {
				curSentenceType = SentenceType_RMC;
			} else if (strcmp(termBuffer + 2, "GGA") == 0) {
				curSentenceType = SentenceType_GGA;
			} else if (strcmp(termBuffer + 2, "GSA") == 0) {
				curSentenceType = SentenceType_GSA;
			} else if (strcmp(termBuffer + 2, "GSV") == 0) {
				curSentenceType = SentenceType_GSV;
			} else if (strcmp(termBuffer + 2, "VTG") == 0) {
				curSentenceType = SentenceType_VTG;
			} else {
				curSentenceType = SentenceType_Other;
			}
			for (customCandidates = customElts; customCandidates != nullptr &&
				strcmp(customCandidates->sentenceName, termBuffer) < 0;
				customCandidates = customCandidates->next);
			if (customCandidates != nullptr && strcmp(customCandidates->sentenceName, termBuffer) > 0) {
				customCandidates = nullptr;
			}
			return false;
		}
	}

	if (curSentenceType != SentenceType_Other && termBuffer[0]) {
		if (curSentenceType == SentenceType_RMC) {
			switch(curTermNumber) {
				case 1:
					time.setTime(termBuffer);
					break;
				case 2:
					sentenceHasFix = (termBuffer[0] == 'A');
					break;
				case 3:
					location.setLatitude(termBuffer);
					break;
				case 4:
					location.rawNewLatData.negative = (termBuffer[0] == 'S');
					break;
				case 5:
					location.setLongitude(termBuffer);
					break;
				case 6:
					location.rawNewLngData.negative = (termBuffer[0] == 'W');
					break;
				case 7:
					speed.set(termBuffer);
					break;
				case 8:
					course.set(termBuffer);
					break;
				case 9:
					date.setDate(termBuffer);
					break;
				default:
					break;
			}
		} else if (curSentenceType == SentenceType_GGA) {
			switch(curTermNumber) {
				case 1:
					time.setTime(termBuffer);
					break;
				case 2:
					location.setLatitude(termBuffer);
					break;
				case 3:
					location.rawNewLatData.negative = (termBuffer[0] == 'S');
					break;
				case 4:
					location.setLongitude(termBuffer);
					break;
				case 5:
					location.rawNewLngData.negative = (termBuffer[0] == 'W');
					break;
				case 6:
					sentenceHasFix = (termBuffer[0] > '0');
					break;
				case 7:
					satellites.set(termBuffer);
					break;
				case 8:
					hdop.set(termBuffer);
					break;
				case 9:
					altitude.set(termBuffer);
					break;
				default:
					break;
			}
		} else if (curSentenceType == SentenceType_GSA) {
			switch(curTermNumber) {
				case 1:
					if (termBuffer[0] != '\0')
						gsa.mode = termBuffer[0];
					break;
				case 2:
					gsa.fixType = atoi(termBuffer);
					break;
				case 3: case 4: case 5: case 6:
				case 7: case 8: case 9: case 10:
				case 11: case 12: case 13: case 14: {
					int index = curTermNumber - 3;
					if (index >= 0 && index < 12)
						gsa.satPrn[index] = atoi(termBuffer);
					break;
				}
				case 15:
					gsa.pdop = atof(termBuffer);
					break;
				case 16:
					gsa.hdop = atof(termBuffer);
					break;
				case 17:
					gsa.vdop = atof(termBuffer);
					break;
				default:
					break;
			}
		} else if (curSentenceType == SentenceType_GSV) {
			switch(curTermNumber) {
				case 1:
					gsv.totalMessages = atoi(termBuffer);
					break;
				case 2:
					gsv.messageNumber = atoi(termBuffer);
					break;
				case 3:
					gsv.satellitesInView = atoi(termBuffer);
					break;
				default: {
					int fieldIndex = curTermNumber - 4;
					int satIndex = fieldIndex / 4;
					int field = fieldIndex % 4;
					if (satIndex < 4) {
						switch (field) {
							case 0:
								gsv.satellites[satIndex].prn = atoi(termBuffer);
								break;
							case 1:
								gsv.satellites[satIndex].elevation = atoi(termBuffer);
								break;
							case 2:
								gsv.satellites[satIndex].azimuth = atoi(termBuffer);
								break;
							case 3:
								gsv.satellites[satIndex].snr = atoi(termBuffer);
								break;
						}
					}
					break;
				}
			}
		} else if (curSentenceType == SentenceType_VTG) {
			switch(curTermNumber) {
				case 1:
					vtg.trueTrack = atof(termBuffer);
					break;
				case 2:
					break;
				case 3:
					vtg.magneticTrack = atof(termBuffer);
					break;
				case 4:
					break;
				case 5:
					vtg.speedKnots = atof(termBuffer);
					break;
				case 6:
					break;
				case 7:
					vtg.speedKmph = atof(termBuffer);
					break;
				case 8:
					break;
				default:
					break;
			}
		}
	}

	for (GPSCustom *p = customCandidates; p != nullptr && strcmp(p->sentenceName, customCandidates->sentenceName) == 0 &&
	p->termNumber <= curTermNumber; p = p->next) {
		if (p->termNumber == curTermNumber) {
			p->set(termBuffer);
		}
	}
	return false;
}

// --- 以下、各サブクラスの実装 ---

void GPSLocation::setLatitude(const char *term) {
	gpsParseDegrees(term, rawNewLatData);
}
void GPSLocation::setLongitude(const char *term) {
	gpsParseDegrees(term, rawNewLngData);
}
void GPSLocation::commit() {
	rawLatData = rawNewLatData;
	rawLngData = rawNewLngData;
	lastCommitTime = millis();
	valid = updated = true;
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

void GPSTime::setTime(const char *term) {
	newTime = static_cast<uint32_t>(gpsParseDecimal(term));
}
void GPSTime::commit() {
	time = newTime;
	lastCommitTime = millis();
	valid = updated = true;
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

void GPSDate::setDate(const char *term) {
	newDate = atol(term);
}
void GPSDate::commit() {
	date = newDate;
	lastCommitTime = millis();
	valid = updated = true;
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

void GPSDecimal::set(const char *term) {
	newval = gpsParseDecimal(term);
}
void GPSDecimal::commit() {
	val = newval;
	lastCommitTime = millis();
	valid = updated = true;
}

void GPSInteger::set(const char *term) {
	newval = atol(term);
}
void GPSInteger::commit() {
	val = newval;
	lastCommitTime = millis();
	valid = updated = true;
}

GPSCustom::GPSCustom(GPSNMEA &gps, const char *sentenceName, int termNumber) {
	begin(gps, sentenceName, termNumber);
}
void GPSCustom::begin(GPSNMEA &gps, const char *sentenceName, int termNumber) {
	lastCommitTime = 0;
	updated = valid = false;
	this->sentenceName = sentenceName;
	this->termNumber = termNumber;
	memset(stagingBuffer, '\0', sizeof(stagingBuffer));
	memset(buffer, '\0', sizeof(buffer));
	gps.insertCustom(this, sentenceName, termNumber);
}
void GPSCustom::commit() {
	strcpy(buffer, stagingBuffer);
	lastCommitTime = millis();
	valid = updated = true;
}
void GPSCustom::set(const char *term) {
	strncpy(stagingBuffer, term, sizeof(stagingBuffer) - 1);
}

void GPSNMEA::insertCustom(GPSCustom *pElt, const char *sentenceName, int termNumber) {
	GPSCustom **ppElt;
	for (ppElt = &customElts; *ppElt != nullptr; ppElt = &((*ppElt)->next)) {
		int cmp = strcmp(sentenceName, (*ppElt)->sentenceName);
		if (cmp < 0 || (cmp == 0 && termNumber < (*ppElt)->termNumber))
			break;
	}
	pElt->next = *ppElt;
	*ppElt = pElt;
}
