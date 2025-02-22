#include "GPSNMEA.hpp"
#include "GPSUtils.hpp"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

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
	customElts(NULL),
	customCandidates(NULL)
{
	termBuffer[0] = '\0';
}

void GPSNMEA::reset() {
	parity = 0;
	isChecksumTerm = false;
	curTermNumber = curTermOffset = 0;
	sentenceHasFix = false;
	encodedCharCount = sentencesWithFixCount = failedChecksumCount = passedChecksumCount = 0;
	termBuffer[0] = '\0';
	customElts = NULL;
	customCandidates = NULL;
}

bool GPSNMEA::encode(char c) {
	++encodedCharCount;

	switch(c) {
		case ',':
			if (!isChecksumTerm) {
				parity ^= (uint8_t)c;
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
				parity ^= c;
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
		byte chksum = 16 * fromHex(termBuffer[0]) + fromHex(termBuffer[1]);
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
					speed.commit();   // RMC の速度
					course.commit();  // RMC の航向
					break;
				case SentenceType_GGA:
					time.commit();
					if (sentenceHasFix) {
						location.commit();
					}
					satellites.commit();  // GGA の衛星数
					hdop.commit();        // GGA のHDOP
					altitude.commit();    // GGA の標高
					break;
				default:
					break;
			}
			for (GPSCustom *p = customCandidates;
			p != NULL && strcmp(p->sentenceName, customCandidates->sentenceName) == 0;
			p = p->next) {
				p->commit();
			}
			return true;
		} else {
			failedChecksumCount++;
		}
		return false;
	}

	if (curTermNumber == 0) {
		// 文識別子の判定
		if ((strchr("GB", termBuffer[0]) != NULL) &&
			(strchr("PNABLD", termBuffer[1]) != NULL) &&
			(strcmp(termBuffer + 2, "RMC") == 0)) {
			curSentenceType = SentenceType_RMC;
		} else if ((strchr("GB", termBuffer[0]) != NULL) &&
			(strchr("PNABLD", termBuffer[1]) != NULL) &&
			(strcmp(termBuffer + 2, "GGA") == 0)) {
			curSentenceType = SentenceType_GGA;
		} else {
			curSentenceType = SentenceType_Other;
		}
		for (customCandidates = customElts; customCandidates != NULL &&
			strcmp(customCandidates->sentenceName, termBuffer) < 0;
			customCandidates = customCandidates->next);
		if (customCandidates != NULL && strcmp(customCandidates->sentenceName, termBuffer) > 0) {
			customCandidates = NULL;
		}
		return false;
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
		}
	}

	for (GPSCustom *p = customCandidates;
	p != NULL && strcmp(p->sentenceName, customCandidates->sentenceName) == 0 &&
	p->termNumber <= curTermNumber;
	p = p->next) {
		if (p->termNumber == curTermNumber) {
			p->set(termBuffer);
		}
	}
	return false;
}

//
// GPSLocation の実装
//
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

//
// GPSTime の実装
//
void GPSTime::setTime(const char *term) {
	newTime = (uint32_t)gpsParseDecimal(term);
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

//
// GPSDate の実装
//
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

//
// GPSDecimal の実装
//
void GPSDecimal::set(const char *term) {
	newval = gpsParseDecimal(term);
}
void GPSDecimal::commit() {
	val = newval;
	lastCommitTime = millis();
	valid = updated = true;
}

//
// GPSInteger の実装
//
void GPSInteger::set(const char *term) {
	newval = atol(term);
}
void GPSInteger::commit() {
	val = newval;
	lastCommitTime = millis();
	valid = updated = true;
}

//
// GPSCustom の実装
//
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

//
// GPSNMEA 内部：カスタムフィールド登録
//
void GPSNMEA::insertCustom(GPSCustom *pElt, const char *sentenceName, int termNumber) {
	GPSCustom **ppElt;
	for (ppElt = &customElts; *ppElt != NULL; ppElt = &((*ppElt)->next)) {
		int cmp = strcmp(sentenceName, (*ppElt)->sentenceName);
		if (cmp < 0 || (cmp == 0 && termNumber < (*ppElt)->termNumber))
			break;
	}
	pElt->next = *ppElt;
	*ppElt = pElt;
}
