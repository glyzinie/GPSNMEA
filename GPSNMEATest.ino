#include <Arduino.h>
#include "GPSNMEA.hpp"

// ------------------------------------------------------
// ハードウェア構成によってはSoftwareSerialを使うことも可能です。
// 例：SoftwareSerial gpsSerial(RXピン, TXピン);
// ------------------------------------------------------

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
// Arduino UNOなどで、SoftwareSerialを使用する場合の例
#include <SoftwareSerial.h>
SoftwareSerial gpsSerial(4, 3); // (RX=4, TX=3) 実際のピンは環境に合わせて設定
#endif

// GPSNMEAクラスのインスタンス
GPSNMEA gps;

void setup() {
	Serial.begin(115200);     // PCへデバッグ出力用

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
	gpsSerial.begin(9600);    // GPSモジュールの通信速度に合わせる（例: 9600bps）
	// (UnoやNanoなどハードウェアシリアルが1系統しかない場合)
#else
	// ハードウェアSerial1が存在するボード(MegaやLeonardoなど)なら
	// Serial1.begin(9600);  // 必要に応じて有効化
#endif

	// 初期化メッセージ
	Serial.println("GPSNMEA Test Start");
}

void loop() {
	// ------------------------------------------------------
	// 1) GPSから受信して GPSNMEA::encode() に渡す
	// ------------------------------------------------------
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
	// SoftwareSerialでの読み取り
	while (gpsSerial.available() > 0) {
		char c = gpsSerial.read();
		// GPSNMEAのencodeに1文字ずつ渡す
		if (gps.encode(c)) {
			// 1文(センテンス)の解析が完了したらtrueが返る
			printGPSInfo();
		}
	}
#else
	// 例: MegaやLeonardo等でSerial1をGPSに接続している場合
	while (Serial1.available() > 0) {
		char c = Serial1.read();
		if (gps.encode(c)) {
			printGPSInfo();
		}
	}
#endif
}

// ------------------------------------------------------
// 2) 受信結果を表示する関数
// ------------------------------------------------------
void printGPSInfo() {
	// 位置情報
	if (gps.location.isValid()) {
		double lat = gps.location.lat();
		double lng = gps.location.lng();
		Serial.print("Lat: ");
		Serial.print(lat, 6);
		Serial.print("  Lng: ");
		Serial.print(lng, 6);
	} else {
		Serial.print("Lat: ---  Lng: ---");
	}

	// 日時情報（RMCにて更新）
	if (gps.date.isValid() && gps.time.isValid()) {
		Serial.print("  Date: ");
		Serial.print(gps.date.year());
		Serial.print("/");
		Serial.print(gps.date.month());
		Serial.print("/");
		Serial.print(gps.date.day());
		Serial.print("  Time: ");
		Serial.print((int)gps.time.hour());
		Serial.print(":");
		Serial.print((int)gps.time.minute());
		Serial.print(":");
		Serial.print((int)gps.time.second());
		// centisecond も見たければ
		// Serial.print(".");
		// Serial.print((int)gps.time.centisecond());
	} else {
		Serial.print("  Date/Time: ---");
	}

	// 衛星数・HDOP・高度(GGA文)
	if (gps.satellites.isValid()) {
		Serial.print("  Sat: ");
		Serial.print(gps.satellites.value());
	}
	if (gps.hdop.isValid()) {
		Serial.print("  HDOP: ");
		Serial.print(gps.hdop.value() / 100.0, 2); // gpsParseDecimal()は100倍の値
	}
	if (gps.altitude.isValid()) {
		Serial.print("  Alt: ");
		Serial.print(gps.altitude.value() / 100.0, 2);
		Serial.print("m");
	}

	// 移動速度 (RMC文より)
	if (gps.speed.isValid()) {
		// speed.value()は "ノット" を100倍した値
		double speedKnots = gps.speed.value() / 100.0;
		// 例: ノット→km/hに変換
		double speedKmh = speedKnots * 1.852;
		Serial.print("  Speed: ");
		Serial.print(speedKmh, 2);
		Serial.print(" km/h");
	}

	// 進行方向 (course) → 16方位へ
	if (gps.course.isValid()) {
		double courseDeg = gps.course.value() / 100.0; // 度
		const char* direction = gpsCardinal(courseDeg);
		Serial.print("  Dir: ");
		Serial.print(direction);
	}

	// 結果を1行で表示
	Serial.println();
}

