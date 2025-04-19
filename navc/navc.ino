// navc.ino

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include "BMI088.h"
#include "bmm150.h"
#include "bmm150_defs.h"
#include <RTClib.h>                // ← include RTClib for DS3231

// Define Serial1 for GPS on PB7 (RX) and PB6 (TX)
HardwareSerial Serial1(PB7, PB6);  // GPS: RX, TX

// Use Serial2 for communication with the FC
#define SerialNavc Serial2

const int LED_PIN   = PC13;   // On‑board LED
const int SD_CS_PIN = PA4;    // SD CS

// Sensor objects
Adafruit_BMP280 bmp;           // BMP280 @0x77
Bmi088Accel    accel(Wire, 0x19);
Bmi088Gyro     gyro(Wire, 0x69);
BMM150         mag;            // BMM150 @0x13

RTC_DS3231     rtc;            // ← use DS3231 instead of PCF8563

String lastLat      = "0";
String lastLon      = "0";
int    satellites   = 0;
String lastAltitude = "0";
String nmeaSentence = "";
bool   hasFix       = false;

#define SEALEVELPRESSURE_HPA 1013.25F

// ─── NMEA parsing ────────────────────────────────────

void splitSentence(const String &sentence, String fields[], int maxFields) {
  int idx = 0; String cur;
  for (int i = 0; i < sentence.length() && idx < maxFields; i++) {
    if (sentence[i] == ',') {
      fields[idx++] = cur;
      cur = "";
    } else cur += sentence[i];
  }
  if (idx < maxFields) fields[idx] = cur;
}

void parseGNRMC(const String &s) {
  String f[15]; splitSentence(s, f, 15);
  if (f[2] == "A") {
    lastLat = f[3] + f[4];
    lastLon = f[5] + f[6];
    hasFix  = true;
  } else {
    hasFix = false;
    lastLat = lastLon = "0";
  }
}

void parseGNGGA(const String &s) {
  String f[15]; splitSentence(s, f, 15);
  if (f[7].toInt() > 0) {
    satellites   = f[7].toInt();
    lastAltitude = f[9].length() ? f[9] : "0";
    hasFix       = true;
  } else {
    satellites   = 0;
    lastAltitude = "0";
    if (!hasFix) lastLat = lastLon = "0";
  }
}

// ─── NAVC class ─────────────────────────────────────

class NAVC {
  unsigned long lastSuccessTime = 0;

  String collectData() {
    // ← read DS3231 timestamp first
    DateTime now = rtc.now();
    char ts[20];
    snprintf(ts, sizeof(ts),
             "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
    String timestamp = String(ts);

    // sensor reads
    accel.readSensor();
    gyro.readSensor();
    mag.read_mag_data();
    float magX = mag.raw_mag_data.raw_datax;
    float magY = mag.raw_mag_data.raw_datay;
    float magZ = mag.raw_mag_data.raw_dataz;
    float temp     = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0F;
    float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    // build CSV, *prefixing* with timestamp
    String packet = timestamp + "," +
                    String(accel.getAccelX_mss(), 2) + "," +
                    String(accel.getAccelY_mss(), 2) + "," +
                    String(accel.getAccelZ_mss(), 2) + "," +
                    String(round(gyro.getGyroX_rads() * 180.0/PI)) + "," +
                    String(round(gyro.getGyroY_rads() * 180.0/PI)) + "," +
                    String(round(gyro.getGyroZ_rads() * 180.0/PI)) + "," +
                    String(temp, 2) + "," +
                    String(pressure, 2) + "," +
                    String(altitude, 2) + "," +
                    String(magX, 2) + "," +
                    String(magY, 2) + "," +
                    String(magZ, 2) + "," +
                    lastLat + "," + lastLon + "," +
                    String(satellites) + "," +
                    lastAltitude;

    if (packet.length() > 255) packet = packet.substring(0, 255);
    return packet;
  }

  void sendPacket(const String &data) {
    String framed = "<" + data + ">";
    digitalWrite(LED_PIN, HIGH);

    // to FC
    SerialNavc.print(framed);
    SerialNavc.flush();

    // to USB‑Serial
    Serial.println(framed);

    // to SD
    File f = SD.open("data_log.txt", FILE_WRITE);
    if (f) {
      f.println(framed);
      f.close();
    }

    digitalWrite(LED_PIN, LOW);
  }

public:
  void processRequest() {
    unsigned long now = millis();

    if (SerialNavc.available() > 32) {
      while (SerialNavc.available()) SerialNavc.read();
    }

    if (SerialNavc.available()) {
      String req = SerialNavc.readStringUntil('\n');
      req.trim();
      if (req == "REQUEST_DATA") {
        sendPacket(collectData());
        lastSuccessTime = now;
      }
      while (SerialNavc.available()) SerialNavc.read();
    }
    else if (now - lastSuccessTime >= 20) {
      sendPacket(collectData());
      lastSuccessTime = now;
    }
  }
} navc;

// ─── setup & loop ────────────────────────────────────

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  Serial1.begin(9600);
  SerialNavc.begin(115200);

  // SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed!");
    while (1) delay(10);
  }

  Serial.println("Starting Autonomous NAVC...");

  // I2C on PB8/PB9
  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin();
  Wire.setClock(100000);

  // BMP280
  if (!bmp.begin(0x77)) while (1) delay(10);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  // BMI088
  if (accel.begin() < 0) while (1) delay(10);
  if (gyro.begin()  < 0) while (1) delay(10);

  // BMM150
  if (mag.initialize() != BMM150_OK) while (1) delay(10);

  // ← initialize DS3231
  if (!rtc.begin()) {
    Serial.println("RTC init failed!");
    while (1) delay(10);
  }
  // only set the RTC once — comment out after initial flash:
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void loop() {
  // GPS parsing
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '$')        nmeaSentence = "$";
    else if (c == '\n' && nmeaSentence.length()) {
      if (nmeaSentence.startsWith("$GNRMC")) parseGNRMC(nmeaSentence);
      else if (nmeaSentence.startsWith("$GNGGA")) parseGNGGA(nmeaSentence);
      nmeaSentence = "";
    }
    else if (nmeaSentence.length()) {
      nmeaSentence += c;
    }
  }

  navc.processRequest();
}
