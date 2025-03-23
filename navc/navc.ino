#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "BMI088.h"

// Define Serial1 as UART1 on PB6 (TX) and PB7 (RX) for GPS
HardwareSerial Serial1(PB7, PB6);  // RX, TX

// Define UART for NAVC-to-FC communication
#define SerialNavc Serial2
const int LED_PIN = PC13; // Onboard LED for data transmission indicator

// Global variables for parsed GPS data
String lastLat = "0";
String lastLon = "0";
int satellites = 0;
String lastAltitude = "0"; // GPS altitude
String nmeaSentence = "";
bool hasFix = false;

// Global sensor objects
Adafruit_BMP280 bmp;
Bmi088Accel accel(Wire, 0x18);
Bmi088Gyro gyro(Wire, 0x68);

#define SEALEVELPRESSURE_HPA (1013.25)

// Helper function to split NMEA sentence into fields
void splitSentence(String sentence, String fields[], int maxFields) {
  int fieldIndex = 0;
  String currentField = "";
  for (int i = 0; i < sentence.length() && fieldIndex < maxFields; i++) {
    if (sentence[i] == ',') {
      fields[fieldIndex] = currentField;
      currentField = "";
      fieldIndex++;
    } else {
      currentField += sentence[i];
    }
  }
  if (fieldIndex < maxFields) {
    fields[fieldIndex] = currentField;
  }
}

// Parse $GNRMC sentence for latitude and longitude
void parseGNRMC(String sentence) {
  String fields[15];
  splitSentence(sentence, fields, 15);
  if (fields[2] == "A") {
    lastLat = fields[3] + fields[4];
    lastLon = fields[5] + fields[6];
    hasFix = true;
  } else {
    hasFix = false;
    lastLat = "0";
    lastLon = "0";
  }
}

// Parse $GNGGA sentence for satellites and altitude
void parseGNGGA(String sentence) {
  String fields[15];
  splitSentence(sentence, fields, 15);
  if (fields[7] != "" && fields[7].toInt() > 0) {
    satellites = fields[7].toInt();
    if (fields[9] != "") lastAltitude = fields[9];
    hasFix = true;
  } else {
    satellites = 0;
    lastAltitude = "0";
    if (!hasFix) {
      lastLat = "0";
      lastLon = "0";
    }
  }
}

// NAVC class
class NAVC {
private:
  uint32_t id = 1;
  unsigned long lastSendTime = 0; // For autonomous sending

  String collectData() {
    accel.readSensor();
    gyro.readSensor();
    float temp = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0F;
    float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    String packet = String(accel.getAccelX_mss(), 2) + "," + 
                    String(accel.getAccelY_mss(), 2) + "," + 
                    String(accel.getAccelZ_mss(), 2) + "," + 
                    String(round(gyro.getGyroX_rads() * (180.0 / PI))) + "," + 
                    String(round(gyro.getGyroY_rads() * (180.0 / PI))) + "," + 
                    String(round(gyro.getGyroZ_rads() * (180.0 / PI))) + "," +
                    String(temp, 2) + "," + String(pressure, 2) + "," + String(altitude, 2) + "," +
                    "0.0,0.0,0.0," + lastLat + "," + lastLon + "," + String(satellites) + "," + lastAltitude;

    if (packet.length() > 255) {
      packet = packet.substring(0, 255);
    }
    return packet;
  }

  void sendPacket(const String& packet) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("NAVC sending: " + packet);
    SerialNavc.println(packet);
    delay(10); // Ensure transmission
    digitalWrite(LED_PIN, LOW);
  }

public:
  void processRequest() {
    unsigned long currentTime = millis();

    // Check for incoming request first
    if (SerialNavc.available() > 0) {
      String request = SerialNavc.readStringUntil('\n');
      request.trim();
      Serial.println("NAVC received: " + request);
      if (request == "REQUEST_DATA") {
        String packet = collectData();
        sendPacket(packet);
        lastSendTime = currentTime; // Reset timer to avoid immediate autonomous send
      }
      while (SerialNavc.available() > 0) SerialNavc.read(); // Clear buffer
    }
    // Autonomous send only if no request was processed and 500ms has passed
    else if (currentTime - lastSendTime >= 500) {
      String packet = collectData();
      sendPacket(packet);
      lastSendTime = currentTime;
    }
  }
};

// Global instance
NAVC navc;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600); // GPS
  SerialNavc.begin(115200); // NAVC-to-FC
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("Starting NMEA parsing and NAVC on STM32F4 Black Pill...");

  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin();
  Wire.setClock(100000); // 100kHz

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 init failed!");
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  int status = accel.begin();
  if (status < 0) {
    Serial.println("Accel init failed!");
    while (1) delay(10);
  }

  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro init failed!");
    while (1) delay(10);
  }
}

void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '$') {
      nmeaSentence = "$";
    } else if (c == '\n' && nmeaSentence.length() > 0) {
      if (nmeaSentence.startsWith("$GNRMC")) parseGNRMC(nmeaSentence);
      else if (nmeaSentence.startsWith("$GNGGA")) parseGNGGA(nmeaSentence);
      nmeaSentence = "";
    } else if (nmeaSentence.length() > 0) {
      nmeaSentence += c;
    }
  }

  navc.processRequest();
}