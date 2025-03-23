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

// Base Sensor class
class Sensor {
public:
  virtual String read() = 0;
  virtual ~Sensor() {}
};

// Real sensor classes
class RealAccelerometer : public Sensor {
public:
  String read() override {
    accel.readSensor();
    float acc_x = accel.getAccelX_mss();
    float acc_y = accel.getAccelY_mss();
    float acc_z = accel.getAccelZ_mss();
    // No velocity data from BMI088, so zero out
    return String(acc_x, 1) + "|" + String(acc_y, 1) + "|" + String(acc_z, 1) + "|0.0|0.0|0.0";
  }
};

class RealGyroscope : public Sensor {
public:
  String read() override {
    gyro.readSensor();
    // Convert radians/sec to degrees/sec (common in flight controllers)
    float gyro_x = gyro.getGyroX_rads() * (180.0 / PI);
    float gyro_y = gyro.getGyroY_rads() * (180.0 / PI);
    float gyro_z = gyro.getGyroZ_rads() * (180.0 / PI);
    return String(gyro_x, 1) + "|" + String(gyro_y, 1) + "|" + String(gyro_z, 1);
  }
};

class RealMagnetometer : public Sensor {
public:
  String read() override {
    // Magnetometer not used, return zeros
    return "0.0|0.0|0.0";
  }
};

class RealBarometer : public Sensor {
public:
  String read() override {
    float pressure = bmp.readPressure() / 100.0F; // hPa
    float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    return String(pressure, 2) + "|" + String(altitude, 1);
  }
};

class RealPressureSensor : public Sensor {
public:
  String read() override {
    float pressure = bmp.readPressure() / 100.0F; // hPa
    return String(pressure, 2);
  }
};

class RealGPS : public Sensor {
public:
  String read() override {
    if (!hasFix) {
      return "0|0|0|0";
    }
    return lastLat + "|" + lastLon + "|" + String(satellites) + "|" + lastAltitude;
  }
};

class RealTempSensor : public Sensor {
public:
  String read() override {
    // Use BMP280 temperature (more accurate for environmental sensing)
    float temp = bmp.readTemperature();
    return String(temp, 1);
  }
};

// NAVC class
class NAVC {
private:
  Sensor* accelerometer;
  Sensor* gyroscope;
  Sensor* magnetometer;
  Sensor* barometer;
  Sensor* pressureSensor;
  Sensor* gps;
  Sensor* tempSensor;
  uint32_t id = 1;
  unsigned long lastSendTime = 0; // Added for autonomous sending

public:
  NAVC() {
    accelerometer = new RealAccelerometer();
    gyroscope = new RealGyroscope();
    magnetometer = new RealMagnetometer();
    barometer = new RealBarometer();
    pressureSensor = new RealPressureSensor();
    gps = new RealGPS();
    tempSensor = new RealTempSensor();
  }

  ~NAVC() {
    delete accelerometer;
    delete gyroscope;
    delete magnetometer;
    delete barometer;
    delete pressureSensor;
    delete gps;
    delete tempSensor;
  }

  String collectData() {
    // Get current time (using millis-based mission time since no RTC is present)
    unsigned long currentTime = millis();
    uint32_t secondsTotal = currentTime / 1000;
    uint32_t hours = secondsTotal / 3600;
    uint32_t minutes = (secondsTotal % 3600) / 60;
    uint32_t seconds = secondsTotal % 60;
    // Fake date since no RTC; using a fixed date for format consistency
    String dateStamp = "[2025/03/22 " + String(hours) + ":" + String(minutes) + ":" + String(seconds) + "]";

    // IMU data (accel and gyro)
    accel.readSensor();
    gyro.readSensor();
    String IMU = String(accel.getAccelX_mss()) + "," + String(accel.getAccelY_mss()) + "," + 
                 String(accel.getAccelZ_mss()) + "," + 
                 String(round(gyro.getGyroX_rads() * (180.0 / PI))) + "," + 
                 String(round(gyro.getGyroY_rads() * (180.0 / PI))) + "," + 
                 String(round(gyro.getGyroZ_rads() * (180.0 / PI))) + "," + 
                 String(bmp.readTemperature()); // Using BMP280 temp as accel temp substitute

    // BME data (barometer)
    String BME = String(bmp.readTemperature()) + "," + 
                 String(bmp.readPressure() / 100.0F) + "," + 
                 String(bmp.readAltitude(SEALEVELPRESSURE_HPA)) + "," + 
                 "0.0"; // No humidity sensor, so using 0.0

    // GPS data
    String GPSVal = String(hasFix) + "," + 
                    String(hasFix ? 1 : 0) + "," + // Assuming quality 1 if fix, 0 if not
                    lastLat + "," + 
                    lastLon + "," + 
                    "0.0" + "," + // No speed data available
                    lastAltitude + "," + 
                    String(satellites);

    // Combine into final packet
    String Packet = dateStamp + " " + IMU + "," + BME + "," + GPSVal;
    if (Packet.length() > 255) {
      Packet = Packet.substring(0, 255);
    }
    return Packet;
  }

  void processRequest() {
    unsigned long currentTime = millis();
    // Send data every 100ms autonomously
    if (currentTime - lastSendTime >= 100) {
      String sensorData = collectData();
      digitalWrite(LED_PIN, HIGH);
      Serial.println("NAVC sending: " + sensorData);
      SerialNavc.println(sensorData);
      delay(10); // Small delay to ensure transmission
      digitalWrite(LED_PIN, LOW);
      lastSendTime = currentTime;
    }

    // Still handle incoming requests if desired
    if (SerialNavc.available() > 0) {
      String request = SerialNavc.readStringUntil('\n');
      request.trim();
      Serial.println("NAVC received: " + request);
      if (request == "REQUEST_DATA") {
        String sensorData = collectData();
        digitalWrite(LED_PIN, HIGH);
        Serial.println("NAVC sending: " + sensorData);
        SerialNavc.println(sensorData);
        delay(10); // Small delay to ensure transmission
        digitalWrite(LED_PIN, LOW);
      }
    }
  }
};

// Global instance
NAVC navc;

void setup() {
  Serial.begin(115200);
  // Removed: while (!Serial) delay(10); // Allow code to run without Serial Monitor
  Serial1.begin(9600);
  SerialNavc.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("Starting NMEA parsing and NAVC on STM32F4 Black Pill...");

  // Initialize I2C
  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin();
  Wire.setClock(100000); // 100kHz, compatible with both sensors

  // Initialize BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 init failed!");
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  // Initialize BMI088 Accelerometer
  int status = accel.begin();
  if (status < 0) {
    Serial.println("Accel init failed!");
    while (1) delay(10);
  }

  // Initialize BMI088 Gyroscope
  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro init failed!");
    while (1) delay(10);
  }
}

void loop() {
  // Parse GPS data from Serial1
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

  // Handle NAVC requests and autonomous sending
  navc.processRequest();
}