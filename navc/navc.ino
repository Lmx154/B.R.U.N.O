#include <Arduino.h>

// Define UART for NAVC-to-FC communication
#define SerialNavc Serial2
const int LED_PIN = PC13; // Onboard LED for data transmission indicator

// Base Sensor class
class Sensor {
public:
    virtual String read() = 0;    // Pure virtual function for reading sensor data
    virtual ~Sensor() {}          // Virtual destructor for proper cleanup
};

// SimulatedAccelerometer class
class SimulatedAccelerometer : public Sensor {
public:
    String read() override {
        float acc_x = random(-100, 100) / 10.0;
        float acc_y = random(-100, 100) / 10.0;
        float acc_z = random(-100, 100) / 10.0;
        float vel_x = random(-50, 50) / 10.0;
        float vel_y = random(-50, 50) / 10.0;
        float vel_z = random(-50, 50) / 10.0;
        return String(acc_x, 1) + "|" + String(acc_y, 1) + "|" + String(acc_z, 1) + "|" +
               String(vel_x, 1) + "|" + String(vel_y, 1) + "|" + String(vel_z, 1);
    }
};

// SimulatedGyroscope class
class SimulatedGyroscope : public Sensor {
public:
    String read() override {
        float pitch = random(-1000, 1000) / 10.0;
        float roll = random(-1000, 1000) / 10.0;
        float yaw = random(-1000, 1000) / 10.0;
        return String(pitch, 1) + "|" + String(roll, 1) + "|" + String(yaw, 1);
    }
};

// SimulatedMagnetometer class
class SimulatedMagnetometer : public Sensor {
public:
    String read() override {
        float mag_x = random(-500, 500) / 10.0;
        float mag_y = random(-500, 500) / 10.0;
        float mag_z = random(-500, 500) / 10.0;
        return String(mag_x, 1) + "|" + String(mag_y, 1) + "|" + String(mag_z, 1);
    }
};

// SimulatedBarometer class
class SimulatedBarometer : public Sensor {
public:
    String read() override {
        float pressure = random(95000, 105000) / 100.0;
        float altitude = random(0, 10000) / 10.0;
        return String(pressure, 2) + "|" + String(altitude, 1);
    }
};

// SimulatedPressureSensor class
class SimulatedPressureSensor : public Sensor {
public:
    String read() override {
        return String(random(95000, 105000) / 100.0, 2);
    }
};

// SimulatedGPS class
class SimulatedGPS : public Sensor {
public:
    String read() override {
        float lat = 37.77 + (random(-100, 100) / 10000.0);
        float lon = -122.43 + (random(-100, 100) / 10000.0);
        uint8_t satellites = random(4, 12);
        return String(lat, 2) + "|" + String(lon, 2) + "|" + String(satellites);
    }
};

// SimulatedTempSensor class
class SimulatedTempSensor : public Sensor {
public:
    String read() override {
        return String(random(200, 300) / 10.0, 1);
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

public:
    NAVC() {
        accelerometer = new SimulatedAccelerometer();
        gyroscope = new SimulatedGyroscope();
        magnetometer = new SimulatedMagnetometer();
        barometer = new SimulatedBarometer();
        pressureSensor = new SimulatedPressureSensor();
        gps = new SimulatedGPS();
        tempSensor = new SimulatedTempSensor();
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
        unsigned long currentTime = millis();
        uint32_t minute = (currentTime / 60000) % 60;
        uint32_t second = (currentTime / 1000) % 60;
        String missionTime = String(minute) + "m" + String(second) + "s";

        String data = "[id:" + String(id) + ",mt:" + missionTime + ",con:1,";
        data += accelerometer->read() + ",";
        data += gyroscope->read() + ",";
        data += magnetometer->read() + ",";
        data += barometer->read() + ",";
        data += pressureSensor->read() + ",";
        data += gps->read() + ",";
        data += tempSensor->read() + ",";
        data += String(random(370, 420) / 100.0, 2) + ",";
        data += String(minute) + ",";
        data += String(second) + "]";
        return data;
    }

    void processRequest() {
        if (SerialNavc.available() > 0) {
            String request = SerialNavc.readStringUntil('\n');
            request.trim();
            Serial.println("NAVC received: " + request); // Debug: Show received command
            if (request == "REQUEST_DATA") {
                String sensorData = collectData();
                digitalWrite(LED_PIN, HIGH);
                Serial.println("NAVC sending: " + sensorData); // Debug: Show data being sent
                SerialNavc.println(sensorData);
                delay(100);
                digitalWrite(LED_PIN, LOW);
            }
        }
    }
};

// Global instance
NAVC navc;

void setup() {
    Serial.begin(115200);    // Serial for debugging
    SerialNavc.begin(115200); // Serial2 for NAVC communication
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

void loop() {
    navc.processRequest();
    delay(100);
}