#include <Arduino.h>

// Use Serial1 for NAVC-to-FC communication (UART0 on Pico 2, GP0/TX, GP1/RX)
#define SerialNavc Serial1

// LED pin definition
const int LED_PIN = 25; // GP25 for LED indicator

// Base Sensor class (interface)
class Sensor {
public:
    virtual String read() = 0;
    virtual ~Sensor() {}
};

// Simulated Barometer (BMI088)
class SimulatedBarometer : public Sensor {
public:
    String read() override {
        float pressure = random(95000, 105000) / 100.0;
        float altitude = random(0, 10000) / 10.0;
        return String(pressure, 2) + "|" + String(altitude, 1);
    }
};

// Simulated Magnetometer (IIS2MDCTR)
class SimulatedMagnetometer : public Sensor {
public:
    String read() override {
        float mag_x = random(-500, 500) / 10.0;
        float mag_y = random(-500, 500) / 10.0;
        float mag_z = random(-500, 500) / 10.0;
        return String(mag_x, 1) + "|" + String(mag_y, 1) + "|" + String(mag_z, 1);
    }
};

// Simulated Pressure Sensor (MPRLS0025PA00001A)
class SimulatedPressureSensor : public Sensor {
public:
    String read() override {
        float pressure = random(95000, 105000) / 100.0;
        return String(pressure, 2);
    }
};

// Simulated GPS Receiver (u-blox MAX-8Q)
class SimulatedGPS : public Sensor {
private:
    uint8_t satellites = 8;

public:
    String read() override {
        float lat = 37.7749 + (random(-100, 100) / 10000.0);
        float lon = -122.4194 + (random(-100, 100) / 10000.0);
        satellites = random(4, 12);
        return String(lat, 2) + "|" + String(lon, 2) + "|" + String(satellites);
    }
};

// Simulated Temperature Sensor (TMP100)
class SimulatedTempSensor : public Sensor {
public:
    String read() override {
        float temp = random(200, 300) / 10.0;
        return String(temp, 1);
    }
};

// Simulated BMI088 Accelerometer
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

// Simulated BMI088 Gyroscope
class SimulatedGyroscope : public Sensor {
public:
    String read() override {
        float pitch = random(-1000, 1000) / 10.0;
        float roll = random(-1000, 1000) / 10.0;
        float yaw = random(-1000, 1000) / 10.0;
        return String(pitch, 1) + "|" + String(roll, 1) + "|" + String(yaw, 1);
    }
};

// Navigation Computer (NAVC)
class NAVC {
private:
    Sensor* barometer;
    Sensor* magnetometer;
    Sensor* pressureSensor;
    Sensor* gps;
    Sensor* tempSensor;
    Sensor* accelerometer;
    Sensor* gyroscope;
    uint32_t id = 1;

public:
    NAVC() {
        barometer = new SimulatedBarometer();
        magnetometer = new SimulatedMagnetometer();
        pressureSensor = new SimulatedPressureSensor();
        gps = new SimulatedGPS();
        tempSensor = new SimulatedTempSensor();
        accelerometer = new SimulatedAccelerometer();
        gyroscope = new SimulatedGyroscope();
    }

    ~NAVC() {
        delete barometer;
        delete magnetometer;
        delete pressureSensor;
        delete gps;
        delete tempSensor;
        delete accelerometer;
        delete gyroscope;
    }

    String collectData() {
        unsigned long currentTime = millis();
        uint32_t minute = (currentTime / 60000) % 60;
        uint32_t second = (currentTime / 1000) % 60;
        String missionTime = String(minute) + "m" + String(second) + "s";

        // Positional data with '|' as delimiter
        String data = "[id:" + String(id) + ",";
        data += "mt:" + missionTime + ","; // Shortened mission_time to mt
        data += "con:1,"; // Shortened connected to con
        data += accelerometer->read() + ","; // acc_x|acc_y|acc_z|vel_x|vel_y|vel_z
        data += gyroscope->read() + ",";     // pitch|roll|yaw
        data += magnetometer->read() + ",";  // mag_x|mag_y|mag_z
        data += barometer->read() + ",";     // baro_press|altitude
        data += pressureSensor->read() + ",";// press
        data += gps->read() + ",";           // lat|lon|sat
        data += tempSensor->read() + ",";    // temp
        data += "bat:" + String(random(370, 420) / 100.0, 2) + ","; // Shortened battery to bat
        data += "min:" + String(minute) + ","; // Shortened minute to min
        data += "sec:" + String(second) + "]"; // Shortened second to sec

        return data;
    }

    void processRequest() {
        if (SerialNavc.available() > 0) {
            String request = SerialNavc.readStringUntil('\n');
            request.trim();
            if (request == "REQUEST_DATA") {
                String sensorData = collectData();
                digitalWrite(LED_PIN, HIGH); // Turn LED on
                SerialNavc.println(sensorData); // Send data to FC via UART
                delay(100); // Keep LED on for 100ms to make it visible
                digitalWrite(LED_PIN, LOW); // Turn LED off
            }
        }
    }
};

// Global NAVC instance
NAVC navc;

void setup() {
    SerialNavc.begin(115200, SERIAL_8N1);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
}

void loop() {
    navc.processRequest();
    delay(100);
}