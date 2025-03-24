#include <Arduino.h>
#include <RadioLib.h>
#include <HardwareTimer.h>

// LoRa pins
#define NSS   PA4
#define DIO0  PA8
#define NRST  PA9
#define DIO1  PA10
SX1276 radio = new Module(NSS, DIO0, NRST, DIO1);

// Servo pin and PWM settings
#define SERVO_PIN PB10          // Servo connected to PB10 (Timer 2, Channel 3)
const int pwmFreq = 200;        // 200 Hz PWM frequency for servo
HardwareTimer *servoTimer = nullptr; // HardwareTimer object for PWM
const int closedDuty = 71;      // 20% duty cycle (0.20 * 255)
const int openDuty = 112;       // 50% duty cycle (0.50 * 255)
bool isServoOpen = false;       // Tracks if servo is currently open

// Altitude settings (in meters)
const float TARGET_ALTITUDE = 1828.8; // Target altitude in meters (above sea level)
float launchAltitude = 0.0;          // Launch site altitude in meters (set during setup)

// Global flag for LoRa operation completion
volatile bool operationDone = false;

// Communication interface
class Communication {
public:
    virtual void send(const String& data) = 0;
    virtual String receive() = 0;
    virtual void setup() = 0;
    virtual ~Communication() {}
};

// UART communication with NAVC
class UartComm {
public:
    void requestData() {
        Serial2.println("REQUEST_DATA");
    }
    String receiveData() {
        if (Serial2.available() > 0) {
            String data = Serial2.readStringUntil('\n');
            data.trim();
            Serial.println("FC received from NAVC: " + data);
            while (Serial2.available() > 0) Serial2.read();
            return data;
        }
        return "";
    }
};

// LoRa communication with Ground Station
class LoraComm : public Communication {
private:
    String receivedData;
    uint8_t fcAddress = 0xA2;            // FC address
    uint8_t groundStationAddress = 0xA1; // Ground Station address

    String getTimestamp() {
        unsigned long currentTime = millis();
        int seconds = (currentTime / 1000) % 60;
        int minutes = (currentTime / 60000) % 60;
        int hours = (currentTime / 3600000) % 24;
        return String("[2025/03/22 ") + (hours < 10 ? "0" : "") + hours + ":" +
               (minutes < 10 ? "0" : "") + minutes + ":" + 
               (seconds < 10 ? "0" : "") + seconds + "]";
    }

public:
    void setup() override {
        Serial.println("Initializing LoRa...");
        int state = radio.begin(915.0);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.println("LoRa init failed: " + String(state));
            while (true);
        }
        radio.setSpreadingFactor(7);
        radio.setBandwidth(250.0);
        radio.setCodingRate(5);
        radio.setSyncWord(0xAB);
        radio.setNodeAddress(fcAddress);
        radio.setDio0Action([]() { operationDone = true; }, RISING);
        radio.startReceive();
        Serial.println("LoRa initialized, address: " + String(fcAddress));
    }

    void send(const String& data) override {
        String dataCopy = data;
        String packet = getTimestamp() + " " + dataCopy;
        Serial.println(packet);
        operationDone = false;
        int state = radio.startTransmit(dataCopy, groundStationAddress);
        if (state == RADIOLIB_ERR_NONE) {
            unsigned long startTime = millis();
            while (!operationDone && millis() - startTime < 1000) {
                delay(1);
            }
            radio.finishTransmit();
        } else {
            Serial.println("Transmit failed: " + String(state));
        }
        radio.startReceive();
    }

    String receive() override {
        if (operationDone) {
            operationDone = false;
            int state = radio.readData(receivedData);
            if (state == RADIOLIB_ERR_NONE) {
                String packet = getTimestamp() + " " + receivedData;
                Serial.println("FC received from GS: " + packet);
                radio.startReceive();
                return receivedData;
            } else {
                Serial.println("Receive failed: " + String(state));
            }
            radio.startReceive();
        }
        return "";
    }
};

// FC class
class FC {
private:
    UartComm uart;
    LoraComm lora;
    bool buzzerActive = false;
    unsigned long buzzerStartTime = 0;
    bool launchAltitudeSet = false; // Flag to set initial altitude once

    // Parse NAVC data to extract altitude (index 8 in the packet)
    float getAltitude(const String& data) {
        int commaCount = 0;
        String value = "";
        for (int i = 0; i < data.length(); i++) {
            if (data[i] == ',') {
                commaCount++;
                if (commaCount == 8) { // Altitude is the 9th value (index 8)
                    i++; // Skip the comma
                    while (i < data.length() && data[i] != ',') {
                        value += data[i];
                        i++;
                    }
                    return value.toFloat(); // Altitude already in meters from BMP280
                }
            }
        }
        return 0.0; // Default if parsing fails
    }

    // Control servo based on altitude
    void controlServo(float currentAltitude) {
        // Set launch altitude on first valid reading
        if (!launchAltitudeSet && currentAltitude > 0) {
            launchAltitude = currentAltitude;
            launchAltitudeSet = true;
            Serial.println("Launch altitude set to: " + String(launchAltitude) + " m");
        }

        // Ensure servo stays closed until target altitude is reached
        if (!isServoOpen && currentAltitude < TARGET_ALTITUDE && launchAltitudeSet) {
            servoTimer->setPWM(3, SERVO_PIN, pwmFreq, closedDuty * 100 / 255); // Keep servo closed
        }

        // Open servo at target altitude and leave it open
        if (!isServoOpen && currentAltitude >= TARGET_ALTITUDE && launchAltitudeSet) {
            servoTimer->setPWM(3, SERVO_PIN, pwmFreq, openDuty * 100 / 255); // Open servo
            isServoOpen = true;
            Serial.println("Servo opened at altitude: " + String(currentAltitude) + " m");
        }
    }

public:
    void setup() {
        // Initialize LoRa
        lora.setup();

        // Initialize buzzer pin
        pinMode(PB13, OUTPUT);
        digitalWrite(PB13, LOW);

        // Initialize servo PWM on PB10 (Timer 2, Channel 3)
        pinMode(SERVO_PIN, OUTPUT);
        servoTimer = new HardwareTimer(TIM2);
        servoTimer->setPWM(3, SERVO_PIN, pwmFreq, closedDuty * 100 / 255); // Start and rest at closed
        Serial.println("Servo initialized on PB10, resting at closed duty");
    }

    void loop() {
        // Request and process NAVC data
        uart.requestData();
        String navcData = uart.receiveData();
        if (navcData != "") {
            lora.send(navcData);
            float altitude = getAltitude(navcData); // Extract altitude
            controlServo(altitude);                 // Control servo based on altitude
        }

        // Process LoRa commands from Ground Station
        String command = lora.receive();
        if (command != "") {
            if (command == "BUZZER_ON") {
                digitalWrite(PB13, HIGH);
                Serial.println("Buzzer activated");
                if (digitalRead(PB13) == HIGH) {
                    buzzerStartTime = millis();
                    buzzerActive = true;
                    lora.send("BUZZER_ON_ACK");
                } else {
                    lora.send("BUZZER_ON_ERR");
                    Serial.println("Buzzer failed to activate");
                }
            }
        }

        // Turn off buzzer after 2 seconds
        if (buzzerActive && millis() - buzzerStartTime >= 2000) {
            digitalWrite(PB13, LOW);
            buzzerActive = false;
            Serial.println("Buzzer deactivated");
        }
    }
};

// Global instance
FC fc;

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
    fc.setup();
}

void loop() {
    fc.loop();
    delay(10); // Reduced delay for faster response
}