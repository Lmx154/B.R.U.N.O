#include <Arduino.h>
#include <RadioLib.h>

// LoRa pins
#define NSS   PA4
#define DIO0  PA8
#define NRST  PA9
#define DIO1  PA10
SX1276 radio = new Module(NSS, DIO0, NRST, DIO1);

// Global flag for operation completion
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
        Serial.println("FC sent: REQUEST_DATA to NAVC");
    }
    String receiveData() {
        if (Serial2.available() > 0) {
            String data = Serial2.readStringUntil('\n');
            data.trim();
            Serial.println("FC received from NAVC: " + data);
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

public:
    void setup() override {
        int state = radio.begin(915.0);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.print(F("FC LoRa init failed, code "));
            Serial.println(state);
            while (true);
        }
        // Optimize LoRa for speed
        radio.setSpreadingFactor(7);  // Lower SF for faster transmission
        radio.setBandwidth(250.0);    // Higher bandwidth for speed
        radio.setCodingRate(5);       // Minimal error correction for speed
        radio.setSyncWord(0xAB);
        radio.setNodeAddress(fcAddress);
        radio.setDio0Action([]() { operationDone = true; }, RISING);
        radio.startReceive();
        Serial.println(F("FC LoRa initialized successfully"));
    }

    void send(const String& data) override {
        String dataCopy = data;
        Serial.print("FC sending to ground station: ");
        Serial.print(dataCopy.length());
        Serial.println(" bytes");
        Serial.println(dataCopy);
        int state = radio.startTransmit(dataCopy, groundStationAddress);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.print(F("FC LoRa send failed, code "));
            Serial.println(state);
        } else {
            Serial.println("FC LoRa send initiated");
        }
    }

    String receive() override {
        if (operationDone) {
            operationDone = false;
            int state = radio.readData(receivedData);
            if (state == RADIOLIB_ERR_NONE) {
                Serial.println("FC LoRa received: " + receivedData);
                radio.startReceive();
                return receivedData;
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

public:
    void setup() {
        lora.setup();
    }
    void loop() {
        uart.requestData();
        String navcData = uart.receiveData();
        if (navcData != "") {
            lora.send(navcData);
            // Wait for transmission with timeout
            unsigned long startTime = millis();
            while (!operationDone && millis() - startTime < 500) { // 500ms timeout
                delay(1);
            }
            if (!operationDone) {
                Serial.println("Transmission timeout - resetting LoRa");
                radio.startReceive(); // Reset LoRa state
            }
            operationDone = false;
        }
        lora.receive(); // Check for commands (if any)
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
    delay(50); // Request data every 50ms for faster telemetry
}