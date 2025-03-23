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

// LoRa communication class
class LoraComm : public Communication {
private:
    String receivedData;
    uint8_t groundStationAddress = 0xA1; // Ground Station address
    uint8_t fcAddress = 0xA2;            // FC address

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
        radio.setNodeAddress(groundStationAddress);
        radio.setDio0Action([]() { operationDone = true; }, RISING);
        radio.startReceive();
        Serial.println("LoRa initialized, address: " + String(groundStationAddress));
    }

    void send(const String& data) override {
        String dataCopy = data;
        String packet = getTimestamp() + " " + dataCopy;
        Serial.println("GS sending: " + packet);
        operationDone = false;
        int state = radio.startTransmit(dataCopy, fcAddress);
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
                Serial.println(packet);
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

// Ground Station class
class GroundStation {
private:
    LoraComm lora;

public:
    void setup() {
        lora.setup();
    }
    void loop() {
        String message = lora.receive();
        if (message != "") {
            // Process silently unless it's an ACK/ERR
            if (message == "BUZZER_ON_ACK" || message == "BUZZER_ON_ERR") {
                Serial.println("GS received: " + message);
            }
        }

        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            if (input == "BUZZER_ON") {
                lora.send("BUZZER_ON");
            }
        }
    }
};

// Global instance
GroundStation groundStation;

void setup() {
    Serial.begin(115200);
    groundStation.setup();
}

void loop() {
    groundStation.loop();
    delay(10); // Reduced delay for faster response
}