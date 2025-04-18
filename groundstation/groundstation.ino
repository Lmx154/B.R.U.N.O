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
    virtual void send(String data) = 0;     // accept by value
    virtual String receive() = 0;
    virtual void setup() = 0;
    virtual ~Communication() {}
};

// LoRa communication class
class LoraComm : public Communication {
private:
    String receivedData;
    const uint8_t groundStationAddress = 0xA1; // Ground Station address
    const uint8_t fcAddress              = 0xA2; // FC address

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

    // Take data by value so it can bind to RadioLib's String& API
    void send(String data) override {
        int state = radio.transmit(data, fcAddress);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.println("Transmit failed: " + String(state));
        }
        radio.startReceive();
    }

    String receive() override {
        if (!operationDone) {
            return "";
        }
        operationDone = false;
        int state = radio.receive(receivedData);
        radio.startReceive();
        if (state == RADIOLIB_ERR_NONE) {
            return receivedData;
        } else {
            Serial.println("Receive failed: " + String(state));
            return "";
        }
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
        // Check for incoming telemetry from FC
        String message = lora.receive();
        if (message.length() > 0) {
            // ONLY print the raw payload
            Serial.println(message);
        }

        // Forward any serial input as commands
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            if (input.length() > 0) {
                lora.send(input);
            }
        }
    }
};

GroundStation groundStation;

void setup() {
    Serial.begin(115200);
    groundStation.setup();
}

void loop() {
    groundStation.loop();
    delay(10);
}
