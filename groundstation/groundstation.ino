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

public:
    void setup() override {
        int state = radio.begin(915.0);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.print(F("Ground LoRa init failed, code "));
            Serial.println(state);
            while (true);
        }
        radio.setSpreadingFactor(7);
        radio.setBandwidth(250.0);
        radio.setCodingRate(5);
        radio.setSyncWord(0xAB);
        radio.setNodeAddress(groundStationAddress);
        radio.setDio0Action([]() { operationDone = true; }, RISING);
        radio.startReceive();
        Serial.println(F("Ground LoRa initialized successfully"));
    }

    void send(const String& data) override {
        String dataCopy = data;
        Serial.println("Ground sending: " + dataCopy);
        int state = radio.startTransmit(dataCopy, fcAddress);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.print(F("Ground LoRa send failed, code "));
            Serial.println(state);
        } else {
            Serial.println("Ground LoRa send initiated");
        }
    }

    String receive() override {
        if (operationDone) {
            operationDone = false;
            int state = radio.readData(receivedData);
            if (state == RADIOLIB_ERR_NONE) {
                Serial.print("Ground LoRa received ");
                Serial.print(receivedData.length());
                Serial.println(" bytes");
                Serial.println("Ground LoRa received: " + receivedData);
                Serial.print("RSSI: ");
                Serial.print(radio.getRSSI());
                Serial.print(" dBm, SNR: ");
                Serial.print(radio.getSNR());
                Serial.println(" dB");
                radio.startReceive();
                return receivedData;
            } else {
                Serial.print(F("Ground LoRa read failed, code "));
                Serial.println(state);
                radio.startReceive();
            }
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
        // **Receive and process messages from FC**
        String message = lora.receive();
        if (message != "") {
            if (message.startsWith("[")) { // Telemetry starts with "["
                Serial.println("Received telemetry: " + message);
            } else if (message == "BUZZER_ON_ACK") {
                Serial.println("Buzzer command acknowledged");
            } else if (message == "BUZZER_ON_ERR") {
                Serial.println("Buzzer command failed");
            } else {
                Serial.println("Received unknown message: " + message);
            }
        }

        // **Send command from software via serial input**
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
    delay(20); // Check for packets every 20ms
}