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
    }
    String receiveData() {
        if (Serial2.available() > 0) {
            String data = Serial2.readStringUntil('\n');
            data.trim();
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
        int state = radio.begin(915.0);
        if (state != RADIOLIB_ERR_NONE) {
            while (true);
        }
        radio.setSpreadingFactor(7);
        radio.setBandwidth(250.0);
        radio.setCodingRate(5);
        radio.setSyncWord(0xAB);
        radio.setNodeAddress(fcAddress);
        radio.setDio0Action([]() { operationDone = true; }, RISING);
        radio.startReceive();
    }

    void send(const String& data) override {
        String dataCopy = data;
        String packet = getTimestamp() + " " + dataCopy;
        Serial.println(packet); // Single timestamp per packet
        operationDone = false;
        int state = radio.startTransmit(dataCopy, groundStationAddress); // Send raw data
        if (state == RADIOLIB_ERR_NONE) {
            unsigned long startTime = millis();
            while (!operationDone && millis() - startTime < 1000) {
                delay(1);
            }
            radio.finishTransmit();
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

public:
    void setup() {
        lora.setup();
        pinMode(PB13, OUTPUT);
        digitalWrite(PB13, LOW);
    }

    void loop() {
        uart.requestData();
        String navcData = uart.receiveData();
        if (navcData != "") {
            lora.send(navcData); // Send directly without buffering
        }

        String command = lora.receive();
        if (command != "") {
            if (command == "BUZZER_ON") {
                digitalWrite(PB13, HIGH);
                if (digitalRead(PB13) == HIGH) {
                    buzzerStartTime = millis();
                    buzzerActive = true;
                    lora.send("BUZZER_ON_ACK");
                } else {
                    lora.send("BUZZER_ON_ERR");
                }
            }
        }

        if (buzzerActive && millis() - buzzerStartTime >= 2000) {
            digitalWrite(PB13, LOW);
            buzzerActive = false;
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
    delay(50);
}