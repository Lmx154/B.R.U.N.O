#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// MAC addresses
uint8_t fcMacAddress[] = {0xE8, 0x06, 0x90, 0x95, 0xEE, 0xB0};    // FC MAC
uint8_t groundStationMacAddress[] = {0x24, 0xEC, 0x4A, 0x2C, 0x4F, 0x38}; // Ground Station MAC

// Structure to send/receive data via ESP-NOW (reduced message size)
typedef struct struct_message {
    char message[240]; // Reduced to fit within ESP-NOW limit
    int rssi;
} struct_message;

struct_message sendMessage;
struct_message receivedMessage;

esp_now_peer_info_t peerInfoFC;

// Callback function executed when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback function executed when data is received
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
    Serial.print("Received from MAC: ");
    Serial.println(macStr);

    if (len <= sizeof(receivedMessage)) {
        memcpy(&receivedMessage, incomingData, sizeof(receivedMessage));
        receivedMessage.message[sizeof(receivedMessage.message) - 1] = '\0';
        Serial.println("Telemetry Data: " + String(receivedMessage.message) + ", rssi: " + String(receivedMessage.rssi));
    }
}

// Base Communication class (interface)
class Communication {
public:
    virtual void send(const String& data) = 0;
    virtual String receive() = 0;
    virtual ~Communication() {}
};

// ESP-NOW communication
class EspNowComm : public Communication {
public:
    EspNowComm() {}

    void setup() {
        WiFi.mode(WIFI_STA);
        Serial.print("This Board MAC Address: ");
        Serial.println(WiFi.macAddress());
        WiFi.setChannel(0); // Match the working example's channel
        if (esp_now_init() != ESP_OK) {
            Serial.println("Error initializing ESP-NOW");
            return;
        }
        esp_now_register_send_cb(OnDataSent);
        esp_now_register_recv_cb(OnDataRecv);

        // Register FC as a peer
        memcpy(peerInfoFC.peer_addr, fcMacAddress, 6);
        peerInfoFC.channel = 0;
        peerInfoFC.encrypt = false;
        if (esp_now_add_peer(&peerInfoFC) != ESP_OK) {
            Serial.println("Failed to add FC peer");
        } else {
            Serial.println("FC peer added successfully");
        }
    }

    void send(const String& data) override {
        if (data.length() >= sizeof(sendMessage.message)) {
            Serial.println("Data too long for buffer, truncating");
            strncpy(sendMessage.message, data.substring(0, sizeof(sendMessage.message) - 1).c_str(), sizeof(sendMessage.message));
        } else {
            strncpy(sendMessage.message, data.c_str(), sizeof(sendMessage.message));
        }
        sendMessage.message[sizeof(sendMessage.message) - 1] = '\0';
        sendMessage.rssi = 0;
        esp_err_t result = esp_now_send(fcMacAddress, (uint8_t *)&sendMessage, sizeof(sendMessage));
        if (result != ESP_OK) {
            Serial.println("ESP-NOW send failed, error code: " + String(result));
        } else {
            Serial.println("Sent: " + String(sendMessage.message));
        }
    }

    String receive() override {
        String data = String(receivedMessage.message);
        receivedMessage.message[0] = '\0';
        return data;
    }

    void sendCommand(const String& command) {
        send(command);
    }

    String receiveCommand() {
        return receive();
    }
};

// Ground Station
class GroundStation {
private:
    Communication* comm;

public:
    GroundStation(Communication* communication) : comm(communication) {}

    void setup() {
        static_cast<EspNowComm*>(comm)->setup();
    }

    void receiveAndForward() {
        // Data is handled in OnDataRecv callback
    }

    void sendCommand(const String& command) {
        static_cast<EspNowComm*>(comm)->sendCommand(command);
    }
};

// Global instances
EspNowComm espNowComm;
GroundStation groundStation(&espNowComm);

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Ground Station Started");
    groundStation.setup();
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.startsWith("SET ACTION")) {
            groundStation.sendCommand(command);
        }
    }
    delay(100);
}