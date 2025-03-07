#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Use UART0 for FC-to-NAVC communication (default pins for ESP32-S3-Zero)
#define FC_TX 43  // GPIO43 (TX)
#define FC_RX 44  // GPIO44 (RX)
HardwareSerial SerialNavc(0); // UART0 on ESP32-S3

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

esp_now_peer_info_t peerInfoGroundStation;

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
        Serial.println("Received Command: " + String(receivedMessage.message));
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
private:
    String lastReceivedCommand;
    int32_t rssi = -50;

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

        // Register Ground Station as a peer
        memcpy(peerInfoGroundStation.peer_addr, groundStationMacAddress, 6);
        peerInfoGroundStation.channel = 0;
        peerInfoGroundStation.encrypt = false;
        if (esp_now_add_peer(&peerInfoGroundStation) != ESP_OK) {
            Serial.println("Failed to add Ground Station peer");
        } else {
            Serial.println("Ground Station peer added successfully");
        }
    }

    void send(const String& data) override {
        rssi = random(-120, -30);
        if (data.length() >= sizeof(sendMessage.message)) {
            Serial.println("Data too long for buffer, truncating");
            strncpy(sendMessage.message, data.substring(0, sizeof(sendMessage.message) - 1).c_str(), sizeof(sendMessage.message));
        } else {
            strncpy(sendMessage.message, data.c_str(), sizeof(sendMessage.message));
        }
        sendMessage.message[sizeof(sendMessage.message) - 1] = '\0';
        sendMessage.rssi = rssi;
        esp_err_t result = esp_now_send(groundStationMacAddress, (uint8_t *)&sendMessage, sizeof(sendMessage));
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
        lastReceivedCommand = command;
    }

    String receiveCommand() {
        String command = lastReceivedCommand;
        lastReceivedCommand = "";
        return command;
    }

    int32_t getRssi() { return rssi; }
};

// Flight Controller (FC)
class FC {
private:
    Communication* comm;
    bool action1 = false;
    bool action2 = false;
    bool action3 = false;

public:
    FC(Communication* communication) : comm(communication) {}

    void setup() {
        static_cast<EspNowComm*>(comm)->setup();
    }

    void transmit(const String& data) {
        comm->send(data);
    }

    int32_t getRssi() {
        return static_cast<EspNowComm*>(comm)->getRssi();
    }

    void processCommand(const String& command) {
        if (command.startsWith("SET ACTION")) {
            int actionNum = command.substring(10, 11).toInt();
            bool value = command.substring(13, 14).toInt();
            String confirmation;
            switch (actionNum) {
                case 1: action1 = value; confirmation = "ACTION 1 = " + String(action1) + " EXECUTED"; break;
                case 2: action2 = value; confirmation = "ACTION 2 = " + String(action2) + " EXECUTED"; break;
                case 3: action3 = value; confirmation = "ACTION 3 = " + String(action3) + " EXECUTED"; break;
                default: confirmation = "INVALID ACTION"; break;
            }
            comm->send(confirmation);
        }
    }

    void checkCommands() {
        String command = static_cast<EspNowComm*>(comm)->receive();
        if (command != "" && command.startsWith("SET ACTION")) {
            processCommand(command);
        }
    }

    void requestDataFromNavc() {
        SerialNavc.println("REQUEST_DATA");
    }

    String receiveDataFromNavc() {
        if (SerialNavc.available() > 0) {
            String data = SerialNavc.readStringUntil('\n');
            data.trim();
            return data;
        }
        return "";
    }
};

// Global instances
EspNowComm espNowComm;
FC fc(&espNowComm);

void setup() {
    Serial.begin(115200);
    SerialNavc.begin(115200, SERIAL_8N1, FC_RX, FC_TX);
    delay(1000);
    Serial.println("FC Started");
    fc.setup();
}

void loop() {
    fc.requestDataFromNavc();
    Serial.println("Sent REQUEST_DATA to NAVC");
    String navcData = fc.receiveDataFromNavc();
    if (navcData != "") {
        Serial.println("Received from NAVC: " + navcData);
        fc.transmit(navcData); // Send telemetry to Ground Station
    } else {
        Serial.println("No data from NAVC");
    }
    fc.checkCommands();
    delay(250); // 1000ms / 4 = 250ms for 4 packets per second
}