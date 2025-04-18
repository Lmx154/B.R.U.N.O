#include <Arduino.h>
#include <RadioLib.h>
#include <HardwareTimer.h>

// ─── LoRa Configuration ────────────────────────────────────────────
#define NSS    PA4
#define DIO0   PA8
#define NRST   PA9
#define DIO1   PA10
SX1276 radio = new Module(NSS, DIO0, NRST, DIO1);

// ─── Servo Configuration ──────────────────────────────────────────
#define SERVO_PIN PB10
const int pwmFreq    = 200;   // Hz
const int closedDuty = 71;    // ~20%
const int openDuty   = 112;   // ~44%
HardwareTimer *servoTimer = nullptr;
bool isServoOpen       = false;

// ─── Altitude Settings ────────────────────────────────────────────
const float TARGET_ALTITUDE = 1828.8;  // meters
float launchAltitude        = 0.0;

// ─── Parser Ring Buffer ──────────────────────────────────────────
#define MAX_FRAMES     16
#define MAX_FRAME_LEN 128

static char frames[MAX_FRAMES][MAX_FRAME_LEN];
static volatile int frameHead = 0, frameTail = 0;

// Frame builder
static char currFrame[MAX_FRAME_LEN];
static int  currIndex = 0;

// Called automatically by Arduino when Serial2 has data
void serialEvent2() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '<') {
      currIndex = 0;
      currFrame[currIndex++] = c;
    }
    else if (currIndex > 0) {
      if (currIndex < MAX_FRAME_LEN - 1) {
        currFrame[currIndex++] = c;
      }
      if (c == '>') {
        currFrame[currIndex] = '\0';
        // enqueue
        strncpy(frames[frameHead], currFrame, MAX_FRAME_LEN);
        frameHead = (frameHead + 1) % MAX_FRAMES;
        currIndex = 0;
      }
    }
  }
}

// ─── LoRa Communication ────────────────────────────────────────────
class LoraComm {
  String receivedData;
  const uint8_t fcAddr = 0xA2, gsAddr = 0xA1;

public:
  void setup() {
    radio.begin(915.0);
    radio.setSpreadingFactor(7);
    radio.setBandwidth(250.0);
    radio.setCodingRate(5);
    radio.setSyncWord(0xAB);
    radio.setNodeAddress(fcAddr);
    radio.setDio0Action([](){}, RISING);
    radio.startReceive();
  }

  void send(String data) {
    // transmit() is blocking and returns an error code
    int16_t state = radio.transmit(data, gsAddr);
    if (state != RADIOLIB_ERR_NONE) {
      Serial.println("LoRa TX err: " + String(state));
    }
    // restart receive
    radio.startReceive();
  }

  String receive() {
    // non-blocking check
    if (!radio.available()) {
      return "";
    }
    int16_t state = radio.receive(receivedData);
    radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      return receivedData;
    }
    return "";
  }
};

// ─── Flight Computer ──────────────────────────────────────────────
class FC {
  LoraComm lora;
  bool     buzzerActive = false;
  unsigned long buzzerStart = 0;

  // extract 9th CSV field (BMP280 altitude)
  float getAltitude(const String &d) {
    int commas = 0;
    for (int i = 0; i < d.length(); i++) {
      if (d[i] == ',' && ++commas == 8) {
        int j = d.indexOf(',', i + 1);
        return d.substring(i + 1, j < 0 ? d.length() : j).toFloat();
      }
    }
    return 0.0f;
  }

  void controlServo(float alt) {
    if (launchAltitude == 0.0f && alt > 0.0f) {
      launchAltitude = alt;
      Serial.println("Launch alt: " + String(launchAltitude));
    }
    if (!isServoOpen && alt < TARGET_ALTITUDE) {
      servoTimer->setPWM(3, SERVO_PIN, pwmFreq, closedDuty * 100 / 255);
    }
    else if (!isServoOpen && alt >= TARGET_ALTITUDE) {
      servoTimer->setPWM(3, SERVO_PIN, pwmFreq, openDuty * 100 / 255);
      isServoOpen = true;
      Serial.println("Servo opened at alt: " + String(alt));
    }
  }

public:
  void setup() {
    // LoRa & buzzer & servo
    lora.setup();
    pinMode(PB13, OUTPUT); digitalWrite(PB13, LOW);
    pinMode(SERVO_PIN, OUTPUT);
    servoTimer = new HardwareTimer(TIM2);
    servoTimer->setPWM(3, SERVO_PIN, pwmFreq, closedDuty * 100 / 255);

    // Serial2 for NAVC
    Serial2.begin(115200);
  }

  void loop() {
    // one packet per pass
    if (frameTail != frameHead) {
      char *pkt = frames[frameTail++];
      if (frameTail >= MAX_FRAMES) frameTail = 0;

      // print raw NAVC packet
      Serial.write(pkt, strlen(pkt));
      Serial.write("\r\n");

      // strip framing and forward
      String payload = String(pkt).substring(1, strlen(pkt) - 1);
      lora.send(payload);
      controlServo(getAltitude(payload));
    }

    // Ground‑Station commands
    String cmd = lora.receive();
    if (cmd == "BUZZER_ON") {
      digitalWrite(PB13, HIGH);
      buzzerActive = true;
      buzzerStart  = millis();
      lora.send("BUZZER_ON_ACK");
    }
    if (buzzerActive && millis() - buzzerStart >= 2000) {
      digitalWrite(PB13, LOW);
      buzzerActive = false;
      Serial.println("Buzzer off");
    }
  }
};

FC fc;

void setup() {
  // USB‑CDC
  Serial.begin(921600);
  // start the FC
  fc.setup();
}

void loop() {
  fc.loop();
  // let serialEvent2 run
  delay(1);
}
