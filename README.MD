# B.R.U.N.O Telemetry System

This repository contains the three key components of the B.R.U.N.O telemetry system:

```
b.r.u.n.o/
├── README.MD
├── fc/
│   └── fc.ino
├── groundstation/
│   └── groundstation.ino
└── navc/
    └── navc.ino
```

Each component runs on an STM32 “Black Pill” board (NAVC and FC) or any Arduino‑compatible board with an SX1276 LoRa module (Ground Station).

---

## 1. Navigation Computer (NAVC) — `navc/navc.ino`

### Role & Overview
- **Reads real sensors** over I²C and UART:
  - **BMP280** barometer/temperature (@ 0x77)
  - **BMI088** accelerometer (@ 0x19) & gyroscope (@ 0x69)
  - **BMM150** magnetometer (@ 0x13)
  - **PCF8563** RTC for timestamping (@ 0x51)
  - **GPS** NMEA sentences on `Serial1` (PB7 RX, PB6 TX)

- **Data handling**:
  - Parses GNRMC/GNGGA for latitude, longitude, satellites, altitude
  - Reads sensors at up to 50 Hz
  - Prefixes each record with an RTC timestamp (`YYYY‑MM‑DD HH:MM:SS`)
  - Frames the CSV string in `<...>` for easy UART parsing

- **Outputs**:
  - **Serial2** (`SerialNavc`) at 115200 baud for FC requests or autonomous sends every 20 ms
  - **USB‑CDC** (`Serial`) mirroring each packet for debug / serial monitor
  - **SD card** `"data_log.txt"` with one `<packet>` per line

### Packet Format
```
<YYYY‑MM‑DD HH:MM:SS,
 accelX,accelY,accelZ,
 gyroX,gyroY,gyroZ,
 temp,pressure,altitude,
 magX,magY,magZ,
 gpsLat,gpsLon,satCount,gpsAlt>
```
- **Comma‑separated**, max length 255 chars  
- All floats printed with 2 decimal places (angles rounded to integers)  

---

## 2. Flight Controller (FC) — `fc/fc.ino`

### Role & Overview
- **UART listener** on `Serial2` at 115200 baud:
  - Implements `serialEvent2()` ISR to buffer framed `<…>` packets into a ring buffer
  - Avoids line‑based reads so it won’t drop frames at high rates

- **LoRa uplink** via SX1276 (`RadioLib`):
  - Configured at 915 MHz, SF7, BW 250 kHz, CR 4/5, sync word 0xAB
  - Sends only **payloads** (no brackets) over LoRa after dequeuing each NAVC frame
  - Prints that payload once to USB‑CDC (`Serial`) for local monitoring

- **Actuation**:
  - **Servo** on PB10 (TIM2 CH3) stays closed until baro altitude ≥ 1828.8 m, then opens
  - **Buzzer** on PB13 via LoRa “BUZZER_ON” command and auto‑off after 2 s

### Key Data Flow
1. **NAVC → FC**  
   FC calls `Serial2.begin(115200)` and `serialEvent2()` fills `frames[]`
2. **FC → LoRa**  
   In `loop()`, on each available frame:  
   - Strip `< >`  
   - `radio.transmit(payload, groundStationAddr)`  
   - `Serial.println(payload)`  
3. **LoRa → Ground Station**  
   Uses `SX1276.receive()` in non‑blocking mode

---

## 3. Ground Station — `groundstation/groundstation.ino`

### Role & Overview
- **LoRa receiver** via SX1276:
  - Same RF settings as FC
  - On successful `receive()`, prints **raw payload** directly to Serial (no extra formatting)

- **Command uplink**:
  - Reads any user‑typed line from USB‑CDC (`Serial.readStringUntil('\n')`)
  - Sends it verbatim over LoRa to FC  

---

## Communication Summary

| Link               | Protocol    | Port/Interface | Baud/Freq    | Frame Style          |
|--------------------|-------------|----------------|--------------|----------------------|
| NAVC → FC          | UART        | Serial2 (PA3/PA2 on F401)  | 115200 baud | `<CSV…>` frames      |
| FC → Ground Station| LoRa (SX1276)| SPI + DIO0 etc.| 915 MHz, SF7 | raw CSV payload      |
| Ground Station → FC| LoRa        | —              | 915 MHz, SF7 | raw command string   |

---

### Getting Started

1. **Wire up sensors** on I²C (PB8=SCL, PB9=SDA) and GPS on Serial1.
2. **Configure SD card** CS on PA4.
3. **Load** `navc/navc.ino` onto NAVC board.
4. **Wire** SX1276 to FC (SPI + DIO0/DIO1), load `fc/fc.ino`.
5. **Wire** SX1276 to Ground Station, load `groundstation/groundstation.ino`.
6. **Open** USB‑CDC at 921600 baud for FC, 115200 baud for NAVC & GS.

With this setup, NAVC will pump out timestamped telemetry at up to 50 Hz, the FC will reliably forward each packet over LoRa (and actuate servo/buzzer), and the Ground Station will display the live stream.