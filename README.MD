# B.R.U.N.O Flight Computer & Ground Station

## Overview

The **B.R.U.N.O** project implements a two-board flight computer system and a matching ground station for telemetry in rocketry applications. It consists of three main components:

* **NAVC** (`navc/navc.ino`): Autonomous sensor data collection, packaging, and local SD logging.
* **FC** (`fc/fc.ino`): Flight computer that reads NAVC packets over UART, transmits telemetry via LoRa, and controls onboard actuators (servo & buzzer).
* **Ground Station** (`groundstation/groundstation.ino`): Receives telemetry via LoRa, displays raw payloads over USB, and forwards user commands back to the FC.

All telemetry packets are framed and sent to the **Sentinel** software for parsing and visualization.

## Directory Structure

```
lmx154-b.r.u.n.o/
├── fc/
│   └── fc.ino               # Flight computer firmware
├── groundstation/
│   └── groundstation.ino    # Ground station firmware
└── navc/
    └── navc.ino             # Autonomous NAVC data logger
```

## Prerequisites

* Arduino IDE (or PlatformIO)
* Libraries:

  * RadioLib
  * HardwareTimer
  * Adafruit\_BMP280
  * BMI088 (Accel & Gyro)
  * BMM150
  * RTClib
  * TinyGPSPlus
  * SD
  * Wire, SPI

## Hardware Connections

### NAVC Board

* **GPS**: UART1 on PB7 (RX) / PB6 (TX)
* **I²C Bus** (PB8=SCL, PB9=SDA):

  * BMI088 Accelerometer (0x19)
  * BMI088 Gyroscope    (0x69)
  * BMM150 Magnetometer (0x13)
  * BMP280 Barometer    (0x77)
  * DS3231 RTC
* **SD Card**: SPI (`MOSI`, `MISO`, `SCK`), CS = PA4
* **NAVC → FC**: UART2 on PA3 (TX) / PA2 (RX)
* **Status LED**: PC13

### Flight Computer (FC) Board

* **LoRa (SX1276)** over SPI:

  * NSS  = PA4
  * DIO0 = PA8 (IRQ)
  * NRST = PA9
  * DIO1 = PA10
* **Servo PWM**: TIM2 channel 3 → PB10
* **Buzzer**: PB13 → digital output
* **NAVC Input**: Serial2 (PA3/PA2)
* **USB-CDC Debug**: Serial (PA9/PA10 on USB port)

### Ground Station Board

* **LoRa (SX1276)** pins identical to FC
* **USB-Serial**: Serial (for console I/O)

## Protocols & Buses

| Interface | Purpose                                      |
| --------- | -------------------------------------------- |
| **SPI**   | LoRa radio (SX1276) & SD card (NAVC)         |
| **I²C**   | BMP280, BMI088, BMM150, DS3231 RTC (NAVC)    |
| **UART1** | GPS → NAVC                                   |
| **UART2** | NAVC ↔ FC (telemetry frames & commands)      |
| **LoRa**  | FC ↔ Ground Station (915 MHz, SPI interface) |
| **PWM**   | Servo control (200 Hz)                       |

## Module Descriptions

### NAVC (`navc/navc.ino`)

The NAVC module samples sensors at regular intervals or on command:

1. **Timestamp (RTC)**: `YYYY-MM-DD HH:MM:SS` from DS3231.
2. **GPS** (TinyGPSPlus): latitude, longitude, satellites, GPS altitude.
3. **IMU** (BMI088): accelX, accelY, accelZ \[m/s²]; gyroX, gyroY, gyroZ \[°/s].
4. **Magnetometer** (BMM150): magX, magY, magZ \[raw units].
5. **Barometer** (BMP280): temperature \[°C], pressure \[hPa], altitude \[m].

After reading, it builds a CSV packet:

```
timestamp,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,temp,pressure,altBMP,magX,magY,magZ,lat,lng,sat,altGps
```

Packets are framed `< ... >` then:

* Sent to FC over Serial2
* Logged to `data_log.txt` on SD card
* Echoed over USB-Serial

#### Data Packet Format

| Pos | Field     | Type   | Units/Format                |
| --- | --------- | ------ | --------------------------- |
| 1   | timestamp | String | `YYYY-MM-DD HH:MM:SS`       |
| 2   | accelX    | float  | m/s²                        |
| 3   | accelY    | float  | m/s²                        |
| 4   | accelZ    | float  | m/s²                        |
| 5   | gyroX     | int    | °/s (rounded)               |
| 6   | gyroY     | int    | °/s (rounded)               |
| 7   | gyroZ     | int    | °/s (rounded)               |
| 8   | temp      | float  | °C                          |
| 9   | pressure  | float  | hPa                         |
| 10  | altBMP    | float  | m (relative to 1013.25 hPa) |
| 11  | magX      | float  | raw                         |
| 12  | magY      | float  | raw                         |
| 13  | magZ      | float  | raw                         |
| 14  | lat       | double | Decimal degrees             |
| 15  | lng       | double | Decimal degrees             |
| 16  | sat       | int    | Number of satellites        |
| 17  | altGps    | float  | m                           |

### Flight Computer (`fc/fc.ino`)

* Buffers incoming `<...>` frames from NAVC on Serial2.
* For each frame:

  1. Strips `< >`, forwards payload via LoRa to ground.
  2. Prints payload over USB-Serial.
  3. Extracts 9th CSV field (BMP280 altitude) and invokes `controlServo()`:

     * Records `launchAltitude` on first positive altitude.
     * Keeps servo closed until reaching `TARGET_ALTITUDE` (1828.8 m).
     * Opens servo (PWM duty change) at target, logs event.
* Listens for LoRa commands from ground:

  * `BUZZER_ON` → activates buzzer on PB13 for 2 s and replies `BUZZER_ON_ACK`.

### Ground Station (`groundstation/groundstation.ino`)

* Initializes LoRa with address 0xA1.
* Loop:

  1. Checks for incoming LoRa telemetry, prints raw payload on USB.
  2. Reads USB-Serial for user commands (e.g. `BUZZER_ON`), sends via LoRa to FC.

## Integration with Sentinel

All framed payloads (`<...>`) sent by NAVC → FC → ground station are consumed by **Sentinel**. Sentinel parses the CSV string according to the field map above, converting each entry to its appropriate data type for real-time display, logging, and analysis.

## Usage

1. **Flash** each board with its respective `.ino` file.
2. **NAVC**: Insert microSD, power on, auto-starts logging and packet emission.
3. **FC**: Power on, opens USB-Serial at 921600 baud for logs.
4. **Ground Station**: Connect via USB at 115200 baud, run console, view telemetry, send commands.

---

*Happy flying!*
