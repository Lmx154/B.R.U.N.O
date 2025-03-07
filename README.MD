# LMX154-B.R.U.N.O Project README

The LMX154-B.R.U.N.O project is a telemetry system comprising three main components: the Navigation Computer (NAVC), the Flight Controller (FC), and the Ground Station. These components work together to collect, transmit, and display sensor data using a combination of UART and ESP-NOW communication protocols. This README provides detailed documentation for each component based on their respective code files: `navc.ino`, `fc.ino`, and `groundstation.ino`.

## Directory Structure

```
lmx154-b.r.u.n.o/
├── README.MD
├── fc/
│   └── fc.ino
├── groundstation/
│   └── groundstation.ino
└── navc/
    └── navc.ino
```

---

## NAVC (Navigation Computer) - `navc.ino`

The NAVC, implemented in `navc.ino`, is the core component responsible for **gathering telemetry data from simulated sensors**, **creating a compact data packet**, and **sending it to the FC** via UART upon request. This section places special emphasis on these critical roles.

### Role and Functionality

- **Telemetry Data Collection**:  
  The NAVC simulates a variety of sensors, including:
  - **Accelerometer**: Measures X, Y, Z acceleration (g) and velocity (m/s).
  - **Gyroscope**: Measures pitch, roll, and yaw (degrees).
  - **Magnetometer**: Measures magnetic field strength in X, Y, Z axes (µT).
  - **Barometer**: Measures atmospheric pressure (hPa) and altitude (m).
  - **Pressure Sensor**: Provides additional pressure readings (hPa).
  - **GPS**: Provides latitude, longitude (degrees), and the number of satellites.
  - **Temperature Sensor**: Measures temperature (°C).
  - These sensors are implemented as classes (e.g., `SimulatedAccelerometer`, `SimulatedBarometer`) that generate random but realistic values for demonstration purposes.

- **Data Packet Creation**:  
  The NAVC collects data from all sensors and formats it into a **compact, position-based string** optimized for transmission. This packet uses square brackets `[]` to enclose the data, commas `,` to separate major sections, and pipes `|` to separate values within sensor groups. The format is designed to minimize size while preserving all necessary telemetry information. An example packet is:

```
[id:1,mt:11m1s,con:1,3.3|-1.9|7.3|-3.8|2.2|0.8,11.2|2.1|-21.4,26.4|-49.5|-42.8,1034.61|69.2,975.34,37.77|-122.43|11,28.8,4.03,11,1]
```

This format is explained in detail below under **Data Format**.

- **Sending Data to the FC**:  
  The NAVC communicates with the FC via **UART** using `Serial1` (pins GP0/TX and GP1/RX on the Pico 2). It listens for a `"REQUEST_DATA"` command from the FC. Upon receiving this command:
  - It collects the latest sensor data using the `collectData()` method.
  - It formats the data into the telemetry packet.
  - It sends the packet to the FC over UART using `SerialNavc.println()`.
  - An LED on pin GP25 blinks (on for 100ms) to indicate data transmission.

### Implementation Details

- **Code Structure**:  
  The `NAVC` class manages sensor instances and orchestrates data collection and transmission. The `processRequest()` method checks for incoming UART commands every 100ms and responds accordingly.

- **Key Features**:  
  - Uses simulated sensor classes for flexibility and testing.
  - Includes mission time (`mt:MMmSSs`) and connection status (`con:1`) in the packet.
  - Employs an LED indicator for visual feedback during transmission.

### Data Format Emphasis

The telemetry packet is a critical aspect of the NAVC's role, as it defines how data is structured and transmitted. Here's an in-depth look:

- **Example Packet**:  

```
[id:1,mt:11m1s,con:1,3.3|-1.9|7.3|-3.8|2.2|0.8,11.2|2.1|-21.4,26.4|-49.5|-42.8,1034.61|69.2,975.34,37.77|-122.43|11,28.8,4.03,11,1]
```

- **Structure Breakdown**:  
  - **Enclosure**: Square brackets `[]` mark the packet's start and end.
  - **Section Separator**: Commas `,` separate major data groups (e.g., ID, mission time, sensor readings).
  - **Value Separator**: Pipes `|` separate individual values within sensor groups (e.g., `3.3|-1.9|7.3` for accelerometer data).

- **Components**:  
  - `id:1`: Device ID (fixed at 1).
  - `mt:11m1s`: Mission time (e.g., 11 minutes, 1 second).
  - `con:1`: Connection status (1 = connected).
  - `3.3|-1.9|7.3|-3.8|2.2|0.8`: Accelerometer and velocity (acc_x|acc_y|acc_z|vel_x|vel_y|vel_z).
  - `11.2|2.1|-21.4`: Gyroscope (pitch|roll|yaw).
  - `26.4|-49.5|-42.8`: Magnetometer (mag_x|mag_y|mag_z).
  - `1034.61|69.2`: Barometer (pressure|altitude).
  - `975.34`: Pressure sensor reading.
  - `37.77|-122.43|11`: GPS (latitude|longitude|satellites).
  - `28.8`: Temperature.
  - `4.03`: Battery voltage.
  - `11`: Current minute.
  - `1`: Current second.

- **Position-Based Data Format Chart**:  
  The following chart maps each position in the packet to its corresponding sensor value, providing a clear reference for interpreting the data:

```
Position-Based Data Format Chart
Delimiter: |
[id:1,mt:MMmSSs,con:1,  1| 2| 3| 4| 5| 6,  7| 8| 9, 10|11|12, 13|14, 15, 16|17|18, 19, 20, 21, 22]

Old Name            | New Position/Abbreviation | Description
--------------------|---------------------------|--------------------
id                  | id:1                      | Device ID
mission_time        | mt:MMmSSs                 | Mission time (MM minutes, SS seconds)
connected           | con:1                     | Connection status (1 = connected)
acceleration_x      | 1                         | X-axis acceleration (g)
acceleration_y      | 2                         | Y-axis acceleration (g)
acceleration_z      | 3                         | Z-axis acceleration (g)
velocity_x          | 4                         | X-axis velocity (m/s)
velocity_y          | 5                         | Y-axis velocity (m/s)
velocity_z          | 6                         | Z-axis velocity (m/s)
pitch               | 7                         | Pitch angle (degrees)
roll                | 8                         | Roll angle (degrees)
yaw                 | 9                         | Yaw angle (degrees)
mag_x               | 10                        | X-axis magnetic field (uT)
mag_y               | 11                        | Y-axis magnetic field (uT)
mag_z               | 12                        | Z-axis magnetic field (uT)
baro_press          | 13                        | Barometric pressure (hPa)
altitude            | 14                        | Altitude (m)
press               | 15                        | Pressure sensor reading (hPa)
latitude            | 16                        | Latitude (degrees)
longitude           | 17                        | Longitude (degrees)
satellites          | 18                        | Number of GPS satellites
temp                | 19                        | Temperature (C)
battery             | 20                        | Battery voltage (V)
minute              | 21                        | Current minute
second              | 22                        | Current second
```

This chart is essential for downstream components (FC and Ground Station) to parse and interpret the telemetry data accurately.

## FC (Flight Controller) - `fc.ino`

The FC, implemented in `fc.ino`, serves as an intermediary between the NAVC and the Ground Station. It requests data from the NAVC, forwards it to the Ground Station, and processes commands received from the Ground Station.

### Role and Functionality Based on Code

- **Requesting Data from NAVC**:
  - The FC uses UART (HardwareSerial SerialNavc on UART0, pins GPIO43/TX and GPIO44/RX on the ESP32-S3) to communicate with the NAVC.
  - Every 250ms, it sends a "REQUEST_DATA" command to the NAVC via `SerialNavc.println("REQUEST_DATA")` (see `fc.requestDataFromNavc()` in the loop()).
  - It then listens for the NAVC's response using `SerialNavc.readStringUntil('\n')` and retrieves the telemetry packet.
  
- **Forwarding Data to Ground Station**:
  - Upon receiving the telemetry packet from the NAVC, the FC transmits it to the Ground Station using ESP-NOW.
  - The FC class's `transmit()` method calls `EspNowComm::send()`, which packages the data into a `struct_message` (with a 240-character limit for the message and an RSSI value) and sends it to the Ground Station's MAC address (0x24, 0xEC, 0x4A, 0x2C, 0x4F, 0x38).
  
- **Processing Commands from Ground Station**:
  - The FC listens for commands via ESP-NOW using the `OnDataRecv` callback, which populates the `receivedMessage` struct.
  - The `FC::checkCommands()` method checks for commands like "SET ACTION X Y" (e.g., "SET ACTION 1 1" to enable Action 1).
  - It processes these commands in `processCommand()`, updating boolean flags (`action1`, `action2`, `action3`) and sending confirmation messages back to the Ground Station (e.g., "ACTION 1 = 1 EXECUTED").

### Implementation Details

- The `EspNowComm` class handles ESP-NOW setup, peer registration (Ground Station as a peer), and data transmission/reception.
- The `FC` class integrates UART and ESP-NOW communication, ensuring seamless data flow from NAVC to Ground Station.
- Simulated RSSI values (random between -120 and -30) are included in transmissions for demonstration.

### Key Features

- Acts as a bridge between UART (NAVC) and ESP-NOW (Ground Station).
- Supports bidirectional communication with the Ground Station, handling both telemetry forwarding and command processing.
- Operates at 4 packets per second (250ms delay) to balance responsiveness and bandwidth.

## Ground Station - `groundstation.ino`

The Ground Station, implemented in `groundstation.ino`, receives telemetry data from the FC and sends commands to control actions on the FC.

### Role and Functionality Based on Code

- **Receiving Telemetry Data**:
  - The Ground Station uses ESP-NOW to receive data from the FC (MAC address 0xE8, 0x06, 0x90, 0x95, 0xEE, 0xB0).
  - The `OnDataRecv` callback processes incoming packets, extracting the telemetry string and RSSI from the `struct_message`.
  - It logs the received data to the Serial Monitor with the sender's MAC address, e.g., "Telemetry Data: [id:1,...], rssi: -50".
  
- **Sending Commands to FC**:
  - Commands are input via the Serial Monitor (e.g., "SET ACTION 1 1") and read every 100ms in the loop().
  - The `GroundStation::sendCommand()` method uses `EspNowComm::send()` to transmit these commands to the FC via ESP-NOW.
  - The `OnDataSent` callback confirms whether the transmission was successful.

### Implementation Details

- The `EspNowComm` class initializes ESP-NOW, registers the FC as a peer, and handles bidirectional communication.
- The `GroundStation` class provides a simple interface for receiving telemetry (handled by callbacks) and sending commands.
- The Ground Station operates in Wi-Fi Station mode (WIFI_STA) with channel 0, matching the FC's configuration.

### Key Features

- Displays telemetry data with RSSI for signal strength monitoring.
- Allows user interaction via serial commands to control FC actions.
- Uses lightweight ESP-NOW packets (240-character limit) to ensure efficient wireless communication.

## Communication Protocols

### UART Communication between NAVC and FC

- **Setup**:
  - NAVC uses Serial1 (Pico 2, GP0/TX, GP1/RX).
  - FC uses HardwareSerial SerialNavc (ESP32-S3, GPIO43/TX, GPIO44/RX).
  - Both operate at 115200 baud with 8N1 configuration.
  
- **Process**:
  - The FC sends "REQUEST_DATA" every 250ms to the NAVC.
  - The NAVC responds with the telemetry packet when it receives this command.
  - Data is transmitted as a single line terminated by \n, ensuring reliable parsing.
  
- **Purpose**:
  This UART link enables the FC to retrieve real-time sensor data from the NAVC, forming the foundation of the telemetry pipeline.

### ESP-NOW Bridge between FC and Ground Station

- **Setup**:
  - Both FC and Ground Station use ESP-NOW over Wi-Fi in Station mode (WIFI_STA), channel 0.
  - MAC addresses are predefined:
    - FC: E8:06:90:95:EE:B0
    - Ground Station: 24:EC:4A:2C:4F:38
  - Peers are registered using `esp_now_add_peer()` with each other's MAC addresses.
  
- **Data Transmission**:
  - FC to Ground Station: The FC sends telemetry packets (up to 240 characters) with an RSSI value using `esp_now_send()`.
  - Ground Station to FC: The Ground Station sends commands (e.g., "SET ACTION 1 1") using the same function.
  - Packets are structured as `struct_message` with a `char message[240]` and `int rssi`.
  
- **Callbacks**:
  - `OnDataSent`: Logs transmission status (success or failure).
  - `OnDataRecv`: Processes incoming data, logging the sender's MAC and payload.
  
- **Purpose**:
  The ESP-NOW bridge provides a low-latency, Wi-Fi-independent wireless link for telemetry and command exchange, optimizing for efficiency with a compact data format.

## Conclusion

The LMX154-B.R.U.N.O project demonstrates a cohesive telemetry system:

- The NAVC excels at gathering sensor data, formatting it into a compact packet, and sending it to the FC via UART.
- The FC bridges the NAVC and Ground Station, managing UART requests and ESP-NOW transmissions while processing commands.
- The Ground Station receives and displays telemetry while enabling user control over the FC.

The emphasized data format and detailed communication protocols (UART and ESP-NOW) ensure this README serves as a comprehensive reference for understanding and maintaining the project.