# ESP32 Prius Gen2 CAN Battery Monitor

## Overview

This project monitors the Toyota Prius Gen2 Battery ECUs using dual CAN buses:

- **Battery 1** via ESP32 internal CAN controller  
- **Battery 2** via external MCP2515 CAN module

It fetches data for pack voltage, current, SOC, CCL/CDL, temperatures, fault codes, and individual module voltages (14 blocks). The pack voltage and current come from standard periodic CAN IDs, while the block voltages are obtained using UDS (mode 0x22) requests (PID 0xCE) via ISO-TP.

Data is displayed live via a built-in Wi-Fi web server (`http://192.168.4.1/`) hosted by the ESP32 in Soft-AP mode.

## Hardware Used

- **ESP32 Dev Board** (with integrated CAN controller)
- **MCP2515 CAN Bus Module** (connected via SPI)
- **CAN Transceivers** for both CAN interfaces
- **12V to 3.3V/5V power supply**
- Proper wiring to the Prius battery ECU CAN lines

## Wiring Overview

| ESP32 Pin | Connected To       | Function                      |
|-----------|--------------------|-------------------------------|
| GPIO16    | Battery 1 CAN RX   | Internal CAN Receive          |
| GPIO17    | Battery 1 CAN TX   | Internal CAN Transmit         |
| SPI Pins  | MCP2515 (CS, SCK...) | For Battery 2 ISO-TP         |
| CAN H/L   | Both CAN buses     | Connected to Prius ECU(s)     |

## Software & Features

- Dual-CAN bus support (internal + MCP2515)
- UDS request: `0x02 21 CE` for block voltages
- Flow-control frames (`0x30 00 05`) for ISO-TP
- Parsing of periodic CAN messages (pack voltage, current, SOC)
- Accurate decoding of **14 block voltages**
- Web interface AutoHTML dashboard for live data

## Decoding References

- **CAN message map and UDS PIDs** for Prius Gen2 via SavvyCAN and EAA-PHEV documentation  
  :contentReference[oaicite:0]{index=0}  
- **Example block voltage decoding** via video:  
  [SavvyCAN decoding block voltages on Prius Gen2﻿ – YouTube]()
 *(video source for reference only)*

## Installation

1. Install Arduino IDE and ESP32 board definitions.
2. Add required libraries:
   - `ACAN_ESP32`  
   - `mcp_can`  
   - `ESPAsyncWebServer`  
   - `WiFi`
3. Download or clone this repository.
4. Open `Esp32PriusMonitor.ino` in Arduino IDE.
5. Select ESP32 Dev Module board and appropriate COM port.
6. Upload the code to the ESP32.

## Usage

1. Power up the ESP32 and MCP2515 module, connect to the CAN bus.
2. Connect to the Wi-Fi SSID **`PriusMonitor`** (Password: `hybridpower`).
3. Open browser to `http://192.168.4.1/`.
4. Monitor real-time metrics:
   - Pack voltage/current, SOC, CCL/CDL, temperatures
   - Sum of all 14 block voltages and detailed block readings

## Example Serial Output

