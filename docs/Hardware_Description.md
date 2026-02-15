# Hardware Description

## 1. Overview

The system is based on an **ESP32-S3-N16R8 development board** connected to a 
**MAX3232 RS232 level converter module with DB9 female connector**.

It allows monitoring and controlling:

- Icom CI-V radios (via CT-17 interface)
- Elecraft KX2 / KX3 CAT RS232 ports
- RS232-USB adapters for laboratory testing

---

## 2. Microcontroller

Board used:

ESP32-S3-N16R8 Development Board

Key properties:
- USB native programming interface
- 3.3V logic levels
- Hardware UART2 used for RS232 communication

From the firmware:

Default UART configuration:
- UART2
- RX pin = GPIO 9
- TX pin = GPIO 10
- 8N1 format
- Default RS232 baud = 9600 (configurable)

---

## 3. RS232 Interface Module

Module:
MAX3232 RS232 level shifter with DB9 female connector

Function:
Converts ESP32 3.3V UART signals to true RS232 voltage levels (+/- voltage).

---

## 4. Electrical Wiring

### ESP32 to MAX3232 connections

ESP32 GPIO10 (UART2 TX)  ->  MAX3232 T1IN  
ESP32 GPIO9  (UART2 RX)  ->  MAX3232 R1OUT  
ESP32 GND                ->  MAX3232 GND  
ESP32 5V                 ->  MAX3232 VCC  

⚠ Important:
The MAX3232 module must support 3.3V logic levels.
Most standard MAX3232 modules work at 3.3V–5V.

---

## 5. DB9 Connector Pinout

DB9 Female (DCE type typical configuration):

Pin 2 – RXD  
Pin 3 – TXD  
Pin 5 – GND  

### Logical Signal Flow

ESP32 TX  → DB9 Pin 3  
ESP32 RX  ← DB9 Pin 2  
GND       → DB9 Pin 5  

---

## 6. Power Supply

### Internal

- ESP32 powered via USB (5V)
- MAX3232 powered from ESP32 5V rail

### External (Ham Radio typical environment)

- 12V DC shack power supply
- Use a 12V → 5V DC-DC converter (buck regulator)
- Feed regulated 5V to ESP32 5V input

Recommended:
Use a filtered buck converter to avoid RF noise.

---

## 7. System Block Diagram

              USB (115200)

PC ───────────────────────────┐

│

┌─────▼─────┐

│ ESP32-S3 │

│ UART2 │

└─────┬─────┘

│ 3.3V TTL

▼

┌─────────────┐

│ MAX3232 │

│ RS232 Level │

└─────┬───────┘

│ RS232

▼

DB9 Connector

│

▼

Radio / CT-17 / KX2 / PC



---

## 9. Default Communication Parameters

USB Serial:
- 115200 baud
- Used for control and monitoring via Arduino IDE

RS232 Port:
- Default: 9600 baud
- Configurable via firmware command:

BAUD <value>

Example:
BAUD 38400

---

## 10. What Must Not Be Forgotten

✔ Common Ground between all devices  
✔ Correct RS232 cable type (straight vs null-modem)  
✔ Flow control disabled  
✔ Correct baud rate on radio  
✔ CI-V address configuration (Icom)  
✔ CAT enabled in radio menu (Elecraft)  
✔ Clean 5V supply (avoid RF noise from cheap converters)  

Optional but recommended:
✔ TVS protection diode on RS232 lines  
✔ Ferrite beads for RF suppression in ham environment  

---

## 11. Typical Use Cases

- CI-V multi-radio monitoring via CT-17
- Elecraft CAT debugging
- RS232 lab analyzer
- Protocol reverse engineering
- Ham radio development and diagnostics



