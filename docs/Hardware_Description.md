# Hardware Description

ESP32 connected to MAX3232 RS232 level converter.

ESP32 UART2 TX -> MAX3232 -> DB9 Pin 3
ESP32 UART2 RX -> MAX3232 -> DB9 Pin 2
GND shared

DB9 Pinout:
Pin 2 - RXD
Pin 3 - TXD
Pin 5 - GND

Default USB Serial: 115200 baud
Default RS232: 9600 baud (configurable via BAUD command)

Compatible with:
- Icom CT-17 CI-V interface
- Elecraft RS232 CAT ports
- RS232-USB adapter for lab testing
