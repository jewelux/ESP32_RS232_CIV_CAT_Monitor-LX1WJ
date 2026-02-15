# ESP32_RS232_CIV_CAT_Monitor-LX1WJ

Ham Radio Utility for monitoring and controlling Icom CI-V and Elecraft CAT radios using an ESP32 and RS232 interface.

Author: LX1WJ

---

## Overview

This project provides a dual-protocol RS232 monitor and command tool for:

- Icom CI-V (binary HEX protocol)
- Elecraft KX2 / KX3 (ASCII CAT protocol)

Architecture:

USB Serial (PC) <-> ESP32 <-> RS232 (MAX3232) <-> Radio

Features:
- Strict protocol selection (PROTO ICOM / PROTO ELE)
- Strict TX format selection (HEX / ASCII)
- No automatic command completion
- CI-V decoding and echo filtering
- Elecraft CAT decoding
- Timestamped monitoring
- RS232 hardware UART support

---

## Getting Started

Upload the sketch in:

src/RS232_Raw_Monitor_Icom_Elecraft_V2_ManualTX.ino

Open Serial Monitor at 115200 baud.

Example (Elecraft):

PROTO ELE
TXFMT ASCII
BAUD 38400
FA;

---

## Documentation

See the docs folder for:
- Hardware description
- Usage guide
- Icom CI-V working guide
- Elecraft CAT working guide
- RS232 test report

---

## License

License will be provided separately by the author.

73 de LX1WJ
