# JEXFYZ Firmware Notes

## 2026-10-01 — FAN / PELTIER OUTPUT MAPPING FIX

- Confirmed bench behavior: App FAN ON was physically activating the Peltier output.
- BLE/P1 command names remain logically correct:
  - `FAN:ON/OFF` = Fan
  - `PELTIER:ON/OFF` = Peltier
- Corrected physical output mapping in `JEXFYZ-V2.3.ino`:
  - `PIN_FAN = 6`
  - `PIN_PELTIER = 5`
- Existing Peltier safety prerequisites remain unchanged: Fan ON, valid NTC, safe hotside temperature, and OTA protection.
- The repository's existing `JEXFYZ.cpp.bin` is a prebuilt binary and was **not** modified because this environment does not provide a verified Arduino-ESP32 3.3.7 compiler toolchain.
- The corrected source is now present in this firmware repository and is ready to compile/flash with the project's ArduinoDroid / Arduino-ESP32 Core 3.3.7 toolchain.
