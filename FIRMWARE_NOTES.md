# JEXFYZ Firmware Notes

## 2026-10-01 — FULL EXECUTION / AUTHORITATIVE SOURCE RECONCILIATION

- Current firmware revision: **V2.3**.
- Dedicated firmware repository currently contains the corrected **V2.3 source** (JEXFYZ-V2.3.ino) and the existing production BIN (JEXFYZ.cpp.bin). Older notes describing this repository as binary-only are superseded by the current repository state.
- Physical GPIO contract is frozen as:
  - **GPIO5 = PELTIER**
  - **GPIO6 = FAN**
- App BLE/P1 command vocabulary remains logical and unchanged:
  - `FAN:ON/OFF` -> logical Fan state -> `PIN_FAN` -> GPIO6
  - `PELTIER:ON/OFF` -> logical Peltier state -> `PIN_PELTIER` -> GPIO5
- V2.3 source now contains a compile-time `static_assert` that rejects accidental reversal of the two critical output pins.
- Static source audit confirms the Fan PWM writer uses `PIN_FAN`, while the Peltier soft-start/output path uses `PIN_PELTIER`.
- Existing Peltier prerequisites remain intact: OTA inactive, valid NTC, safe Hotside temperature, and Fan ON.
- The production `JEXFYZ.cpp.bin` was **not regenerated or modified** in this execution because a verified Arduino-ESP32 Core 3.3.7 compiler toolchain is not available here.
- Source hardening commit: `21ceb9ceb3f7b7701a88367a0dbd13090949acc2`.
- ArduinoDroid/Core 3.3.7 compile is still pending.
- Physical flash/runtime validation is still pending. Do not call the GPIO correction hardware-validated until the corrected binary is flashed and both outputs are tested.

## HISTORICAL RECORD — 2026-10-01 — INITIAL FAN / PELTIER OUTPUT FIX

- Confirmed bench symptom: App FAN ON physically activated the Peltier output.
- Logical App/BLE command mapping was kept unchanged.
- Corrected source mapping is GPIO5 = PELTIER and GPIO6 = FAN.
- Production BIN was intentionally not modified without a verified Core 3.3.7 compile.

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
