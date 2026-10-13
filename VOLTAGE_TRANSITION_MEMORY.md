# JE X FYZ / JE Cooler — Voltage Transition & Peltier Soft-Start Memory

Updated: 2026-10-13 (Asia/Jakarta)
Status: Fix implemented in V6.3 source; compile/runtime/hardware result not yet validated for this revision.

## Incident report

[USER-REPORTED — 2026-10-13] On the physical hardware, the user selected 5 V, turned Peltier and Fan ON, then selected 12 V while the 5-second Peltier soft-start was still active. The hardware immediately restarted. The exact reset reason from that reproduction has not yet been supplied.

The failure path is consistent with a power-transient hypothesis: the prior firmware's voltage transition turned both selector outputs OFF for its 200 ms break-before-make interval, but it intentionally preserved the Peltier PWM state. The new rail could therefore be enabled while Peltier PWM remained at an in-progress duty cycle. PWM duty alone does not limit the peak voltage of an individual ON pulse; changing the rail can sharply change the Peltier's current demand. A supply dip/brownout is plausible but not proven until reset diagnostics are captured.

## Source correction applied

Source: [firmware/JEXFYZ-V6.3.ino](https://github.com/TenzoNkz/JEXFYZ-App/blob/main/firmware/JEXFYZ-V6.3.ino)
Regression contract: [test/v6_3_qc_contract_test.dart](https://github.com/TenzoNkz/JEXFYZ-App/blob/main/test/v6_3_qc_contract_test.dart)

The revised transition contract is:

1. When a valid voltage transition is accepted while the logical Peltier state is ON, immediately command physical Peltier PWM on GPIO5 to 0% before de-asserting selector outputs. Keep the logical Peltier request ON so the App's state doesn't falsely change to OFF. The Fan remains on and is forced to 100% as before.
2. De-assert both 9 V/12 V selector outputs and retain the existing 200 ms break-before-make period.
3. Activate the accepted target rail only after the existing validation and transition checks.
4. Keep Peltier PWM at 0% for a post-transition settle interval equal to the already-established 1-second voltage guard (`PELTIER_VOLTAGE_SETTLE_MS = VOLTAGE_GUARD_MS`). This reuses the known project guard rather than introducing an unrelated guessed duration.
5. Start an entirely new 5-second, cooperative soft-start from 0% PWM after the settle interval. This applies whether the old duty was still ramping or had already reached 100%, making each rail change start from a known physical output state.
6. If thermal/sensor safety, an explicit Peltier-OFF request, or OTA safety turns the logical Peltier state OFF, the normal stop helper cancels the pending voltage pause and prevents automatic restart.
7. The change remains non-blocking: no `delay()` or timing `while()` was introduced.

## Timing contract

- Selector break-before-make: 200 ms (existing setting).
- Post-selector Peltier settle: 1,000 ms, aliased to the current shared voltage guard.
- Peltier PWM soft-start after settling: 5,000 ms.
- The 1-second interval is a conservative software dwell using the existing guard; it is not laboratory proof of the actual supply rail's electrical settling time. Hardware validation is still required.

## Regression safeguards

- The source must call `pausePeltierForVoltageTransition()` after a transition is accepted and before either selector output is switched OFF.
- The pause helper must set the pause state, cancel the prior ramp timer, and write GPIO5 PWM to 0%.
- While `voltageTransition` is active, `processPeltierSoftStart()` must fail closed at 0% PWM.
- After selector completion, Peltier may not ramp until `PELTIER_VOLTAGE_SETTLE_MS` has elapsed from `lastVoltageTransitionCompleteMs`; it must then call `beginPeltierSoftStart()`.
- Keep the 200 ms break-before-make, 1-second voltage guard, 5-second soft start, existing Hotside/sensor latches, safe output path, and non-blocking main loop.
- Keep the `RESET_REASON` / boot serial diagnostic mapping for `BROWNOUT`. If the hardware restarts again, record `BOOT: RESET_REASON=...` and the App `RESET_REASON` telemetry before drawing a stronger root-cause conclusion.

## Validation status and release boundary

- [SOURCE FIX IMPLEMENTED] A static source correction and regression contract have been committed.
- [TEST ADDED — NOT RUN] `test/v6_3_qc_contract_test.dart` asserts Peltier is paused before selector outputs move, waits the settle interval, restarts a fresh ramp, and retains reset-reason diagnostics. Test execution is not claimed.
- [NOT VERIFIED] Firmware compilation for this revision, physical reproduction, actual brownout behavior, temperature/cooling behavior, and repeated manual/Adaptive voltage transitions have not been validated.
- The previous user report that the earlier V6.3 version compiled does not establish that this new revision compiles.
- No BIN was generated or published, no Firebase metadata was changed, and no device was flashed/OTA-updated in this change.

## Future-change rule

Do not remove the Peltier pause or preserve its old PWM across a rail transition without discussion and explicit user approval. Any change to transition sequencing or timing must update the regression contract and this memory first. Treat reset causality as a hypothesis until real reset diagnostics and, ideally, supply/Peltier current measurements support it.
