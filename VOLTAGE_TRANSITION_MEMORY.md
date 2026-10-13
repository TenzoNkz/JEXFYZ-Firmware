# JE X FYZ / JE Cooler — Voltage Transition & Peltier Soft-Start Memory

Updated: 2026-10-13 (Asia/Jakarta)
Status: Conditional resume rule implemented in V6.3 source; compile/test/runtime/hardware result not yet validated for this revision.
Active source blob: `bb76b9c4e78b0e77d23bd82b13d23da6f623e985`.
Regression contract blob: `0244ceaff0aa2366c3ebe2e0c13e7684f5b085cf`.

## Incident report

[USER-REPORTED — 2026-10-13] The physical hardware restarted when changing from 5 V to 12 V with Peltier and Fan ON while the 5-second Peltier soft-start was still active. The exact reset reason from that reproduction has not been supplied. A supply transient/brownout remains a plausible hypothesis, not a confirmed cause.

## Current transition contract

This rule applies to normal voltage changes requested in Manual Volt and while Adaptive is active. It is based on the Peltier ramp state **when the voltage transition is accepted**:

1. If Peltier is logically ON, capture whether `peltierSoftStarting` is true before changing ramp state; then force physical GPIO5 PWM to 0% before selector outputs are de-asserted. Preserve the logical ON request so the App does not report a false OFF. Fan remains enabled and is forced to 100%.
2. Keep the existing 200 ms break-before-make selector interval.
3. After selector completion, retain the existing 1,000 ms rail-settle wait through `PELTIER_VOLTAGE_SETTLE_MS = VOLTAGE_GUARD_MS`.
4. If the saved flag says the 5-second soft-start was still in progress, begin a **new full 5-second ramp from 0%** after settling.
5. If the saved flag says the soft-start had already completed, restore **100% PWM directly** after settling; do not add another 5-second ramp.
6. If Peltier is OFF, do not enable it because of a voltage change. If an explicit Peltier-OFF request, thermal/sensor safety, or OTA safe-output path stops it during the transition, clear the saved resume state so it cannot restart automatically.
7. Adaptive entry/exit remains its separate safe-5V mode handoff: it intentionally switches Peltier OFF and, when allowed, restores it through the normal OFF→ON 5-second soft-start. Adaptive's subsequent ordinary rail changes use the conditional rule above. Adaptive thermal HOTSTEP safety may deliberately force Peltier OFF and must take priority.
8. The transition remains cooperative/non-blocking; no `delay()` or timing `while()` is introduced.

## Timing contract

- Selector break-before-make: 200 ms.
- Post-selector settle: 1,000 ms using the pre-existing shared guard.
- Interrupted ramp: fresh 5,000 ms soft-start after settling.
- Completed ramp: PWM returns directly to 100% after settling.
- The 1-second interval is an existing software guard, not laboratory proof of the rail's electrical settling time. Hardware validation remains required.

## Regression safeguards

- In `requestVoltage()`, accepted transition state is recorded before `pausePeltierForVoltageTransition()`; the pause must happen before selector outputs are switched OFF.
- The pause helper must snapshot `peltierSoftStarting` into `peltierVoltageResumeSoftStart` before clearing the active ramp and forcing PWM to 0%.
- While `voltageTransition` is active, Peltier processing must fail closed.
- After `PELTIER_VOLTAGE_SETTLE_MS`, resume branches on the captured flag: a fresh ramp only for the previously active soft-start; otherwise full duty is restored.
- `stopPeltierOutput()` must clear both pending pause and saved-resume state.
- Preserve the 200 ms deadtime, 1-second guard, 5-second OFF→ON soft-start, Hotside/sensor safety latches, safe output path, and cooperative main loop.
- Keep serial boot diagnostics and App `RESET_REASON` telemetry for `BROWNOUT`. If the hardware restarts again, record `BOOT: RESET_REASON=...` and the App telemetry before drawing a stronger root-cause conclusion.

## Validation status and release boundary

- [SOURCE FIX IMPLEMENTED] V6.3 now distinguishes an interrupted ramp from a completed ramp.
- [REGRESSION CONTRACT UPDATED — NOT RUN] The static Flutter test contract checks state capture, ordering, both resumption paths, stop-state cleanup, timing, and reset-reason diagnostics.
- [STATIC REVIEW ONLY] No firmware compile, Dart/Flutter test, physical reproduction, supply/current measurement, thermal regression, or repeated Manual/Adaptive rail test has been performed for this revision.
- The earlier user-reported compile result applied to a prior V6.3 revision and does not certify the current source.
- No BIN was generated or published, no Firebase metadata was changed, and no device was flashed or OTA-updated by this change.

## Future-change rule

Do not remove the state snapshot, unconditionally restart the 5-second soft-start, or restore full duty without respecting Peltier-OFF/safety cancellation. Any transition timing or behavior change must update the source-level contract and this memory first. Reset causality remains a hypothesis until real diagnostics and, ideally, supply/Peltier current measurements support it.
