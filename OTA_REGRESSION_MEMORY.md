# JE X FYZ — Online OTA Regression Memory (Artifact Repository)

Updated: 2026-10-13 (Asia/Jakarta)

This repository primarily stores the OTA binary, not editable firmware source. Canonical source and the full engineering incident log are in [TenzoNkz/JEXFYZ-App / OTA_REGRESSION_MEMORY.md](https://github.com/TenzoNkz/JEXFYZ-App/blob/main/OTA_REGRESSION_MEMORY.md).

Related electrical/selector restart issue is tracked separately in [VOLTAGE_TRANSITION_MEMORY.md](https://github.com/TenzoNkz/JEXFYZ-App/blob/main/VOLTAGE_TRANSITION_MEMORY.md).

## Incident record

- User reported that V6.3 connected to Wi-Fi (blue indicator) then showed red at the GitHub connection stage after a change to CA-based HTTPS validation.
- Source at the time contained an embedded ISRG Root X1 certificate applied with `cloudClient.setCACert(GITHUB_ROOT_CA)`, plus an OTA-only NTP time-sync gate (`configTime()`, `TIME_SYNCING`, 20-second timeout).
- Source was restored to `cloudClient.setInsecure()`; the CA PEM and OTA-only NTP gate were removed. Blue Wi-Fi feedback now precedes the GitHub GET attempt.
- User reports the corrected V6.3 compiled successfully and Online OTA could connect to GitHub again.
- Do not claim an independently reproduced build or end-to-end OTA completion from this note. A full transfer, `Update.end()`, successful restart, and confirmation of the installed version need separate evidence.
- The change removed CA validation and the NTP gate together, so available evidence does not isolate either as the sole cause. The user attributes the failure to the CA addition; preserve that as the user's conclusion, not as a single-variable controlled experiment.

## Required guardrails

1. Do not reintroduce a custom CA, certificate pinning, `setCACert()`, or OTA-only NTP/TLS-time gate without discussion and explicit user authorization.
2. Keep raw GitHub BIN URL allowlisting, redirect blocking, file-length/size validation, ESP image magic-byte validation, streaming update lifecycle, safe outputs, radio handoff, and failure recovery intact.
3. `setInsecure()` does not authenticate the TLS peer certificate. URL allowlisting alone is not cryptographic proof of server identity.
4. Do not change `JEXFYZ.cpp.bin`, Firebase/production release metadata, or release state unless explicitly authorized.
5. A binary upload timestamp is not proof that the file matches the active source blob. Record source commit/blob, toolchain, compile result, and artifact checksum when a future build/release is authorized.

## Artifact checkpoint

- Latest observed BIN upload commit at the time of this record: [`b345637acfdbd3f80ae6159a14cc09ddbc05f934`](https://github.com/TenzoNkz/JEXFYZ-Firmware/commit/b345637acfdbd3f80ae6159a14cc09ddbc05f934).
- The artifact was not modified by the documentation update.
- Source-to-BIN correspondence is not independently confirmed in this note.
