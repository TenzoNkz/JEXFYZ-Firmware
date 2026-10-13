# JE X FYZ Firmware Artifact Repository

This repository serves the firmware binary used for JE X FYZ / JE Cooler Online OTA. The deployable artifact path is `JEXFYZ.cpp.bin`.

## Source-of-truth boundary

- Canonical editable firmware source and QC/test contracts live in [TenzoNkz/JEXFYZ-App](https://github.com/TenzoNkz/JEXFYZ-App), notably [`firmware/JEXFYZ-V6.3.ino`](https://github.com/TenzoNkz/JEXFYZ-App/blob/main/firmware/JEXFYZ-V6.3.ino).
- This repository's `JEXFYZ.cpp.bin` is a binary artifact, not the editable source. An upload timestamp alone does not establish which exact source blob or compile configuration produced it.
- The latest observed BIN upload in the repository history is commit [`b345637acfdbd3f80ae6159a14cc09ddbc05f934`](https://github.com/TenzoNkz/JEXFYZ-Firmware/commit/b345637acfdbd3f80ae6159a14cc09ddbc05f934), which changes `JEXFYZ.cpp.bin`. Its source-blob/build provenance has not been independently verified in this record. Do not assume it necessarily corresponds to the current V6.3 source without matching provenance.
- This documentation update does not replace, rebuild, or modify the existing BIN.

## Online OTA connection regression memory

V6.3 source was restored to the previously working `cloudClient.setInsecure()` path after the CA certificate validation and OTA-only NTP time-sync gate were associated with a user-reported GitHub connection failure. The user reports that V6.3 then compiled successfully and Online OTA could connect to GitHub again. This is user-reported; full BIN transfer, installation, reboot, and post-reboot version confirmation are not independently proven by this repository change.

Read the canonical [OTA Regression Memory in the source repository](https://github.com/TenzoNkz/JEXFYZ-App/blob/main/OTA_REGRESSION_MEMORY.md) before changing or publishing OTA firmware.

## Release guardrails

- Do not replace/publish `JEXFYZ.cpp.bin`, change Firebase/production metadata, flash a device, or perform a release without explicit authorization.
- Keep the raw GitHub BIN URL allowlist, redirect blocking, image/size checks, safe outputs, OTA recovery, and radio handoff intact.
- `setInsecure()` keeps TLS encryption but skips server certificate identity validation. URL allowlisting does not replace cryptographic server authentication.
- Build/test/release claims must be labeled accurately. A source contract test is not proof that a test was executed; a successful compile is not proof of successful end-to-end OTA installation.
