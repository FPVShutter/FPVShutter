# DJI R SDK host tests

Desktop checks for the Osmo Action 4 / 5 Pro / 6 backend (`src/dji_rsdk_*`),
no hardware needed. Needs `g++` (Linux, macOS, WSL or MSYS2).

    sh tools/rsdk_host_test/run.sh

- `protocol_test.cpp` checks the frame layer against DJI's own example frames
  (byte-for-byte), plus the stream reassembler, the status-push parser and the
  record-command polarity.
- `sim_test.cpp` runs the real backend and shared BLE transport against a fake
  camera: handshake, subscription, telemetry, start/stop, keep-alive, watchdog
  recovery, rejection backoff.
- `stub/` holds just enough of Arduino / NimBLE / FreeRTOS to compile those
  files on a PC. It is not used by the firmware build.

PlatformIO only builds `src/`, so this folder doesn't affect `pio run`.
