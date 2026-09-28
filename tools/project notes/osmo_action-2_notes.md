# Osmo Action 2 — DUML-over-BLE findings
 
Verified on a borrowed DJI Osmo Action 2 against FPVShutter firmware, 2026-09-24.
Backend: `dji_action_camera.cpp` on the shared `dji_duml_transport.cpp`.
 
## Same as the Osmo Nano
- GATT service `0xFFF0`: commands written without response to `FFF5`, notifications on `FFF4`.
- Pairing: write `[01 00]` to `FFF4`, then `07/45` with identifier + token. The camera replies `07/45` payload `00 01` = already paired (fast path, survives a camera power cycle).
- Record: `02/02` from app `0x02` to camera `0x01`, flags `0x40`, payload `01` start / `00` stop.
## Different from the Nano: telemetry is polled, never pushed
The Action 2 sent no unsolicited `02/80` or `0D/02` in 20+ s, and never answered a `02/80` query.
 
| Query (flags `0x20`) | To | Every | Reply (flags `0xC0`), after 11-byte header | Use |
|---|---|---|---|---|
| `02/70` payload `01` | `0x01` | 500 ms | `[00 result][state]...`, state at `pData[12]`: `01` idle, `41` starting, `81` recording, `C1` saving (~150–500 ms after stop) | Record state |
| `0D/02` payload `00 00 00 00` | `0x05` | 5 s | result byte then battery struct; battery % at `pData[32]` | Battery |
| `02/71` (empty) | `0x01` | 3 s, standby only | `00 \| 01 \| total MB u32 \| free MB u32 \| remaining shots u32 \| remaining rec time s u32`, so remaining time = `pData[25..28]`, free MB = `pData[17..20]` | Remaining time |
 
Sample `02/71` reply: `55 1F 04 4E 01 02 00 06 C0 02 71 00 01 21 59 00 00 A8 2B 00 00 00 00 00 00 E4 02 00 00 ..` → 22817 MB total, 11176 MB free, 740 s remaining, which matched the camera screen's 12:20. The `02/71` field order matches DJI's `02/80` "Camera State Info" struct (dji-firmware-tools camera dissector).
 
## Heartbeat
Every 1 s: `00/2B` payload `01 01` to `0xF0` (flags `0x40`), plus an empty `00/00` to `0x28` (flags `0x00`). The camera answers the latter from `0x28`. This is what DJI's own remote sends; RotorREC reports it as confirmed on the Action 2. The shared 15 s `00/F1` keep-alive also still runs.
 
## Watchdog lessons
- **Timer race (affected both DJI backends):** `lastRxMs` is written by the NimBLE host task while `dumlUpdate()` reads `now` at the top of its tick. A reply landing in between made `now - lastRxMs` wrap to about 4294967 s and fire the stale-link watchdog on a healthy link. Fixed by snapshotting `lastRxMs` and clamping a negative age to 0. The elapsed-record clock (`_recordStartMs`) got the same clamp.
- **Action backend sets `softRecoverOnStale`:** after 15 s of silence it re-sends the pairing PIN in place, and only does a full reconnect if nothing answers within `DJI_SOFT_RECOVER_GRACE_MS` (5 s).
## Reconnect after a camera power cycle
The link dropped (`reason=520`). Attempts then failed with `status=13` (camera off) and `574`/`520` (camera advertising but still booting), and it reconnected about 33 s later with "already paired" and no on-screen approval.
 
## Open items
- A start command landing inside the `C1` saving window (Crash Flip re-arm) isn't bench-testable. Stop-on-disarm's delay should keep a start out of that window.
- BLE connect blocks `loop()` for up to 10 s per attempt while the camera is off. Non-blocking connect is planned for a separate session.
- Action 3/4/5 Pro/6 are untested. Set `DJI_ACTION_FRAME_DISCOVERY` to 1 in `config.h` to log each new message type once.
## Sources
- flosean/RotorREC: Action 2 hardware results (protocol facts only; the repo has no licence, no code used)
- FLORIANSV35/controle-dji-action2 (MIT)
- rover1312/shutterlink commit `0ca5fa1` (upstream Action 2 pairing/record)
- o-gs/dji-firmware-tools `comm_dissector/wireshark/dji-dumlv1-camera.lua`