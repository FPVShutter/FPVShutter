# Osmo Action 4 / 5 Pro / 6 — DJI R SDK backend notes
 
Started 2026-09-26. Backend: `src/dji_rsdk_camera.cpp` (camera type **3**, "DJI Osmo Action 4+"), frame layer `src/dji_rsdk_protocol.cpp`, built on the shared BLE link in `dji_duml_transport.cpp` via the new `DjiLinkHooks`.
**Status: CONNECTS on a real Action 4 (fw AC203-03.04.80.15) as of 2026-09-26: handshake, model detection, status subscription and telemetry confirmed. Record start/stop and reconnect not yet tested.**
 
## Why a separate backend
The Action 4 / 5 Pro / 6 and Osmo 360 are covered by DJI's official remote protocol (the "DJI R SDK" protocol, published in [dji-sdk/Osmo-GPS-Controller-Demo](https://github.com/dji-sdk/Osmo-GPS-Controller-Demo); demo code MIT, protocol docs under DJI's EULA). DJI lists the Osmo Nano as "not supported yet", so the Nano stays on DUML.
It uses the **same GATT service as DUML** (`FFF0`, notify `FFF4`, write `FFF5`), so connect, reconnect, the auth timeout, the watchdog and the FC-UART bench guard are all reused. Only the protocol on top is new.
 
DUML shouldn't be needed for the basics: one R SDK status push carries record state, record time, remaining time, SD free space, battery, mode/res/fps and overheat state. The backend logs the first DUML frame it sees on the link in case the camera talks both.
 
## Framing (verified against DJI's own frames)
`AA | u16 ver/len (len = low 10 bits) | CmdType | ENC | RES[3] | u16 SEQ | u16 CRC16 | CmdSet CmdID payload | u32 CRC32`, all little-endian.
- CmdType bit 5 = response frame; bits 4:0 = 0 no reply, 1 optional, 2 required.
- CRC-16/ARC (poly 0xA001 reflected) over bytes 0–9, **seed 0x3AA3**.
- CRC-32 (0xEDB88320 reflected) over everything before it, **seed 0x3AA3, no final XOR**.
- Byte-for-byte match with DJI's mode-switch example and all 10 frames in `test/connect_cmd_frame_builder/connect_cmd_frame.txt`.
## Commands used
| Key | Direction | Use |
|---|---|---|
| `0019` | both | Connection handshake (below) |
| `1D05` | → cam | Subscribe to status: push_mode 3 (periodic + on change), freq 20 (fixed 2 Hz) |
| `1D02` | cam → | Status push, 38 bytes: `[0]` mode, `[1]` status (3 = recording, 5 = pre-record), `[2]` res, `[3]` fps idx, `[4]` EIS, `[5..6]` record time s, `[15..18]` SD free MB, `[23..26]` remaining time s, `[28]` power mode (3 = sleep), `[30]` temp (2 = too hot to record), `[37]` battery % |
| `1D06` | cam → | Mode name / parameter strings (logged only) |
| `1D03` | → cam | Record: device_id u32, **0 = start, 1 = stop** (opposite to DUML 02/02), reserved[4] |
| `0000` | → cam | Version query; also used as the 15 s keep-alive |
 
## Handshake (0019)
1. We send a request: device_id `0x12345678`, our BLE MAC, fw 0, verify_mode `0` (the camera decides from its pairing history and prompts only if it doesn't know us), random 4-digit verify_data.
2. The camera replies with a response frame, ret_code 0.
3. The camera sends **its own** 0019 command frame: verify_mode 2, verify_data 0 = allowed / 1 = rejected, device_id = model (`FF33` Action 4, `FF44` 5 Pro, `FF55` 6, `FF66` Osmo 360).
4. We reply on the camera's SEQ, then subscribe (1D05) and send a version query.
- Rejection → auto-reconnect pauses 60 s (DJI says not to re-ask a camera that refused). A user "Use"/pair click overrides the pause.
## Scan filter
DJI's demo accepts manufacturer data `AA 08 xx xx FA` (DJI company ID + byte 4 = `0xFA`). Fallbacks are name prefixes and the plain DJI company ID.
 
## Bench log 1 (2026-09-26)
- The camera advertises as `OsmoAction4-FA06`. The BLE connection opened (MTU 256), then the lookup of service `FFF0` got **no reply**. After about 4.5 s the camera ended the link (`reason=531`, remote user terminated) and the next attempts failed with `status=574`. The firmware reported "not a DJI Osmo camera", which was misleading.
- Fix under test: the R SDK backend now discovers all services (`DjiDumlSession::fullServiceDiscovery`), as DJI's demo and Android remotes do, instead of NimBLE's by-UUID lookup. A camera that hangs up mid-discovery now shows "camera dropped the connection". BLE-level pairing is logged (`RSDK: BLE security change`), because NimBLE auto-pairs on a camera security request and DJI's demo never pairs.
- The user set `DJI_RSDK_VERIFY_MODE` to 1 (always prompt).
## Bench log 2 (2026-09-26): connected
- Full service discovery found 3 services: `0x1800`, `0x1801`, `0xfff0`. There was no BLE security change.
- The camera answered our 0019 request with `ret 0` about 200 ms later. Its own 0019 (verify_mode 2, verify_data 0) arrived about 17 s later, after the on-screen approval with verify_mode 1.
- The camera's device_id is `0x0000FF33` (bytes `33 FF 00 00`). DJI's demo record command uses `0x33FF0000` (bytes `00 00 FF 33`), so start/stop will show whether the camera checks this field.
- The 1D05 subscribe gets a 5-byte response (`00 00 00 00 00`). 1D02 pushes arrive with CmdType `0x01`.
- First push: video mode, res 95 (2.7K 4:3), fps idx 5 (50 fps), EIS 0, battery 85 %, 20458 s remaining, 243882 MB free.
- Version reply: product `DJI-ACTION4`, `SDK-v1.1 DEBUG AC203-03.04.80.15`.
## Values to confirm on the bench
- Record command device_id: DJI's demo hard-codes `0x33FF0000` for the OA4. The demo also writes the id the other way round elsewhere, which suggests the camera ignores the field. Configurable as `DJI_RSDK_CAMERA_DEVICE_ID`.
- Whether verify_mode 0 shows the pairing prompt on a first connection (DJI's doc says yes). If not, set `DJI_RSDK_VERIFY_MODE` to 1.
- The camera's actual device_id in its 0019 (logged as `camera device_id 0x…`).
- Whether record time in `1D02` counts during recording (a local-clock fallback covers it if it reads 0).
- Behaviour when the camera sleeps (power mode 3). Waking it needs a 2 s BLE advert `[0x0A, 0xFF, 'W','K','P', mac reversed]`. Not implemented yet, but useful for arm-to-record.
## Bench aid
`DJI_RSDK_FRAME_DISCOVERY` = 1 (config.h) logs the first frame of each R SDK message type with a hex dump.
Host tests: `sh tools/rsdk_host_test/run.sh` (protocol vectors plus a fake-camera simulation of the whole backend).