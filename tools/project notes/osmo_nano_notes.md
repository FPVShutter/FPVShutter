# Osmo Nano — DUML-over-BLE findings
 
Backend: `src/dji_nano_camera.cpp` on the shared `src/dji_duml_transport.cpp`. All of this is hardware-verified on the Osmo Nano.
 
## Pushed by the camera (no query needed)
| Frame | Contents |
|---|---|
| `0D/02` from `0x05` | Battery push. `pData[31]` = battery %. While charging, `pData[12]` and `pData[16..17]` vary on every push (probably charge current / voltage) and `pData[28]` flips between `08` and `12` (probably a charging flag). |
| `02/80` from `0x01` (73 bytes) | Status push. `pData[11]` record state (`01` idle, `41` starting, `81` recording), `pData[15]` work mode (`01` video, `00` photo), `pData[16..19]` card total MB, `pData[20..23]` card free MB, `pData[28..31]` remaining record time in s. Nothing else in the frame changes when resolution / fps change; only remaining time moves, because the bitrate changes. |
 
Other one-off frames at connect: `07/45` pairing reply, `48→02 00/81` (an ID string, ASCII "ow001"), `88→02 00/74`, `28→02 00/F1`.
 
## Resolution / fps: `02/19` Video Format Get (bench-verified 2026-09-27)
The Nano never pushes these. Query with app `0x02` → camera `0x01`, flags `0x20`, set `0x02`, id `0x19`, empty payload.
The camera answers twice with the request's sequence number:
1. a short ack, payload `01` (14-byte frame)
2. the result, payload `00 | resolution | fps index | 00 00 00` (19-byte frame)
The values use **the same enums as DJI's R SDK 1D02 push** (Osmo-GPS-Controller-Demo `enums_logic.h`):
 
| Camera menu | Reply payload |
|---|---|
| 2.7K 4:3 50 fps | `00 5F 05 00 00 00` |
| 4K 16:9 60 fps | `00 10 06 00 00 00` |
| 2.7K 4:3 60 fps | `00 5F 06 00 00 00` |
| 2.7K 16:9 60 fps | `00 2D 06 00 00 00` |
| 1080p 50 fps | `00 0A 05 00 00 00` |
| 1080p 60 fps | `00 0A 06 00 00 00` |
| Slo-mo 1080p 120 fps | `00 0A 07 00 00 04` |
| Slo-mo 1080p 240 fps | `00 0A 08 00 00 04` |
| Slo-mo 2.7K 16:9 120 fps | `00 2D 07 00 00 04` |
| Slo-mo 4K 16:9 120 fps | `00 10 07 00 00 04` |
| SuperNight | `00 0A 03 00 00 04` (1080p 30) / `00 10 03 00 00 04` (4K 30) |
| Timelapse / Hyperlapse | `00 0A 03 00 00 04` / `00 10 03 00 00 04`, the same as SuperNight |
 
The last payload byte is `04` for a "special" video mode (slo-mo, SuperNight, timelapse, hyperlapse) and `00` for normal video. The other two trailing bytes stayed `00` (meaning unknown).
 
## Telling the special modes apart: `02/6D` Video Record Mode Get
Same addressing as `02/19`. The camera always sends the short ack (payload `01`). Only timelapse and hyperlapse follow it with data, about 30 ms later:
 
| Mode | Payload |
|---|---|
| Timelapse | `00 04 00 14 00 58 02 …`: mode `04`, u16 `0x0014` = 20 (probably interval ×0.1 s = 2 s), u16 `0x0258` = 600 (probably duration in s) |
| Hyperlapse Auto / ×5 / ×10 / ×15 / ×30 | `00 0B 00 <rate> 00 …` with rate `00` / `05` / `0A` / `0F` / `1E` |
 
Normal video, slo-mo and SuperNight only get the ack. In static timelapse the `02/80` push reports work mode `00` (photo), because it shoots stills, so timelapse has to win over `02/80`'s photo. The Nano backend's MODE element is:
- flag `04` and `02/6D` data mode `04` → `TLAPSE` (checked first, even when `02/80` says photo)
- `02/80` work mode `00` → `PHOTO`
- flag `00` → `VIDEO`
- flag `04` and fps > 60 → `SLOMO` (SuperNight, timelapse and hyperlapse are all capped at 30 fps; confirmed by the user for SuperNight)
- flag `04`, `02/6D` data mode `0B` → `HLAPSE`
- flag `04`, `02/6D` ack with no data within 700 ms → `NIGHT`
`02/6D` is only polled while a special mode at ≤60 fps is active, alternating with `02/19` so each still runs every `DJI_NANO_FORMAT_POLL_MS`.
 
Other gets tried: `02/11` Camera Work Mode Get never got a reply.
 
## Bench aids
- `DJI_NANO_FRAME_DISCOVERY` (config.h): 1 = hex-dump the first frame of each type, 2 = also re-dump when a type's bytes change (ignoring the seq and CRC). Shared logger: `dumlFrameLog()`.
- Command IDs come from o-gs/dji-firmware-tools `comm_dissector/wireshark/dji-dumlv1-camera.lua`, which has names for the camera set but no layout for the `02/19` reply.
## Open
- The Action 2 may answer the same `02/19` query. Untested.
- Only one timelapse setting was captured (param 20 / 600). The other timelapse intervals should only change the param.