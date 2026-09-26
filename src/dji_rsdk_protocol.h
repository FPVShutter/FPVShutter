// ============================================================================
// dji_rsdk_protocol.h — DJI R SDK frame layer (0xAA framing) for Osmo cameras
// ============================================================================
// Pure framing / payload helpers for the protocol DJI documents in its
// official "Osmo GPS Controller" demo (github.com/dji-sdk/Osmo-GPS-Controller-
// Demo, docs/protocol.md + docs/protocol_data_segment.md). DJI lists it for
// the Osmo Action 4 / 5 Pro / 6 and Osmo 360; the Osmo Nano is listed as
// "not supported yet" (it stays on the DUML backend).
//
// No Arduino / NimBLE dependencies on purpose: this file is unit-tested on
// a desktop compiler against DJI's own example frames (see the checks
// described in dji_rsdk_camera.cpp). Written from the protocol docs; the
// CRC parameters (CRC-16/ARC and CRC-32 reflected, both seeded 0x3AA3)
// were confirmed against DJI's worked mode-switch example frame.
//
// Frame layout (little-endian throughout):
//   off 0   SOF 0xAA
//   off 1   u16 ver/len  — [15:10] version (0), [9:0] total frame length
//   off 3   CmdType      — [4:0] response type (0 none, 1 optional, 2+ required)
//                          [5]   0 = command frame, 1 = response frame
//   off 4   ENC          — 0 = no encryption
//   off 5   RES[3]
//   off 8   u16 SEQ      — a response reuses the command's SEQ
//   off 10  u16 CRC-16   — over bytes 0..9
//   off 12  CmdSet, CmdID, payload...
//   end-4   u32 CRC-32   — over everything before it
// ============================================================================

#ifndef DJI_RSDK_PROTOCOL_H
#define DJI_RSDK_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

// ── Frame constants ─────────────────────────────────────────────────────────
static const uint8_t  RSDK_SOF          = 0xAA;
static const size_t   RSDK_HEADER_LEN   = 12;     // SOF .. CRC-16
static const size_t   RSDK_OVERHEAD     = 12 + 2 + 4;  // header + set/id + CRC-32
static const size_t   RSDK_MIN_FRAME    = 16;     // header + CRC-32 (no set/id)
static const size_t   RSDK_MAX_FRAME    = 1023;   // 10-bit length field

// CmdType values used by DJI's demo (enums_logic.h there).
static const uint8_t  RSDK_CMD_NO_RESPONSE     = 0x00;
static const uint8_t  RSDK_CMD_RESPONSE_OR_NOT = 0x01;
static const uint8_t  RSDK_CMD_WAIT_RESULT     = 0x02;
static const uint8_t  RSDK_ACK_NO_RESPONSE     = 0x20;
static const uint8_t  RSDK_FRAME_TYPE_RESPONSE = 0x20;  // bit 5 of CmdType

// ── Command set / id pairs used by this project ────────────────────────────
// (CmdSet << 8 | CmdID), as in DJI's docs ("0019", "1D02", ...).
static const uint16_t RSDK_VERSION_QUERY   = 0x0000;
static const uint16_t RSDK_CONNECT         = 0x0019;
static const uint16_t RSDK_POWER_MODE      = 0x001A;
static const uint16_t RSDK_STATUS_PUSH     = 0x1D02;
static const uint16_t RSDK_RECORD_CONTROL  = 0x1D03;
static const uint16_t RSDK_MODE_SWITCH     = 0x1D04;
static const uint16_t RSDK_STATUS_SUBSCRIBE= 0x1D05;
static const uint16_t RSDK_STATUS_PUSH_NEW = 0x1D06;

// Camera device IDs from the connection handshake (protocol_data_segment.md).
static const uint16_t RSDK_DEVID_ACTION4    = 0xFF33;
static const uint16_t RSDK_DEVID_ACTION5PRO = 0xFF44;
static const uint16_t RSDK_DEVID_ACTION6    = 0xFF55;
static const uint16_t RSDK_DEVID_OSMO360    = 0xFF66;

// ── CRCs ────────────────────────────────────────────────────────────────────
uint16_t rsdkCrc16(const uint8_t *data, size_t len);   // CRC-16/ARC, seed 0x3AA3
uint32_t rsdkCrc32(const uint8_t *data, size_t len);   // CRC-32 reflected, seed 0x3AA3, no final XOR

// ── Build ───────────────────────────────────────────────────────────────────
/// Builds one complete frame into `out`. Returns the frame length, or 0 if
/// it would not fit in `cap` / exceed RSDK_MAX_FRAME.
size_t rsdkBuildFrame(uint8_t *out, size_t cap, uint8_t cmdType, uint16_t seq,
                      uint8_t cmdSet, uint8_t cmdId,
                      const uint8_t *payload, size_t payloadLen);

// ── Parse ───────────────────────────────────────────────────────────────────
struct RsdkFrame {
    uint8_t        cmdType;
    uint16_t       seq;
    uint8_t        cmdSet;
    uint8_t        cmdId;
    const uint8_t *payload;      // points into the caller's / reassembler's buffer
    size_t         payloadLen;

    bool     isResponse() const { return (cmdType & RSDK_FRAME_TYPE_RESPONSE) != 0; }
    uint16_t key()        const { return (uint16_t)((cmdSet << 8) | cmdId); }
};

/// Validates one complete frame (SOF, length, both CRCs) and fills `out`.
bool rsdkParseFrame(const uint8_t *data, size_t len, RsdkFrame &out);

/// Stream reassembler: BLE notifications are not guaranteed to line up with
/// frame boundaries (a long frame can span several notifications, or two
/// short ones can share one), so bytes are accumulated here and complete,
/// CRC-valid frames are handed to the callback. Resyncs on 0xAA after
/// garbage or a CRC failure by dropping one byte at a time.
typedef void (*RsdkFrameHandler)(const RsdkFrame &frame, void *ctx);

class RsdkReassembler {
public:
    void     reset() { _len = 0; }
    void     feed(const uint8_t *data, size_t len, RsdkFrameHandler handler, void *ctx);
    uint32_t crcErrors()    const { return _crcErrors; }
    uint32_t droppedBytes() const { return _droppedBytes; }
private:
    uint8_t  _buf[RSDK_MAX_FRAME + 1];
    size_t   _len = 0;
    uint32_t _crcErrors = 0;
    uint32_t _droppedBytes = 0;
    void     drop(size_t n);
};

// ── Payloads ────────────────────────────────────────────────────────────────

/// 0019 connection request, command frame (33 bytes). `mac` is in display
/// order (aa:bb:... → mac[0]=0xaa), matching the doc's example.
size_t rsdkBuildConnectRequest(uint8_t *out, uint32_t deviceId, const uint8_t mac[6],
                               uint8_t verifyMode, uint16_t verifyData);

/// 0019 connection response frame payload (9 bytes): device_id, ret_code,
/// reserved[4] (reserved[0] = camera index, 0 = single camera).
size_t rsdkBuildConnectResponse(uint8_t *out, uint32_t deviceId, uint8_t retCode,
                                uint8_t cameraIndex);

/// The camera's own 0019 command frame (verify_mode 2 = verification result).
struct RsdkConnectRequest {
    uint32_t deviceId;
    uint8_t  verifyMode;
    uint16_t verifyData;   // with verifyMode 2: 0 = allowed, 1 = rejected
};
bool rsdkParseConnectRequest(const uint8_t *p, size_t n, RsdkConnectRequest &out);

/// 1D03 record control payload (9 bytes). NOTE the polarity: DJI's R SDK
/// uses 0 = START, 1 = STOP (the DUML 02/02 command is the other way round).
size_t rsdkBuildRecordControl(uint8_t *out, uint32_t deviceId, bool start);

/// 1D05 status subscription payload (6 bytes). DJI: push_freq only accepts
/// 20 (= 2 Hz); push_mode 3 = periodic + an extra push on every change.
size_t rsdkBuildStatusSubscribe(uint8_t *out, uint8_t pushMode, uint8_t pushFreq);

/// 1D02 camera status push (command frame from the camera, >= 38 bytes).
struct RsdkCameraStatus {
    uint8_t  cameraMode;      // 0x01 video, 0x00 slow-mo, 0x05 photo, ...
    uint8_t  cameraStatus;    // 0 screen off, 1 live view, 2 playback, 3 shooting/recording, 5 pre-recording
    uint8_t  videoResolution; // 10 1080p, 16 4K 16:9, 45 2.7K 16:9, 95 2.7K 4:3, 103 4K 4:3, ...
    uint8_t  fpsIdx;          // 1 24, 2 25, 3 30, 4 48, 5 50, 6 60, 10 100, 7 120, 19 200, 8 240
    uint8_t  eisMode;         // 0 off, 1 RS, 2 HS, 3 RS+, 4 HB
    uint16_t recordTimeS;     // current recording time (incl. pre-record), seconds
    uint32_t remainCapacityMb;
    uint32_t remainPhotos;
    uint32_t remainTimeS;     // remaining record time, seconds
    uint8_t  userMode;
    uint8_t  powerMode;       // 0 normal, 3 sleep
    uint8_t  tempOver;        // 0 normal, 1 warm, 2 too hot to record, 3 shutting down
    uint8_t  batteryPercent;
};
static const size_t RSDK_STATUS_PUSH_MIN_LEN = 38;
bool rsdkParseCameraStatus(const uint8_t *p, size_t n, RsdkCameraStatus &out);

/// Human-readable model for a camera device ID (either half, either byte order, is
/// checked, since DJI's own demo writes the 0xFF33 id both ways round).
const char *rsdkModelName(uint32_t deviceId);

/// Frame-rate index → fps (0 if unknown), for logs / future OSD use.
uint16_t rsdkFpsFromIdx(uint8_t idx);

#endif // DJI_RSDK_PROTOCOL_H
