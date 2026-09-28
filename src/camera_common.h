// ============================================================================
// camera_common.h — Shared types for camera BLE backends
// ============================================================================
// Every camera backend (DJI Osmo, GoPro) implements the same function set so
// the camera manager can dispatch to the active one at runtime.
// ============================================================================

#ifndef CAMERA_COMMON_H
#define CAMERA_COMMON_H

#include <Arduino.h>
#include "config.h"

// ──────────────────────────────────────────────────────────────────────────────
// Camera State (parsed from telemetry notifications)
// ──────────────────────────────────────────────────────────────────────────────

enum CameraRecordingState : uint8_t {
    CAM_STATE_UNKNOWN   = 0,
    CAM_STATE_STANDBY   = 1,
    CAM_STATE_RECORDING = 2,
    CAM_STATE_ERROR     = 3,
};

struct CameraTelemetry {
    uint8_t              batteryPercent;
    uint16_t             recTimeSeconds;    // now: estimated REMAINING record time
    CameraRecordingState state;
    bool                 dataValid;
    char                 model[24];

    uint8_t   captureMode;   // 1 = video, 0 = photo (raw byte, mapped in UI layer)
    uint16_t  storageRaw;    // free-storage counter, unit still TBD
    bool      photoPending;  // true briefly after a photo capture (pData[24:27] != 0)

    // ── Video format / camera health (optional, per backend) ────────────────
    // Filled only by backends whose protocol actually reports them; left at
    // the "unknown" value otherwise. The OSD element renderer and both UIs
    // show a placeholder for unknown fields. Labels are short, UPPERCASE and
    // OSD-safe (Betaflight fonts have no real lowercase glyphs).
    char      modeLabel[8];   // "VIDEO" / "SLOMO" / "TLAPSE" / "PHOTO"... ("" = unknown)
    char      resLabel[6];    // "4K" / "2.7K" / "1080P"                  ("" = unknown)
    char      aspectLabel[6]; // "16:9" / "4:3" / "9:16"                  ("" = unknown)
    uint16_t  fps;            // frames per second                        (0 = unknown)
    char      eisLabel[8];    // "RS+" / "HS" / "RS" / "HB" / "OFF"        ("" = unknown)
    uint8_t   tempState;      // 0 normal/unknown, 1 warm, 2 too hot to record, 3 overheat shutdown
    uint32_t  freeMb;         // free card space in MB           (CAM_FREE_MB_UNKNOWN = unknown)

    CameraTelemetry()
        : batteryPercent(255), recTimeSeconds(0),
          state(CAM_STATE_UNKNOWN), dataValid(false),
          captureMode(1), storageRaw(0), photoPending(false),
          fps(0), tempState(0), freeMb(0xFFFFFFFFu) {
        model[0] = '\0';
        modeLabel[0] = resLabel[0] = aspectLabel[0] = eisLabel[0] = '\0';
    }
};

static const uint32_t CAM_FREE_MB_UNKNOWN = 0xFFFFFFFFu;

// ──────────────────────────────────────────────────────────────────────────────
// BLE Connection State
// ──────────────────────────────────────────────────────────────────────────────

enum BleConnectionState : uint8_t {
    BLE_DISCONNECTED,
    BLE_SCANNING,
    BLE_CONNECTING,
    BLE_AUTHENTICATING,   // DJI: pairing handshake / GoPro: waiting for approval
    BLE_CONNECTED,        // Ready to send commands
};

#endif // CAMERA_COMMON_H
