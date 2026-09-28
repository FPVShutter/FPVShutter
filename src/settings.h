// ============================================================================
// settings.h — Persistent runtime configuration (NVS via Preferences)
// ============================================================================
// Everything the user can change from the Web UI lives here and survives
// reboots. Defaults come from config.h.
//
//   • Camera brand selection (DJI Osmo Nano / DJI Osmo Action / GoPro HERO8+)
//   • Record switch: RC channel index, threshold, debounce
//   • Record-on-arm (+ optional stop on disarm)
//   • Wi-Fi AP credentials for the Web UI
//   • OSD content assignment for Custom Message slots 1..4
// ============================================================================

#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include "config.h"

// ──────────────────────────────────────────────────────────────────────────────
// Camera brands
// ──────────────────────────────────────────────────────────────────────────────
// Split the old single CAMERA_DJI value into CAMERA_DJI_NANO and
// CAMERA_DJI_ACTION (Action 2), both hardware-verified — see
// dji_nano_camera.h / dji_action_camera.h. Existing numeric values are preserved for NVS
// backward compatibility: anything already saved as camera=0 keeps meaning
// exactly what it meant before (this project's actual paired camera, the
// Nano) and camera=1 is untouched. CAMERA_DJI_ACTION is a new value (2), so
// no migration is needed for existing saved settings/paired cameras.
// CAMERA_DJI_RSDK (3) follows the same rule: a new value, nothing migrates.
enum CameraType : uint8_t {
    CAMERA_DJI_NANO   = 0,   // DJI Osmo Nano (DUML over BLE) — hardware-verified
    CAMERA_GOPRO      = 1,   // GoPro HERO8+ (Open GoPro BLE API)
    CAMERA_DJI_ACTION = 2,   // DJI Osmo Action 2 (DUML over BLE, polled telemetry) — hardware-verified
    CAMERA_DJI_RSDK   = 3,   // DJI Osmo Action 4 / 5 Pro / 6, Osmo 360 (DJI R SDK over BLE) — not yet bench-tested
};

// ──────────────────────────────────────────────────────────────────────────────
// BLE TX power levels
// ──────────────────────────────────────────────────────────────────────────────
// PROTOTYPE: simple 3-step picker (rather than a raw dBm value or a dynamic
// scan-vs-connected scheme) for tuning how hard the C3's shared 2.4GHz radio
// drives BLE. Lower levels trade BLE range for less RF footprint/current
// draw — useful when a co-located ELRS receiver is being desensed by a
// high-power, close-proximity BLE radio. HIGH reproduces the previous
// hardcoded behaviour (ESP_PWR_LVL_P9) exactly, so existing installs are
// unaffected until the user changes it.
enum BlePowerLevel : uint8_t {
    BLE_POWER_LOW    = 0,   // ~ -9 dBm — shortest range, least RF footprint
    BLE_POWER_MEDIUM = 1,   // ~  0 dBm — balanced
    BLE_POWER_HIGH   = 2,   // ~ +9 dBm — longest range (previous fixed default)
};

// ──────────────────────────────────────────────────────────────────────────────
// OSD elements (building blocks of Betaflight Custom Message 1..4)
// ──────────────────────────────────────────────────────────────────────────────
// Each custom-message slot is an optional short label followed by up to
// OSD_ELEMS_PER_SLOT elements, joined with single spaces and cut to
// OSD_MAX_TEXT_LEN (whole elements only — one that doesn't fit is dropped,
// never split). See osd_slots.cpp for the exact text of each element.
//
// The numeric IDs are persisted in NVS and used by the Web UI / Bench
// Console: never renumber, only append. The browser-side preview
// (web_assets.h and docs/app.js) mirrors this list and the formatter.

#define OSD_ELEMS_PER_SLOT   4
#define OSD_LABEL_MAX_LEN    6

enum OsdElement : uint8_t {
    OSD_EL_NONE      = 0,
    OSD_EL_CAM_STATE = 1,   // "REC" / "STBY" / "CAM OFF" / "CAM SCAN" / "CAM PAIR"
    OSD_EL_REC_TIME  = 2,   // "12:34" elapsed while recording, "2H33M" remaining in standby
    OSD_EL_CAM_BATT  = 3,   // "85%"
    OSD_EL_LINK      = 4,   // "READY" / "PAIR" / "CONN" / "SCAN" / "OFF"
    OSD_EL_FC_VOLT   = 5,   // "15.8V"
    OSD_EL_ARM       = 6,   // "ARMED" / "DISARMED" / "FC NOLINK"
    OSD_EL_MODE      = 7,   // "VIDEO" / "SLOMO" / "HLAPSE" / "PHOTO" ...
    OSD_EL_RES       = 8,   // "4K" / "2.7K" / "1080P"
    OSD_EL_ASPECT    = 9,   // "16:9" / "4:3" / "9:16"
    OSD_EL_FPS       = 10,  // "60FPS"
    OSD_EL_FORMAT    = 11,  // "4K60" (resolution + fps in one)
    OSD_EL_EIS       = 12,  // "RS+" / "HS" / "RS" / "HB" / "EIS OFF"
    OSD_EL_SD_FREE   = 13,  // "112G" / "850M" free on the card
    OSD_EL_TEMP      = 14,  // "" when normal, "WARM" / "HOT" / "OVERHEAT" as an alert

    OSD_EL_COUNT     = 15,
};

// Legacy single-choice slot contents (firmware <= v2.3). Still accepted by
// the settings API as "slotN" and migrated from NVS on first boot — each
// maps onto an element list + label that renders the same text.
enum OsdSlotContent : uint8_t {
    OSD_SLOT_OFF        = 0,
    OSD_SLOT_CAM_STATUS = 1,   // "REC 85% 12:34" / "STBY 85% 2H33M"
    OSD_SLOT_REC_TIME   = 2,   // "REC 12:34"
    OSD_SLOT_BATTERY    = 3,   // "BAT 85%"
    OSD_SLOT_LINK       = 4,   // "LINK READY"
    OSD_SLOT_FC_BATT    = 5,   // "FC 15.8V"
    OSD_SLOT_ARM_STATE  = 6,   // "ARMED" / "DISARMED"

    OSD_SLOT_COUNT      = 7,
};

struct OsdSlotConfig {
    uint8_t elem[OSD_ELEMS_PER_SLOT];     // OsdElement, OSD_EL_NONE = unused
    char    label[OSD_LABEL_MAX_LEN + 1]; // optional prefix, e.g. "CAM" ("" = none)
};

/// Fill `out` with the element list + label equivalent to a legacy
/// OsdSlotContent value (unknown values → empty slot).
void osdSlotFromLegacy(uint8_t legacy, OsdSlotConfig &out);

/// Make a user-supplied label OSD-safe in place: uppercase, printable ASCII
/// only (no '"' or '\\' so it can be echoed in JSON), max OSD_LABEL_MAX_LEN.
void osdSanitizeLabel(char *label);

// ──────────────────────────────────────────────────────────────────────────────
// Saved camera registry (auto-learned during scans, persisted)
// ──────────────────────────────────────────────────────────────────────────────

#define MAX_SAVED_CAMERAS 4

struct SavedCamera {
    uint8_t type;              // CameraType
    bool    active;            // The one the ESP32 should connect to
    char    mac[18];           // "AA:BB:CC:DD:EE:FF"
    char    name[24];          // Advertised name (may be empty)
};

// ──────────────────────────────────────────────────────────────────────────────
// Settings container
// ──────────────────────────────────────────────────────────────────────────────

struct ShutterSettings {
    CameraType camera;               // Active camera backend
    uint8_t    auxChannelIndex;      // RC channel used as record switch (0-based)
    uint16_t   rcThresholdUs;        // µs above which the switch is ON
    uint16_t   debounceMs;           // Switch debounce time
    bool       recordOnArm;          // Start recording when FC arms
    bool       stopOnDisarm;         // Stop recording when FC disarms (needs recordOnArm)
    uint16_t   stopOnDisarmDelayMs;  // Grace period before stop-on-disarm fires (0 = instant)
    bool       scanAll;              // Accept any advertiser during discovery
                                     // (otherwise filters by MAC OUI / name / mfr)
    char       apSsid[33];           // SoftAP SSID for the Web UI
    char       apPass[65];           // SoftAP password (min 8 chars, or empty = open)
    OsdSlotConfig osd[4];            // Custom Message 1..4 layout (elements + label)
    uint8_t    wifiSwitchCh;         // Optional RC channel toggling Wi-Fi (255 =
                                     // no AUX toggle configured). Only takes
                                     // effect while wifiApEnabled is true — see
                                     // below.
    bool       wifiApEnabled;        // Master Wi-Fi AP switch. false = the AP
                                     // never starts (full radio-off), regardless
                                     // of wifiSwitchCh. true = previous/default
                                     // behaviour — AP allowed to run, optionally
                                     // still toggled in-field by wifiSwitchCh.
    BlePowerLevel blePower;          // BLE TX power (Low/Medium/High)

    SavedCamera cams[MAX_SAVED_CAMERAS];
    uint8_t     camCount;
};

/// Load settings from NVS (or defaults on first boot).
void settingsLoad();

/// Persist current settings to NVS.
void settingsSave();

/// Reset to compile-time defaults (does not save automatically).
void settingsReset();

/// Access the live settings struct.
ShutterSettings& settingsGet();

/// Human-readable name of a camera type ("DJI Osmo Nano" / "DJI Osmo Action" / "GoPro").
const char* cameraTypeName(CameraType type);

/// Human-readable name of a BLE TX power level ("Low" / "Medium" / "High").
const char* blePowerName(BlePowerLevel level);

#endif // SETTINGS_H