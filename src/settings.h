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
// PROTOTYPE: split the old single CAMERA_DJI value into CAMERA_DJI_NANO
// (hardware-verified) and CAMERA_DJI_ACTION (assumed compatible, untested —
// see dji_action_camera.h). Existing numeric values are preserved for NVS
// backward compatibility: anything already saved as camera=0 keeps meaning
// exactly what it meant before (this project's actual paired camera, the
// Nano) and camera=1 is untouched. CAMERA_DJI_ACTION is a new value (2), so
// no migration is needed for existing saved settings/paired cameras.
enum CameraType : uint8_t {
    CAMERA_DJI_NANO   = 0,   // DJI Osmo Nano (DUML over BLE) — hardware-verified
    CAMERA_GOPRO      = 1,   // GoPro HERO8+ (Open GoPro BLE API)
    CAMERA_DJI_ACTION = 2,   // DJI Osmo Action family (DUML over BLE) — assumed compatible, untested
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
// OSD slot contents (which info goes into Betaflight Custom Message 1..4)
// ──────────────────────────────────────────────────────────────────────────────

enum OsdSlotContent : uint8_t {
    OSD_SLOT_OFF        = 0,   // Leave this custom message untouched/empty
    OSD_SLOT_CAM_STATUS = 1,   // "REC 85% 12:34" / "STBY ..." (combined)
    OSD_SLOT_REC_TIME   = 2,   // "REC 12:34"
    OSD_SLOT_BATTERY    = 3,   // "BAT 85%"
    OSD_SLOT_LINK       = 4,   // "LINK READY" / "SCAN" / "OFF"
    OSD_SLOT_FC_BATT    = 5,   // "FC 15.8V"
    OSD_SLOT_ARM_STATE  = 6,   // "ARMED" / "DISARMED"

    OSD_SLOT_COUNT      = 7,
};

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
    uint8_t    osdSlot[4];           // OsdSlotContent for Custom Message 1..4
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