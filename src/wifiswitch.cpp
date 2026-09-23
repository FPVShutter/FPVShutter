// ============================================================================
// wifiswitch.cpp — Toggle the Web-UI Wi-Fi AP from a spare radio switch
// ============================================================================
// PROTOTYPE: the optional AUX toggle below now only takes effect while the
// new master Wi-Fi AP switch (settingsGet().wifiApEnabled) is enabled. When
// the master is disabled, wifiSwitchUpdate() returns immediately — the AUX
// channel can turn the AP off (as it always could) but can never turn it
// back on while the master itself is off. The master switch is applied
// directly from main.cpp (boot) and api_core.cpp (live changes via the Web
// UI / bench console), not from here.
// ============================================================================

#include "wifiswitch.h"
#include "config.h"
#include "settings.h"
#include "web_server.h"

// ──────────────────────────────────────────────────────────────────────────────
// Internal state
// ──────────────────────────────────────────────────────────────────────────────

static bool     _rcValid      = false;
static bool     _rawOn        = false;    // Un-debounced switch position
static bool     _stableOn     = true;     // Debounced position (boot default: on)
static uint32_t _lastRawChange= 0;

/// Minimum time a new raw position must hold before it is accepted.  Uses the
/// configured debounce but never less than 600 ms — you don't want Wi-Fi
/// flapping because of a bumpy thumb.
#define WIFI_SWITCH_DEBOUNCE_MS 600

// ──────────────────────────────────────────────────────────────────────────────
// Public API
// ──────────────────────────────────────────────────────────────────────────────

void wifiSwitchInit() {
    _rcValid = false;
    _stableOn = true;   // Assumed AUX position at boot (lock-out protection);
                         // irrelevant while wifiSwitchCh is unset, and
                         // irrelevant while the master AP switch is off,
                         // since wifiSwitchUpdate() won't act on it either way.
}

void wifiSwitchFeedRc(uint16_t rcValueUsec) {
    _rcValid = true;
    _rawOn = (rcValueUsec > settingsGet().rcThresholdUs);
    if (_rawOn != _stableOn && _lastRawChange == 0) {
        // Potential transition — start the qualification window.
        _lastRawChange = millis();
    }
}

void wifiSwitchUpdate() {
    const ShutterSettings &cfg = settingsGet();
    if (!cfg.wifiApEnabled) return;      // Master switch off — AUX cannot override
    if (cfg.wifiSwitchCh > 15) return;   // No AUX toggle configured — AP just stays on

    if (!_rcValid) return;

    uint32_t now = millis();

    // If the raw position flipped back during the window, re-arm.
    if (_rawOn == _stableOn) {
        _lastRawChange = 0;
        return;
    }

    if (_lastRawChange != 0 &&
        now - _lastRawChange >= WIFI_SWITCH_DEBOUNCE_MS) {
        _stableOn      = _rawOn;
        _lastRawChange = 0;

        DBG("WIFI-SWITCH: %s", _stableOn ? "ON" : "OFF");
        if (_stableOn) {
            webStart();    // Bring SoftAP + DNS back up
        } else {
            webStop();     // Powers down the whole Wi-Fi radio
        }
    }
}