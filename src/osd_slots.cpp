// ============================================================================
// osd_slots.cpp — Betaflight Custom Message 1..4 content manager
// ============================================================================
// Each slot is an optional label plus up to OSD_ELEMS_PER_SLOT elements
// (OsdElement in settings.h), rendered by osd_format.cpp.
//
// Betaflight limits each custom message to 16 characters (MAX_NAME_LENGTH),
// same as pilot/craft name.
// ============================================================================

#include "osd_slots.h"
#include "config.h"
#include "settings.h"
#include "camera_manager.h"
#include "fc_status.h"
#include "recorder.h"
#include "msp_protocol.h"
#include "osd_format.h"

// ──────────────────────────────────────────────────────────────────────────────
// Internal state
// ──────────────────────────────────────────────────────────────────────────────

static uint32_t _lastPush        = 0;
static char     _lastSent[4][OSD_MAX_TEXT_LEN + 1] = {"", "", "", ""};
static bool     _firstRun        = true;

// ──────────────────────────────────────────────────────────────────────────────
// Public API
// ──────────────────────────────────────────────────────────────────────────────

void osdSlotsInit() {
    for (int i = 0; i < 4; i++) _lastSent[i][0] = '\0';
}

const char* osdSlotText(uint8_t slot) {
    if (slot >= 4) return "";
    return _lastSent[slot];
}

void osdSlotsUpdate() {
    uint32_t now = millis();
    if (!_firstRun && now - _lastPush < OSD_UPDATE_INTERVAL_MS) return;
    bool forcePush = _firstRun;
    _firstRun = false;
    _lastPush = now;

    const ShutterSettings &cfg = settingsGet();
    const FcTelemetry &fc = fcGetTelemetry();

    OsdContext ctx;
    ctx.link     = camGetState();
    ctx.camReady = camIsReady();
    ctx.tel      = &camGetTelemetry();
    ctx.fcAlive  = fc.fcAlive;
    ctx.armed    = fc.armed;
    ctx.vbat10   = fc.vbat10;

    for (uint8_t slot = 0; slot < 4; slot++) {
        // Only push a slot when its text actually changed.
        char buf[OSD_MAX_TEXT_LEN + 1];
        osdFormatSlot(cfg.osd[slot], ctx, buf, sizeof(buf));

        if (forcePush || strcmp(buf, _lastSent[slot]) != 0) {
            strlcpy(_lastSent[slot], buf, sizeof(_lastSent[slot]));
            mspSendSetText(MSP2TEXT_CUSTOM_MSG_0 + slot, buf);
        }
    }
}
