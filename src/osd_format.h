// ============================================================================
// osd_format.h — Pure text formatter for the OSD custom-message elements
// ============================================================================
// No I/O and no globals: takes a snapshot of camera/FC state plus one slot's
// layout and produces the exact string that goes out over MSP2_SET_TEXT.
// Kept separate from osd_slots.cpp so it can be unit-tested on the host
// (tools/osd_host_test) and so the browser previews have ONE reference to
// mirror. If you change any text here, change osdRender*() in web_assets.h
// and docs/app.js to match — the host test compares them against this.
// ============================================================================

#ifndef OSD_FORMAT_H
#define OSD_FORMAT_H

#include <stdint.h>
#include <stddef.h>
#include "camera_common.h"
#include "settings.h"

/// Everything the element formatter needs, gathered once per update.
struct OsdContext {
    BleConnectionState     link;       // camGetState()
    bool                   camReady;   // camIsReady()
    const CameraTelemetry *tel;        // camGetTelemetry()
    bool                   fcAlive;
    bool                   armed;
    uint16_t               vbat10;     // FC battery, 0.1 V units (0 = unknown)
};

/// Text for one element. `hideCamData` = the slot already shows why the
/// camera is down (it contains CAM_STATE or LINK), so camera-data elements
/// render "" instead of a placeholder while the camera isn't ready.
void osdFormatElement(uint8_t element, const OsdContext &ctx, bool hideCamData,
                      char *out, size_t outLen);

/// Full text of one custom-message slot: label + elements, single-space
/// separated, whole elements only, max OSD_MAX_TEXT_LEN characters.
void osdFormatSlot(const OsdSlotConfig &slot, const OsdContext &ctx,
                   char *out, size_t outLen);

/// Bitmask (bit n = OsdElement n) of elements the given camera backend can
/// actually fill. The UIs use it to flag elements that will only ever show
/// a placeholder on the selected camera.
uint32_t osdElementSupportMask(uint8_t cameraType);

/// Human-readable duration for goggle glances — "45S" / "12M" / "2H" / "2H33M".
void osdFormatHuman(uint32_t seconds, char *out, size_t len);

#endif // OSD_FORMAT_H
