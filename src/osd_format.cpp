// ============================================================================
// osd_format.cpp — Pure text formatter for the OSD custom-message elements
// ============================================================================
// All text is UPPERCASE on purpose: Betaflight/analog OSD fonts repurpose the
// lowercase ASCII tiles (0x60-0x7F) for heading/compass and unit icons, so a
// lowercase "h"/"m"/"s" renders as the wrong icon on real hardware. The
// Bench Console's live preview draws with the real font to show exactly this.
// ============================================================================

#include "osd_format.h"
#include <stdio.h>
#include <string.h>

void osdFormatHuman(uint32_t seconds, char *out, size_t len) {
    if (seconds < 60) { snprintf(out, len, "%uS", (unsigned)seconds); return; }
    unsigned h = seconds / 3600;
    unsigned m = (seconds % 3600) / 60;
    if (h > 0) {
        if (m > 0) snprintf(out, len, "%uH%uM", h, m);
        else       snprintf(out, len, "%uH", h);
    } else {
        snprintf(out, len, "%uM", m);
    }
}

// ── Settings helpers (declared in settings.h; here so they stay host-testable)
void osdSlotFromLegacy(uint8_t legacy, OsdSlotConfig &out) {
    memset(&out, 0, sizeof(out));
    switch (legacy) {
        case OSD_SLOT_CAM_STATUS:
            out.elem[0] = OSD_EL_CAM_STATE;
            out.elem[1] = OSD_EL_CAM_BATT;
            out.elem[2] = OSD_EL_REC_TIME;
            break;
        case OSD_SLOT_REC_TIME:
            strlcpy(out.label, "REC", sizeof(out.label));
            out.elem[0] = OSD_EL_REC_TIME;
            break;
        case OSD_SLOT_BATTERY:
            strlcpy(out.label, "BAT", sizeof(out.label));
            out.elem[0] = OSD_EL_CAM_BATT;
            break;
        case OSD_SLOT_LINK:
            strlcpy(out.label, "LINK", sizeof(out.label));
            out.elem[0] = OSD_EL_LINK;
            break;
        case OSD_SLOT_FC_BATT:
            strlcpy(out.label, "FC", sizeof(out.label));
            out.elem[0] = OSD_EL_FC_VOLT;
            break;
        case OSD_SLOT_ARM_STATE:
            out.elem[0] = OSD_EL_ARM;
            break;
        case OSD_SLOT_OFF:
        default:
            break;
    }
}

void osdSanitizeLabel(char *label) {
    size_t o = 0;
    for (size_t i = 0; label[i] && o < OSD_LABEL_MAX_LEN; i++) {
        char c = label[i];
        if (c >= 'a' && c <= 'z') c -= 32;           // OSD fonts: uppercase only
        if (c < 0x20 || c > 0x5F || c == '"' || c == '\\') continue;
        label[o++] = c;
    }
    label[o] = '\0';
    // No leading/trailing spaces — the joiner adds its own separator.
    while (o > 0 && label[o - 1] == ' ') label[--o] = '\0';
    size_t lead = 0;
    while (label[lead] == ' ') lead++;
    if (lead) memmove(label, label + lead, o - lead + 1);
}

static bool isCameraDataElement(uint8_t el) {
    switch (el) {
        case OSD_EL_REC_TIME: case OSD_EL_CAM_BATT: case OSD_EL_MODE:
        case OSD_EL_RES:      case OSD_EL_ASPECT:   case OSD_EL_FPS:
        case OSD_EL_FORMAT:   case OSD_EL_EIS:      case OSD_EL_SD_FREE:
        case OSD_EL_TEMP:
            return true;
        default:
            return false;
    }
}

static void put(char *out, size_t len, const char *s) {
    snprintf(out, len, "%s", s);
}

void osdFormatElement(uint8_t el, const OsdContext &ctx, bool hideCamData,
                      char *out, size_t len) {
    if (!len) return;
    out[0] = '\0';
    const CameraTelemetry &tel = *ctx.tel;

    if (!ctx.camReady && hideCamData && isCameraDataElement(el)) return;

    switch (el) {
        case OSD_EL_CAM_STATE:
            switch (ctx.link) {
                case BLE_DISCONNECTED:   put(out, len, "CAM OFF");  return;
                case BLE_SCANNING:       put(out, len, "CAM SCAN"); return;
                case BLE_CONNECTING:     put(out, len, "CAM CONN"); return;
                case BLE_AUTHENTICATING: put(out, len, "CAM PAIR"); return;
                default: break;
            }
            if (!ctx.camReady)                         put(out, len, "CAM PAIR");
            else if (tel.state == CAM_STATE_RECORDING) put(out, len, "REC");
            else if (tel.state == CAM_STATE_STANDBY)   put(out, len, "STBY");
            else if (tel.state == CAM_STATE_ERROR)     put(out, len, "CAM ERR");
            else                                       put(out, len, "CAM ???");
            return;

        case OSD_EL_REC_TIME: {
            if (!ctx.camReady || !tel.dataValid) { put(out, len, "--:--"); return; }
            uint16_t t = tel.recTimeSeconds;
            if (tel.state == CAM_STATE_RECORDING) {
                snprintf(out, len, "%02u:%02u", t / 60, t % 60);   // elapsed
            } else {
                osdFormatHuman(t, out, len);                       // remaining
            }
            return;
        }

        case OSD_EL_CAM_BATT:
            if (ctx.camReady && tel.batteryPercent <= 100) snprintf(out, len, "%u%%", tel.batteryPercent);
            else                                           put(out, len, "--%");
            return;

        case OSD_EL_LINK:
            switch (ctx.link) {
                case BLE_CONNECTED:      put(out, len, "READY"); return;
                case BLE_AUTHENTICATING: put(out, len, "PAIR");  return;
                case BLE_CONNECTING:     put(out, len, "CONN");  return;
                case BLE_SCANNING:       put(out, len, "SCAN");  return;
                default:                 put(out, len, "OFF");   return;
            }

        case OSD_EL_FC_VOLT:
            if (ctx.vbat10 > 0) snprintf(out, len, "%u.%uV", ctx.vbat10 / 10, ctx.vbat10 % 10);
            else                put(out, len, "--.-V");
            return;

        case OSD_EL_ARM:
            if (!ctx.fcAlive)   put(out, len, "FC NOLINK");
            else if (ctx.armed) put(out, len, "ARMED");
            else                put(out, len, "DISARMED");
            return;

        case OSD_EL_MODE:
            put(out, len, (ctx.camReady && tel.modeLabel[0]) ? tel.modeLabel : "--");
            return;

        case OSD_EL_RES:
            put(out, len, (ctx.camReady && tel.resLabel[0]) ? tel.resLabel : "--");
            return;

        case OSD_EL_ASPECT:
            put(out, len, (ctx.camReady && tel.aspectLabel[0]) ? tel.aspectLabel : "--");
            return;

        case OSD_EL_FPS:
            if (ctx.camReady && tel.fps) snprintf(out, len, "%uFPS", tel.fps);
            else                         put(out, len, "--FPS");
            return;

        case OSD_EL_FORMAT:
            if (ctx.camReady && tel.resLabel[0] && tel.fps) snprintf(out, len, "%s%u", tel.resLabel, tel.fps);
            else                                            put(out, len, "--");
            return;

        case OSD_EL_EIS:
            if (!ctx.camReady || !tel.eisLabel[0]) put(out, len, "--");
            else if (strcmp(tel.eisLabel, "OFF") == 0) put(out, len, "EIS OFF");
            else put(out, len, tel.eisLabel);
            return;

        case OSD_EL_SD_FREE: {
            uint32_t mb = tel.freeMb;
            if (!ctx.camReady || mb == CAM_FREE_MB_UNKNOWN) { put(out, len, "--"); return; }
            if (mb < 1024) {
                snprintf(out, len, "%uM", (unsigned)mb);
            } else if (mb < 10 * 1024) {
                uint32_t tenths = (mb * 10 + 512) / 1024;          // rounded 0.1 GB
                snprintf(out, len, "%u.%uG", (unsigned)(tenths / 10), (unsigned)(tenths % 10));
            } else {
                snprintf(out, len, "%uG", (unsigned)((mb + 512) / 1024));
            }
            return;
        }

        case OSD_EL_TEMP:
            // Alert-only: nothing at all while the camera runs normally.
            if (!ctx.camReady) return;
            if (tel.tempState == 1)      put(out, len, "WARM");
            else if (tel.tempState == 2) put(out, len, "HOT");
            else if (tel.tempState == 3) put(out, len, "OVERHEAT");
            return;

        case OSD_EL_NONE:
        default:
            return;
    }
}

void osdFormatSlot(const OsdSlotConfig &slot, const OsdContext &ctx,
                   char *out, size_t outLen) {
    if (!outLen) return;
    out[0] = '\0';
    const size_t maxLen = (outLen - 1 < OSD_MAX_TEXT_LEN) ? outLen - 1 : OSD_MAX_TEXT_LEN;

    bool hideCamData = false;
    for (int i = 0; i < OSD_ELEMS_PER_SLOT; i++) {
        if (slot.elem[i] == OSD_EL_CAM_STATE || slot.elem[i] == OSD_EL_LINK) hideCamData = true;
    }

    size_t used = 0;
    auto append = [&](const char *part) -> bool {
        size_t n = strlen(part);
        if (!n) return true;                        // empty part: skip, keep going
        size_t need = n + (used ? 1 : 0);
        if (used + need > maxLen) return false;     // whole elements only
        if (used) out[used++] = ' ';
        memcpy(out + used, part, n);
        used += n;
        out[used] = '\0';
        return true;
    };

    if (!append(slot.label)) return;
    for (int i = 0; i < OSD_ELEMS_PER_SLOT; i++) {
        if (slot.elem[i] == OSD_EL_NONE || slot.elem[i] >= OSD_EL_COUNT) continue;
        char part[OSD_MAX_TEXT_LEN + 1];
        osdFormatElement(slot.elem[i], ctx, hideCamData, part, sizeof(part));
        if (!append(part)) break;                   // drop this and everything after
    }
}

uint32_t osdElementSupportMask(uint8_t cameraType) {
    uint32_t m = (1u << OSD_EL_CAM_STATE) | (1u << OSD_EL_REC_TIME) | (1u << OSD_EL_CAM_BATT) |
                 (1u << OSD_EL_LINK) | (1u << OSD_EL_FC_VOLT) | (1u << OSD_EL_ARM);
    switch (cameraType) {
        case CAMERA_DJI_NANO:
            m |= (1u << OSD_EL_MODE) |                         // 02/80 work mode
                 (1u << OSD_EL_RES) | (1u << OSD_EL_ASPECT) |  // 02/19 video format
                 (1u << OSD_EL_FPS) | (1u << OSD_EL_FORMAT);
            break;
        case CAMERA_DJI_ACTION:
            m |= (1u << OSD_EL_SD_FREE);                       // 02/71 free MB
            break;
        case CAMERA_DJI_RSDK:
            m |= (1u << OSD_EL_MODE) | (1u << OSD_EL_RES) | (1u << OSD_EL_ASPECT) |
                 (1u << OSD_EL_FPS) | (1u << OSD_EL_FORMAT) | (1u << OSD_EL_EIS) |
                 (1u << OSD_EL_SD_FREE) | (1u << OSD_EL_TEMP);  // 1D02 status push
            break;
        default:
            break;
    }
    return m;
}
