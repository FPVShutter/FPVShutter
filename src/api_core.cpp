// ============================================================================
// api_core.cpp — Shared configuration-surface logic (HTTP + Serial transports)
// ============================================================================
// Both the Wi-Fi REST API (web_server.cpp) and the Web Serial bench protocol
// (serial_config.cpp) expose the exact same configuration surface. This file
// holds that logic ONCE so the two transports can never drift out of sync.
// Every function here is transport-agnostic — plain strings/buffers in,
// no WebServer or Serial calls inside it.
//
// PROTOTYPE: camera-type validity checks generalized from the old 2-way
// (CAMERA_DJI / CAMERA_GOPRO) to the 3-way CameraType enum, and lastError
// lookup now goes through camera_manager's new camGetLastError() instead of
// calling each backend's own djiGetLastError()/gpGetLastError() directly —
// this file no longer needs to include any individual backend header at
// all, DJI or GoPro. This also means DJI Osmo Action is already selectable
// via {"camera":2} (bench console / REST API) even before the Web UI grows
// a third brand pill — see PROTOTYPE_NOTES.md.
//
// PROTOTYPE: two new settings keys, both aimed at reducing 2.4GHz
// contention/interference with a co-located ELRS receiver:
//   • "wifiApEnabled" (bool) — master Wi-Fi AP switch. Hybrid with the
//     existing optional "wifiSwitch" AUX channel: that AUX channel still
//     works exactly as it always did as an in-field on/off toggle, but
//     only while this master switch is true. Turning it off is a hard
//     override — the AP cannot be brought back up by the AUX channel while
//     the master is off. Applying the change live (starting/stopping the
//     AP itself) is deferred to the caller via apNeedsRestart/apShouldStop,
//     the same pattern already used for ssid/pass changes, so the "ok"
//     response has a chance to go out over the AP before it's torn down.
//   • "blePower" (0/1/2 = Low/Medium/High, BlePowerLevel) — applied
//     immediately via camSetBlePower(), no reconnect needed.
// ============================================================================

#include "api_core.h"
#include "config.h"
#include "settings.h"
#include "camera_manager.h"
#include "fc_status.h"
#include "recorder.h"
#include "osd_slots.h"
#include "osd_format.h"
#include "cam_registry.h"
#include "scan_results.h"
#include "json_scan.h"
#include "web_server.h"   // sanitizeDeviceName(), webApIp(), webIsUp(), webInit()
#include <WiFi.h>

// ──────────────────────────────────────────────────────────────────────────────
// Status
// ──────────────────────────────────────────────────────────────────────────────

static bool cameraTypeValid(long t) {
    return t == CAMERA_DJI_NANO || t == CAMERA_GOPRO || t == CAMERA_DJI_ACTION ||
           t == CAMERA_DJI_RSDK;
}

size_t apiBuildStatusJson(char *buf, size_t bufLen) {
    const CameraTelemetry &tel = camGetTelemetry();
    const FcTelemetry &fc = fcGetTelemetry();
    const ShutterSettings &cfg = settingsGet();

    BleConnectionState st = camGetState();
    static const char *kStateNames[] =
        {"OFF", "SCANNING", "CONNECTING", "PAIRING", "CONNECTED"};

    int batt = tel.batteryPercent <= 100 ? tel.batteryPercent : -1;

    char cams[512];
    {
        size_t o = 0;
        o += snprintf(cams + o, sizeof(cams) - o, "[");
        bool online = camIsReady();
        for (uint8_t i = 0; i < cfg.camCount; i++) {
            const SavedCamera &c = cfg.cams[i];
            const char *nm = c.name[0] ? c.name : c.mac;
            char safeName[sizeof(c.name)];
            strlcpy(safeName, nm, sizeof(safeName));
            for (char *q = safeName; *q; q++) {
                if (*q == '"' || *q == '\\') { memmove(q + 1, q, strlen(q)); *q = ' '; q++; }
            }
            o += snprintf(cams + o, sizeof(cams) - o,
                          "%s{\"t\":%u,\"n\":\"%s\",\"m\":\"%s\",\"a\":%s,\"on\":%s}",
                          i ? "," : "", c.type, safeName, c.mac,
                          c.active ? "true" : "false",
                          (c.active && online) ? "true" : "false");
            if (o >= sizeof(cams) - 8) break;
        }
        o += snprintf(cams + o, sizeof(cams) - o, "]");
    }

    char pending[640] = "[]";
    {
        ScanResult const *sorted[MAX_SCAN_RESULTS];
        uint8_t n = scanResultsGetSortedByRssi(sorted, MAX_SCAN_RESULTS);
        size_t o = 0;
        o += snprintf(pending + o, sizeof(pending) - o, "[");
        for (uint8_t i = 0; i < n; i++) {
            const char *typeStr = cameraTypeName((CameraType)sorted[i]->type);
            char safeName[sizeof(sorted[i]->name)];
            strlcpy(safeName, sorted[i]->name, sizeof(safeName));
            for (char *q = safeName; *q; q++) {
                if (*q == '"' || *q == '\\') { memmove(q + 1, q, strlen(q)); *q = ' '; q++; }
            }
            // "ty" = numeric CameraType. Front-ends must use this (not the
            // display string "t") as the Pair & Save type — a string match
            // on "GoPro" can't tell a Nano from an Action.
            char entry[128];
            int el = snprintf(entry, sizeof(entry),
                "%s{\"mac\":\"%s\",\"n\":\"%s\",\"t\":\"%s\",\"ty\":%u,\"r\":%d}",
                i ? "," : "", sorted[i]->mac, safeName, typeStr,
                (unsigned)sorted[i]->type, sorted[i]->rssi);
            // Append whole entries only, always leaving room for the closing
            // ']' — a truncated entry would make the entire status JSON
            // unparseable in the browser.
            if (el <= 0 || (size_t)el >= sizeof(entry) ||
                o + (size_t)el + 2 > sizeof(pending)) break;
            memcpy(pending + o, entry, (size_t)el);
            o += (size_t)el;
            pending[o] = '\0';
        }
        if (o < sizeof(pending) - 1) snprintf(pending + o, sizeof(pending) - o, "]");
    }

    char safeErr[64] = "";
    {
        const char *lastErr = camGetLastError();
        size_t i = 0;
        for (; lastErr[i] && i < sizeof(safeErr) - 1; i++) {
            char ch = lastErr[i];
            safeErr[i] = (ch == '"' || ch == '\\') ? ' ' : ch;
        }
        safeErr[i] = '\0';
    }

    // OSD layout: [{"e":[1,3,2,0],"l":"CAM"},...]. Labels are sanitized on
    // the way in (osdSanitizeLabel: no quotes/backslashes), so no escaping.
    char osdCfg[4 * (24 + OSD_LABEL_MAX_LEN) + 4];
    {
        size_t o = 0;
        o += snprintf(osdCfg + o, sizeof(osdCfg) - o, "[");
        for (uint8_t i = 0; i < 4 && o < sizeof(osdCfg); i++) {
            const OsdSlotConfig &sc = cfg.osd[i];
            o += snprintf(osdCfg + o, sizeof(osdCfg) - o, "%s{\"e\":[%u,%u,%u,%u],\"l\":\"%s\"}",
                          i ? "," : "", sc.elem[0], sc.elem[1], sc.elem[2], sc.elem[3], sc.label);
        }
        if (o < sizeof(osdCfg)) snprintf(osdCfg + o, sizeof(osdCfg) - o, "]");
    }

    // Optional video-format telemetry ("" / -1 / 0 = not reported).
    long freeMb = tel.freeMb == CAM_FREE_MB_UNKNOWN ? -1L : (long)tel.freeMb;

    uint32_t heap = ESP.getFreeHeap();

    size_t w = snprintf(buf, bufLen,
        "{\"heap\":%u,"
        "\"cam\":{\"type\":%d,\"name\":\"%s\",\"state\":%d,"
        "\"stateName\":\"%s\",\"batt\":%d,\"recTime\":%u,\"valid\":%s,\"recording\":%s,\"ready\":%s,\"cst\":%u,"
        "\"model\":\"%s\",\"mode\":\"%s\",\"res\":\"%s\",\"ar\":\"%s\",\"fps\":%u,"
        "\"eis\":\"%s\",\"temp\":%u,\"freeMb\":%ld},"
        "\"rec\":{\"desired\":%s,\"switchOn\":%s,\"roa\":%s,\"sod\":%s,\"sodDelay\":%u,\"rcValue\":%u,"
        "\"auxCh\":%u,\"thr\":%u,\"deb\":%u},"
        "\"osdCfg\":%s,\"osdSup\":%lu,"
        "\"osd\":[\"%s\",\"%s\",\"%s\",\"%s\"],"
        "\"wifiSwitch\":%d,\"wifiOn\":%s,\"wifiApEnabled\":%s,"
        "\"blePower\":%d,\"blePowerName\":\"%s\",\"scanAll\":%s,"
        "\"lastError\":\"%s\","
        "\"cams\":%s,\"pending_cams\":%s,\"scanning\":%s,"
        "\"fc\":{\"alive\":%s,\"armed\":%s,\"vbat10\":%u,\"rssi\":%u,"
        "\"cycle\":%u,\"api\":\"%s\",\"fw\":\"%s\",\"board\":\"%s\"},"
        "\"sys\":{\"heap\":%u,\"uptime\":%lu,\"ip\":\"%s\",\"sta\":%d,\"version\":\"%s\"}}",
        heap,
        (int)cfg.camera, camGetName(), (int)st, kStateNames[st], batt,
        tel.recTimeSeconds, tel.dataValid ? "true" : "false",
        // The CAMERA's own recording state (every backend sets it), as
        // opposed to rec.desired (what we asked for). recTime means elapsed
        // while this is true and remaining while false, so the UIs format
        // it from this rather than from rec.desired.
        tel.state == CAM_STATE_RECORDING ? "true" : "false",
        camIsReady() ? "true" : "false", (unsigned)tel.state,
        tel.model, tel.modeLabel, tel.resLabel, tel.aspectLabel, tel.fps,
        tel.eisLabel, tel.tempState, freeMb,
        recorderDesiredRecording() ? "true" : "false",
        recorderSwitchOn() ? "true" : "false",
        cfg.recordOnArm ? "true" : "false",
        cfg.stopOnDisarm ? "true" : "false",
        cfg.stopOnDisarmDelayMs,
        recorderLastRcValue(),
        cfg.auxChannelIndex, cfg.rcThresholdUs, cfg.debounceMs,
        osdCfg, (unsigned long)osdElementSupportMask(cfg.camera),
        osdSlotText(0), osdSlotText(1), osdSlotText(2), osdSlotText(3),
        (cfg.wifiSwitchCh <= 15) ? (int)cfg.wifiSwitchCh : -1,
        webIsUp() ? "true" : "false",
        cfg.wifiApEnabled ? "true" : "false",
        (int)cfg.blePower, blePowerName(cfg.blePower),
        cfg.scanAll ? "true" : "false",
        safeErr,
        cams, pending,
        scanResultsIsScanning() ? "true" : "false",
        fc.fcAlive ? "true" : "false", fc.armed ? "true" : "false",
        fc.vbat10, fc.rssi / 10, fc.cycleTimeUs,
        fcApiVersion(), fcFirmwareVersion(), fcBoardName(),
        heap, millis() / 1000UL, webApIp(),
        WiFi.softAPgetStationNum(),
        FIRMWARE_VERSION);
    return w;
}

// ──────────────────────────────────────────────────────────────────────────────
// Settings
// ──────────────────────────────────────────────────────────────────────────────

bool apiApplySettings(const String &body, bool &apNeedsRestart, bool &apShouldStop,
                       char *errBuf, size_t errBufLen) {
    apNeedsRestart = false;
    apShouldStop = false;

    if (!jsonHas(body, "camera") && !jsonHas(body, "auxChannel") &&
        !jsonHas(body, "recordOnArm") && !jsonHas(body, "ssid") &&
        !jsonHas(body, "slot0") && !jsonHas(body, "slot1") &&
        !jsonHas(body, "slot2") && !jsonHas(body, "slot3") &&
        !jsonHas(body, "osd0") && !jsonHas(body, "osd1") &&
        !jsonHas(body, "osd2") && !jsonHas(body, "osd3") &&
        !jsonHas(body, "lbl0") && !jsonHas(body, "lbl1") &&
        !jsonHas(body, "lbl2") && !jsonHas(body, "lbl3") &&
        !jsonHas(body, "wifiSwitch") && !jsonHas(body, "wifiApEnabled") &&
        !jsonHas(body, "blePower") && !jsonHas(body, "scanAll") &&
        !jsonHas(body, "threshold") && !jsonHas(body, "debounce") &&
        !jsonHas(body, "stopOnDisarm") && !jsonHas(body, "stopOnDisarmDelay")) {
        if (errBuf) strlcpy(errBuf, "no recognized keys", errBufLen);
        return false;
    }

    ShutterSettings &cfg = settingsGet();

    if (jsonHas(body, "camera")) {
        long cam = jsonGetNum(body, "camera");
        if (!cameraTypeValid(cam)) {
            if (errBuf) strlcpy(errBuf, "invalid camera type", errBufLen);
            return false;
        }
        if ((CameraType)cam != cfg.camera) {
            camSetCamera((CameraType)cam);
        }
    }

    if (jsonHas(body, "auxChannel")) {
        long ch = jsonGetNum(body, "auxChannel");
        if (ch < 0 || ch > 15) { if (errBuf) strlcpy(errBuf, "channel out of range", errBufLen); return false; }
        cfg.auxChannelIndex = (uint8_t)ch;
    }
    if (jsonHas(body, "threshold")) {
        long t = jsonGetNum(body, "threshold");
        if (t < 1200 || t > 1800) { if (errBuf) strlcpy(errBuf, "threshold out of range", errBufLen); return false; }
        cfg.rcThresholdUs = (uint16_t)t;
    }
    if (jsonHas(body, "debounce")) {
        long d = jsonGetNum(body, "debounce");
        if (d < 50 || d > 1000) { if (errBuf) strlcpy(errBuf, "debounce out of range", errBufLen); return false; }
        cfg.debounceMs = (uint16_t)d;
    }

    if (jsonHas(body, "recordOnArm"))
        cfg.recordOnArm = jsonGetBool(body, "recordOnArm");
    if (jsonHas(body, "stopOnDisarm"))
        cfg.stopOnDisarm = jsonGetBool(body, "stopOnDisarm");
    if (jsonHas(body, "stopOnDisarmDelay")) {
        long d = jsonGetNum(body, "stopOnDisarmDelay");
        if (d < 0 || d > 15000) { if (errBuf) strlcpy(errBuf, "stop-on-disarm delay out of range", errBufLen); return false; }
        cfg.stopOnDisarmDelayMs = (uint16_t)d;
    }

    if (jsonHas(body, "scanAll"))
        cfg.scanAll = jsonGetBool(body, "scanAll");

    if (jsonHas(body, "wifiSwitch")) {
        long ch = jsonGetNum(body, "wifiSwitch");
        if ((ch >= 0 && ch <= 15) || ch == -1) {
            cfg.wifiSwitchCh = (ch == -1) ? 255 : (uint8_t)ch;
        } else {
            if (errBuf) strlcpy(errBuf, "wifi switch channel out of range", errBufLen);
            return false;
        }
    }

    // Master Wi-Fi AP switch. Only the transition matters — applying the
    // radio change itself is deferred to the caller (web_server.cpp /
    // serial_config.cpp) via apNeedsRestart/apShouldStop, so the response
    // to THIS request can be sent before the AP potentially goes away.
    if (jsonHas(body, "wifiApEnabled")) {
        bool newVal = jsonGetBool(body, "wifiApEnabled");
        if (newVal != cfg.wifiApEnabled) {
            cfg.wifiApEnabled = newVal;
            if (newVal) {
                apNeedsRestart = true;   // AP was off — bring it up
            } else {
                apShouldStop = true;     // AP was allowed — tear it down
            }
        }
    }

    if (jsonHas(body, "blePower")) {
        long p = jsonGetNum(body, "blePower");
        if (p < BLE_POWER_LOW || p > BLE_POWER_HIGH) {
            if (errBuf) strlcpy(errBuf, "invalid BLE power level", errBufLen);
            return false;
        }
        cfg.blePower = (BlePowerLevel)p;
        camSetBlePower(cfg.blePower);   // Applies live immediately, no reconnect
    }

    // OSD layout. Validate every slot into a copy first, so a bad value
    // leaves the live layout untouched.
    //   "osdN": "e1,e2,e3,e4"  OsdElement ids, up to OSD_ELEMS_PER_SLOT
    //   "lblN": "CAM"          optional prefix label (sanitized, max 6)
    //   "slotN": 0..6          legacy single-choice preset (older UIs)
    OsdSlotConfig newOsd[4];
    memcpy(newOsd, cfg.osd, sizeof(newOsd));
    for (uint8_t s = 0; s < 4; s++) {
        char key[8];
        snprintf(key, sizeof(key), "slot%d", s);
        if (jsonHas(body, key)) {
            long v = jsonGetNum(body, key);
            if (v < 0 || v >= OSD_SLOT_COUNT) {
                if (errBuf) strlcpy(errBuf, "invalid slot content", errBufLen);
                return false;
            }
            osdSlotFromLegacy((uint8_t)v, newOsd[s]);
        }
        snprintf(key, sizeof(key), "osd%d", s);
        if (jsonHas(body, key)) {
            String list = jsonGetStr(body, key);
            uint8_t el[OSD_ELEMS_PER_SLOT] = {0};
            int n = 0;
            long cur = -1;
            for (unsigned i = 0; i <= list.length(); i++) {
                char c = i < list.length() ? list[i] : ',';
                if (c >= '0' && c <= '9') {
                    cur = (cur < 0 ? 0 : cur * 10) + (c - '0');
                    if (cur > 255) cur = 255;
                } else if (c == ',') {
                    if (cur >= 0) {
                        if (cur >= OSD_EL_COUNT || n >= OSD_ELEMS_PER_SLOT) {
                            if (errBuf) strlcpy(errBuf, cur >= OSD_EL_COUNT ? "invalid OSD element"
                                                                          : "too many OSD elements", errBufLen);
                            return false;
                        }
                        el[n++] = (uint8_t)cur;
                    }
                    cur = -1;
                } else if (c != ' ') {
                    if (errBuf) strlcpy(errBuf, "bad OSD element list", errBufLen);
                    return false;
                }
            }
            memcpy(newOsd[s].elem, el, sizeof(el));
        }
        snprintf(key, sizeof(key), "lbl%d", s);
        if (jsonHas(body, key)) {
            String lbl = jsonGetStr(body, key);
            char tmp[33];
            strlcpy(tmp, lbl.c_str(), sizeof(tmp));
            osdSanitizeLabel(tmp);
            strlcpy(newOsd[s].label, tmp, sizeof(newOsd[s].label));
        }
    }
    memcpy(cfg.osd, newOsd, sizeof(cfg.osd));

    if (jsonHas(body, "ssid")) {
        String ssid = jsonGetStr(body, "ssid");
        ssid.trim();
        if (ssid.length() < 1 || ssid.length() > 32) {
            if (errBuf) strlcpy(errBuf, "SSID must be 1-32 chars", errBufLen);
            return false;
        }
        strlcpy(cfg.apSsid, ssid.c_str(), sizeof(cfg.apSsid));
        apNeedsRestart = true;
    }
    if (jsonHas(body, "pass")) {
        String pass = jsonGetStr(body, "pass");
        pass.trim();
        if (pass.length() > 0 && pass.length() < 8) {
            if (errBuf) strlcpy(errBuf, "password must be empty or 8-64 chars", errBufLen);
            return false;
        }
        strlcpy(cfg.apPass, pass.c_str(), sizeof(cfg.apPass));
        apNeedsRestart = true;
    }

    settingsSave();
    DBG("API: settings saved");
    return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// Camera registry
// ──────────────────────────────────────────────────────────────────────────────

bool apiApplyCamera(const String &body, bool &alreadyScanning,
                     char *errBuf, size_t errBufLen) {
    alreadyScanning = false;

    if (jsonHas(body, "scan")) {
        if (scanResultsIsScanning()) { alreadyScanning = true; return true; }
        camStartUserScan();
        return true;

    } else if (jsonHas(body, "select")) {
        long idx = jsonGetNum(body, "select", -1);
        if (idx < 0 || !camRegistrySelect((uint8_t)idx)) {
            if (errBuf) strlcpy(errBuf, "invalid camera index", errBufLen);
            return false;
        }
        camKick();
        return true;

    } else if (jsonHas(body, "remove")) {
        long idx = jsonGetNum(body, "remove", -1);
        if (idx < 0 || !camRegistryRemove((uint8_t)idx)) {
            if (errBuf) strlcpy(errBuf, "invalid camera index", errBufLen);
            return false;
        }
        camDisconnect();
        return true;

    } else if (jsonHas(body, "pair")) {
        String mac = jsonGetStr(body, "mac");
        long type = jsonGetNum(body, "type", -1);

        if (mac.length() != 17) {
            if (errBuf) strlcpy(errBuf, "pair: invalid MAC length", errBufLen);
            return false;
        }
        for (int i = 0; i < 17; i++) {
            char c = mac.charAt(i);
            if (i % 3 == 2) {
                if (c != ':') { if (errBuf) strlcpy(errBuf, "pair: invalid MAC format", errBufLen); return false; }
            } else {
                if (!isHexadecimalDigit(c)) { if (errBuf) strlcpy(errBuf, "pair: invalid MAC character", errBufLen); return false; }
            }
        }

        if (!cameraTypeValid(type)) {
            if (errBuf) strlcpy(errBuf, "pair: mac and type required", errBufLen);
            return false;
        }

        ScanResult const *sorted[MAX_SCAN_RESULTS];
        uint8_t scount = scanResultsGetSortedByRssi(sorted, MAX_SCAN_RESULTS);
        bool isOnline = false;
        char nameBuf[24] = "";
        for (uint8_t i = 0; i < scount; i++) {
            if (strcasecmp(sorted[i]->mac, mac.c_str()) == 0 &&
                sorted[i]->type == (uint8_t)type) {
                isOnline = true;
                sanitizeDeviceName(nameBuf, sorted[i]->name, sizeof(nameBuf));
                break;
            }
        }
        if (!isOnline) {
            if (errBuf) strlcpy(errBuf, "Cannot pair: device is offline or not in range", errBufLen);
            return false;
        }
        if (!camRegistrySave((uint8_t)type, mac.c_str(),
                              nameBuf[0] ? nameBuf : mac.c_str())) {
            if (errBuf) strlcpy(errBuf, "pair: registry full or invalid", errBufLen);
            return false;
        }
        for (uint8_t i = 0; i < settingsGet().camCount; i++) {
            if (strcasecmp(settingsGet().cams[i].mac, mac.c_str()) == 0 &&
                settingsGet().cams[i].type == (uint8_t)type) {
                camRegistrySelect(i);
                camKick();
                break;
            }
        }
        return true;
    }

    if (errBuf) strlcpy(errBuf, "expected scan, select, remove, or pair", errBufLen);
    return false;
}

// ──────────────────────────────────────────────────────────────────────────────
// Start / stop / reboot
// ──────────────────────────────────────────────────────────────────────────────

bool apiApplyCommand(const String &cmd, bool &shouldReboot,
                      char *errBuf, size_t errBufLen) {
    shouldReboot = false;
    if (cmd == "start") {
        recorderManualStart();
        return true;
    } else if (cmd == "stop") {
        recorderManualStop();
        return true;
    } else if (cmd == "reboot") {
        shouldReboot = true;
        return true;
    }
    if (errBuf) strlcpy(errBuf, "unknown command", errBufLen);
    return false;
}