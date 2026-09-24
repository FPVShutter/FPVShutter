// ============================================================================
// dji_action_camera.cpp — DUML-over-BLE backend for DJI Osmo Action cameras
// ============================================================================
// Target hardware: DJI Osmo Action 2 (the first Action-line camera this
// fork has on the bench). Built on the shared dji_duml_transport module,
// so transport-level behaviour (GATT connect, pairing arm + 07/45 PIN,
// keep-alive, staleness watchdog, FC-UART-passthrough reconnect guard) is
// identical to the hardware-verified Nano backend.
//
// What's different from the Nano, and why
// ---------------------------------------
// The Nano PUSHES its telemetry unsolicited (02/80 general status, 0D/02
// battery). The Action 2 instead answers explicit queries:
//
//   • 02/70  record-state query   → reply payload [result][state]...
//            state 0x01 idle, 0x41 starting, 0x81 recording, 0xC1 saving
//   • 0D/02  battery query        → reply payload [result][battery struct],
//            battery % at payload[21] (= pData[32]; the unsolicited Nano
//            push has no result byte, which is why it sits at pData[31])
//
//   • 02/71  SD card info query   → reply [result][insert?][total MB u32]
//            [free MB u32][remaining shots u32][remaining rec time s u32],
//            remaining record time at pData[25..28] (standby only). Found
//            on the bench 2026-09-24: the Action 2 neither pushes nor
//            answers 02/80, but 02/71 carries the same four SD fields in
//            the same order as DJI's 02/80 "Camera State Info" struct.
//
// All are sent from djiActionUpdate() once the pairing session is up, and
// their replies also keep the DJI_LINK_STALE_MS watchdog fed.
//
// Provenance of those protocol facts (facts only — no code copied):
//   • RotorREC (github.com/flosean/RotorREC) — reports Action 2 pairing,
//     start/stop, polled recording state, battery and keep-alive as
//     confirmed on real hardware (docs/verification-status.md).
//   • controle-dji-action2 (github.com/FLORIANSV35/controle-dji-action2,
//     MIT) — independent Action 2 ESP32-C3 project using the same 02/70
//     status poll and 02/02 record command.
//   • This repo's own upstream (rover1312/shutterlink, commit 0ca5fa1
//     "working dji action 2 changes") — the pairing handshake and 02/02
//     record opcode the shared transport already uses were developed
//     against an Action 2 there.
//
// The Nano-style 02/80 push parser is kept only as a harmless fallback for
// other Action models: this project's Action 2 never pushed 02/80 and never
// answered a 02/80 query on the bench.
//
// STATUS: HARDWARE-VERIFIED on this project's own (borrowed) Action 2,
// 2026-09-24: pairing ("already paired" fast path, no re-approval after a
// camera power cycle), start/stop, 02/70 record state including the brief
// C1 "saving" state, 0D/02 battery, 02/71 remaining time (740 s matched the
// camera's own 12:20), the 1 Hz 00/2B heartbeat (camera answers 00/00 from
// 0x28), and automatic reconnect after a camera power cycle (~33 s, almost
// all of it camera boot time). Not bench-testable: a start command landing
// inside the ~150-500 ms C1 window (Crash Flip re-arm); stop-on-disarm's
// delay makes that unlikely in practice. Action 3/4/5/6 remain untested.
// ============================================================================

#include "dji_action_camera.h"
#include "dji_duml_transport.h"
#include "cam_registry.h"
#include "scan_results.h"
#include "settings.h"
#include "serial_config.h"
#include <NimBLEDevice.h>

// ──────────────────────────────────────────────────────────────────────────────
// Internal state
// ──────────────────────────────────────────────────────────────────────────────
static DjiDumlSession  _session;
static CameraTelemetry _telemetry;

static bool     _wasRecording  = false;
static uint32_t _recordStartMs = 0;

// Polled-telemetry bookkeeping (see header comment).
static uint32_t _lastStatusPoll    = 0;
static uint32_t _lastBatteryPoll   = 0;
static bool     _polledStatusSeen  = false;  // a 02/70 reply arrived this session
static uint32_t _statusReplies     = 0;
static uint8_t  _lastStatusRaw     = 0xFF;
static uint16_t _remainingSec      = 0;      // standby estimate (02/71 reply, or 02/80 push)
static bool     _remainingKnown    = false;
static uint32_t _lastRemainPoll    = 0;
static uint32_t _lastHeartbeat     = 0;

static const char *MODEL_NAME = "DJI Osmo Action";

// DUML addressing used for the queries. Sender 0x02 (app) matches what the
// shared transport already uses for pairing/record; receivers are the
// camera's own modules as seen in Action 2 captures.
static const uint8_t DUML_SENDER_APP      = 0x02;
static const uint8_t DUML_RX_CAMERA       = 0x01;  // 02/xx camera control/status
static const uint8_t DUML_RX_BATTERY      = 0x05;  // 0D/xx battery
static const uint8_t DUML_FLAGS_REQUEST   = 0x20;  // request, reply wanted
static const uint8_t DUML_FLAG_REPLY_BIT  = 0x80;  // set on camera replies (0xC0)

// ──────────────────────────────────────────────────────────────────────────────
// Forward declarations
// ──────────────────────────────────────────────────────────────────────────────
void        djiActionStartScan();
static void notifyCallback(NimBLERemoteCharacteristic *pChar, uint8_t *pData,
                            size_t length, bool isNotify);
static bool isDjiActionDevice(NimBLEAdvertisedDevice *device);
static void resetSessionTelemetry();

// ──────────────────────────────────────────────────────────────────────────────
// BLE Callbacks
// ──────────────────────────────────────────────────────────────────────────────
class ActionClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient *pClient) override {
        DBG("ACTION: Connected to camera");
    }
    void onDisconnect(NimBLEClient *pClient) {
        DBG("ACTION: Disconnected from camera (%lu status replies this session)",
            (unsigned long)_statusReplies);
        _session.bleState = BLE_DISCONNECTED;
        _session.sessionEstablished = false;
        _session.pControlChar   = nullptr;
        _session.pTelemetryChar = nullptr;
        _session.pAuthChar      = nullptr;
        _telemetry.dataValid = false;
        _telemetry.state     = CAM_STATE_UNKNOWN;
        resetSessionTelemetry();
    }
};
static ActionClientCallbacks _clientCallbacks;

// RTOS-SAFE scan callback — see dji_duml_transport.h's notes on the NimBLE
// host task; same rules as the original file (no NVS/Serial/heavy work in
// here, copy NimBLE's std::string temporaries before use).
class ActionAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice *advertisedDevice) override {
        bool isMatched = isDjiActionDevice(advertisedDevice);
        if (!isMatched && scanResultsIsShowAll()) {
            std::string addr = advertisedDevice->getAddress().toString();
            isMatched = !addr.empty();
        }
        if (!isMatched) return;

        std::string macStr  = advertisedDevice->getAddress().toString();
        std::string nameStr = advertisedDevice->haveName()
                              ? advertisedDevice->getName() : "";
        int8_t rssi = advertisedDevice->getRSSI();

        scanResultsAdd(CAMERA_DJI_ACTION, macStr.c_str(), nameStr.c_str(), rssi, isMatched);
        camRegistryRemember(CAMERA_DJI_ACTION, macStr.c_str(), nameStr.c_str());

        if (camRegistryMayAutoConnect(CAMERA_DJI_ACTION, macStr.c_str())) {
            NimBLEDevice::getScan()->stop();
            _session.targetAddress    = advertisedDevice->getAddress();
            _session.hasTargetAddress = true;
            _session.doConnect        = true;
        }
    }
};
static ActionAdvertisedDeviceCallbacks _scanCallbacks;

// ──────────────────────────────────────────────────────────────────────────────
// Device identification — RENAME-PROOF
// ──────────────────────────────────────────────────────────────────────────────
// Name prefixes: the "rishavhs..." entries are the upstream author's own
// renamed Action 2 (see header). The OUI list is still kept IDENTICAL to
// dji_nano_camera.cpp's on purpose — OUIs usually track a shared radio
// module rather than a consumer model, and discovery also matches on the
// brand-wide 0xFFF0 service / DJI company ID anyway, so narrowing it buys
// nothing but risk. The user picks the camera from the scan list either
// way; which backend drives it is decided by the brand they scanned with.
static const char *DJI_ACTION_NAME_PREFIXES[] = {
    "Osmo Action", "DJI Action", "OSMO ACTION", "DJI ACTION",
    "Action 2", "action2", "Action 4", "Action 5", "OsmoAction",
    "rishavhsAction2", "RishavhsAction2", "RISHAVHSACTION2",
};
static const size_t DJI_ACTION_NAME_PREFIX_COUNT =
    sizeof(DJI_ACTION_NAME_PREFIXES) / sizeof(DJI_ACTION_NAME_PREFIXES[0]);

static const char *DJI_ACTION_OUI_PREFIXES[] = {
    "34:d2:62", "60:60:1f", "ec:9e:ea",
};
static const size_t DJI_ACTION_OUI_COUNT =
    sizeof(DJI_ACTION_OUI_PREFIXES) / sizeof(DJI_ACTION_OUI_PREFIXES[0]);

static bool isDjiActionDevice(NimBLEAdvertisedDevice *device) {
    if (settingsGet().scanAll) {
        std::string addr = device->getAddress().toString();
        return !addr.empty();
    }

    // Signal 1: name prefix match.
    if (device->haveName()) {
        std::string name = device->getName();
        for (size_t i = 0; i < DJI_ACTION_NAME_PREFIX_COUNT; i++) {
            if (name.find(DJI_ACTION_NAME_PREFIXES[i]) != std::string::npos) return true;
        }
    }

    // Signal 2: MAC OUI match.
    std::string addr = device->getAddress().toString();
    for (size_t i = 0; i < DJI_ACTION_OUI_COUNT; i++) {
        if (addr.rfind(DJI_ACTION_OUI_PREFIXES[i], 0) == 0) return true;
    }

    // Signal 3: advertised service UUID 0xFFF0 (shared — see transport module).
    if (device->isAdvertisingService(DJI_DUML_ADV_SERVICE_UUID)) return true;

    // Signal 4: manufacturer data contains DJI / Xtra company id (shared).
    std::string mfr = device->getManufacturerData();
    if (!mfr.empty()) {
        if (dumlBytesContain(mfr, DJI_DUML_COMPANY_ID))   return true;
        if (dumlBytesContain(mfr, DJI_DUML_XTRA_COMPANY)) return true;
    }

    return false;
}

// ──────────────────────────────────────────────────────────────────────────────
// Bench aid: first-seen inbound frame logger
// ──────────────────────────────────────────────────────────────────────────────
#if DJI_ACTION_FRAME_DISCOVERY
static uint32_t _seenFrameKeys[32];
static uint8_t  _seenFrameCount = 0;

/// Logs the first frame of every distinct (flags, set, id, sender) combo
/// with a hex dump. Called from the NimBLE host task — only a small table
/// scan + DBG, same weight as the per-frame logging the original
/// dji_camera.cpp already did in this callback.
static void logFirstSeenFrame(const char *charName, const uint8_t *pData,
                               size_t length, const DumlFrameHeader &hdr) {
    uint32_t key = ((uint32_t)hdr.flags << 24) | ((uint32_t)hdr.cmdSet << 16) |
                   ((uint32_t)hdr.cmdId << 8) | pData[4];
    for (uint8_t i = 0; i < _seenFrameCount; i++) {
        if (_seenFrameKeys[i] == key) return;
    }
    if (_seenFrameCount >= sizeof(_seenFrameKeys) / sizeof(_seenFrameKeys[0])) return;
    _seenFrameKeys[_seenFrameCount++] = key;

    char hexBuf[3 * 48 + 1] = "";
    size_t hexLen = 0;
    for (size_t i = 0; i < length && i < 48; i++) {
        hexLen += snprintf(hexBuf + hexLen, sizeof(hexBuf) - hexLen, "%02X ", pData[i]);
    }
    DBG("ACTION: new frame [%s] %02X->%02X flags=%02X set=%02X id=%02X len=%u: %s%s",
        charName, pData[4], pData[5], hdr.flags, hdr.cmdSet, hdr.cmdId,
        (unsigned)length, hexBuf, length > 48 ? "..." : "");
}
#endif

// ──────────────────────────────────────────────────────────────────────────────
// Telemetry helpers
// ──────────────────────────────────────────────────────────────────────────────
static void resetSessionTelemetry() {
    _polledStatusSeen = false;
    _statusReplies    = 0;
    _lastStatusRaw    = 0xFF;
    _wasRecording     = false;
    _lastStatusPoll   = 0;
    _lastBatteryPoll  = 0;
    _remainingKnown   = false;
    _remainingSec     = 0;
    _lastRemainPoll   = 0;
    _lastHeartbeat    = 0;
}

/// Single place that turns "is the camera recording?" into telemetry, so
/// both the polled 02/70 path and the 02/80 fallback share the elapsed-time
/// edge detection (same semantics as the Nano: elapsed while recording,
/// camera's own remaining estimate while in standby).
static void applyRecordingState(bool isRecording) {
    _telemetry.state = isRecording ? CAM_STATE_RECORDING : CAM_STATE_STANDBY;
    if (isRecording && !_wasRecording) {
        _recordStartMs = millis();
    }
    _wasRecording = isRecording;
    if (isRecording) {
        _telemetry.recTimeSeconds = (uint16_t)((millis() - _recordStartMs) / 1000);
    } else {
        // Standby shows the camera's remaining-time estimate. That only
        // exists once a 02/71 reply (or 02/80 push) has arrived; otherwise report 0 rather
        // than leaving the last ELAPSED value masquerading as "remaining".
        _telemetry.recTimeSeconds = _remainingKnown ? _remainingSec : 0;
    }
    _telemetry.dataValid = true;
}

static void sendQuery(uint8_t receiver, uint8_t cmdSet, uint8_t cmdId,
                      const uint8_t *payload, size_t payloadLen) {
    if (!_session.pControlChar || !_session.pControlChar->canWriteNoResponse()) return;
    uint8_t packet[32];
    size_t len = dumlBuildPacket(packet, DUML_SENDER_APP, receiver,
                                  _session.sequenceCounter++, DUML_FLAGS_REQUEST,
                                  cmdSet, cmdId, payload, payloadLen);
    _session.pControlChar->writeValue(packet, len, false);
}

// ──────────────────────────────────────────────────────────────────────────────
// Notification Callback
// ──────────────────────────────────────────────────────────────────────────────
static void notifyCallback(NimBLERemoteCharacteristic *pChar, uint8_t *pData,
                            size_t length, bool isNotify) {
    const char *charName = (pChar == _session.pAuthChar) ? "FFF4"
                         : (pChar == _session.pTelemetryChar) ? "FFF3" : "FFF5";
    _session.lastRxMs = millis();  // Any inbound traffic proves the link is alive

    DumlFrameHeader hdr;
    if (!dumlParseHeader(pData, length, hdr)) {
        dumlLogNonDuml(charName, pData, length);
        return;
    }

#if DJI_ACTION_FRAME_DISCOVERY
    logFirstSeenFrame(charName, pData, length, hdr);
#endif

    if (dumlHandlePairingFrame(_session, hdr, pData, length)) return;

    const bool isReply = (hdr.flags & DUML_FLAG_REPLY_BIT) != 0;

    // Record-control response (02/02 reply): first payload byte is the
    // camera's reply code (MEDIA_PROTOCOL.md):
    //   00 = OK, d8 = resource busy, d9 = wrong state, df = wrong param,
    //   e3 = bad param, e0 = not supported, silence = receiver missing.
    // An OK only confirms the command was accepted — the actual state
    // change shows up on the next 02/70 poll (~500 ms later).
    if (isReply && hdr.cmdSet == 0x02 && hdr.cmdId == 0x02 && length >= 12) {
        uint8_t reply = pData[11];
        switch (reply) {
            case 0x00: DBG("ACTION: record command OK"); break;
            case 0xd8: DBG("ACTION: record cmd: resource not ready"); break;
            case 0xd9: DBG("ACTION: record cmd: wrong state (already rec?)"); break;
            case 0xdf: DBG("ACTION: record cmd: wrong parameter"); break;
            case 0xe3: DBG("ACTION: record cmd: bad/missing parameter"); break;
            case 0xe0: DBG("ACTION: record cmd: NOT SUPPORTED by camera"); break;
            default:   DBG("ACTION: record cmd reply 0x%02X", reply); break;
        }
        return;
    }

    // Record-state poll reply (02/70). Payload: [result][state][...].
    // Needs at least 6 payload bytes (13 + 6 = 19 total) and result 0x00.
    if (isReply && hdr.cmdSet == 0x02 && hdr.cmdId == 0x70 &&
        length >= 19 && pData[11] == 0x00) {
        uint8_t st = pData[12];
        _statusReplies++;
        if (st != _lastStatusRaw) {
            DBG("ACTION: 02/70 state %02X -> %02X (%s)", _lastStatusRaw, st,
                st == 0x01 ? "idle" : st == 0x41 ? "starting" :
                st == 0x81 ? "recording" : st == 0xC1 ? "saving" : "unknown");
            _lastStatusRaw = st;
        }
        switch (st) {
            case 0x81: _polledStatusSeen = true; applyRecordingState(true);  break;
            case 0x01:                                                       // idle
            case 0xC1: _polledStatusSeen = true; applyRecordingState(false); break; // saving = stopped
            case 0x41: _polledStatusSeen = true; break;  // starting — too brief, keep previous
            default:   break;                            // unknown — ignore, don't invent state
        }
        return;
    }

    // Battery query reply (0D/02, flags 0xC0). Payload: [result][struct...],
    // battery % at payload[21] = pData[32]; needs 13 + 22 = 35 bytes.
    if (isReply && hdr.cmdSet == 0x0D && hdr.cmdId == 0x02 &&
        length >= 35 && pData[11] == 0x00 && pData[32] <= 100) {
        if (_telemetry.batteryPercent != pData[32]) {
            DBG("ACTION: battery %u%%", pData[32]);
        }
        _telemetry.batteryPercent = pData[32];
        _telemetry.dataValid = true;
        return;
    }

    // Battery push (flags=0x00, 0D/02) — Nano layout, no result byte.
    // Kept in case the Action also pushes it; harmless if it never does.
    if (hdr.flags == 0x00 && hdr.cmdSet == 0x0D && hdr.cmdId == 0x02 &&
        length >= 32 && pData[31] <= 100) {
        _telemetry.batteryPercent = pData[31];
        _telemetry.dataValid = true;
        return;
    }

    // SD card info reply (02/71, flags 0xC0) — remaining record time.
    // Bench capture (Action 2, 2026-09-24), payload after the header:
    //   00 | 01 | 21 59 00 00 | A8 2B 00 00 | 00 00 00 00 | E4 02 00 00
    //   res  ?    total 22817MB  free 11176MB  shots 0       740 s remaining
    // Same field order as DJI's 02/80 Camera State Info (sd total, sd free,
    // remained shots, remained time — dji-firmware-tools camera dissector).
    // The Action 2 never answered a 02/80 query, so this is the source.
    if (isReply && hdr.cmdSet == 0x02 && hdr.cmdId == 0x71) {
        if (length >= 31 && pData[11] == 0x00) {
            uint32_t freeMb = (uint32_t)pData[17] | ((uint32_t)pData[18] << 8) |
                              ((uint32_t)pData[19] << 16) | ((uint32_t)pData[20] << 24);
            uint32_t remain = (uint32_t)pData[25] | ((uint32_t)pData[26] << 8) |
                              ((uint32_t)pData[27] << 16) | ((uint32_t)pData[28] << 24);
            uint16_t rem = remain > 0xFFFF ? 0xFFFF : (uint16_t)remain;
            if (!_remainingKnown || rem != _remainingSec) {
                DBG("ACTION: 02/71 remaining %lus (%lu MB free)",
                    (unsigned long)remain, (unsigned long)freeMb);
            }
            _remainingSec   = rem;
            _remainingKnown = true;
            _telemetry.storageRaw = freeMb > 0xFFFF ? 0xFFFF : (uint16_t)freeMb;
            if (_telemetry.state != CAM_STATE_RECORDING) {
                _telemetry.recTimeSeconds = _remainingSec;
            }
        } else {
            DBG("ACTION: 02/71 reply code 0x%02X len=%u (not usable)",
                length > 11 ? pData[11] : 0xFF, (unsigned)length);
        }
        return;
    }

    // General Status Push (flags=0x00, 02/80) — Nano offsets, FALLBACK only.
    // pData[11] bit 0x80 = recording; pData[15] video/photo; pData[20:21]
    // storage counter; pData[28:29] remaining record time (standby).
    if (hdr.flags == 0x00 && hdr.cmdSet == 0x02 && hdr.cmdId == 0x80 && length >= 30) {
        _telemetry.captureMode = pData[15];
        _telemetry.storageRaw  = pData[20] | (pData[21] << 8);

        if (!_polledStatusSeen) {
            uint8_t recByte = pData[11];
            if (recByte & 0x80)       applyRecordingState(true);
            else if (recByte == 0x01) applyRecordingState(false);
        }
        _remainingSec   = pData[28] | (pData[29] << 8);
        _remainingKnown = true;
        if (_telemetry.state != CAM_STATE_RECORDING) {
            _telemetry.recTimeSeconds = _remainingSec;
        }
        _telemetry.dataValid = true;
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Public API
// ──────────────────────────────────────────────────────────────────────────────
void djiActionInit() {
    DBG("ACTION: Backend ready (NimBLE stack shared)");
    _session   = DjiDumlSession();
    // The Action 2 only answers queries, so a quiet link usually means its
    // DUML session lapsed, not that BLE died — try re-auth in place first.
    _session.softRecoverOnStale = true;
    _telemetry = CameraTelemetry();
    resetSessionTelemetry();
#if DJI_ACTION_FRAME_DISCOVERY
    _seenFrameCount = 0;
#endif
}

void djiActionUpdate() {
    dumlUpdate(_session, _telemetry, CAMERA_DJI_ACTION, MODEL_NAME,
               serialConfigFcUartActive(), &_clientCallbacks, notifyCallback);

    if (!djiActionIsReady()) return;

    uint32_t now = millis();

    // Keep the elapsed-time clock ticking between replies (the Nano gets
    // this for free from its ~1 Hz push; here the 500 ms poll drives it,
    // but a dropped reply shouldn't freeze the OSD counter).
    // _recordStartMs is written from the NimBLE callback task, so it can be
    // a few ms newer than `now` — clamp instead of letting the unsigned
    // subtraction wrap (same race as the transport watchdog fix).
    if (_telemetry.state == CAM_STATE_RECORDING) {
        int32_t el = (int32_t)(now - _recordStartMs);
        _telemetry.recTimeSeconds = (uint16_t)((el > 0 ? (uint32_t)el : 0) / 1000);
    }

    // Remote heartbeat, 1 Hz: 00/2B payload {01 01} to 0xF0, plus an empty
    // 00/00 to 0x28. This is what DJI's own Bluetooth remote sends; RotorREC
    // lists it as hardware-confirmed keep-alive on the Action 2 (it also
    // lights the camera's Bluetooth icon). Without it the camera appears to
    // drop the remote session after a while and stop answering our polls,
    // which is what tripped the reconnect loop. The shared 00/F1 keep-alive
    // (every 15 s, Nano-verified) still goes out from the transport too.
    if (_lastHeartbeat == 0 || now - _lastHeartbeat >= DJI_ACTION_HEARTBEAT_MS) {
        _lastHeartbeat = now ? now : 1;
        static const uint8_t hb[] = {0x01, 0x01};
        if (_session.pControlChar && _session.pControlChar->canWriteNoResponse()) {
            uint8_t packet[32];
            size_t len = dumlBuildPacket(packet, DUML_SENDER_APP, 0xF0,
                                          _session.sequenceCounter++, 0x40,
                                          0x00, 0x2B, hb, sizeof(hb));
            _session.pControlChar->writeValue(packet, len, false);
            len = dumlBuildPacket(packet, DUML_SENDER_APP, 0x28,
                                   _session.sequenceCounter++, 0x00,
                                   0x00, 0x00, nullptr, 0);
            _session.pControlChar->writeValue(packet, len, false);
        }
    }

    if (now - _lastStatusPoll >= DJI_ACTION_STATUS_POLL_MS) {
        _lastStatusPoll = now;
        static const uint8_t q[] = {0x01};
        sendQuery(DUML_RX_CAMERA, 0x02, 0x70, q, sizeof(q));
    }

    // First battery query goes out on the first ready tick (_lastBatteryPoll
    // is reset to 0 per session), then every DJI_ACTION_BATTERY_POLL_MS.
    if (_lastBatteryPoll == 0 || now - _lastBatteryPoll >= DJI_ACTION_BATTERY_POLL_MS) {
        _lastBatteryPoll = now ? now : 1;
        static const uint8_t q[] = {0x00, 0x00, 0x00, 0x00};
        sendQuery(DUML_RX_BATTERY, 0x0D, 0x02, q, sizeof(q));
    }

    // Remaining record time only matters in standby (while recording the
    // OSD shows locally-counted elapsed time), so only ask for it then.
    // First query on the first ready tick, then every DJI_ACTION_REMAIN_POLL_MS.
    if (_telemetry.state != CAM_STATE_RECORDING &&
        (_lastRemainPoll == 0 || now - _lastRemainPoll >= DJI_ACTION_REMAIN_POLL_MS)) {
        _lastRemainPoll = now ? now : 1;
        sendQuery(DUML_RX_CAMERA, 0x02, 0x71, nullptr, 0);
    }
}

void djiActionStartScan() {
    dumlStartScan(_session, &_scanCallbacks);
}

// Record control: 02/02, payload 0x01 start / 0x00 stop, app (0x02) ->
// camera (0x01). Same opcode the upstream Action 2 work and both reference
// projects use; identical bytes to the Nano backend.
static bool sendRecord(bool start) {
    if (_session.bleState != BLE_CONNECTED || !_session.sessionEstablished) return false;

    uint8_t packet[32];
    uint8_t payload[] = { (uint8_t)(start ? 0x01 : 0x00) };
    size_t len = dumlBuildPacket(packet, DUML_SENDER_APP, DUML_RX_CAMERA,
                                  _session.sequenceCounter++,
                                  0x40, 0x02, 0x02, payload, sizeof(payload));

    if (_session.pControlChar && _session.pControlChar->canWriteNoResponse()) {
        _session.pControlChar->writeValue(packet, len, false);
        DBG("ACTION: Sent %s Record to FFF5", start ? "Start" : "Stop");
        // Poll state promptly instead of waiting out the rest of the interval.
        _lastStatusPoll = millis() - DJI_ACTION_STATUS_POLL_MS + 150;
        return true;
    }
    return false;
}

bool djiActionSendStartRecord() { return sendRecord(true); }
bool djiActionSendStopRecord()  { return sendRecord(false); }

BleConnectionState djiActionGetState() { return _session.bleState; }
const CameraTelemetry& djiActionGetTelemetry() { return _telemetry; }
bool djiActionIsReady() { return (_session.bleState == BLE_CONNECTED) && _session.sessionEstablished; }
const char* djiActionGetLastError() { return _session.lastError; }

void djiActionTargetMac(const char *mac) {
    dumlTargetMac(_session, mac, "ACTION");
}
