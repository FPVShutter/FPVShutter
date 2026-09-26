// ============================================================================
// dji_rsdk_camera.cpp — DJI R SDK backend: Osmo Action 4 / 5 Pro / 6, Osmo 360
// ============================================================================
// Why a separate backend from dji_action_camera.cpp
// -------------------------------------------------
// The Action 2 (and the Nano) speak DUML: 0x55 frames, 07/45 PIN pairing,
// 02/02 record, polled/pushed DUML telemetry. From the Action 4 on, DJI
// publishes an official remote-control protocol instead — the "DJI R SDK"
// protocol in DJI's Osmo GPS Controller demo (github.com/dji-sdk/Osmo-GPS-
// Controller-Demo, MIT for the code, DJI EULA for the protocol docs). It
// runs over the SAME GATT service (0xFFF0: notify FFF4, write FFF5), so the
// BLE link comes from dji_duml_transport unchanged and only the protocol on
// top differs (see DjiLinkHooks there).
//
// What the camera gives us here is better than DUML ever did:
//   • 1D02 status push, subscribed via 1D05 at 2 Hz + on every change:
//     mode, recording status, resolution, fps, EIS, CURRENT RECORD TIME
//     (from the camera itself), SD free MB, remaining record time, power
//     mode, overheat state and battery % — all in one frame.
//   • 1D03 record start/stop with a result code.
//   • 0019 handshake that reports which camera model answered.
//
// Handshake (0019), per DJI's protocol_data_segment.md:
//   1. We send a connection request: our device_id + MAC, verify_mode,
//      a random 4-digit verify code. (verify_mode 0 = camera decides from
//      its pairing history: silent if it knows us, on-screen prompt if not.)
//   2. Camera answers with a 0019 response frame (ret_code 0 = OK).
//   3. Camera then sends ITS OWN 0019 command frame, verify_mode 2,
//      verify_data 0 = allowed / 1 = rejected, device_id = camera model.
//   4. We answer that with a 0019 response frame on the camera's SEQ.
//   Then: subscribe to status (1D05 mode 3, freq 20).
// A rejected camera must not be asked again straight away (DJI's note), so
// a rejection pauses automatic reconnects for DJI_RSDK_REJECT_BACKOFF_MS.
//
// Threading: notifications arrive on the NimBLE host task. The callback only
// parses and sets flags/telemetry; every WRITE happens from loop() context
// (the transport hooks and djiRsdkUpdate()), never from inside the callback.
//
// STATUS: frame layer unit-tested against DJI's own test frames (all ten
// connect vectors in test/connect_cmd_frame_builder plus the documented
// mode-switch example, byte-for-byte). The handshake/telemetry logic has
// NOT run on a real camera yet — first bench target is the Osmo Action 4.
// DJI_RSDK_FRAME_DISCOVERY (config.h) is on for that bring-up.
// ============================================================================

#include "dji_rsdk_camera.h"
#include "dji_rsdk_protocol.h"
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
static RsdkReassembler _rx;

static const char *MODEL_NAME = "DJI Osmo Action 4+";   // until the handshake names it

enum RsdkHandshake : uint8_t {
    HS_IDLE = 0,
    HS_SEND_REQUEST,   // link up, request not sent yet
    HS_WAIT_CAMERA,    // request sent, waiting for the camera's verdict
    HS_ACCEPT,         // camera allowed us — answer it from loop()
    HS_DONE,           // session established
    HS_REJECTED,       // camera (user) said no
    HS_FAILED,         // camera answered our request with an error code
};
static volatile uint8_t  _hs            = HS_IDLE;
static volatile uint16_t _camSeq        = 0;     // SEQ of the camera's own 0019 request
static volatile uint32_t _camDeviceId   = 0;     // device_id the camera reported
static volatile uint8_t  _hsRetCode     = 0;
static volatile bool     _reAckPending  = false; // camera re-sent 0019 after HS_DONE
static uint32_t          _linkUpMs      = 0;
static uint16_t          _verifyCode    = 0;
static uint32_t          _rejectBackoffUntil = 0;  // 0 = no backoff

static volatile bool     _needSubscribe = false;
static bool              _loggedDuml    = false;

// Last status push (for change-only logging) + local elapsed fallback.
static RsdkCameraStatus  _last;
static bool              _lastValid     = false;
static bool              _wasRecording  = false;
static uint32_t          _recordStartMs = 0;
static uint32_t          _statusPushes  = 0;
static char              _lastModeName[22] = "";

// ──────────────────────────────────────────────────────────────────────────────
// Forward declarations
// ──────────────────────────────────────────────────────────────────────────────
static void notifyCallback(NimBLERemoteCharacteristic *pChar, uint8_t *pData,
                            size_t length, bool isNotify);
static bool isRsdkDevice(NimBLEAdvertisedDevice *device);
static void resetSessionState();

// ──────────────────────────────────────────────────────────────────────────────
// Sending (loop() context only)
// ──────────────────────────────────────────────────────────────────────────────
static bool sendFrame(uint8_t cmdType, uint16_t seq, uint16_t key,
                      const uint8_t *payload, size_t len) {
    NimBLERemoteCharacteristic *c = _session.pControlChar;
    if (!c) return false;
    uint8_t buf[96];
    size_t n = rsdkBuildFrame(buf, sizeof(buf), cmdType, seq,
                              (uint8_t)(key >> 8), (uint8_t)key, payload, len);
    if (!n) return false;
    // Same write style as the DUML backends on FFF5 (no response: never
    // blocks loop()); fall back to write-with-response if that's all the
    // characteristic offers.
    if (c->canWriteNoResponse()) return c->writeValue(buf, n, false);
    if (c->canWrite())           return c->writeValue(buf, n, true);
    return false;
}

static uint16_t nextSeq() {
    uint16_t s = _session.sequenceCounter++;
    if (_session.sequenceCounter == 0) _session.sequenceCounter = 1;
    return s;
}

static void sendSubscribe() {
    uint8_t p[6];
    size_t n = rsdkBuildStatusSubscribe(p, 3 /* periodic + on change */, 20 /* 2 Hz */);
    sendFrame(RSDK_CMD_NO_RESPONSE, nextSeq(), RSDK_STATUS_SUBSCRIBE, p, n);
    DBG("RSDK: subscribed to camera status (1D05, 2 Hz + on change)");
}

static void sendVersionQuery() {
    sendFrame(RSDK_CMD_RESPONSE_OR_NOT, nextSeq(), RSDK_VERSION_QUERY, nullptr, 0);
}

/// Our own BLE MAC in display order — the camera stores its pairing record
/// against it, so it must be stable (it is: the C3's public address).
static void ownMac(uint8_t out[6]) {
    memset(out, 0, 6);
    std::string a = NimBLEDevice::getAddress().toString();
    unsigned b[6];
    if (sscanf(a.c_str(), "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
        for (int i = 0; i < 6; i++) out[i] = (uint8_t)b[i];
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Transport hooks
// ──────────────────────────────────────────────────────────────────────────────
static void hookLinkUp(DjiDumlSession &s) {
    _rx.reset();
    resetSessionState();
    _hs       = HS_SEND_REQUEST;
    _linkUpMs = millis();
}

static void hookAuthTick(DjiDumlSession &s, uint32_t now) {
    switch (_hs) {
        case HS_SEND_REQUEST: {
            if (now - _linkUpMs < DJI_RSDK_CONNECT_DELAY_MS) break;
            uint8_t mac[6], p[40];
            ownMac(mac);
            _verifyCode = (uint16_t)(esp_random() % 10000);
            size_t n = rsdkBuildConnectRequest(p, DJI_RSDK_REMOTE_DEVICE_ID, mac,
                                               DJI_RSDK_VERIFY_MODE, _verifyCode);
            if (sendFrame(RSDK_CMD_WAIT_RESULT, nextSeq(), RSDK_CONNECT, p, n)) {
                DBG("RSDK: connection request sent (verify code %04u) — if the camera "
                    "shows a pairing prompt, accept it on the camera", _verifyCode);
                _hs = HS_WAIT_CAMERA;
            } else {
                DBG("RSDK: could not write connection request");
            }
            break;
        }

        case HS_WAIT_CAMERA:
            // The shared 30 s auth timeout handles "no answer"; leave the
            // user a reason just before it fires.
            if (now - s.authStartMs >= 29000 && s.lastError[0] == '\0') {
                s.setError("no approval from camera");
            }
            break;

        case HS_ACCEPT: {
            uint8_t p[9];
            size_t n = rsdkBuildConnectResponse(p, DJI_RSDK_REMOTE_DEVICE_ID, 0x00, 0);
            sendFrame(RSDK_ACK_NO_RESPONSE, _camSeq, RSDK_CONNECT, p, n);

            const char *m = rsdkModelName(_camDeviceId);
            char model[sizeof(_telemetry.model)];
            snprintf(model, sizeof(model), "DJI %s", m ? m : "Osmo (R SDK)");
            strlcpy(_telemetry.model, model, sizeof(_telemetry.model));
            DBG("RSDK: *** CONNECTED — %s (camera device_id 0x%08lX) ***",
                model, (unsigned long)_camDeviceId);

            s.resetError();
            s.sessionEstablished = true;
            s.bleState           = BLE_CONNECTED;
            s.lastRxMs           = now;
            s.lastKeepAlive      = now;
            _hs                  = HS_DONE;
            _needSubscribe       = true;
            break;
        }

        case HS_REJECTED:
        case HS_FAILED:
            if (_hs == HS_REJECTED) {
                DBG("RSDK: camera REJECTED the connection — pausing auto-reconnect %lus",
                    (unsigned long)(DJI_RSDK_REJECT_BACKOFF_MS / 1000));
                s.setError("camera rejected pairing");
            } else {
                DBG("RSDK: connection request refused, ret_code 0x%02X", _hsRetCode);
                s.setError("camera refused connection");
            }
            _rejectBackoffUntil = (now + DJI_RSDK_REJECT_BACKOFF_MS) | 1;
            _hs = HS_IDLE;
            if (s.pClient) s.pClient->disconnect();
            s.bleState = BLE_DISCONNECTED;
            s.lastReconnectAttempt = now;
            break;

        default:
            break;
    }
}

// No documented keep-alive in the R SDK (DJI's demo sends none; the 2 Hz
// status push keeps the link busy). A version query every
// BLE_KEEPALIVE_INTERVAL_MS is a harmless request/response that also feeds
// the watchdog if pushes ever pause.
static void hookKeepAlive(DjiDumlSession &s) { sendVersionQuery(); }

// Link quiet but BLE up: most likely the subscription lapsed (or the camera
// went to sleep). Re-subscribing is cheap; if nothing answers, the shared
// watchdog does the full reconnect after DJI_SOFT_RECOVER_GRACE_MS.
static void hookSoftRecover(DjiDumlSession &s) { sendSubscribe(); }

static const DjiLinkHooks RSDK_HOOKS = {
    hookLinkUp, hookAuthTick, hookKeepAlive, hookSoftRecover,
};

// ──────────────────────────────────────────────────────────────────────────────
// BLE callbacks
// ──────────────────────────────────────────────────────────────────────────────
class RsdkClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient *pClient) override {
        DBG("RSDK: BLE connected");
    }
    // DJI's demo never pairs/encrypts at the BLE level, but NimBLE starts
    // pairing by itself if the camera sends an SMP Security Request (our
    // stack is configured for bonding because GoPro needs it). Log it so a
    // bench log shows whether that happens with the Action 4.
    void onAuthenticationComplete(ble_gap_conn_desc *desc) override {
        DBG("RSDK: BLE security change: encrypted=%u authenticated=%u bonded=%u",
            desc->sec_state.encrypted, desc->sec_state.authenticated, desc->sec_state.bonded);
    }
    void onDisconnect(NimBLEClient *pClient) {
        DBG("RSDK: disconnected (%lu status pushes this session, %lu CRC errors)",
            (unsigned long)_statusPushes, (unsigned long)_rx.crcErrors());
        _session.bleState           = BLE_DISCONNECTED;
        _session.sessionEstablished = false;
        _session.pControlChar       = nullptr;
        _session.pTelemetryChar     = nullptr;
        _session.pAuthChar          = nullptr;
        _telemetry.dataValid        = false;
        _telemetry.state            = CAM_STATE_UNKNOWN;
        if (_hs != HS_REJECTED && _hs != HS_FAILED) _hs = HS_IDLE;
    }
};
static RsdkClientCallbacks _clientCallbacks;

// RTOS-safe scan callback: same rules as the other DJI backends (copy
// NimBLE's temporaries, no NVS/heavy work here).
class RsdkAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice *advertisedDevice) override {
        bool isMatched = isRsdkDevice(advertisedDevice);
        if (!isMatched && scanResultsIsShowAll()) {
            std::string addr = advertisedDevice->getAddress().toString();
            isMatched = !addr.empty();
        }
        if (!isMatched) return;

        std::string macStr  = advertisedDevice->getAddress().toString();
        std::string nameStr = advertisedDevice->haveName() ? advertisedDevice->getName() : "";
        int8_t rssi = advertisedDevice->getRSSI();

        scanResultsAdd(CAMERA_DJI_RSDK, macStr.c_str(), nameStr.c_str(), rssi, isMatched);
        camRegistryRemember(CAMERA_DJI_RSDK, macStr.c_str(), nameStr.c_str());

        if (camRegistryMayAutoConnect(CAMERA_DJI_RSDK, macStr.c_str())) {
            NimBLEDevice::getScan()->stop();
            _session.targetAddress    = advertisedDevice->getAddress();
            _session.hasTargetAddress = true;
            _session.doConnect        = true;
        }
    }
};
static RsdkAdvertisedDeviceCallbacks _scanCallbacks;

// ──────────────────────────────────────────────────────────────────────────────
// Device identification
// ──────────────────────────────────────────────────────────────────────────────
// DJI's own demo accepts a camera when its manufacturer data is
// [AA 08 xx xx FA ...] (DJI company id 0x08AA, byte 4 = 0xFA). That's the
// primary signal; names and the plain DJI company id are fallbacks so a
// camera with an unexpected advert still shows up in the scan list.
static const char *RSDK_NAME_PREFIXES[] = {
    "Osmo Action", "OsmoAction", "OSMO ACTION", "DJI Action", "Action 4",
    "Action 5", "Action 6", "Osmo 360", "OSMO 360", "OA4", "OA5", "OA6",
};

static bool isRsdkDevice(NimBLEAdvertisedDevice *device) {
    if (settingsGet().scanAll) {
        std::string addr = device->getAddress().toString();
        return !addr.empty();
    }

    std::string mfr = device->getManufacturerData();
    if (mfr.size() >= 5 && (uint8_t)mfr[0] == 0xAA && (uint8_t)mfr[1] == 0x08 &&
        (uint8_t)mfr[4] == 0xFA) {
        return true;
    }

    if (device->haveName()) {
        std::string name = device->getName();
        for (size_t i = 0; i < sizeof(RSDK_NAME_PREFIXES) / sizeof(RSDK_NAME_PREFIXES[0]); i++) {
            if (name.find(RSDK_NAME_PREFIXES[i]) != std::string::npos) return true;
        }
    }

    if (device->isAdvertisingService(DJI_DUML_ADV_SERVICE_UUID)) return true;
    if (!mfr.empty() && (dumlBytesContain(mfr, DJI_DUML_COMPANY_ID) ||
                         dumlBytesContain(mfr, DJI_DUML_XTRA_COMPANY))) return true;
    return false;
}

// ──────────────────────────────────────────────────────────────────────────────
// Inbound frames (NimBLE host task)
// ──────────────────────────────────────────────────────────────────────────────
#if DJI_RSDK_FRAME_DISCOVERY
static uint32_t _seenKeys[32];
static uint8_t  _seenCount = 0;

/// Logs the first frame of every distinct (set, id, command/response)
/// combination with a hex dump of its payload — the bring-up aid for a new
/// camera model, same idea as DJI_ACTION_FRAME_DISCOVERY.
static void logFirstSeen(const RsdkFrame &f) {
    uint32_t key = ((uint32_t)f.key() << 8) | (f.isResponse() ? 1 : 0);
    for (uint8_t i = 0; i < _seenCount; i++) if (_seenKeys[i] == key) return;
    if (_seenCount >= sizeof(_seenKeys) / sizeof(_seenKeys[0])) return;
    _seenKeys[_seenCount++] = key;

    char hexBuf[3 * 48 + 1] = "";
    size_t hexLen = 0;
    for (size_t i = 0; i < f.payloadLen && i < 48; i++) {
        hexLen += snprintf(hexBuf + hexLen, sizeof(hexBuf) - hexLen, "%02X ", f.payload[i]);
    }
    DBG("RSDK: new frame %04X %s type=%02X seq=%u len=%u: %s%s", f.key(),
        f.isResponse() ? "resp" : "cmd ", f.cmdType, f.seq, (unsigned)f.payloadLen,
        hexBuf, f.payloadLen > 48 ? "..." : "");
}
#endif

static const char *statusName(uint8_t s) {
    switch (s) {
        case 0x00: return "screen off";
        case 0x01: return "live view";
        case 0x02: return "playback";
        case 0x03: return "RECORDING";
        case 0x05: return "pre-recording";
        default:   return "?";
    }
}

static void applyStatus(const RsdkCameraStatus &st) {
    _statusPushes++;
    const bool rec = (st.cameraStatus == 0x03);

    if (!_lastValid || st.cameraStatus != _last.cameraStatus || st.cameraMode != _last.cameraMode ||
        st.videoResolution != _last.videoResolution || st.fpsIdx != _last.fpsIdx ||
        st.eisMode != _last.eisMode) {
        DBG("RSDK: status %s | mode 0x%02X res %u fps %u eis %u | batt %u%% | "
            "remain %lus / %lu MB", statusName(st.cameraStatus), st.cameraMode,
            st.videoResolution, rsdkFpsFromIdx(st.fpsIdx), st.eisMode, st.batteryPercent,
            (unsigned long)st.remainTimeS, (unsigned long)st.remainCapacityMb);
    }
    if (_lastValid && st.tempOver != _last.tempOver) {
        DBG("RSDK: camera temperature state %u (%s)", st.tempOver,
            st.tempOver == 0 ? "normal" : st.tempOver == 1 ? "warm" :
            st.tempOver == 2 ? "TOO HOT TO RECORD" : "OVERHEAT SHUTDOWN");
    }
    if (_lastValid && st.powerMode != _last.powerMode) {
        DBG("RSDK: camera power mode %u (%s)", st.powerMode,
            st.powerMode == 3 ? "sleep" : "normal");
    }
    _last = st;
    _lastValid = true;

    if (rec && !_wasRecording) _recordStartMs = millis();
    _wasRecording = rec;

    _telemetry.state = rec ? CAM_STATE_RECORDING : CAM_STATE_STANDBY;
    if (rec) {
        // The camera reports its own record time — prefer it. Fall back to
        // a local clock only if it reads 0 well into a recording.
        uint32_t local = (millis() - _recordStartMs) / 1000;
        _telemetry.recTimeSeconds = (st.recordTimeS == 0 && local > 2)
                                    ? (uint16_t)local : st.recordTimeS;
    } else {
        _telemetry.recTimeSeconds = st.remainTimeS > 0xFFFF ? 0xFFFF : (uint16_t)st.remainTimeS;
    }
    if (st.batteryPercent <= 100) _telemetry.batteryPercent = st.batteryPercent;
    _telemetry.captureMode = (st.cameraMode == 0x05) ? 0 : 1;   // 0x05 = photo
    _telemetry.storageRaw  = st.remainCapacityMb > 0xFFFF ? 0xFFFF
                                                          : (uint16_t)st.remainCapacityMb;
    _telemetry.dataValid   = true;
}

static void onFrame(const RsdkFrame &f, void *) {
#if DJI_RSDK_FRAME_DISCOVERY
    logFirstSeen(f);
#endif
    switch (f.key()) {
        case RSDK_CONNECT:
            if (f.isResponse()) {
                // Camera's answer to OUR request: device_id(4) ret_code(1) ...
                uint8_t ret = f.payloadLen >= 5 ? f.payload[4] : 0xFF;
                if (ret == 0x00) {
                    DBG("RSDK: camera accepted the request, waiting for its verdict...");
                } else if (_hs == HS_WAIT_CAMERA || _hs == HS_SEND_REQUEST) {
                    _hsRetCode = ret;
                    _hs = HS_FAILED;
                }
            } else {
                RsdkConnectRequest req;
                if (!rsdkParseConnectRequest(f.payload, f.payloadLen, req)) break;
                if (req.verifyMode != 2) {
                    DBG("RSDK: unexpected camera 0019 verify_mode %u", req.verifyMode);
                    break;
                }
                _camDeviceId = req.deviceId;
                _camSeq      = f.seq;
                if (_hs == HS_DONE) {
                    _reAckPending = true;         // answer again from loop()
                } else if (req.verifyData == 0) {
                    _hs = HS_ACCEPT;
                } else {
                    _hs = HS_REJECTED;
                }
            }
            break;

        case RSDK_STATUS_PUSH: {
            RsdkCameraStatus st;
            if (rsdkParseCameraStatus(f.payload, f.payloadLen, st)) applyStatus(st);
            else DBG("RSDK: short 1D02 push (%u bytes)", (unsigned)f.payloadLen);
            break;
        }

        case RSDK_STATUS_PUSH_NEW: {
            // Mode name + parameter strings (e.g. "Video" / "4K 60fps").
            // Logged on change only; handy to confirm settings on the bench.
            if (f.payloadLen >= 25 && f.payload[0] == 0x01) {
                uint8_t nl = f.payload[1] > 20 ? 20 : f.payload[1];
                char name[22] = "";
                memcpy(name, &f.payload[2], nl);
                name[nl] = '\0';
                char param[22] = "";
                if (f.payloadLen >= 26 && f.payload[23] == 0x02) {
                    uint8_t pl = f.payload[24] > 20 ? 20 : f.payload[24];
                    if ((size_t)25 + pl <= f.payloadLen) { memcpy(param, &f.payload[25], pl); param[pl] = '\0'; }
                }
                if (strcmp(name, _lastModeName) != 0) {
                    strlcpy(_lastModeName, name, sizeof(_lastModeName));
                    DBG("RSDK: camera mode \"%s\" %s", name, param);
                }
            }
            break;
        }

        case RSDK_RECORD_CONTROL:
            if (f.isResponse()) {
                uint8_t ret = f.payloadLen ? f.payload[0] : 0xFF;
                DBG("RSDK: record command %s (ret 0x%02X)",
                    ret == 0 ? "OK" : ret == 1 ? "parse error" : ret == 2 ? "FAILED" : "error", ret);
            }
            break;

        case RSDK_VERSION_QUERY:
            if (f.isResponse() && f.payloadLen >= 18) {
                static bool logged = false;
                if (!logged) {
                    logged = true;
                    char pid[17]; memcpy(pid, &f.payload[2], 16); pid[16] = '\0';
                    char ver[48] = "";
                    size_t vl = f.payloadLen - 18;
                    if (vl > sizeof(ver) - 1) vl = sizeof(ver) - 1;
                    for (size_t i = 0; i < vl; i++) {
                        char c = (char)f.payload[18 + i];
                        ver[i] = (c >= 32 && c < 127) ? c : ' ';
                    }
                    ver[vl] = '\0';
                    DBG("RSDK: camera product \"%s\" version \"%s\"", pid, ver);
                }
            }
            break;

        default:
            break;
    }
}

static void notifyCallback(NimBLERemoteCharacteristic *pChar, uint8_t *pData,
                            size_t length, bool isNotify) {
    _session.lastRxMs = millis();   // any inbound traffic proves the link is alive

    // Does this camera ALSO talk DUML on the link? Worth knowing once
    // (a valid DUML header CRC makes a false positive very unlikely).
    if (!_loggedDuml && length >= 13 && pData[0] == 0x55 && dumlCrc8(pData, 3) == pData[3]) {
        _loggedDuml = true;
        dumlLogNonDuml("DUML-on-RSDK-link", pData, length);
    }

    _rx.feed(pData, length, onFrame, nullptr);
}

// ──────────────────────────────────────────────────────────────────────────────
// Public API
// ──────────────────────────────────────────────────────────────────────────────
static void resetSessionState() {
    _needSubscribe = false;
    _reAckPending  = false;
    _lastValid     = false;
    _wasRecording  = false;
    _statusPushes  = 0;
    _lastModeName[0] = '\0';
}

void djiRsdkInit() {
    DBG("RSDK: Backend ready (Osmo Action 4 / 5 Pro / 6, Osmo 360 — DJI R SDK)");
    _session = DjiDumlSession();
    _session.hooks = &RSDK_HOOKS;
    _session.fullServiceDiscovery = true;   // see DjiDumlSession::fullServiceDiscovery
    // The camera pushes status at 2 Hz, so a quiet link means the push
    // subscription lapsed (or the camera slept) before it means BLE died:
    // re-subscribe in place first.
    _session.softRecoverOnStale = true;
    _telemetry = CameraTelemetry();
    _rx.reset();
    _hs = HS_IDLE;
    _rejectBackoffUntil = 0;
    resetSessionState();
#if DJI_RSDK_FRAME_DISCOVERY
    _seenCount = 0;
#endif
}

void djiRsdkUpdate() {
    uint32_t now = millis();
    bool backoff = false;
    if (_rejectBackoffUntil) {
        if ((int32_t)(now - _rejectBackoffUntil) < 0) backoff = true;
        else _rejectBackoffUntil = 0;
    }

    dumlUpdate(_session, _telemetry, CAMERA_DJI_RSDK, MODEL_NAME,
               serialConfigFcUartActive() || backoff, &_clientCallbacks, notifyCallback);

    if (!djiRsdkIsReady()) return;

    if (_needSubscribe) {
        _needSubscribe = false;
        sendSubscribe();
        sendVersionQuery();
    }
    if (_reAckPending) {
        _reAckPending = false;
        uint8_t p[9];
        size_t n = rsdkBuildConnectResponse(p, DJI_RSDK_REMOTE_DEVICE_ID, 0x00, 0);
        sendFrame(RSDK_ACK_NO_RESPONSE, _camSeq, RSDK_CONNECT, p, n);
        DBG("RSDK: camera re-sent its connection request — answered again");
    }

    // Keep the OSD's elapsed counter moving between 2 Hz pushes when the
    // camera's own record time isn't usable (see applyStatus()).
    if (_telemetry.state == CAM_STATE_RECORDING && _lastValid && _last.recordTimeS == 0) {
        int32_t el = (int32_t)(now - _recordStartMs);
        _telemetry.recTimeSeconds = (uint16_t)((el > 0 ? (uint32_t)el : 0) / 1000);
    }
}

void djiRsdkStartScan() {
    dumlStartScan(_session, &_scanCallbacks);
}

// Record control: 1D03, record_ctrl 0 = START, 1 = STOP (opposite polarity
// to DUML's 02/02). The state change itself arrives on the next 1D02 push,
// which the subscription sends immediately on change.
static bool sendRecord(bool start) {
    if (!djiRsdkIsReady()) return false;
    uint8_t p[9];
    size_t n = rsdkBuildRecordControl(p, DJI_RSDK_CAMERA_DEVICE_ID, start);
    bool ok = sendFrame(RSDK_CMD_RESPONSE_OR_NOT, nextSeq(), RSDK_RECORD_CONTROL, p, n);
    DBG("RSDK: sent %s record%s", start ? "START" : "STOP", ok ? "" : " (write FAILED)");
    return ok;
}

bool djiRsdkSendStartRecord() { return sendRecord(true); }
bool djiRsdkSendStopRecord()  { return sendRecord(false); }

BleConnectionState djiRsdkGetState() { return _session.bleState; }
const CameraTelemetry& djiRsdkGetTelemetry() { return _telemetry; }
bool djiRsdkIsReady() { return _session.bleState == BLE_CONNECTED && _session.sessionEstablished; }
const char* djiRsdkGetLastError() { return _session.lastError; }

void djiRsdkTargetMac(const char *mac) {
    _rejectBackoffUntil = 0;   // a user-initiated connect overrides the backoff
    dumlTargetMac(_session, mac, "RSDK");
}
