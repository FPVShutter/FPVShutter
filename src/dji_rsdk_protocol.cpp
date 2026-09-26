// ============================================================================
// dji_rsdk_protocol.cpp — DJI R SDK frame layer (see dji_rsdk_protocol.h)
// ============================================================================

#include "dji_rsdk_protocol.h"
#include <string.h>

// ── Little-endian helpers ───────────────────────────────────────────────────
static inline void putU16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
static inline void putU32(uint8_t *p, uint32_t v) {
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}
static inline uint16_t getU16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t getU32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// ── CRCs (bitwise; frames are short, a 1 KB table isn't worth the flash) ───
uint16_t rsdkCrc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0x3AA3;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

uint32_t rsdkCrc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0x00003AA3;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320UL : (crc >> 1);
    }
    return crc;
}

// ── Build ───────────────────────────────────────────────────────────────────
size_t rsdkBuildFrame(uint8_t *out, size_t cap, uint8_t cmdType, uint16_t seq,
                      uint8_t cmdSet, uint8_t cmdId,
                      const uint8_t *payload, size_t payloadLen) {
    const size_t total = RSDK_OVERHEAD + payloadLen;
    if (total > cap || total > RSDK_MAX_FRAME) return 0;

    out[0] = RSDK_SOF;
    putU16(&out[1], (uint16_t)(total & 0x03FF));   // version 0
    out[3] = cmdType;
    out[4] = 0x00;                                  // ENC: none
    out[5] = out[6] = out[7] = 0x00;                // RES
    putU16(&out[8], seq);
    putU16(&out[10], rsdkCrc16(out, 10));
    out[12] = cmdSet;
    out[13] = cmdId;
    if (payloadLen) memcpy(&out[14], payload, payloadLen);
    putU32(&out[total - 4], rsdkCrc32(out, total - 4));
    return total;
}

// ── Parse ───────────────────────────────────────────────────────────────────
bool rsdkParseFrame(const uint8_t *data, size_t len, RsdkFrame &out) {
    if (len < RSDK_MIN_FRAME || data[0] != RSDK_SOF) return false;
    const size_t flen = getU16(&data[1]) & 0x03FF;
    if (flen != len) return false;
    if (getU16(&data[10]) != rsdkCrc16(data, 10)) return false;
    if (getU32(&data[len - 4]) != rsdkCrc32(data, len - 4)) return false;
    if (len < RSDK_OVERHEAD) return false;          // no room for CmdSet/CmdID

    out.cmdType    = data[3];
    out.seq        = getU16(&data[8]);
    out.cmdSet     = data[12];
    out.cmdId      = data[13];
    out.payload    = &data[14];
    out.payloadLen = len - RSDK_OVERHEAD;
    return true;
}

void RsdkReassembler::drop(size_t n) {
    if (n >= _len) { _droppedBytes += _len; _len = 0; return; }
    memmove(_buf, _buf + n, _len - n);
    _len -= n;
    _droppedBytes += n;
}

void RsdkReassembler::feed(const uint8_t *data, size_t len,
                           RsdkFrameHandler handler, void *ctx) {
    while (len > 0) {
        size_t room = sizeof(_buf) - _len;
        size_t n = len < room ? len : room;
        memcpy(_buf + _len, data, n);
        _len += n; data += n; len -= n;

        for (;;) {
            // Resync: frames always start with SOF.
            size_t skip = 0;
            while (skip < _len && _buf[skip] != RSDK_SOF) skip++;
            if (skip) drop(skip);
            if (_len < 3) break;

            const size_t flen = getU16(&_buf[1]) & 0x03FF;
            if (flen < RSDK_OVERHEAD) { drop(1); continue; }   // not a real header
            if (_len < RSDK_HEADER_LEN) break;
            if (getU16(&_buf[10]) != rsdkCrc16(_buf, 10)) {    // bad header: resync early
                _crcErrors++; drop(1); continue;
            }
            if (_len < flen) break;                            // wait for the rest

            RsdkFrame f;
            if (rsdkParseFrame(_buf, flen, f)) {
                if (handler) handler(f, ctx);
                // Consume without counting it as dropped.
                memmove(_buf, _buf + flen, _len - flen);
                _len -= flen;
            } else {
                _crcErrors++;
                drop(1);
            }
        }
        // Buffer full of an unfinishable frame: shouldn't happen (a valid
        // length always fits), but never wedge — start over.
        if (_len == sizeof(_buf)) drop(_len);
    }
}

// ── Payloads ────────────────────────────────────────────────────────────────
size_t rsdkBuildConnectRequest(uint8_t *out, uint32_t deviceId, const uint8_t mac[6],
                               uint8_t verifyMode, uint16_t verifyData) {
    memset(out, 0, 33);
    putU32(&out[0], deviceId);
    out[4] = 6;                       // mac_addr_len
    memcpy(&out[5], mac, 6);          // mac_addr[16], rest zero
    putU32(&out[21], 0);              // fw_version: 0 = no firmware-update pushes
    out[25] = 0;                      // conidx (reserved)
    out[26] = verifyMode;
    putU16(&out[27], verifyData);
    // out[29..32] reserved = 0
    return 33;
}

size_t rsdkBuildConnectResponse(uint8_t *out, uint32_t deviceId, uint8_t retCode,
                                uint8_t cameraIndex) {
    putU32(&out[0], deviceId);
    out[4] = retCode;
    out[5] = cameraIndex;
    out[6] = out[7] = out[8] = 0;
    return 9;
}

bool rsdkParseConnectRequest(const uint8_t *p, size_t n, RsdkConnectRequest &out) {
    if (n < 29) return false;
    out.deviceId   = getU32(&p[0]);
    out.verifyMode = p[26];
    out.verifyData = getU16(&p[27]);
    return true;
}

size_t rsdkBuildRecordControl(uint8_t *out, uint32_t deviceId, bool start) {
    putU32(&out[0], deviceId);
    out[4] = start ? 0x00 : 0x01;
    out[5] = out[6] = out[7] = out[8] = 0;
    return 9;
}

size_t rsdkBuildStatusSubscribe(uint8_t *out, uint8_t pushMode, uint8_t pushFreq) {
    out[0] = pushMode;
    out[1] = pushFreq;
    out[2] = out[3] = out[4] = out[5] = 0;
    return 6;
}

bool rsdkParseCameraStatus(const uint8_t *p, size_t n, RsdkCameraStatus &out) {
    if (n < RSDK_STATUS_PUSH_MIN_LEN) return false;
    out.cameraMode       = p[0];
    out.cameraStatus     = p[1];
    out.videoResolution  = p[2];
    out.fpsIdx           = p[3];
    out.eisMode          = p[4];
    out.recordTimeS      = getU16(&p[5]);
    out.remainCapacityMb = getU32(&p[15]);
    out.remainPhotos     = getU32(&p[19]);
    out.remainTimeS      = getU32(&p[23]);
    out.userMode         = p[27];
    out.powerMode        = p[28];
    out.tempOver         = p[30];
    out.batteryPercent   = p[37];
    return true;
}

const char *rsdkModelName(uint32_t deviceId) {
    // Check both halves, each in both byte orders: DJI's demo writes the
    // OA4 id as 0x33FF0000 in one command and 0xFF330000 in another.
    const uint16_t lo = (uint16_t)(deviceId & 0xFFFF), hi = (uint16_t)(deviceId >> 16);
    const uint16_t cands[4] = { lo, hi, (uint16_t)((lo << 8) | (lo >> 8)),
                                (uint16_t)((hi << 8) | (hi >> 8)) };
    for (int i = 0; i < 4; i++) {
        switch (cands[i]) {
            case RSDK_DEVID_ACTION4:    return "Osmo Action 4";
            case RSDK_DEVID_ACTION5PRO: return "Osmo Action 5 Pro";
            case RSDK_DEVID_ACTION6:    return "Osmo Action 6";
            case RSDK_DEVID_OSMO360:    return "Osmo 360";
        }
    }
    return nullptr;
}

uint16_t rsdkFpsFromIdx(uint8_t idx) {
    switch (idx) {
        case 1: return 24;  case 2: return 25;  case 3: return 30;  case 4: return 48;
        case 5: return 50;  case 6: return 60;  case 10: return 100; case 7: return 120;
        case 19: return 200; case 8: return 240;
        default: return 0;
    }
}
