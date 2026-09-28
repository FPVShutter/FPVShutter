// ============================================================================
// settings.cpp — NVS-backed persistent configuration
// ============================================================================

#include "settings.h"
#include "cam_registry.h"
#include <Preferences.h>

static ShutterSettings _s;
static Preferences _prefs;

static void applyDefaults() {
    _s.camera              = (CameraType)DEFAULT_CAMERA_TYPE;
    _s.auxChannelIndex     = DEFAULT_AUX_CHANNEL_INDEX;
    _s.rcThresholdUs       = DEFAULT_RC_THRESHOLD_US;
    _s.debounceMs          = DEFAULT_RC_DEBOUNCE_MS;
    _s.recordOnArm         = DEFAULT_RECORD_ON_ARM;
    _s.stopOnDisarm        = DEFAULT_STOP_ON_DISARM;
    _s.stopOnDisarmDelayMs = DEFAULT_STOP_ON_DISARM_DELAY_MS;   // Default Grace Period is 0ms, so instant stop of recording like the original behaviour
    _s.scanAll             = DEFAULT_SCAN_ALL;
    _s.wifiSwitchCh        = DEFAULT_WIFI_SWITCH_CH;
    _s.wifiApEnabled       = DEFAULT_WIFI_AP_ENABLED;
    _s.blePower            = (BlePowerLevel)DEFAULT_BLE_POWER;

    strlcpy(_s.apSsid, WIFI_AP_DEFAULT_SSID, sizeof(_s.apSsid));
    strlcpy(_s.apPass, WIFI_AP_DEFAULT_PASS, sizeof(_s.apPass));

    static const uint8_t kDefaultSlots[4] =
        { DEFAULT_OSD_SLOT_1, DEFAULT_OSD_SLOT_2, DEFAULT_OSD_SLOT_3, DEFAULT_OSD_SLOT_4 };
    for (int i = 0; i < 4; i++) {
        osdSlotFromLegacy(kDefaultSlots[i], _s.osd[i]);
    }
    _s.camCount = 0;
    memset(_s.cams, 0, sizeof(_s.cams));
}

void settingsLoad() {
    applyDefaults();

    if (!_prefs.begin("fpvshutter", false)) {
        DBG("SETTINGS: NVS open failed — using defaults");
        return;
    }

    _s.camera              = (CameraType)_prefs.getUChar("camera", _s.camera);
    _s.auxChannelIndex     = _prefs.getUChar("auxCh", _s.auxChannelIndex);
    _s.rcThresholdUs       = _prefs.getUShort("thr", _s.rcThresholdUs);
    _s.debounceMs          = _prefs.getUShort("deb", _s.debounceMs);
    _s.recordOnArm         = _prefs.getBool("roa", _s.recordOnArm);
    _s.stopOnDisarm        = _prefs.getBool("sod", _s.stopOnDisarm);
    _s.stopOnDisarmDelayMs = _prefs.getUShort("sodDelay", _s.stopOnDisarmDelayMs);
    _s.scanAll             = _prefs.getBool("scanAll", _s.scanAll);
    _s.wifiSwitchCh        = _prefs.getUChar("wifiCh", _s.wifiSwitchCh);
    _s.wifiApEnabled       = _prefs.getBool("apEn", _s.wifiApEnabled);
    _s.blePower            = (BlePowerLevel)_prefs.getUChar("blePwr", _s.blePower);

    char buf[65] = {0};
    if (_prefs.getString("ssid", buf, sizeof(buf)) > 0) strlcpy(_s.apSsid, buf, sizeof(_s.apSsid));
    buf[0] = 0;
    if (_prefs.getString("apkey", buf, sizeof(buf)) > 0) strlcpy(_s.apPass, buf, sizeof(_s.apPass));

    // OSD layout: "osdN" = element bytes, "osdlN" = label. Firmware <= v2.3
    // stored one OsdSlotContent byte as "slotN" instead — migrate that the
    // first time (the old key is left in place, so a downgrade still works).
    for (int i = 0; i < 4; i++) {
        char key[8], lkey[8];
        snprintf(key,  sizeof(key),  "osd%d",  i);
        snprintf(lkey, sizeof(lkey), "osdl%d", i);
        if (_prefs.isKey(key)) {
            uint8_t el[OSD_ELEMS_PER_SLOT] = {0};
            _prefs.getBytes(key, el, sizeof(el));
            for (int e = 0; e < OSD_ELEMS_PER_SLOT; e++) {
                _s.osd[i].elem[e] = el[e] < OSD_EL_COUNT ? el[e] : OSD_EL_NONE;
            }
            _s.osd[i].label[0] = '\0';
            _prefs.getString(lkey, _s.osd[i].label, sizeof(_s.osd[i].label));
            osdSanitizeLabel(_s.osd[i].label);
        } else {
            char legacyKey[8];
            snprintf(legacyKey, sizeof(legacyKey), "slot%d", i);
            if (_prefs.isKey(legacyKey)) {
                uint8_t v = _prefs.getUChar(legacyKey, OSD_SLOT_OFF);
                if (v < OSD_SLOT_COUNT) osdSlotFromLegacy(v, _s.osd[i]);
            }
        }
    }

    // Sanity clamps
    if (_s.auxChannelIndex > 15)            _s.auxChannelIndex = 15;
    if (_s.rcThresholdUs < 1200)            _s.rcThresholdUs = 1200;
    if (_s.rcThresholdUs > 1800)            _s.rcThresholdUs = 1800;
    if (_s.debounceMs < 50)                 _s.debounceMs = 50;
    if (_s.debounceMs > 1000)               _s.debounceMs = 1000;
    if (_s.stopOnDisarmDelayMs > 15000)     _s.stopOnDisarmDelayMs = 15000;
    if (_s.wifiSwitchCh > 15 && _s.wifiSwitchCh != 255)
                                            _s.wifiSwitchCh = 255;
    if (_s.blePower > BLE_POWER_HIGH)       _s.blePower = BLE_POWER_HIGH;

    _prefs.end();

    // Saved camera registry lives in the same NVS namespace.
    camRegistryLoad();

    DBG("SETTINGS: Loaded (cam=%u aux=%u thr=%u deb=%u roa=%d apEn=%d blePwr=%s)",
        _s.camera, _s.auxChannelIndex, _s.rcThresholdUs, _s.debounceMs, _s.recordOnArm,
        _s.wifiApEnabled, blePowerName(_s.blePower));
}

void settingsSave() {
    if (!_prefs.begin("fpvshutter", false)) {
        DBG("SETTINGS: NVS open failed — not saved");
        return;
    }
    _prefs.putUChar("camera", _s.camera);
    _prefs.putUChar("auxCh", _s.auxChannelIndex);
    _prefs.putUShort("thr", _s.rcThresholdUs);
    _prefs.putUShort("deb", _s.debounceMs);
    _prefs.putBool("roa", _s.recordOnArm);
    _prefs.putBool("sod", _s.stopOnDisarm);
    _prefs.putUShort("sodDelay", _s.stopOnDisarmDelayMs);
    _prefs.putBool("scanAll", _s.scanAll);
    _prefs.putUChar("wifiCh", _s.wifiSwitchCh);
    _prefs.putBool("apEn", _s.wifiApEnabled);
    _prefs.putUChar("blePwr", _s.blePower);
    _prefs.putString("ssid", _s.apSsid);
    _prefs.putString("apkey", _s.apPass);

    for (int i = 0; i < 4; i++) {
        char key[8], lkey[8];
        snprintf(key,  sizeof(key),  "osd%d",  i);
        snprintf(lkey, sizeof(lkey), "osdl%d", i);
        _prefs.putBytes(key, _s.osd[i].elem, OSD_ELEMS_PER_SLOT);
        _prefs.putString(lkey, _s.osd[i].label);
    }

    _prefs.end();
    DBG("SETTINGS: Saved");
}

void settingsReset() {
    applyDefaults();
}

ShutterSettings& settingsGet() {
    return _s;
}

const char* cameraTypeName(CameraType type) {
    switch (type) {
        case CAMERA_GOPRO:      return "GoPro";
        case CAMERA_DJI_ACTION: return "DJI Osmo Action";
        case CAMERA_DJI_RSDK:   return "DJI Osmo Action 4+";
        case CAMERA_DJI_NANO:
        default:                return "DJI Osmo Nano";
    }
}

const char* blePowerName(BlePowerLevel level) {
    switch (level) {
        case BLE_POWER_LOW:    return "Low";
        case BLE_POWER_MEDIUM: return "Medium";
        case BLE_POWER_HIGH:
        default:                return "High";
    }
}
