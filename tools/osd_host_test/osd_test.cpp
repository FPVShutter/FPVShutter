// Host test driver for src/osd_format.cpp. Reads one case per line on stdin:
//   link ready valid cst batt recTime fps temp freeMb fcAlive armed vbat10 e0 e1 e2 e3 |mode|res|ar|eis|label
// and prints the rendered slot text per line, wrapped in [] so trailing
// spaces would show. Driven by compare.js, which renders the same cases
// with docs/osd-format.js and the copy embedded in src/web_assets.h.
#include "osd_format.h"
#include <iostream>
#include <sstream>
#include <string>
unsigned long g_millis = 0;

int main() {
    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;
        size_t bar = line.find('|');
        std::istringstream in(line.substr(0, bar));
        long v[16];
        for (int i = 0; i < 16; i++) in >> v[i];
        std::string rest = line.substr(bar + 1), f[5];
        for (int i = 0; i < 5; i++) {
            size_t p = rest.find('|');
            f[i] = rest.substr(0, p);
            rest = p == std::string::npos ? "" : rest.substr(p + 1);
        }
        CameraTelemetry tel;
        tel.dataValid = v[2];
        tel.state = (CameraRecordingState)v[3];
        tel.batteryPercent = v[4] < 0 ? 255 : (uint8_t)v[4];
        tel.recTimeSeconds = (uint16_t)v[5];
        tel.fps = (uint16_t)v[6];
        tel.tempState = (uint8_t)v[7];
        tel.freeMb = v[8] < 0 ? CAM_FREE_MB_UNKNOWN : (uint32_t)v[8];
        strlcpy(tel.modeLabel, f[0].c_str(), sizeof(tel.modeLabel));
        strlcpy(tel.resLabel, f[1].c_str(), sizeof(tel.resLabel));
        strlcpy(tel.aspectLabel, f[2].c_str(), sizeof(tel.aspectLabel));
        strlcpy(tel.eisLabel, f[3].c_str(), sizeof(tel.eisLabel));
        OsdContext ctx{(BleConnectionState)v[0], v[1] != 0, &tel, v[9] != 0, v[10] != 0, (uint16_t)v[11]};
        OsdSlotConfig sc{};
        for (int i = 0; i < 4; i++) sc.elem[i] = (uint8_t)v[12 + i];
        char lbl[64];
        strlcpy(lbl, f[4].c_str(), sizeof(lbl));
        osdSanitizeLabel(lbl);
        strlcpy(sc.label, lbl, sizeof(sc.label));
        char out[OSD_MAX_TEXT_LEN + 1];
        osdFormatSlot(sc, ctx, out, sizeof(out));
        std::cout << "[" << out << "]\n";
    }
    // Legacy presets must keep rendering what firmware <= v2.3 did.
    return 0;
}
