// ============================================================================
// osd-format.js — Browser mirror of src/osd_format.cpp
// ============================================================================
// Renders Custom Message text client-side, so the OSD editor can preview a
// layout before it's saved (and without a camera, via the sample states).
// MUST produce byte-identical text to osd_format.cpp — tools/osd_host_test
// runs both over the same cases. The on-device Web UI (src/web_assets.h)
// embeds a copy of this file between OSDFMT-BEGIN / OSDFMT-END markers; the
// host test checks that copy too, so update both together.
// ============================================================================
/*OSDFMT-BEGIN*/
(function (root) {
  "use strict";
  var MAX_LEN = 16, ELEMS_PER_SLOT = 4, LABEL_MAX = 6;
  // BLE link states (camera_common.h BleConnectionState order)
  var L_OFF = 0, L_SCAN = 1, L_CONN = 2, L_PAIR = 3, L_READY = 4;
  // CameraRecordingState
  var S_STBY = 1, S_REC = 2, S_ERR = 3;

  // id -> element. Ids match OsdElement in settings.h (never renumber).
  // cam: camera-data element (hidden while the camera is down when the slot
  // also shows CAM STATE / LINK). eg: example text for the picker.
  var ELEMENTS = [
    { id: 0,  key: "none",   name: "(empty)" },
    { id: 1,  key: "state",  name: "Camera state",    eg: "REC / STBY / CAM OFF" },
    { id: 2,  key: "time",   name: "Record time",     eg: "12:34 / 2H33M left", cam: 1 },
    { id: 3,  key: "batt",   name: "Camera battery",  eg: "85%", cam: 1 },
    { id: 4,  key: "link",   name: "Link state",      eg: "READY / SCAN / OFF" },
    { id: 5,  key: "fcv",    name: "FC voltage",      eg: "15.8V" },
    { id: 6,  key: "arm",    name: "Arm state",       eg: "ARMED / DISARMED" },
    { id: 7,  key: "mode",   name: "Camera mode",     eg: "VIDEO / SLOMO / HLAPSE", cam: 1 },
    { id: 8,  key: "res",    name: "Resolution",      eg: "4K / 2.7K / 1080P", cam: 1 },
    { id: 9,  key: "ar",     name: "Aspect ratio",    eg: "16:9 / 4:3", cam: 1 },
    { id: 10, key: "fps",    name: "Frame rate",      eg: "60FPS", cam: 1 },
    { id: 11, key: "fmt",    name: "Res + fps",       eg: "4K60 / 2.7K50", cam: 1 },
    { id: 12, key: "eis",    name: "Stabilisation",   eg: "RS+ / HS / EIS OFF", cam: 1 },
    { id: 13, key: "sd",     name: "Card free",       eg: "112G / 850M", cam: 1 },
    { id: 14, key: "temp",   name: "Overheat alert",  eg: "HOT (blank when OK)", cam: 1 }
  ];

  function pad2(n) { return (n < 10 ? "0" : "") + n; }

  function human(s) {
    s = Math.max(0, s | 0);
    if (s < 60) return s + "S";
    var h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60);
    if (h > 0) return m > 0 ? h + "H" + m + "M" : h + "H";
    return m + "M";
  }

  function sanitizeLabel(str) {
    var out = "";
    str = String(str == null ? "" : str);
    for (var i = 0; i < str.length && out.length < LABEL_MAX; i++) {
      var c = str.charCodeAt(i);
      if (c >= 97 && c <= 122) c -= 32;
      if (c < 0x20 || c > 0x5f || c === 34 || c === 92) continue;
      out += String.fromCharCode(c);
    }
    return out.replace(/^ +| +$/g, "");
  }

  // ctx: { link, ready, valid, cst, batt, recTime, mode, res, ar, fps, eis,
  //        temp, freeMb, fcAlive, armed, vbat10 }
  function element(id, c, hideCam) {
    var el = ELEMENTS[id];
    if (!el || id === 0) return "";
    if (!c.ready && hideCam && el.cam) return "";
    switch (id) {
      case 1:
        if (c.link === L_OFF) return "CAM OFF";
        if (c.link === L_SCAN) return "CAM SCAN";
        if (c.link === L_CONN) return "CAM CONN";
        if (c.link === L_PAIR) return "CAM PAIR";
        if (!c.ready) return "CAM PAIR";
        if (c.cst === S_REC) return "REC";
        if (c.cst === S_STBY) return "STBY";
        if (c.cst === S_ERR) return "CAM ERR";
        return "CAM ???";
      case 2:
        if (!c.ready || !c.valid) return "--:--";
        var t = c.recTime | 0;
        return c.cst === S_REC ? pad2(Math.floor(t / 60)) + ":" + pad2(t % 60) : human(t);
      case 3:
        return (c.ready && c.batt >= 0 && c.batt <= 100) ? c.batt + "%" : "--%";
      case 4:
        return ["OFF", "SCAN", "CONN", "PAIR", "READY"][c.link] || "OFF";
      case 5:
        return c.vbat10 > 0 ? Math.floor(c.vbat10 / 10) + "." + (c.vbat10 % 10) + "V" : "--.-V";
      case 6:
        return !c.fcAlive ? "FC NOLINK" : (c.armed ? "ARMED" : "DISARMED");
      case 7:  return (c.ready && c.mode) ? c.mode : "--";
      case 8:  return (c.ready && c.res) ? c.res : "--";
      case 9:  return (c.ready && c.ar) ? c.ar : "--";
      case 10: return (c.ready && c.fps) ? c.fps + "FPS" : "--FPS";
      case 11: return (c.ready && c.res && c.fps) ? c.res + c.fps : "--";
      case 12:
        if (!c.ready || !c.eis) return "--";
        return c.eis === "OFF" ? "EIS OFF" : c.eis;
      case 13:
        var mb = c.freeMb;
        if (!c.ready || mb == null || mb < 0) return "--";
        if (mb < 1024) return mb + "M";
        if (mb < 10240) { var tn = Math.floor((mb * 10 + 512) / 1024); return Math.floor(tn / 10) + "." + (tn % 10) + "G"; }
        return Math.floor((mb + 512) / 1024) + "G";
      case 14:
        if (!c.ready) return "";
        return ["", "WARM", "HOT", "OVERHEAT"][c.temp] || "";
    }
    return "";
  }

  // slot: { e: [ids], l: "LABEL" } -> { text, dropped: [ids that didn't fit] }
  function slot(cfg, c) {
    var ids = (cfg && cfg.e) || [], label = sanitizeLabel(cfg && cfg.l);
    var hideCam = ids.indexOf(1) >= 0 || ids.indexOf(4) >= 0;
    var out = "", dropped = [], full = false;
    function add(part) {
      if (!part) return true;
      var need = part.length + (out ? 1 : 0);
      if (out.length + need > MAX_LEN) return false;
      out += (out ? " " : "") + part;
      return true;
    }
    add(label);
    for (var i = 0; i < ids.length && i < ELEMS_PER_SLOT; i++) {
      var id = ids[i];
      if (!id || !ELEMENTS[id]) continue;
      if (full) { dropped.push(id); continue; }
      if (!add(element(id, c, hideCam))) { full = true; dropped.push(id); }
    }
    return { text: out, dropped: dropped };
  }

  // Status JSON (/api/status or the serial "status" reply) -> ctx.
  function ctxFromStatus(st) {
    var cam = (st && st.cam) || {}, fc = (st && st.fc) || {};
    var link = typeof cam.state === "number" ? cam.state : L_OFF;
    var ready = typeof cam.ready === "boolean" ? cam.ready : link === L_READY;
    var cst = typeof cam.cst === "number" ? cam.cst : (cam.recording ? S_REC : (cam.valid ? S_STBY : 0));
    return {
      link: link, ready: ready, valid: !!cam.valid, cst: cst,
      batt: typeof cam.batt === "number" ? cam.batt : -1, recTime: cam.recTime | 0,
      mode: cam.mode || "", res: cam.res || "", ar: cam.ar || "", fps: cam.fps | 0,
      eis: cam.eis || "", temp: cam.temp | 0,
      freeMb: typeof cam.freeMb === "number" ? cam.freeMb : -1,
      fcAlive: !!fc.alive, armed: !!fc.armed, vbat10: fc.vbat10 | 0
    };
  }

  // Canned states for previewing without (or regardless of) a camera.
  var base = { link: L_READY, ready: true, valid: true, batt: 85, mode: "VIDEO", res: "4K",
               ar: "16:9", fps: 60, eis: "RS+", temp: 0, freeMb: 114688,
               fcAlive: true, vbat10: 158 };
  function mk(o) { var r = {}, k; for (k in base) r[k] = base[k]; for (k in o) r[k] = o[k]; return r; }
  var SAMPLES = {
    recording: { name: "Sample: recording, armed", ctx: mk({ cst: S_REC, recTime: 754, armed: true }) },
    standby:   { name: "Sample: standby",          ctx: mk({ cst: S_STBY, recTime: 9180, armed: false }) },
    hot:       { name: "Sample: recording, hot",   ctx: mk({ cst: S_REC, recTime: 1392, armed: true, temp: 2, batt: 23, freeMb: 3400 }) },
    offline:   { name: "Sample: camera off",       ctx: mk({ link: L_OFF, ready: false, valid: false, cst: 0, armed: false }) }
  };

  // Bit n of the firmware's "osdSup" = element n is filled by this camera.
  // Fallback for firmware that doesn't send it: everything "supported".
  function supported(st, id) {
    if (!st || typeof st.osdSup !== "number") return true;
    return id === 0 || ((st.osdSup >>> id) & 1) === 1;
  }

  root.OsdFormat = {
    MAX_LEN: MAX_LEN, ELEMS_PER_SLOT: ELEMS_PER_SLOT, LABEL_MAX: LABEL_MAX,
    ELEMENTS: ELEMENTS, SAMPLES: SAMPLES,
    element: element, slot: slot, human: human, sanitizeLabel: sanitizeLabel,
    ctxFromStatus: ctxFromStatus, supported: supported
  };
})(typeof window !== "undefined" ? window : globalThis);
/*OSDFMT-END*/
