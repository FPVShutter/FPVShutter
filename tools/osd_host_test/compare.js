// Renders a batch of cases with osd_format.cpp (via the compiled driver),
// docs/osd-format.js and the copy embedded in src/web_assets.h, and fails
// on any difference. Run through run.sh.
const fs = require("fs");
const path = require("path");
const { execFileSync } = require("child_process");
const SRC = path.join(__dirname, "../../src"), DOCS = path.join(__dirname, "../../docs");

function load(code) {
  const sandbox = {};
  new Function("globalThis", "window", code)(sandbox, sandbox);
  return sandbox.OsdFormat;
}
const docsCode = fs.readFileSync(path.join(DOCS, "osd-format.js"), "utf8");
const assets = fs.readFileSync(path.join(SRC, "web_assets.h"), "utf8");
const m = assets.match(/\/\*OSDFMT-BEGIN\*\/([\s\S]*?)\/\*OSDFMT-END\*\//);
if (!m) { console.error("FAIL: no OSDFMT block in web_assets.h"); process.exit(1); }
const impls = { docs: load(docsCode), webui: load(m[1]) };
const docsBlock = docsCode.match(/\/\*OSDFMT-BEGIN\*\/([\s\S]*?)\/\*OSDFMT-END\*\//)[1];
if (docsBlock.trim() !== m[1].trim()) console.log("note: web_assets.h copy differs textually from docs/osd-format.js (outputs still compared)");

// Deterministic pseudo-random cases + hand-picked edge cases.
let seed = 12345;
const rnd = (n) => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed % n; };
const pick = (a) => a[rnd(a.length)];
const cases = [];
const labels = ["", "CAM", "REC", "bat", "  x y ", "LONGLABEL", "a\"b\\c", "LINK", "FC"];
for (let i = 0; i < 4000; i++) {
  const link = rnd(5);
  const ready = link === 4 ? rnd(4) !== 0 : false;
  cases.push({
    link, ready, valid: rnd(5) !== 0, cst: rnd(4), batt: pick([-1, 0, 7, 85, 100]),
    recTime: pick([0, 5, 59, 60, 61, 754, 3599, 3600, 3660, 9180, 36000, 65535]),
    fps: pick([0, 24, 30, 50, 60, 120, 240]), temp: rnd(5), freeMb: pick([-1, 0, 850, 1023, 1024, 3400, 10239, 10240, 114688, 243882]),
    fcAlive: rnd(4) !== 0, armed: rnd(2) === 1, vbat10: pick([0, 5, 158, 252]),
    e: [rnd(16), rnd(16), rnd(16), rnd(16)].map((x) => (x === 15 ? 0 : x)),
    mode: pick(["", "VIDEO", "SLOMO", "HLAPSE"]), res: pick(["", "4K", "2.7K", "1080P"]),
    ar: pick(["", "16:9", "4:3"]), eis: pick(["", "OFF", "RS+", "HS"]), l: pick(labels),
  });
}
const line = (c) => [c.link, +c.ready, +c.valid, c.cst, c.batt, c.recTime, c.fps, c.temp, c.freeMb,
  +c.fcAlive, +c.armed, c.vbat10, ...c.e].join(" ") + "|" + [c.mode, c.res, c.ar, c.eis, c.l].join("|");
const cOut = execFileSync(process.argv[2], { input: cases.map(line).join("\n") + "\n" }).toString().trim().split("\n");

let fails = 0;
cases.forEach((c, i) => {
  for (const [name, F] of Object.entries(impls)) {
    const js = "[" + F.slot({ e: c.e, l: c.l }, c).text + "]";
    if (js !== cOut[i]) {
      if (fails++ < 10) console.log(`FAIL ${name} case ${i}: C=${cOut[i]} JS=${js}\n  ${line(c)}`);
    }
  }
  if (cOut[i].length - 2 > 16) { fails++; console.log(`FAIL too long: ${cOut[i]}`); }
});

// Legacy presets (firmware <= v2.3 text) through the JS side.
const F = impls.docs, s = F.SAMPLES;
const expect = [
  [{ e: [1, 3, 2], l: "" }, s.recording.ctx, "REC 85% 12:34"],
  [{ e: [1, 3, 2], l: "" }, s.offline.ctx, "CAM OFF"],
  [{ e: [2], l: "REC" }, s.recording.ctx, "REC 12:34"],
  [{ e: [2], l: "REC" }, s.standby.ctx, "REC 2H33M"],
  [{ e: [3], l: "BAT" }, s.standby.ctx, "BAT 85%"],
  [{ e: [4], l: "LINK" }, s.standby.ctx, "LINK READY"],
  [{ e: [5], l: "FC" }, s.standby.ctx, "FC 15.8V"],
  [{ e: [11, 12, 14], l: "" }, s.hot.ctx, "4K60 RS+ HOT"],
];
for (const [cfg, ctx, want] of expect) {
  const got = F.slot(cfg, ctx).text;
  if (got !== want) { fails++; console.log(`FAIL preset ${JSON.stringify(cfg)}: got "${got}" want "${want}"`); }
}
console.log(fails ? `${fails} FAILURES` : `ALL PASS (${cases.length} random cases x ${Object.keys(impls).length} JS copies + presets)`);
process.exit(fails ? 1 : 0);
