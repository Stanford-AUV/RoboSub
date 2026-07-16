"""Live web dashboard for sensors_plot.

A stdlib ThreadingHTTPServer serves a single self-contained HTML page (no
internet, no CDN -- the sub and laptop are on the tether LAN) that polls
/data.json a few times per second and renders every panel on <canvas>.
Rendering happens in the laptop's browser, so the Orin only pays for
serializing ~window-sized arrays.

Open from the laptop:  http://192.168.2.2:8080
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = 8080

# {panels} and {pinger} placeholders are filled by build_page().
_PAGE = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>RoboSub sensors</title>
<style>
  body { background: #111; color: #ddd; font-family: sans-serif; margin: 8px; }
  #grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px; }
  .panel { background: #1a1a1a; border: 1px solid #333; border-radius: 4px;
           padding: 4px 6px 2px; }
  .panel h3 { margin: 0 0 2px; font-size: 12px; font-weight: normal;
              color: #aaa; white-space: nowrap; overflow: hidden; }
  canvas { width: 100%; height: 170px; display: block; }
  #status { font-size: 15px; color: #cda; margin: 4px 2px; font-weight: bold; }
</style>
</head>
<body>
<div id="status">connecting...</div>
<div id="grid"></div>
<script>
"use strict";
const PANELS = {panels};
const PINGER = {pinger};
const WINDOW_S = 20.0;

const grid = document.getElementById("grid");
const canvases = {};
function addPanel(key, title) {
  const div = document.createElement("div");
  div.className = "panel";
  const h = document.createElement("h3");
  h.textContent = title;
  const c = document.createElement("canvas");
  div.appendChild(h); div.appendChild(c); grid.appendChild(div);
  canvases[key] = c;
}
PANELS.forEach((p, idx) => addPanel(idx, p.title));
addPanel("__pinger__", PINGER.title);

function prepCanvas(c) {
  const r = window.devicePixelRatio || 1;
  const w = c.clientWidth, h = c.clientHeight;
  if (c.width !== w * r || c.height !== h * r) { c.width = w * r; c.height = h * r; }
  const g = c.getContext("2d");
  g.setTransform(r, 0, 0, r, 0, 0);
  g.clearRect(0, 0, w, h);
  return [g, w, h];
}

const M = {l: 42, r: 6, t: 4, b: 16};   // plot margins px

function drawAxes(g, w, h, x0, x1, y0, y1) {
  g.strokeStyle = "#333"; g.fillStyle = "#888";
  g.font = "9px sans-serif"; g.lineWidth = 1;
  g.strokeRect(M.l, M.t, w - M.l - M.r, h - M.t - M.b);
  const ny = 4, nx = 5;
  for (let i = 0; i <= ny; i++) {
    const y = M.t + (h - M.t - M.b) * i / ny;
    const v = y1 - (y1 - y0) * i / ny;
    g.strokeStyle = "#282828";
    g.beginPath(); g.moveTo(M.l, y); g.lineTo(w - M.r, y); g.stroke();
    g.textAlign = "right"; g.textBaseline = "middle";
    g.fillText(fmt(v), M.l - 3, y);
  }
  for (let i = 0; i <= nx; i++) {
    const x = M.l + (w - M.l - M.r) * i / nx;
    const v = x0 + (x1 - x0) * i / nx;
    g.strokeStyle = "#282828";
    g.beginPath(); g.moveTo(x, M.t); g.lineTo(x, h - M.b); g.stroke();
    g.textAlign = "center"; g.textBaseline = "top";
    g.fillText(v.toFixed(0), x, h - M.b + 3);
  }
}

function fmt(v) {
  const a = Math.abs(v);
  if (a >= 1000) return v.toFixed(0);
  if (a >= 10) return v.toFixed(1);
  if (a >= 0.01 || a === 0) return v.toFixed(2);
  return v.toExponential(0);
}

function drawSeries(g, w, h, t, ys, colors, x0, x1, y0, y1, labels) {
  const sx = v => M.l + (v - x0) / (x1 - x0) * (w - M.l - M.r);
  const sy = v => M.t + (y1 - v) / (y1 - y0) * (h - M.t - M.b);
  g.save();
  g.beginPath(); g.rect(M.l, M.t, w - M.l - M.r, h - M.t - M.b); g.clip();
  ys.forEach((arr, k) => {
    if (!arr.length) return;
    g.strokeStyle = colors[k]; g.lineWidth = 1.2; g.beginPath();
    for (let i = 0; i < t.length; i++) {
      const x = sx(t[i]), y = sy(arr[i]);
      i ? g.lineTo(x, y) : g.moveTo(x, y);
    }
    g.stroke();
  });
  g.restore();
  g.font = "9px sans-serif"; g.textAlign = "left"; g.textBaseline = "top";
  labels.forEach((lab, k) => {
    g.fillStyle = colors[k];
    g.fillText(lab, M.l + 4, M.t + 3 + 10 * k);
  });
}

function yRange(ys) {
  let lo = Infinity, hi = -Infinity;
  for (const a of ys) for (const v of a) { if (v < lo) lo = v; if (v > hi) hi = v; }
  if (lo === Infinity) { lo = -1; hi = 1; }
  if (hi - lo < 1e-9) { lo -= 0.5; hi += 0.5; }
  const pad = (hi - lo) * 0.08;
  return [lo - pad, hi + pad];
}

let lastOk = 0;
let busy = false;
async function tick() {
  if (busy) return;   // don't pile up fetches if the link is slow
  busy = true;
  try { await tickInner(); } finally { busy = false; }
}
async function tickInner() {
  let d;
  try {
    const r = await fetch("/data.json", {cache: "no-store"});
    d = await r.json();
    lastOk = Date.now();
    document.getElementById("status").textContent =
      "live - t = " + d.t.toFixed(1) + " s"
      + (d.stage ? "   |   stage: " + d.stage : "");
  } catch (e) {
    if (Date.now() - lastOk > 3000)
      document.getElementById("status").textContent = "no data (node down?)";
    return;
  }
  const x1 = Math.max(d.t, WINDOW_S), x0 = x1 - WINDOW_S;
  PANELS.forEach((p, idx) => {
    const c = canvases[idx];
    const [g, w, h] = prepCanvas(c);
    const t = d.series[p.time] || [];
    const ys = p.series.map(s => d.series[s.key] || []);
    const [y0, y1] = yRange(ys);
    drawAxes(g, w, h, x0, x1, y0, y1);
    drawSeries(g, w, h, t, ys, p.series.map(s => s.color), x0, x1, y0, y1,
               p.series.map(s => s.label));
  });
  // pinger panel: fixed 0..1 scale, threshold line, FRONT/BACK label
  const c = canvases["__pinger__"];
  const [g, w, h] = prepCanvas(c);
  const t = d.series[PINGER.time] || [];
  const y0 = -0.05, y1 = 1.05;
  drawAxes(g, w, h, x0, x1, y0, y1);
  drawSeries(g, w, h, t, d.pinger_levels, PINGER.colors, x0, x1, y0, y1,
             PINGER.labels);
  const ty = M.t + (y1 - PINGER.threshold) / (y1 - y0) * (h - M.t - M.b);
  g.strokeStyle = "#999"; g.setLineDash([4, 3]);
  g.beginPath(); g.moveTo(M.l, ty); g.lineTo(w - M.r, ty); g.stroke();
  g.setLineDash([]);
  if (d.front !== null) {
    g.font = "bold 26px sans-serif"; g.textAlign = "left";
    g.textBaseline = "bottom";
    g.fillStyle = d.front ? "#4c4" : "#e55";
    g.fillText(d.front ? "FRONT" : "BACK", M.l + 6, h - M.b - 4);
  }
}
setInterval(tick, 200);
tick();
</script>
</body>
</html>
"""


def build_page(panels, pinger):
    """Fill the page template. panels/pinger are JSON-able specs."""
    return (_PAGE
            .replace("{panels}", json.dumps(panels))
            .replace("{pinger}", json.dumps(pinger)))


def start_server(page, get_data, log):
    """Serve `page` at / and get_data() (a dict) at /data.json, in a
    daemon thread. Returns the server (never raises into the caller's
    thread; bind errors are logged)."""

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ("/", "/index.html"):
                body = page.encode()
                ctype = "text/html; charset=utf-8"
            elif self.path.startswith("/data.json"):
                body = json.dumps(get_data()).encode()
                ctype = "application/json"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):  # quiet
            pass

    try:
        server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    except OSError as e:
        log(f"dashboard server failed to bind :{PORT}: {e}")
        return None
    threading.Thread(target=server.serve_forever, daemon=True).start()
    log(f"live dashboard on http://0.0.0.0:{PORT} "
        f"(from the laptop: http://192.168.2.2:{PORT})")
    return server
