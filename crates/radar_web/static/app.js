/* ESP32-RADAR compact sensor head — live dashboard.
 *
 * Two data paths (see crates/radar_web/src/telemetry.rs for the wire format):
 *   1. WebSocket /ws  — binary telemetry frames:
 *        0x01 StatusFrame      (66 B, all LE)
 *        0x02 WaterfallFrame   (11-B header + n_sub*bins u8, time-major)
 *        0x03 SpectrogramFrame (11-B header + n_freq*bins u8, time-major)
 *   2. HTTP /status  — compact JSON StatusSnapshot, polled every 2 s for the
 *      fields the binary frame doesn't carry (channel, tx_rate, cal_stage, ...).
 *
 * The WS path drives the fast charts; the JSON path fills the header tiles and
 * acts as a fallback if WebSockets are unavailable.
 */
(() => {
  'use strict';

  /* ------------------------------------------------------------------ *
   * constants — must match telemetry.rs exactly                         *
   * ------------------------------------------------------------------ */
  const MAGIC = 0x52544D31;          // 'RTM1'
  const VERSION = 1;
  const K_STATUS = 0x01;
  const K_WATERFALL = 0x02;
  const K_SPECTROGRAM = 0x03;
  const LINK_RX1 = 1, LINK_RX2 = 2, LINK_FUSED = 3;

  const OCC_NAMES = [
    'UNKNOWN', 'EMPTY', 'POSSIBLE PRESENCE', 'STATIC PRESENCE',
    'MOVEMENT', 'STRONG MOVEMENT', 'COMPLEX/MULTIPLE MOVEMENT',
  ];
  // occupancy code → dashboard accent colour
  const OCC_COLORS = ['#8b949e', '#3fb950', '#d29922', '#58a6ff',
    '#f0883e', '#f85149', '#bc4c9c'];

  const H = (el) => document.getElementById(el);

  /* ------------------------------------------------------------------ *
   * colormap (thermal: black → blue → green → yellow → white)           *
   * ------------------------------------------------------------------ */
  const CMAP = (() => {
    const lut = new Uint8Array(256 * 4);
    for (let i = 0; i < 256; i++) {
      const t = i / 255;
      let r, g, b;
      if (t < 0.25) { r = 0; g = 0; b = 90 + t * 660; }
      else if (t < 0.5) { const u = (t - 0.25) * 4; r = 0; g = 90 * u; b = 255 - 165 * u; }
      else if (t < 0.75) { const u = (t - 0.5) * 4; r = 90 * u; g = 255 - 140 * u; b = 255 - 165 * u; }
      else { const u = (t - 0.75) * 4; r = 90 + 165 * u; g = 115 - 115 * u; b = 90 + 165 * u; }
      lut[i * 4] = Math.max(0, Math.min(255, r | 0));
      lut[i * 4 + 1] = Math.max(0, Math.min(255, g | 0));
      lut[i * 4 + 2] = Math.max(0, Math.min(255, b | 0));
      lut[i * 4 + 3] = 255;
    }
    return lut;
  })();

  /* ------------------------------------------------------------------ *
   * state                                                               *
   * ------------------------------------------------------------------ */
  const ROLL_ROWS = 160;             // waterfall/spectrogram history depth
  const ME_WINDOW = 240;             // motion-energy samples kept

  const wf = { [LINK_RX1]: { rows: [], n: 56, scale: 0 }, [LINK_RX2]: { rows: [], n: 56, scale: 0 } };
  const sp = { [LINK_RX1]: { rows: [], n: 32, scale: 0 }, [LINK_RX2]: { rows: [], n: 32, scale: 0 }, [LINK_FUSED]: { rows: [], n: 32, scale: 0 } };
  const me = { t: [], r1: [], r2: [], f: [] };

  let status = {                    // merged binary-frame + JSON snapshot
    radar_active: false, channel: 0, tx_rate_hz: 0,
    cal_stage: 0, cal_active: false, cal_stage_name: '',
    occupancy: 0, confidence: 0, tx_power_db: 0,
    rssi_rx1: 0, rssi_rx2: 0, csi_quality_rx1: 0, csi_quality_rx2: 0,
    sat_rx1: 0, sat_rx2: 0, dyn_rx1: 0, dyn_rx2: 0,
    packet_delivery_pct: 0, paired_frames_s: 0, seq: 0,
    motion_energy_rx1: 0, motion_energy_rx2: 0, motion_energy_fused: 0,
    spectral_entropy: 0, dominant_freq_hz: 0, pca1: 0, pca2: 0,
    correlation: 0, differential: 0,
  };
  let connUp = false;
  let ws = null;

  const CAL_STAGE_NAMES = ['', 'IDENTITY', 'RF POWER', 'BASELINE',
    'MOVING TEST', 'FINGERPRINT'];

  /* ------------------------------------------------------------------ *
   * binary decoders                                                     *
   * ------------------------------------------------------------------ */
  function decodeStatus(dv) {
    const s = {
      occupancy: dv.getUint8(6),
      confidence: dv.getUint8(7),
      tx_power_db: dv.getInt8(8),
      rssi_rx1: dv.getInt8(9),
      rssi_rx2: dv.getInt8(10),
      csi_quality_rx1: dv.getUint8(11),
      csi_quality_rx2: dv.getUint8(12),
      sat_rx1: dv.getUint8(13),
      sat_rx2: dv.getUint8(14),
      dyn_rx1: dv.getUint8(15),
      dyn_rx2: dv.getUint8(16),
      packet_delivery_pct: dv.getUint8(17),
      paired_frames_s: dv.getUint16(18, true),
      seq: dv.getUint32(20, true),
      t_us: Number(dv.getBigUint64(24, true)),
      motion_energy_rx1: dv.getFloat32(32, true),
      motion_energy_rx2: dv.getFloat32(36, true),
      motion_energy_fused: dv.getFloat32(40, true),
      spectral_entropy: dv.getFloat32(44, true),
      dominant_freq_hz: dv.getUint16(48, true),
      pca1: dv.getFloat32(50, true),
      pca2: dv.getFloat32(54, true),
      correlation: dv.getFloat32(58, true),
      differential: dv.getFloat32(62, true),
    };
    Object.assign(status, s);
    pushMotionEnergy(s);
    updateStatusUI();
  }

  function decodeMatrix(dv, kind) {
    // 11-byte header: magic u32 | version u8 | kind u8 | link u8 |
    // n u8 | bins u16 | scale u8
    const link = dv.getUint8(6);
    const n = dv.getUint8(7);
    const bins = dv.getUint16(8, true);
    const scale = dv.getUint8(10);
    const data = new Uint8Array(dv.buffer, dv.byteOffset + 11, n * bins);
    const store = kind === K_WATERFALL ? wf : sp;
    const buf = store[link];
    if (!buf) return;
    buf.n = n;
    buf.scale = scale;
    // time-major: each time bin is `n` consecutive bytes
    for (let row = 0; row < bins; row++) {
      buf.rows.push(data.slice(row * n, row * n + n));
      if (buf.rows.length > ROLL_ROWS) buf.rows.shift();
    }
  }

  function handleFrame(dv) {
    if (dv.byteLength < 6) return;
    if (dv.getUint32(0, true) !== MAGIC) return;
    if (dv.getUint8(4) !== VERSION) return;
    const kind = dv.getUint8(5);
    if (kind === K_STATUS) decodeStatus(dv);
    else if (kind === K_WATERFALL) decodeMatrix(dv, K_WATERFALL);
    else if (kind === K_SPECTROGRAM) decodeMatrix(dv, K_SPECTROGRAM);
  }

  /* ------------------------------------------------------------------ *
   * motion energy rolling series                                        *
   * ------------------------------------------------------------------ */
  function pushMotionEnergy(s) {
    const t = s.t_us / 1e6;
    me.t.push(t); me.r1.push(s.motion_energy_rx1);
    me.r2.push(s.motion_energy_rx2); me.f.push(s.motion_energy_fused);
    if (me.t.length > ME_WINDOW) {
      me.t.shift(); me.r1.shift(); me.r2.shift(); me.f.shift();
    }
  }

  /* ------------------------------------------------------------------ *
   * WebSocket                                                           *
   * ------------------------------------------------------------------ */
  function connectWS() {
    try {
      const proto = location.protocol === 'https:' ? 'wss' : 'ws';
      ws = new WebSocket(`${proto}://${location.host}/ws`);
      ws.binaryType = 'arraybuffer';
      ws.onopen = () => { connUp = true; setConnUI(); };
      ws.onmessage = (e) => {
        if (e.data instanceof ArrayBuffer) handleFrame(new DataView(e.data));
      };
      ws.onclose = () => { connUp = false; setConnUI(); setTimeout(connectWS, 1500); };
      ws.onerror = () => { try { ws.close(); } catch (_) {} };
    } catch (_) {
      setTimeout(connectWS, 3000);
    }
  }

  function setConnUI() {
    const pill = H('conn');
    pill.textContent = connUp ? 'WS LIVE' : 'WS —';
    pill.className = 'pill ' + (connUp ? 'radar-on' : 'radar-off');
  }

  /* ------------------------------------------------------------------ *
   * /status JSON poll (header fields + fallback)                        *
   * ------------------------------------------------------------------ */
  async function pollStatus() {
    try {
      const r = await fetch('/status');
      const j = await r.json();
      Object.assign(status, j);
      status.occupancy = j.occupancy_code;
      status.cal_stage_name = CAL_STAGE_NAMES[j.cal_stage] || '';
      // if WS is down, let the JSON drive the motion-energy plot too
      if (!connUp) { pushMotionEnergy(status); }
      updateStatusUI();
    } catch (_) { /* server unreachable */ }
    setTimeout(pollStatus, 2000);
  }

  /* ------------------------------------------------------------------ *
   * UI builders                                                         *
   * ------------------------------------------------------------------ */
  function buildStatusGrid() {
    const grid = H('statusGrid');
    const defs = [
      ['RADAR', s => s.radar_active ? 'ON' : 'OFF'],
      ['CHANNEL', s => s.channel || '—'],
      ['TX RATE', s => s.tx_rate_hz ? s.tx_rate_hz + ' Hz' : '—'],
      ['TX POWER', s => s.tx_power_db + ' dBm'],
      ['RX1 RSSI', s => s.rssi_rx1 + ' dBm'],
      ['RX2 RSSI', s => s.rssi_rx2 + ' dBm'],
      ['RX1 QUALITY', s => s.csi_quality_rx1 + '%'],
      ['RX2 QUALITY', s => s.csi_quality_rx2 + '%'],
      ['PKT DELIVERY', s => s.packet_delivery_pct + '%'],
      ['PAIRED FPS', s => s.paired_frames_s],
      ['RX1 SAT/DYN', s => s.sat_rx1 + '/' + s.dyn_rx1],
      ['RX2 SAT/DYN', s => s.sat_rx2 + '/' + s.dyn_rx2],
      ['CALIBRATION', s => s.cal_active ? (s.cal_stage_name || 'stage ' + s.cal_stage) : (s.radar_active ? 'done' : 'off')],
      ['SEQ', s => s.seq],
    ];
    for (const [k, f] of defs) {
      const d = document.createElement('div');
      d.className = 'stat';
      d.innerHTML = `<div class="k">${k}</div><div class="v" id="st-${k}">—</div>`;
      grid.appendChild(d);
    }
    grid._defs = defs;
  }

  function buildDiffGrid() {
    const grid = H('diffGrid');
    const defs = [
      ['PCA1', s => s.pca1.toFixed(2)],
      ['PCA2', s => s.pca2.toFixed(2)],
      ['CROSS-LINK CORR', s => s.correlation.toFixed(3)],
      ['DIFFERENTIAL RMS', s => s.differential.toFixed(3)],
      ['SPECTRAL ENTROPY', s => s.spectral_entropy.toFixed(3)],
      ['DOMINANT FREQ', s => s.dominant_freq_hz + ' Hz'],
    ];
    for (const [k, f] of defs) {
      const d = document.createElement('div');
      d.className = 'diff';
      d.innerHTML = `<div class="k">${k}</div><div class="v" id="df-${k}">—</div>`;
      grid.appendChild(d);
      d._get = f;
    }
    grid._defs = defs;
  }

  function updateStatusUI() {
    // header pills
    const rp = H('radarPill');
    const cp = H('calPill');
    if (status.cal_active) {
      rp.className = 'pill radar-cal'; rp.textContent = 'CALIBRATING';
      cp.hidden = false;
      cp.textContent = (status.cal_stage_name || 'stage ' + status.cal_stage);
      cp.className = 'pill radar-cal';
    } else {
      rp.className = 'pill ' + (status.radar_active ? 'radar-on' : 'radar-off');
      rp.textContent = status.radar_active ? 'RADAR ACTIVE' : 'RADAR OFFLINE';
      cp.hidden = true;
    }
    H('footCh').textContent = status.channel || '—';

    // status grid
    const grid = H('statusGrid');
    if (grid._defs) {
      for (const [k, f] of grid._defs) {
        const n = H('st-' + k);
        if (n) n.textContent = f(status);
      }
    }

    // differential tiles
    const dg = H('diffGrid');
    if (dg._defs) {
      for (const [k, f] of dg._defs) {
        const n = H('df-' + k);
        if (n) n.textContent = f(status);
      }
    }

    // occupancy
    const occ = status.occupancy & 7;
    H('occState').textContent = OCC_NAMES[occ] || 'UNKNOWN';
    H('occState').style.color = OCC_COLORS[occ] || '#8b949e';
    H('occBar').style.width = status.confidence + '%';
    H('occBar').style.background = OCC_COLORS[occ] || 'var(--accent)';
    H('occConf').textContent = 'conf ' + status.confidence + '%';
    H('occHint').textContent = occHint(occ);
  }

  function occHint(occ) {
    switch (occ) {
      case 0: return 'waiting for link data';
      case 1: return 'room empty';
      case 2: return 'uncertain signal';
      case 3: return 'baseline drift, no motion';
      case 4: return 'motion detected';
      case 5: return 'strong motion';
      case 6: return 'multiple sources';
      default: return '';
    }
  }

  /* ------------------------------------------------------------------ *
   * canvas rendering                                                    *
   * ------------------------------------------------------------------ */
  function renderMatrix(canvas, buf) {
    const ctx = canvas.getContext('2d');
    const W = canvas.width, Hpx = canvas.height;
    const img = ctx.createImageData(W, Hpx);
    const out = img.data;
    const nrows = buf.rows.length;
    const n = buf.n;
    for (let y = 0; y < Hpx; y++) {
      const src = y - (Hpx - nrows);           // newest at the bottom
      for (let x = 0; x < W; x++) {
        let v = 0;
        if (src >= 0) {
          const row = buf.rows[src];
          v = x < row.length ? row[x] : 0;
        }
        const p = (y * W + x) * 4;
        out[p] = CMAP[v * 4]; out[p + 1] = CMAP[v * 4 + 1];
        out[p + 2] = CMAP[v * 4 + 2]; out[p + 3] = 255;
      }
    }
    ctx.putImageData(img, 0, 0);
  }

  /* per-subcarrier plot — derives a series from the waterfall buffer.
   * RAW I / RAW Q / SANITIZED PHASE are not carried on the WS telemetry
   * (only 8-bit normalized amplitude is), so they are reported honestly as
   * unavailable rather than synthesized (spec §7). */
  const PS_METRICS = {
    amp: { label: 'AMPLITUDE', needScale: true },
    norm: { label: 'NORMALIZED AMPLITUDE', needScale: false },
    deriv: { label: 'TEMPORAL DERIVATIVE', needScale: false },
    rawi: { label: 'RAW I', unavailable: true },
    rawq: { label: 'RAW Q', unavailable: true },
    phase: { label: 'SANITIZED PHASE', unavailable: true },
  };

  function drawPerSubcarrier() {
    const canvas = H('psPlot');
    const ctx = canvas.getContext('2d');
    const W = canvas.width, Hpx = canvas.height;
    const link = parseInt(H('psLink').value, 10);
    const metric = H('psMetric').value;
    const sub = parseInt(H('psSub').value, 10);
    H('psSubVal').textContent = sub;
    const unav = H('psUnavail');
    const md = PS_METRICS[metric];

    if (md.unavailable) {
      unav.style.display = 'block';
      unav.textContent = md.label + ' is not carried on the WS telemetry ' +
        '(the 8-bit waterfall only carries normalized amplitude).';
      ctx.clearRect(0, 0, W, Hpx);
      return;
    }
    unav.style.display = 'none';

    const buf = wf[link];
    const rows = buf.rows;
    const values = [];
    for (let i = 0; i < rows.length; i++) {
      const v = sub < rows[i].length ? rows[i][sub] : 0;
      if (metric === 'amp') values.push(v << buf.scale);
      else values.push(v);                     // norm or deriv base
    }
    if (metric === 'deriv') {
      for (let i = values.length - 1; i >= 1; i--) values[i] = values[i] - values[i - 1];
      values[0] = 0;
    }

    // background + gridlines
    ctx.fillStyle = '#05070a';
    ctx.fillRect(0, 0, W, Hpx);
    ctx.strokeStyle = '#1c2230';
    ctx.lineWidth = 1;
    for (let i = 1; i < 4; i++) {
      ctx.beginPath();
      ctx.moveTo(0, (Hpx / 4) * i);
      ctx.lineTo(W, (Hpx / 4) * i);
      ctx.stroke();
    }

    if (values.length < 2) {
      ctx.fillStyle = '#8b949e';
      ctx.font = '12px monospace';
      ctx.fillText('waiting for CSI…', 10, Hpx / 2);
      return;
    }

    // autoscale
    let lo = Infinity, hi = -Infinity;
    for (const v of values) {
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
    if (hi - lo < 1e-6) { hi = lo + 1; lo -= 1; }
    const pad = (hi - lo) * 0.1;
    lo -= pad; hi += pad;

    ctx.strokeStyle = '#58a6ff';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    const xStep = W / (values.length - 1);
    for (let i = 0; i < values.length; i++) {
      const x = i * xStep;
      const y = Hpx - ((values[i] - lo) / (hi - lo)) * Hpx;
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();

    // axis labels
    ctx.fillStyle = '#8b949e';
    ctx.font = '10px monospace';
    ctx.fillText(md.label + ' · sub ' + sub + ' · ' + linkName(link), 6, 12);
    ctx.fillText('max ' + hi.toFixed(1), 6, Hpx - 6);
    ctx.fillText('min ' + lo.toFixed(1), W - 60, Hpx - 6);
  }

  function linkName(link) {
    return link === LINK_RX1 ? 'RX1' : link === LINK_RX2 ? 'RX2' : 'FUSED';
  }

  /* motion-energy line chart (3 traces) */
  function drawMotionEnergy() {
    const canvas = H('mePlot');
    const ctx = canvas.getContext('2d');
    const W = canvas.width, Hpx = canvas.height;
    ctx.fillStyle = '#05070a';
    ctx.fillRect(0, 0, W, Hpx);
    ctx.strokeStyle = '#1c2230';
    ctx.lineWidth = 1;
    for (let i = 1; i < 4; i++) {
      ctx.beginPath();
      ctx.moveTo(0, (Hpx / 4) * i);
      ctx.lineTo(W, (Hpx / 4) * i);
      ctx.stroke();
    }
    if (me.t.length < 2) {
      ctx.fillStyle = '#8b949e';
      ctx.font = '12px monospace';
      ctx.fillText('waiting for status frames…', 10, Hpx / 2);
      return;
    }
    // autoscale to the max of the three series
    let hi = 0;
    for (const arr of [me.r1, me.r2, me.f]) for (const v of arr) if (v > hi) hi = v;
    if (hi < 1e-6) hi = 1;
    const traces = [
      { data: me.r1, color: '#58a6ff' },
      { data: me.r2, color: '#3fb950' },
      { data: me.f, color: '#d29922' },
    ];
    const t0 = me.t[0], t1 = me.t[me.t.length - 1];
    const span = (t1 - t0) || 1;
    ctx.lineWidth = 1.5;
    for (const tr of traces) {
      ctx.strokeStyle = tr.color;
      ctx.beginPath();
      for (let i = 0; i < tr.data.length; i++) {
        const x = ((me.t[i] - t0) / span) * W;
        const y = Hpx - (tr.data[i] / hi) * (Hpx - 6);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
    ctx.fillStyle = '#8b949e';
    ctx.font = '10px monospace';
    ctx.fillText('max ' + hi.toFixed(1), 6, 12);
  }

  /* ------------------------------------------------------------------ *
   * render loop (throttled to ~15 fps)                                  *
   * ------------------------------------------------------------------ */
  function tick() {
    renderMatrix(H('wf-rx1'), wf[LINK_RX1]);
    renderMatrix(H('wf-rx2'), wf[LINK_RX2]);
    renderMatrix(H('sp-rx1'), sp[LINK_RX1]);
    renderMatrix(H('sp-rx2'), sp[LINK_RX2]);
    renderMatrix(H('sp-fused'), sp[LINK_FUSED]);
    drawPerSubcarrier();
    drawMotionEnergy();
  }

  /* ------------------------------------------------------------------ *
   * boot                                                                *
   * ------------------------------------------------------------------ */
  buildStatusGrid();
  buildDiffGrid();
  H('psSub').addEventListener('input', () => { H('psSubVal').textContent = H('psSub').value; });
  H('psLink').addEventListener('change', drawPerSubcarrier);
  H('psMetric').addEventListener('change', drawPerSubcarrier);
  setConnUI();
  connectWS();
  pollStatus();
  setInterval(tick, 66);
})();
