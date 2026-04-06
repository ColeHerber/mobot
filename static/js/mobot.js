/* ============================================================
   mobot.js — CMU Mobot 2026 · Shared JS Utilities
   ============================================================
   Globals exported to window:
     MobotSSE         class  — SSE client with auto-reconnect
     RingBuffer        class  — rolling data buffer
     fmt(val, dec)            — number formatter
     lineErrorColor(abs, dr)  — line position → hex color
     stateColor(state)        — state name → hex color
     batteryColor(v)          — voltage → hex color
     drawTrendChart(...)      — canvas trend line
     drawArcGauge(...)        — canvas arc gauge
     initHeaderSSE()          — wire up header live stats
   ============================================================ */

"use strict";

/* ── SSE client with auto-reconnect ─────────────────────────── */
class MobotSSE {
    /**
     * @param {string}   url       SSE endpoint e.g. "/api/state"
     * @param {function} onMessage called with parsed JSON on each event
     */
    constructor(url, onMessage) {
        this.url       = url;
        this.onMessage = onMessage || null;
        this._es       = null;
        this._listeners = onMessage ? [onMessage] : [];
        this._retryMs  = 1000;
        this._maxRetry = 15000;
        this._stopped  = false;
        this._connDot  = null;
        this._errCount = 0;
    }

    /** Attach a .conn-dot element for visual connection feedback. */
    setConnDot(el) { this._connDot = el; return this; }

    /** Register an additional callback (returns unsubscribe fn). */
    subscribe(fn) {
        this._listeners.push(fn);
        return () => { this._listeners = this._listeners.filter(f => f !== fn); };
    }

    _setDot(state) {
        if (!this._connDot) return;
        this._connDot.className = 'conn-dot ' + state;
    }

    connect() {
        if (this._stopped) return;
        this._setDot('connecting');
        const es = new EventSource(this.url);
        this._es = es;

        es.onopen = () => {
            this._retryMs = 1000;
            this._setDot('connected');
        };

        es.onmessage = (evt) => {
            try {
                const data = JSON.parse(evt.data);
                this._listeners.forEach(fn => fn(data));
            } catch (_) {}
        };

        es.onerror = () => {
            this._errCount++;
            es.close();
            this._es = null;
            this._setDot('disconnected');
            if (!this._stopped) {
                const delay = this._retryMs;
                this._retryMs = Math.min(this._retryMs * 1.5, this._maxRetry);
                setTimeout(() => this.connect(), delay);
            }
        };
    }

    stop() {
        this._stopped = true;
        if (this._es) { this._es.close(); this._es = null; }
        this._setDot('disconnected');
    }

    get errorCount() { return this._errCount; }

    // Legacy compat: close() alias
    close() { this.stop(); }
}


/* ── Number formatting ───────────────────────────────────────── */
function fmt(val, decimals = 2) {
    if (val === null || val === undefined || !isFinite(val)) return '--';
    return Number(val).toFixed(decimals);
}


/* ── Line position → color ───────────────────────────────────── */
function lineErrorColor(absLinePos, dead_reckon = false) {
    if (dead_reckon) return '#888888';
    const a = Math.abs(absLinePos);
    if (a < 0.1) return '#00ff88';
    if (a < 0.3) return '#88ff00';
    if (a < 0.6) return '#ff9900';
    return '#ff4444';
}


/* ── State name → color ──────────────────────────────────────── */
const STATE_COLORS = {
    'LINE_FOLLOW':   '#00ff88',
    'DEAD_RECKON':   '#666666',
    'REACQUIRE':     '#ff9900',
    'GATE_APPROACH': '#4488ff',
    'INTERSECTION':  '#ff44ff',
};

function stateColor(state) {
    return STATE_COLORS[state] || '#c9d1d9';
}


/* ── Battery voltage → color ─────────────────────────────────── */
function batteryColor(voltage) {
    if (voltage < 6.6)  return '#ff4444';
    if (voltage < 7.0)  return '#ff9900';
    if (voltage < 7.4)  return '#ffdd00';
    return '#00ff88';
}


/* ── Rolling ring buffer ─────────────────────────────────────── */
class RingBuffer {
    /** @param {number} maxLen default 600 = 30 s at 20 Hz */
    constructor(maxLen = 600) {
        this._maxLen = maxLen;
        this._buf    = [];
    }

    push(val) {
        this._buf.push(val);
        if (this._buf.length > this._maxLen) this._buf.shift();
    }

    toArray() { return this._buf.slice(); }

    get length() { return this._buf.length; }

    clear() { this._buf = []; }

    /** Alias for legacy makeRingBuffer().data style access */
    get data() { return this.toArray(); }
}

/** Legacy factory alias */
function makeRingBuffer(size) {
    const rb = new RingBuffer(size);
    return rb;
}


/* ── Trend chart on canvas ───────────────────────────────────── */
/**
 * Draw a scrolling time-series line chart.
 *
 * New signature: drawTrendChart(canvas, xData, yData, yMin, yMax, color, extraLines)
 * Legacy compat:  drawTrendChart(canvas, data, opts)   — auto-detected
 *
 * extraLines / opts.thresholds: [{y|value, color, label}]
 */
function drawTrendChart(canvas, arg1, arg2, yMin, yMax, color, extraLines) {
    // Detect legacy vs new call form
    const isLegacy = !Array.isArray(arg2) && typeof arg2 !== 'number';
    if (isLegacy) {
        _drawTrendChartLegacy(canvas, arg1, arg2 || {});
        return;
    }

    const xData = arg1;
    const yData = arg2;
    extraLines  = extraLines || [];

    const ctx = canvas.getContext('2d');
    const W = canvas.width;
    const H = canvas.height;

    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = '#0d1117';
    ctx.fillRect(0, 0, W, H);

    if (!xData || xData.length < 2) return;

    const PAD = { top: 6, right: 4, bottom: 18, left: 36 };
    const pw = W - PAD.left - PAD.right;
    const ph = H - PAD.top  - PAD.bottom;
    const yRange = (yMax - yMin) || 1;
    const xMin   = xData[0];
    const xRange = (xData[xData.length - 1] - xMin) || 1;

    const toX = t => PAD.left + ((t - xMin) / xRange) * pw;
    const toY = v => PAD.top  + (1 - (v - yMin) / yRange) * ph;

    // Grid + Y labels
    ctx.font = '9px Courier New';
    ctx.textAlign = 'right';
    for (let i = 0; i <= 4; i++) {
        const v  = yMin + (i / 4) * yRange;
        const gy = toY(v);
        ctx.strokeStyle = '#30363d';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(PAD.left, gy);
        ctx.lineTo(W - PAD.right, gy);
        ctx.stroke();
        ctx.fillStyle = '#6e7681';
        ctx.fillText(fmt(v, 2), PAD.left - 2, gy + 3);
    }

    // Extra threshold / horizontal lines
    for (const ln of extraLines) {
        const ly = ln.y !== undefined ? ln.y : ln.value;
        if (ly < yMin || ly > yMax) continue;
        const gy = toY(ly);
        ctx.strokeStyle = ln.color || '#ff4444';
        ctx.setLineDash([4, 4]);
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(PAD.left, gy);
        ctx.lineTo(W - PAD.right, gy);
        ctx.stroke();
        ctx.setLineDash([]);
        if (ln.label) {
            ctx.fillStyle = ln.color || '#ff4444';
            ctx.textAlign = 'left';
            ctx.fillText(ln.label, PAD.left + 2, gy - 2);
            ctx.textAlign = 'right';
        }
    }

    // Data line
    ctx.beginPath();
    ctx.strokeStyle = color || '#00ff88';
    ctx.lineWidth   = 1.5;
    ctx.setLineDash([]);
    for (let i = 0; i < xData.length; i++) {
        const px = toX(xData[i]);
        const py = toY(Math.max(yMin, Math.min(yMax, yData[i])));
        if (i === 0) ctx.moveTo(px, py);
        else         ctx.lineTo(px, py);
    }
    ctx.stroke();

    // X axis labels
    ctx.fillStyle = '#6e7681';
    ctx.textAlign = 'right';
    ctx.fillText('now', W - PAD.right, H - 4);
    ctx.textAlign = 'left';
    ctx.fillText(`-${fmt(xRange, 0)}s`, PAD.left, H - 4);
}

/** Legacy drawTrendChart implementation (array of values + opts object) */
function _drawTrendChartLegacy(canvas, data, opts) {
    const ctx  = canvas.getContext('2d');
    const W    = canvas.width;
    const H    = canvas.height;
    const pad  = { top: 10, right: 8, bottom: 20, left: 40 };

    ctx.clearRect(0, 0, W, H);
    if (opts.bgColor) {
        ctx.fillStyle = opts.bgColor;
        ctx.fillRect(0, 0, W, H);
    }

    const allSeries = opts.series
        ? opts.series
        : [{ data, color: opts.color || '#00ff88', label: opts.label }];

    let yMin = opts.min !== undefined ? opts.min : Infinity;
    let yMax = opts.max !== undefined ? opts.max : -Infinity;
    if (opts.min === undefined || opts.max === undefined) {
        allSeries.forEach(s => {
            (s.data || []).forEach(v => {
                if (v === null || v === undefined) return;
                if (v < yMin) yMin = v;
                if (v > yMax) yMax = v;
            });
        });
        if (yMin === Infinity)  yMin = 0;
        if (yMax === -Infinity) yMax = 1;
        if (yMin === yMax) { yMin -= 0.5; yMax += 0.5; }
        const pad10 = (yMax - yMin) * 0.1;
        if (opts.min === undefined) yMin -= pad10;
        if (opts.max === undefined) yMax += pad10;
    }

    const chartW = W - pad.left - pad.right;
    const chartH = H - pad.top  - pad.bottom;
    const toX = (i, len) => pad.left + (i / (len - 1 || 1)) * chartW;
    const toY = v         => pad.top  + (1 - (v - yMin) / (yMax - yMin)) * chartH;

    ctx.strokeStyle = opts.gridColor || 'rgba(48,54,61,0.8)';
    ctx.lineWidth   = 1;
    for (let gi = 0; gi <= 4; gi++) {
        const gy  = pad.top + (gi / 4) * chartH;
        ctx.beginPath();
        ctx.moveTo(pad.left, gy);
        ctx.lineTo(pad.left + chartW, gy);
        ctx.stroke();
        const val = yMax - (gi / 4) * (yMax - yMin);
        ctx.fillStyle = 'rgba(110,118,129,0.9)';
        ctx.font      = '10px Courier New';
        ctx.textAlign = 'right';
        ctx.fillText(val.toFixed(1), pad.left - 4, gy + 3);
    }

    (opts.thresholds || []).forEach(th => {
        if (th.value < yMin || th.value > yMax) return;
        const ty = toY(th.value);
        ctx.save();
        ctx.strokeStyle = th.color || '#ff4444';
        ctx.lineWidth   = 1;
        if (th.dash) ctx.setLineDash(th.dash);
        ctx.beginPath();
        ctx.moveTo(pad.left, ty);
        ctx.lineTo(pad.left + chartW, ty);
        ctx.stroke();
        ctx.restore();
        if (th.label) {
            ctx.fillStyle = th.color || '#ff4444';
            ctx.font      = '9px Courier New';
            ctx.textAlign = 'left';
            ctx.fillText(th.label, pad.left + 2, ty - 2);
        }
    });

    allSeries.forEach(s => {
        const pts = (s.data || []).filter(v => v !== null && v !== undefined);
        if (pts.length < 2) return;
        ctx.beginPath();
        ctx.strokeStyle = s.color || '#00ff88';
        ctx.lineWidth   = 1.5;
        ctx.setLineDash([]);
        const len = s.data.length;
        let started = false;
        s.data.forEach((v, i) => {
            if (v === null || v === undefined) { started = false; return; }
            const x = toX(i, len);
            const y = toY(v);
            if (!started) { ctx.moveTo(x, y); started = true; }
            else           ctx.lineTo(x, y);
        });
        ctx.stroke();
    });

    ctx.strokeStyle = 'rgba(48,54,61,0.6)';
    ctx.lineWidth   = 1;
    ctx.strokeRect(pad.left, pad.top, chartW, chartH);
}


/* ── Arc gauge ───────────────────────────────────────────────── */
/**
 * Draw a semicircular arc gauge.
 *
 * New signature: drawArcGauge(canvas, value, min, max, zones, label)
 *   zones = [{end, color}]  ordered color bands
 *
 * Legacy signature: drawArcGauge(canvas, value, opts)
 *   opts = { min, max, zones:[{from,to,color}], unit, decimals, ... }
 */
function drawArcGauge(canvas, value, arg3, arg4, arg5, arg6) {
    // Detect call form
    if (typeof arg3 === 'object' && !Array.isArray(arg3) && arg3 !== null && !('end' in arg3)) {
        _drawArcGaugeLegacy(canvas, value, arg3);
        return;
    }

    // New form
    const min    = (typeof arg3 === 'number') ? arg3 : 0;
    const max    = (typeof arg4 === 'number') ? arg4 : 100;
    const zones  = Array.isArray(arg5) ? arg5 : [];
    const label  = arg6 || '';

    const ctx = canvas.getContext('2d');
    const W   = canvas.width;
    const H   = canvas.height;
    const cx  = W / 2;
    const cy  = H * 0.62;
    const R   = Math.min(W, H) * 0.38;

    ctx.clearRect(0, 0, W, H);

    const startAngle = Math.PI * 0.75;
    const endAngle   = Math.PI * 2.25;
    const totalAngle = endAngle - startAngle;
    const range      = max - min || 1;
    const frac       = Math.max(0, Math.min(1, (value - min) / range));

    // Background track
    ctx.beginPath();
    ctx.arc(cx, cy, R, startAngle, endAngle);
    ctx.strokeStyle = '#1c2128';
    ctx.lineWidth   = R * 0.2;
    ctx.lineCap     = 'round';
    ctx.stroke();

    // Zone arcs (dim background)
    let prev = min;
    for (const zone of zones) {
        const zEnd   = Math.min(zone.end, max);
        const aStart = startAngle + ((prev - min) / range) * totalAngle;
        const aEnd   = startAngle + ((zEnd  - min) / range) * totalAngle;
        if (aEnd > aStart) {
            ctx.beginPath();
            ctx.arc(cx, cy, R, aStart, aEnd);
            ctx.strokeStyle = zone.color;
            ctx.lineWidth   = R * 0.18;
            ctx.globalAlpha = 0.25;
            ctx.stroke();
            ctx.globalAlpha = 1;
        }
        prev = zone.end;
    }

    // Active color from zones
    let activeColor = '#00ff88';
    for (const zone of zones) {
        if (value <= zone.end) { activeColor = zone.color; break; }
    }
    if (zones.length && value > zones[zones.length - 1].end) {
        activeColor = zones[zones.length - 1].color;
    }

    // Filled arc
    if (frac > 0) {
        ctx.beginPath();
        ctx.arc(cx, cy, R, startAngle, startAngle + frac * totalAngle);
        ctx.strokeStyle = activeColor;
        ctx.lineWidth   = R * 0.18;
        ctx.lineCap     = 'round';
        ctx.stroke();
    }

    // Value text
    ctx.fillStyle    = activeColor;
    ctx.font         = `bold ${Math.round(R * 0.38)}px Courier New`;
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(fmt(value, 1), cx, cy - R * 0.05);

    // Label
    if (label) {
        ctx.fillStyle    = '#6e7681';
        ctx.font         = `${Math.round(R * 0.18)}px Courier New`;
        ctx.textBaseline = 'middle';
        ctx.fillText(label, cx, cy + R * 0.35);
    }

    // Min / max axis labels
    ctx.fillStyle    = '#6e7681';
    ctx.font         = '9px Courier New';
    ctx.textBaseline = 'top';
    const minX = cx + R * Math.cos(startAngle);
    const minY = cy + R * Math.sin(startAngle) + 4;
    const maxX = cx + R * Math.cos(endAngle);
    const maxY = cy + R * Math.sin(endAngle) + 4;
    ctx.textAlign = 'center';
    ctx.fillText(String(min), minX, minY);
    ctx.fillText(String(max), maxX, maxY);
}

function _drawArcGaugeLegacy(canvas, value, opts) {
    const ctx    = canvas.getContext('2d');
    const W      = canvas.width;
    const H      = canvas.height;
    const cx     = W / 2;
    const cy     = H * 0.62;
    const R      = Math.min(W, H) * 0.38;

    ctx.clearRect(0, 0, W, H);

    const startAngle = Math.PI * 0.75;
    const endAngle   = Math.PI * 2.25;
    const totalAngle = endAngle - startAngle;

    const min  = opts.min !== undefined ? opts.min : 0;
    const max  = opts.max !== undefined ? opts.max : 100;
    const frac = Math.max(0, Math.min(1, (value - min) / (max - min)));

    ctx.beginPath();
    ctx.arc(cx, cy, R, startAngle, endAngle);
    ctx.strokeStyle = opts.trackColor || '#1c2128';
    ctx.lineWidth   = R * 0.2;
    ctx.lineCap     = 'round';
    ctx.stroke();

    const zones = opts.zones || [];
    let activeColor = opts.needleColor || '#00ff88';

    if (zones.length > 0) {
        zones.forEach(z => {
            if (value >= z.from && value <= z.to) activeColor = z.color;
        });
        zones.forEach(z => {
            const zFrom = Math.max(min, z.from);
            const zTo   = Math.min(max, z.to);
            if (zFrom >= zTo) return;
            const aFrom = startAngle + ((zFrom - min) / (max - min)) * totalAngle;
            const aTo   = startAngle + ((zTo   - min) / (max - min)) * totalAngle;
            ctx.beginPath();
            ctx.arc(cx, cy, R, aFrom, aTo);
            ctx.strokeStyle = z.color;
            ctx.lineWidth   = R * 0.18;
            ctx.globalAlpha = 0.25;
            ctx.stroke();
            ctx.globalAlpha = 1;
        });
    }

    ctx.beginPath();
    ctx.arc(cx, cy, R, startAngle, startAngle + frac * totalAngle);
    ctx.strokeStyle = activeColor;
    ctx.lineWidth   = R * 0.18;
    ctx.lineCap     = 'round';
    ctx.stroke();

    ctx.fillStyle    = activeColor;
    ctx.font         = `bold ${Math.round(R * 0.38)}px Courier New`;
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    const decimals = opts.decimals !== undefined ? opts.decimals : 1;
    const display  = opts.label !== undefined
        ? opts.label
        : value.toFixed(decimals) + (opts.unit || '');
    ctx.fillText(display, cx, cy);

    ctx.fillStyle    = 'rgba(110,118,129,0.9)';
    ctx.font         = '10px Courier New';
    ctx.textBaseline = 'top';
    const minX = cx - R * Math.cos(Math.PI * 0.25);
    const minY = cy - R * Math.sin(Math.PI * 0.25) + 4;
    const maxX = cx + R * Math.cos(Math.PI * 0.25);
    ctx.textAlign = 'center';
    ctx.fillText(String(min), minX, minY);
    ctx.fillText(String(max), maxX, minY);
}


/* ── Live header helper ──────────────────────────────────────── */
/**
 * Wire up standard header SSE updater.
 * Requires elements with IDs: hdr-state, hdr-battery, hdr-speed
 * and a .conn-dot element.
 */
function initHeaderSSE() {
    const elState = document.getElementById('hdr-state');
    const elBat   = document.getElementById('hdr-battery');
    const elSpeed = document.getElementById('hdr-speed');
    const dot     = document.querySelector('.conn-dot');

    const sse = new MobotSSE('/api/state', (d) => {
        if (d._sse_error) return;
        if (elState) {
            elState.textContent = d.state || '--';
            elState.style.color = stateColor(d.state);
        }
        if (elBat) {
            const v = d.input_voltage;
            elBat.textContent = v != null ? fmt(v, 1) + ' V' : '--';
            elBat.style.color = v != null ? batteryColor(v) : '';
        }
        if (elSpeed) {
            elSpeed.textContent = d.wheel_velocity_ms != null
                ? fmt(d.wheel_velocity_ms, 2) + ' m/s' : '--';
        }
    });

    if (dot) sse.setConnDot(dot);
    sse.connect();
    return sse;
}


/* ── Utility ─────────────────────────────────────────────────── */
function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }


/* ── Expose as globals ───────────────────────────────────────── */
window.MobotSSE       = MobotSSE;
window.RingBuffer     = RingBuffer;
window.makeRingBuffer = makeRingBuffer;
window.fmt            = fmt;
window.lineErrorColor = lineErrorColor;
window.stateColor     = stateColor;
window.batteryColor   = batteryColor;
window.drawTrendChart = drawTrendChart;
window.drawArcGauge   = drawArcGauge;
window.initHeaderSSE  = initHeaderSSE;
window.clamp          = clamp;
