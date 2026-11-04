/* MTRB web UI: live map, robot, scan and plan over a single WebSocket.
 * Vanilla JS, no build step, no external resources (works offline). */
(function () {
  'use strict';

  var ROBOT_DIAMETER_M = 0.34;
  var TOKEN_KEY = 'mtrb_token';
  var BACKOFF_MIN_MS = 1000;
  var BACKOFF_MAX_MS = 5000;
  var NO_TF_AFTER_S = 3;
  var AGE_GREEN_S = 2;
  var AGE_AMBER_S = 5;
  var SCAN_STALE_S = 3;
  var STALE_ALPHA = 0.25;

  var $ = function (id) { return document.getElementById(id); };

  // ---- pure helpers (no DOM, no state; also exported for Node tests) ----
  function worldToScreen(view, w, h, x, y) {
    return { x: w / 2 + (x - view.cx) * view.scale, y: h / 2 - (y - view.cy) * view.scale };
  }
  function screenToWorld(view, w, h, sx, sy) {
    return { x: view.cx + (sx - w / 2) / view.scale, y: view.cy - (sy - h / 2) / view.scale };
  }
  function isNum(v) { return typeof v === 'number' && isFinite(v); }
  // fetch() puts the token in a header: only printable ASCII is safe.
  function isValidToken(t) { return typeof t === 'string' && /^[\x21-\x7e]+$/.test(t); }
  // Keep only well-formed [x, y] pairs.
  function sanitizePlan(points) {
    var out = [];
    for (var i = 0; i < points.length; i++) {
      var p = points[i];
      if (Array.isArray(p) && isNum(p[0]) && isNum(p[1])) out.push([p[0], p[1]]);
    }
    return out;
  }
  // Keep only finite (x, y) pairs of a flat [x0, y0, ...] array.
  function sanitizeScan(points) {
    var out = [];
    for (var i = 0; i + 1 < points.length; i += 2) {
      if (isNum(points[i]) && isNum(points[i + 1])) out.push(points[i], points[i + 1]);
    }
    return out;
  }

  // Wrap an angle to (-pi, pi].
  function normalizeYaw(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a <= -Math.PI) a += 2 * Math.PI;
    return a;
  }
  var REQUEST_TIMEOUT_MS = 8000;   // default POST timeout
  var CANCEL_TIMEOUT_MS = 3000;    // /api/cancel must fail fast to allow an immediate retry
  var NAV_LABELS = { idle: 'Inactivo', active: 'Navegando', succeeded: 'Llegado',
    canceled: 'Cancelado', failed: 'Fallido' };
  // User-facing Spanish message for a failed command. `status` is the HTTP status
  // (0 = no response), `code` the server's `error` field. Never exposes technical detail.
  function describeApiError(status, code) {
    if (status === 401) return 'Sesión no válida: introduce el token de nuevo.';
    if (status === 409 && code === 'goal_active') return 'Ya hay una navegación activa; cancélala primero.';
    if (status === 409 && code === 'wrong_mode') {
      return 'La pose inicial solo se puede fijar en modo navegación (map).';
    }
    if (status === 409) return 'La acción no es posible en el estado actual del robot.';
    if (status === 503 && code === 'no_map') return 'Aún no hay mapa disponible; espera a recibirlo.';
    if (status === 503 && code === 'dock_unavailable') return 'La posición del dock no está configurada.';
    if (status === 503) return 'Navegación no disponible.';
    if (status === 400 && code === 'invalid_goal') return 'Objetivo no válido.';
    if (status === 400) return 'Petición no válida.';
    if (status === 0) return 'No se pudo contactar con el robot.';
    return 'No se pudo completar la acción (error ' + status + ').';
  }

  // ---- token storage (localStorage may be blocked, e.g. private mode) ----
  function loadToken() {
    try { return window.localStorage.getItem(TOKEN_KEY) || ''; } catch (e) { return ''; }
  }
  function saveToken(t) {
    try { window.localStorage.setItem(TOKEN_KEY, t); } catch (e) { /* ignore */ }
  }
  function clearToken() {
    try { window.localStorage.removeItem(TOKEN_KEY); } catch (e) { /* ignore */ }
  }

  // ---- application state ----
  var state = {
    token: '',
    ws: null,
    connected: false,
    closeNotice: '',     // message kept while a special close (e.g. 1013) lasts
    mapError: '',        // map decode failure, kept until a valid map arrives
    retryMs: BACKOFF_MIN_MS,
    retryTimer: null,
    lastMsgAt: 0,        // performance.now() of last message
    lastPoseAt: 0,
    lastScanAt: 0,
    pose: null,          // {x, y, yaw}
    scan: [],            // flat [x0, y0, ...]
    plan: [],            // [[x, y], ...]
    mapMeta: null,       // last map_meta (awaiting binary)
    map: null,           // {meta, canvas}
    status: null,
    nav: null,
    view: { cx: 0, cy: 0, scale: 40 },  // scale = pixels per meter
    fitted: false,
    dirty: true,
    wasStale: false,
    authFailed: false,
    pending: null,       // {kind: 'goal'|'initialpose', x, y, yaw} awaiting explicit confirmation
    pickInitial: false,  // next tap sets the initial pose instead of a goal
    goalRejected: false, // last goal POST was rejected (400/503) with no goal running
    busy: {},            // action name -> true while its request is in flight
    dockConfirm: false
  };

  // ---- zlib inflate fallback (used only without DecompressionStream) ----
  var inflateRaw = (function () {
    var LBASE = [3,4,5,6,7,8,9,10,11,13,15,17,19,23,27,31,35,43,51,59,67,83,99,115,131,163,195,227,258];
    var LEXT = [0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,0];
    var DBASE = [1,2,3,4,5,7,9,13,17,25,33,49,65,97,129,193,257,385,513,769,1025,1537,2049,3073,
      4097,6145,8193,12289,16385,24577];
    var DEXT = [0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13];
    var ORDER = [16,17,18,0,8,7,9,6,10,5,11,4,12,3,13,2,14,1,15];

    function build(lengths) {
      var count = new Array(16).fill(0), offs = new Array(16).fill(0), syms = [], i;
      for (i = 0; i < lengths.length; i++) count[lengths[i]]++;
      count[0] = 0;
      for (i = 1; i < 16; i++) offs[i] = offs[i - 1] + count[i - 1];
      for (i = 0; i < lengths.length; i++) if (lengths[i]) syms[offs[lengths[i]]++] = i;
      return { count: count, symbols: syms };
    }

    return function (src, start, outSize) {
      var out = new Uint8Array(outSize), op = 0, pos = start, bitBuf = 0, bitCnt = 0;
      function bits(n) {
        while (bitCnt < n) {
          if (pos >= src.length) throw new Error('inflate: truncated');
          bitBuf |= src[pos++] << bitCnt; bitCnt += 8;
        }
        var v = bitBuf & ((1 << n) - 1);
        bitBuf >>>= n; bitCnt -= n;
        return v;
      }
      function decode(h) {
        var code = 0, first = 0, index = 0;
        for (var len = 1; len < 16; len++) {
          code |= bits(1);
          var c = h.count[len];
          if (code - c < first) return h.symbols[index + (code - first)];
          index += c; first += c; first <<= 1; code <<= 1;
        }
        throw new Error('inflate: bad code');
      }
      var fixedL = null, fixedD = null;
      function fixed() {
        if (!fixedL) {
          var l = [], i;
          for (i = 0; i < 144; i++) l.push(8);
          for (; i < 256; i++) l.push(9);
          for (; i < 280; i++) l.push(7);
          for (; i < 288; i++) l.push(8);
          fixedL = build(l);
          fixedD = build(new Array(30).fill(5));
        }
      }
      function codes(lh, dh) {
        for (;;) {
          var sym = decode(lh);
          if (sym < 256) {
            if (op >= outSize) throw new Error('inflate: overflow');
            out[op++] = sym;
          } else if (sym === 256) {
            return;
          } else {
            sym -= 257;
            if (sym >= 29) throw new Error('inflate: bad length');
            var len = LBASE[sym] + bits(LEXT[sym]);
            var ds = decode(dh);
            var dist = DBASE[ds] + bits(DEXT[ds]);
            if (dist > op || op + len > outSize) throw new Error('inflate: bad distance');
            while (len--) { out[op] = out[op - dist]; op++; }
          }
        }
      }
      var last;
      do {
        last = bits(1);
        var type = bits(2);
        if (type === 0) {
          bitBuf = 0; bitCnt = 0;
          var n = src[pos] | (src[pos + 1] << 8);
          pos += 4;
          if (op + n > outSize || pos + n > src.length) throw new Error('inflate: bad stored');
          out.set(src.subarray(pos, pos + n), op); op += n; pos += n;
        } else if (type === 1) {
          fixed(); codes(fixedL, fixedD);
        } else if (type === 2) {
          var nlen = bits(5) + 257, ndist = bits(5) + 1, ncode = bits(4) + 4, i;
          var cl = new Array(19).fill(0);
          for (i = 0; i < ncode; i++) cl[ORDER[i]] = bits(3);
          var clh = build(cl), lens = [];
          while (lens.length < nlen + ndist) {
            var s = decode(clh);
            if (s < 16) { lens.push(s); } else {
              var rep, val = 0;
              if (s === 16) { val = lens[lens.length - 1]; rep = 3 + bits(2); }
              else if (s === 17) { rep = 3 + bits(3); }
              else { rep = 11 + bits(7); }
              while (rep--) lens.push(val);
            }
          }
          codes(build(lens.slice(0, nlen)), build(lens.slice(nlen, nlen + ndist)));
        } else {
          throw new Error('inflate: bad block');
        }
      } while (!last);
      return out;
    };
  })();

  // Pure functions exposed for reuse/tests without a build step.
  window.MTRBPure = {
    worldToScreen: worldToScreen, screenToWorld: screenToWorld, inflateRaw: inflateRaw,
    isValidToken: isValidToken, sanitizePlan: sanitizePlan, sanitizeScan: sanitizeScan,
    normalizeYaw: normalizeYaw, describeApiError: describeApiError
  };

  function zlibDecompress(bytes, expectedSize) {
    if (typeof DecompressionStream === 'function') {
      var stream = new Blob([bytes]).stream().pipeThrough(new DecompressionStream('deflate'));
      return new Response(stream).arrayBuffer().then(function (b) { return new Uint8Array(b); });
    }
    try {
      return Promise.resolve(inflateRaw(bytes, 2, expectedSize));  // skip 2-byte zlib header
    } catch (e) {
      return Promise.reject(e);
    }
  }

  // ---- map rendering cache ----
  function makeCanvas(w, h) {
    if (typeof OffscreenCanvas === 'function') return new OffscreenCanvas(w, h);
    var c = document.createElement('canvas');
    c.width = w; c.height = h;
    return c;
  }

  function buildMapCanvas(meta, cells) {
    var w = meta.width, h = meta.height;
    var canvas = makeCanvas(w, h);
    var ctx = canvas.getContext('2d');
    var img = ctx.createImageData(w, h);
    var px = img.data;
    // Grid row 0 is the bottom of the map; image row 0 is the top: flip vertically.
    for (var j = 0; j < h; j++) {
      var src = (h - 1 - j) * w, dst = j * w * 4;
      for (var i = 0; i < w; i++) {
        var v = cells[src + i] << 24 >> 24;  // uint8 -> int8
        var g = v < 0 ? 128 : (v >= 50 ? 0 : 255);
        var o = dst + i * 4;
        px[o] = g; px[o + 1] = g; px[o + 2] = g; px[o + 3] = 255;
      }
    }
    ctx.putImageData(img, 0, 0);
    return canvas;
  }

  function handleMapBinary(buf) {
    var meta = state.mapMeta;
    if (!meta || buf.byteLength < 5) return;
    var dv = new DataView(buf);
    var version = dv.getUint32(0, true);
    if (version !== meta.version) return;  // stale binary: discard
    var size = meta.width * meta.height;
    zlibDecompress(new Uint8Array(buf, 4), size).then(function (cells) {
      if (state.mapMeta !== meta) return;  // a newer meta arrived meanwhile
      if (cells.length !== size) throw new Error('map size mismatch');
      state.map = { meta: meta, canvas: buildMapCanvas(meta, cells) };
      state.mapError = '';
      if (!state.fitted) fitToMap();
      state.dirty = true;
      updatePanel();
      updateStatus();
    }).catch(function () {
      if (state.mapMeta !== meta) return;
      state.mapError = 'No se pudo leer el mapa recibido.';
      updateStatus();
    });
  }

  // ---- view helpers ----
  var canvas = $('view');
  var ctx = canvas.getContext('2d');

  function resize() {
    var r = canvas.getBoundingClientRect();
    var dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.round(r.width * dpr));
    canvas.height = Math.max(1, Math.round(r.height * dpr));
    state.dirty = true;
  }

  function fitToMap() {
    if (!state.map) return;
    var m = state.map.meta;
    var wm = m.width * m.resolution, hm = m.height * m.resolution;
    state.view.cx = m.origin_x + wm / 2;
    state.view.cy = m.origin_y + hm / 2;
    state.view.scale = Math.min(canvas.width / wm, canvas.height / hm) * 0.95 || 40;
    state.fitted = true;
  }

  function centerOnRobot() {
    if (!state.pose) return;
    state.view.cx = state.pose.x;
    state.view.cy = state.pose.y;
    if (state.view.scale < 30) state.view.scale = 60;
    state.dirty = true;
  }

  // ---- drawing ----
  function draw() {
    var W = canvas.width, H = canvas.height, v = state.view;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = '#5a6572';
    ctx.fillRect(0, 0, W, H);

    // World (x right, y up, meters) -> screen px.
    ctx.setTransform(v.scale, 0, 0, -v.scale, W / 2 - v.cx * v.scale, H / 2 + v.cy * v.scale);

    if (state.map) {
      var m = state.map.meta;
      ctx.imageSmoothingEnabled = false;
      ctx.save();
      // Image top-left corner sits at world (origin_x, origin_y + height * res); flip y back.
      ctx.translate(m.origin_x, m.origin_y + m.height * m.resolution);
      ctx.scale(m.resolution, -m.resolution);
      ctx.drawImage(state.map.canvas, 0, 0);
      ctx.restore();
    }

    var px = 1 / v.scale;  // one screen pixel in meters

    if ($('layer-plan').checked && state.plan.length > 1) {
      ctx.beginPath();
      ctx.moveTo(state.plan[0][0], state.plan[0][1]);
      for (var i = 1; i < state.plan.length; i++) ctx.lineTo(state.plan[i][0], state.plan[i][1]);
      ctx.strokeStyle = '#1e90ff';
      ctx.lineWidth = 3 * px;
      ctx.stroke();
    }

    var now = performance.now();
    var poseStale = now - state.lastPoseAt > NO_TF_AFTER_S * 1000;
    var scanStale = poseStale || now - state.lastScanAt > SCAN_STALE_S * 1000;

    if ($('layer-scan').checked && state.scan.length >= 2) {
      ctx.globalAlpha = scanStale ? STALE_ALPHA : 1;
      ctx.fillStyle = '#ff3b30';
      var s = 3 * px;
      for (var k = 0; k + 1 < state.scan.length; k += 2) {
        ctx.fillRect(state.scan[k] - s / 2, state.scan[k + 1] - s / 2, s, s);
      }
      ctx.globalAlpha = 1;
    }

    var dock = getDock();
    if (dock && $('layer-dock').checked) drawMarker(dock.x, dock.y, dock.yaw, '#f5a623', px, false);
    if (state.pending) {
      drawMarker(state.pending.x, state.pending.y, state.pending.yaw,
        state.pending.kind === 'goal' ? '#1e90ff' : '#c06cff', px, true);
    }

    if (state.pose) {
      ctx.globalAlpha = poseStale ? STALE_ALPHA : 1;
      var r = ROBOT_DIAMETER_M / 2;
      ctx.beginPath();
      ctx.arc(state.pose.x, state.pose.y, r, 0, 2 * Math.PI);
      ctx.fillStyle = 'rgba(46, 204, 113, 0.85)';
      ctx.fill();
      ctx.strokeStyle = '#0b5d2e';
      ctx.lineWidth = 2 * px;
      ctx.stroke();
      // heading line
      ctx.beginPath();
      ctx.moveTo(state.pose.x, state.pose.y);
      ctx.lineTo(state.pose.x + r * Math.cos(state.pose.yaw), state.pose.y + r * Math.sin(state.pose.yaw));
      ctx.strokeStyle = '#0b2e17';
      ctx.lineWidth = 3 * px;
      ctx.stroke();
      ctx.globalAlpha = 1;
    }
  }

  // Marker with an orientation arrow, in world coordinates (px = meters per screen pixel).
  function drawMarker(x, y, yaw, color, px, ghost) {
    var r = ROBOT_DIAMETER_M / 2, len = r * 2.2;
    ctx.save();
    ctx.globalAlpha = ghost ? 0.6 : 1;
    ctx.beginPath();
    ctx.arc(x, y, r, 0, 2 * Math.PI);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.strokeStyle = '#101820';
    ctx.lineWidth = 2 * px;
    ctx.stroke();
    ctx.globalAlpha = 1;
    var ex = x + len * Math.cos(yaw), ey = y + len * Math.sin(yaw), head = 8 * px;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(ex, ey);
    ctx.moveTo(ex, ey);
    ctx.lineTo(ex - head * Math.cos(yaw - 0.5), ey - head * Math.sin(yaw - 0.5));
    ctx.moveTo(ex, ey);
    ctx.lineTo(ex - head * Math.cos(yaw + 0.5), ey - head * Math.sin(yaw + 0.5));
    ctx.strokeStyle = color;
    ctx.lineWidth = 3 * px;
    ctx.stroke();
    ctx.restore();
  }

  // Dock pose, only if the server publishes it in `status.dock` ({x, y, yaw}); never invented.
  function getDock() {
    var d = state.status && state.status.dock;
    return d && isNum(d.x) && isNum(d.y) && isNum(d.yaw) ? d : null;
  }

  // ---- text / status ----
  function showOverlayText(text) {
    var o = $('overlay');
    if (text) { o.textContent = text; o.hidden = false; } else { o.hidden = true; }
  }

  function setBadge(cls, text) {
    var b = $('conn');
    b.className = 'badge ' + cls;
    $('conn-text').textContent = text;
  }

  function updateStatus() {
    var now = performance.now();
    var age = (now - state.lastMsgAt) / 1000;
    var text = null;
    if (state.authFailed) {
      setBadge('badge-red', 'Token inválido');
      text = 'Token inválido. Pulsa «Cambiar token» e introdúcelo de nuevo.';
    } else if (!state.connected) {
      setBadge('badge-red', 'Sin conexión');
      text = state.closeNotice || 'Sin conexión con el robot. Reintentando…';
    } else {
      if (age < AGE_GREEN_S) setBadge('badge-green', 'Conectado');
      else if (age < AGE_AMBER_S) setBadge('badge-amber', 'Datos con retraso');
      else setBadge('badge-red', 'Sin datos');
      if (now - state.lastPoseAt > NO_TF_AFTER_S * 1000) {
        text = 'Sin TF: no se recibe la pose del robot.';
      }
      if (!state.map) {
        text = (state.mapError || 'Esperando mapa…') + (text ? ' ' + text : '');
      } else if (state.mapError) {
        text = state.mapError + (text ? ' ' + text : '');
      }
    }
    showOverlayText(text);
  }

  function updatePanel() {
    var st = state.status || {};
    $('st-mode').textContent = st.mode === 'map' ? 'Navegación (map)' :
      st.mode === 'mapping' ? 'Mapeo (mapping)' : '-';
    $('st-battery').textContent = typeof st.battery_pct === 'number' ?
      st.battery_pct.toFixed(0) + ' %' : '-';
    var nav = state.nav;
    $('st-nav').textContent = nav ? (NAV_LABELS[nav.state] || '-') +
      (isNum(nav.distance_remaining) ? ' (faltan ' + nav.distance_remaining.toFixed(1) + ' m)' : '') : '-';
    var warn = state.navWarnEl;
    if (!warn) {
      warn = state.navWarnEl = document.createElement('small');
      warn.className = 'alert-warning';
      warn.setAttribute('role', 'alert');
    }
    var showWarn = !!nav && nav.state === 'failed' && !state.goalRejected;
    if (showWarn) {
      warn.textContent = ' \u26A0 FALLIDO: Nav2 puede seguir moviendo el robot hasta confirmar la cancelación; ' +
        'pulsa Cancelar navegación si ves que se mueve.';
      $('st-nav').appendChild(warn);
    } else if (warn.parentNode) {
      warn.parentNode.removeChild(warn);
    }
    $('st-pose').textContent = state.pose ? 'x ' + state.pose.x.toFixed(2) + ' m, y ' +
      state.pose.y.toFixed(2) + ' m, ' + (state.pose.yaw * 180 / Math.PI).toFixed(0) + '°' : '-';
    updateActions();
    $('st-map').textContent = state.map ? state.map.meta.width + '×' + state.map.meta.height +
      ' @ ' + state.map.meta.resolution + ' m' : 'Esperando mapa…';
  }

  // ---- message handling ----
  function handleJson(msg) {
    if (!msg || typeof msg.type !== 'string') return;
    switch (msg.type) {
      case 'pose':
        if (isNum(msg.x) && isNum(msg.y) && isNum(msg.yaw)) {
          state.pose = { x: msg.x, y: msg.y, yaw: msg.yaw };
          state.lastPoseAt = performance.now();
          state.dirty = true;
        }
        break;
      case 'scan':
        if (Array.isArray(msg.points)) {
          state.scan = sanitizeScan(msg.points);
          state.lastScanAt = performance.now();
          state.dirty = true;
        }
        break;
      case 'plan':
        if (Array.isArray(msg.points)) { state.plan = sanitizePlan(msg.points); state.dirty = true; }
        break;
      case 'nav':
        state.nav = msg;
        if (msg.state === 'active') state.goalRejected = false;
        break;
      case 'status': state.status = msg; break;
      case 'map_meta':
        if (isNum(msg.version) && isNum(msg.width) && isNum(msg.height) && isNum(msg.resolution) &&
            isNum(msg.origin_x) && isNum(msg.origin_y) && msg.width > 0 && msg.height > 0 &&
            msg.width * msg.height <= 64e6) {
          state.mapMeta = msg;
        }
        break;
      default: break;
    }
  }

  // The first valid message (not onopen) proves the server kept the connection:
  // only then reset the backoff and clear the notice (a 1013 close follows an accept).
  function markConnected() {
    if (state.connected) return;
    state.connected = true;
    state.authFailed = false;
    state.closeNotice = '';
    state.retryMs = BACKOFF_MIN_MS;
    state.lastPoseAt = performance.now();
    hideLogin();
    updateActions();
  }

  function onMessage(ev) {
    state.lastMsgAt = performance.now();
    if (typeof ev.data === 'string') {
      var msg;
      try {
        msg = JSON.parse(ev.data);
        if (msg && typeof msg.type === 'string') markConnected();
        handleJson(msg);
        if (msg && (msg.type === 'status' || msg.type === 'nav' || msg.type === 'pose')) updatePanel();
      } catch (e) { /* a bad message must not break the connection */ }
    } else {
      markConnected();
      try { handleMapBinary(ev.data); } catch (e) { /* ignore malformed map */ }
    }
    updateStatus();
  }

  // ---- connection with backoff ----
  function wsUrl() {
    var proto = location.protocol === 'https:' ? 'wss://' : 'ws://';
    return proto + location.host + '/ws?token=' + encodeURIComponent(state.token);
  }

  function checkTokenValid() {
    // The WS handshake failure does not say why; ask a REST endpoint to tell 401 apart.
    if (!isValidToken(state.token)) return Promise.resolve(false);
    return fetch('/api/state', { headers: { Authorization: 'Bearer ' + state.token } })
      .then(function (r) { return r.status !== 401; })
      .catch(function () { return true; });
  }

  function scheduleRetry() {
    clearTimeout(state.retryTimer);
    state.retryTimer = setTimeout(connect, state.retryMs);
    state.retryMs = Math.min(BACKOFF_MAX_MS, Math.round(state.retryMs * 1.5));
  }

  function connect() {
    if (!state.token) return;
    clearTimeout(state.retryTimer);
    var ws;
    try { ws = new WebSocket(wsUrl()); } catch (e) { scheduleRetry(); return; }
    ws.binaryType = 'arraybuffer';
    state.ws = ws;
    ws.onopen = function () { /* wait for the first valid message: see markConnected */ };
    ws.onmessage = function (ev) { if (state.ws === ws) onMessage(ev); };
    ws.onclose = function (ev) {
      if (state.ws !== ws) return;
      state.connected = false;
      state.ws = null;
      state.closeNotice = (ev && ev.code === 1013) ?
        'Demasiados clientes conectados al robot. Reintentando…' : '';
      updateStatus();
      updateActions();
      checkTokenValid().then(function (ok) {
        if (!ok) {
          state.authFailed = true;
          clearToken();
          state.token = '';
          showLogin('Token inválido. Revisa el token e inténtalo de nuevo.');
          updateStatus();
          return;
        }
        updateStatus();
        scheduleRetry();
      });
    };
    ws.onerror = function () { /* onclose follows */ };
  }

  function disconnect() {
    clearTimeout(state.retryTimer);
    var ws = state.ws;
    state.ws = null;
    state.connected = false;
    if (ws) { try { ws.close(); } catch (e) { /* ignore */ } }
    updateActions();
  }

  // ---- login ----
  function showLogin(errorText) {
    $('login').hidden = false;
    var e = $('login-error');
    if (errorText) { e.textContent = errorText; e.hidden = false; } else { e.hidden = true; }
    $('token-input').value = '';
    $('token-input').focus();
  }
  function hideLogin() { $('login').hidden = true; }

  $('login-form').addEventListener('submit', function (ev) {
    ev.preventDefault();
    var t = $('token-input').value.trim();
    if (!t) return;
    if (!isValidToken(t)) {
      $('login-error').textContent = 'Token con caracteres no válidos. Usa solo caracteres ASCII sin espacios.';
      $('login-error').hidden = false;
      return;
    }
    state.token = t;
    state.authFailed = false;
    saveToken(t);
    state.retryMs = BACKOFF_MIN_MS;
    disconnect();
    connect();
    $('login-error').hidden = true;
  });

  $('btn-token').addEventListener('click', function () {
    disconnect();
    clearToken();
    state.token = '';
    showLogin('');
    updateStatus();
  });

  // ---- interaction: drag, wheel, pinch ----
  var pointers = new Map();
  var pinchDist = 0;

  function localXY(ev) {
    var r = canvas.getBoundingClientRect();
    var k = canvas.width / r.width;
    return { x: (ev.clientX - r.left) * k, y: (ev.clientY - r.top) * k };
  }

  function zoomAt(sx, sy, factor) {
    var v = state.view;
    var w = screenToWorld(v, canvas.width, canvas.height, sx, sy);
    v.scale = Math.min(2000, Math.max(2, v.scale * factor));
    v.cx = w.x - (sx - canvas.width / 2) / v.scale;
    v.cy = w.y + (sy - canvas.height / 2) / v.scale;
    state.dirty = true;
  }

  var TAP_SLOP_PX = 10;       // movement below this still counts as a tap
  var GRAB_RADIUS_PX = 32;    // a drag starting this close to the pending point sets its heading
  var drag = null;            // {id, mode: 'pan'|'orient', x0, y0, moved}
  var multiTouch = false;

  function setPending(kind, x, y, yaw) {
    state.pending = { kind: kind, x: x, y: y, yaw: normalizeYaw(yaw) };
    state.dirty = true;
    updateActions();
  }

  function headingFrom(ev) {
    var p = localXY(ev), w = screenToWorld(state.view, canvas.width, canvas.height, p.x, p.y);
    return Math.atan2(w.y - state.pending.y, w.x - state.pending.x);
  }

  canvas.addEventListener('pointerdown', function (ev) {
    try { canvas.setPointerCapture(ev.pointerId); } catch (e) { /* ignore */ }
    var p = localXY(ev);
    pointers.set(ev.pointerId, p);
    pinchDist = 0;
    if (pointers.size > 1) { multiTouch = true; drag = null; return; }
    multiTouch = false;
    drag = { id: ev.pointerId, mode: 'pan', x0: p.x, y0: p.y, moved: false };
    if (state.pending) {
      var s = worldToScreen(state.view, canvas.width, canvas.height, state.pending.x, state.pending.y);
      var k = canvas.width / (canvas.getBoundingClientRect().width || canvas.width);
      if (Math.hypot(p.x - s.x, p.y - s.y) <= GRAB_RADIUS_PX * k) drag.mode = 'orient';
    }
  });
  canvas.addEventListener('pointermove', function (ev) {
    if (!pointers.has(ev.pointerId)) return;
    var prev = pointers.get(ev.pointerId), cur = localXY(ev);
    pointers.set(ev.pointerId, cur);
    if (pointers.size === 1 && drag && drag.id === ev.pointerId) {
      if (!drag.moved && Math.hypot(cur.x - drag.x0, cur.y - drag.y0) > TAP_SLOP_PX) drag.moved = true;
      if (drag.mode === 'orient') {
        if (drag.moved && state.pending) {
          state.pending.yaw = headingFrom(ev);
          state.dirty = true;
          updateActions();
        }
      } else if (drag.moved) {
        state.view.cx -= (cur.x - prev.x) / state.view.scale;
        state.view.cy += (cur.y - prev.y) / state.view.scale;
        state.dirty = true;
      }
    } else if (pointers.size === 2) {
      var pts = Array.from(pointers.values());
      var d = Math.hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y);
      if (pinchDist > 0 && d > 0) {
        zoomAt((pts[0].x + pts[1].x) / 2, (pts[0].y + pts[1].y) / 2, d / pinchDist);
      }
      pinchDist = d;
    }
  });
  function endPointer(ev) {
    var wasTap = drag && drag.id === ev.pointerId && drag.mode === 'pan' && !drag.moved && !multiTouch &&
      ev.type === 'pointerup';
    var p = pointers.get(ev.pointerId);
    pointers.delete(ev.pointerId);
    pinchDist = 0;
    if (pointers.size === 0) multiTouch = false;
    if (drag && drag.id === ev.pointerId) drag = null;
    if (wasTap && p) {
      // A tap only PREVIEWS the point; nothing is sent until the user confirms.
      var w = screenToWorld(state.view, canvas.width, canvas.height, p.x, p.y);
      var kind = state.pickInitial && canInitialPose() ? 'initialpose' : 'goal';
      setPending(kind, w.x, w.y, state.pending ? state.pending.yaw : 0);
    }
  }
  canvas.addEventListener('pointerup', endPointer);
  canvas.addEventListener('pointercancel', endPointer);
  canvas.addEventListener('wheel', function (ev) {
    ev.preventDefault();
    var p = localXY(ev);
    zoomAt(p.x, p.y, Math.exp(-ev.deltaY * 0.0015));
  }, { passive: false });
  canvas.addEventListener('keydown', function (ev) {
    var step = 40 / state.view.scale, v = state.view;
    if (ev.key === 'ArrowLeft') v.cx -= step;
    else if (ev.key === 'ArrowRight') v.cx += step;
    else if (ev.key === 'ArrowUp') v.cy += step;
    else if (ev.key === 'ArrowDown') v.cy -= step;
    else if (ev.key === '+' || ev.key === '=') zoomAt(canvas.width / 2, canvas.height / 2, 1.25);
    else if (ev.key === '-') zoomAt(canvas.width / 2, canvas.height / 2, 0.8);
    else return;
    ev.preventDefault();
    state.dirty = true;
  });

  $('btn-center').addEventListener('click', centerOnRobot);
  $('layer-scan').addEventListener('change', function () { state.dirty = true; });
  $('layer-plan').addEventListener('change', function () { state.dirty = true; });
  $('layer-dock').addEventListener('change', function () { state.dirty = true; });
  window.addEventListener('resize', function () { resize(); });

  // ---- commands (browser -> robot) ----
  function canInitialPose() { return !!state.status && state.status.mode === 'map'; }

  function showMessage(text, isError) {
    var m = $('msg');
    m.textContent = text;
    m.className = isError ? 'msg msg-error' : 'msg msg-ok';
    m.setAttribute('role', isError ? 'alert' : 'status');
    m.hidden = !text;
  }

  function deg(rad) { return Math.round(rad * 180 / Math.PI); }

  // Enable/disable and label the command widgets from the current state.
  function updateActions() {
    var p = state.pending, init = canInitialPose(), online = !!state.connected;
    if (state.pickInitial && !init) state.pickInitial = false;
    if (p && p.kind === 'initialpose' && !init) { state.pending = p = null; state.dirty = true; }
    $('pending-bar').hidden = !p;
    if (p) {
      $('pending-text').textContent = (p.kind === 'goal' ? 'Objetivo: ' : 'Pose inicial: ') +
        'x ' + p.x.toFixed(2) + ' m, y ' + p.y.toFixed(2) + ' m, ' + deg(p.yaw) + '°';
      $('btn-confirm').textContent = p.kind === 'goal' ? 'Confirmar objetivo' : 'Confirmar pose inicial';
      $('btn-confirm').disabled = !!state.busy.confirm || !online;
      var yawIn = $('yaw-input');
      if (document.activeElement !== yawIn) yawIn.value = String(deg(p.yaw));
    }
    $('btn-discard').disabled = !!state.busy.confirm;
    $('btn-cancel-nav').disabled = !!state.busy.cancel;
    $('btn-dock').disabled = !!state.busy.dock || !online;
    // Cancel stays enabled whenever it is not in flight: stopping must never depend on the link state.
    $('offline-hint').hidden = online;
    $('offline-hint').textContent = online ? '' :
      'Sin conexión con el robot: no se pueden enviar objetivos, regreso al dock ni pose inicial. ' +
      'Cancelar navegación sigue disponible.';
    if (!online && state.dockConfirm) state.dockConfirm = false;
    $('dock-confirm').hidden = !state.dockConfirm;
    $('btn-dock-yes').disabled = !!state.busy.dock || !online;
    var ip = $('btn-initpose');
    ip.disabled = !init || !online;
    ip.setAttribute('aria-pressed', state.pickInitial ? 'true' : 'false');
    ip.textContent = state.pickInitial ? 'Tocar el mapa para la pose inicial' : 'Fijar pose inicial';
    $('initpose-hint').textContent = !online ? 'Sin conexión: no se puede fijar la pose inicial.' : init ? '' : (state.status ?
      'Solo disponible en modo navegación (map); ahora el robot está en mapeo.' :
      'Disponible cuando se conozca el modo del robot.');
    var hasDock = !!getDock();
    $('layer-dock').disabled = !hasDock;
    $('dock-note').hidden = hasDock;
  }

  // POST helper: resolves {ok, status, data, timeout?}; never rejects (network failure -> status 0).
  // A request without answer within `timeoutMs` is aborted and resolves {status: 0, timeout: true},
  // so a half-open Wi-Fi link cannot keep a button locked.
  function postJson(path, body, timeoutMs) {
    var init = { method: 'POST', headers: { Authorization: 'Bearer ' + state.token } };
    var ctl = typeof AbortController === 'function' ? new AbortController() : null;
    if (ctl) init.signal = ctl.signal;
    if (body !== undefined) {
      init.headers['Content-Type'] = 'application/json';
      init.body = JSON.stringify(body);
    }
    return new Promise(function (resolve) {
      var settled = false;
      function finish(res) {
        if (settled) return;
        settled = true;
        clearTimeout(timer);
        resolve(res);
      }
      var timer = setTimeout(function () {
        finish({ ok: false, status: 0, data: {}, timeout: true });
        if (ctl) { try { ctl.abort(); } catch (e) { /* ignore */ } }
      }, timeoutMs || REQUEST_TIMEOUT_MS);
      fetch(path, init).then(function (r) {
        return r.json().catch(function () { return {}; }).then(function (data) {
          finish({ ok: r.ok, status: r.status, data: data || {} });
        });
      }).catch(function () { finish({ ok: false, status: 0, data: {} }); });
    });
  }

  // Run one command with a per-action in-flight lock (prevents double submits).
  function runCommand(name, path, body, okText) {
    if (state.busy[name]) return Promise.resolve(null);
    state.busy[name] = true;
    updateActions();
    var done;
    try {
      done = postJson(path, body, path === '/api/cancel' ? CANCEL_TIMEOUT_MS : REQUEST_TIMEOUT_MS);
    } catch (e) {
      done = Promise.resolve({ ok: false, status: 0, data: {} });
    }
    return done.then(function (res) {
      state.busy[name] = false;
      if (res.ok) {
        if (path === '/api/goal' || path === '/api/dock') state.goalRejected = false;
        showMessage(okText, false);
      } else {
        // Only a validation rejection (400/503) that leaves no goal running may hide the
        // failed warning; a 409 or a network error never proves the robot is stopped.
        if (path === '/api/goal' && (res.status === 400 || res.status === 503) &&
            (!state.nav || (state.nav.state !== 'active' && state.nav.state !== 'failed'))) {
          state.goalRejected = true;
        }
        showMessage(res.timeout ? 'Sin respuesta del robot; pulsa de nuevo.' :
          describeApiError(res.status, res.data && res.data.error), true);
        if (res.status === 401) {
          disconnect();
          clearToken();
          state.token = '';
          showLogin('Token inválido. Introdúcelo de nuevo.');
        }
      }
      updateActions();
      updateStatus();
      return res;
    });
  }

  function confirmPending() {
    var p = state.pending;
    if (!p || state.busy.confirm) return;
    if (!state.connected) { showMessage('Sin conexión con el robot: no se envió el comando.', true); updateActions(); return; }
    if (p.kind === 'initialpose' && !canInitialPose()) { updateActions(); return; }
    var path = p.kind === 'goal' ? '/api/goal' : '/api/initialpose';
    var okText = p.kind === 'goal' ? 'Objetivo enviado.' : 'Pose inicial enviada.';
    return runCommand('confirm', path, { x: p.x, y: p.y, yaw: p.yaw }, okText).then(function (res) {
      // Keep the preview on failure so the user can retry or adjust it.
      if (res && res.ok && state.pending === p) {
        state.pending = null;
        state.pickInitial = false;
        state.dirty = true;
        updateActions();
      }
    });
  }

  $('btn-confirm').addEventListener('click', confirmPending);
  $('btn-discard').addEventListener('click', function () {
    state.pending = null;
    state.dirty = true;
    showMessage('', false);
    updateActions();
  });
  $('btn-cancel-nav').addEventListener('click', function () {
    var r = runCommand('cancel', '/api/cancel', undefined, 'Navegación cancelada.');
    if (r) r.then(function (res) {
      if (res && !res.ok && res.status !== 401 && !res.timeout) {
        showMessage('No se pudo cancelar la navegación. Pulsa «Cancelar navegación» para reintentar. ' +
          describeApiError(res.status, res.data && res.data.error), true);
      }
    });
  });
  $('btn-dock').addEventListener('click', function () {
    if (!state.connected) { updateActions(); return; }
    state.dockConfirm = true;
    updateActions();
    $('btn-dock-yes').focus();
  });
  $('btn-dock-no').addEventListener('click', function () {
    state.dockConfirm = false;
    updateActions();
  });
  $('btn-dock-yes').addEventListener('click', function () {
    if (!state.connected) { state.dockConfirm = false; updateActions(); return; }
    state.dockConfirm = false;
    runCommand('dock', '/api/dock', undefined, 'Volviendo al dock.');
  });
  $('btn-initpose').addEventListener('click', function () {
    if (!canInitialPose() || !state.connected) return;
    state.pickInitial = !state.pickInitial;
    if (!state.pickInitial && state.pending && state.pending.kind === 'initialpose') {
      state.pending = null;
      state.dirty = true;
    }
    if (state.pickInitial && state.pending) state.pending.kind = 'initialpose';
    showMessage(state.pickInitial ? 'Toca el mapa donde está el robot y arrastra para indicar su orientación.' : '', false);
    updateActions();
  });
  $('yaw-input').addEventListener('input', function () {
    var v = parseFloat($('yaw-input').value);
    if (state.pending && isNum(v)) {
      state.pending.yaw = normalizeYaw(v * Math.PI / 180);
      state.dirty = true;
    }
  });
  $('yaw-input').addEventListener('change', updateActions);

  // ---- main loop ----
  function frame() {
    var now = performance.now();
    // Stale pose/scan fade out with time even if no message arrives to mark the view dirty.
    var stale = now - state.lastPoseAt > NO_TF_AFTER_S * 1000;
    if (stale !== state.wasStale) { state.wasStale = stale; state.dirty = true; }
    if (state.dirty) {
      state.dirty = false;
      try { draw(); } catch (e) { ctx.globalAlpha = 1; ctx.setTransform(1, 0, 0, 1, 0, 0); }
    }
    requestAnimationFrame(frame);
  }

  resize();
  setInterval(updateStatus, 500);
  state.token = loadToken();
  if (state.token && !isValidToken(state.token)) { clearToken(); state.token = ''; }
  if (state.token) { connect(); } else { showLogin(''); }
  updateStatus();
  updatePanel();
  requestAnimationFrame(frame);
})();
