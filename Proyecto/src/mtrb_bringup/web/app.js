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
  function clampAbs(v, max) {
    if (!isNum(v)) return 0;
    return Math.max(-max, Math.min(max, v));
  }
  function normalizeYaw(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a <= -Math.PI) a += 2 * Math.PI;
    return a;
  }
  var REQUEST_TIMEOUT_MS = 8000;   // default POST timeout
  var CANCEL_TIMEOUT_MS = 3000;    // /api/cancel and /api/stop must fail fast to allow an immediate retry
  // INITIAL, must match web.yaml (teleop.max_vx / teleop.max_wz). The server saturates anyway;
  // the client never asks for more than this.
  var MSG_TOO_MANY = 'Demasiados intentos con un token incorrecto. Espera un minuto y vuelve a probar.';
  var LOCKOUT_RETRY_MS = 35000;    // after a 429 / close 1008 do not reconnect sooner than this
  var MAX_VX = 0.2;                // m/s
  // INITIAL, must match web.yaml
  var MAX_WZ = 0.8;                // rad/s
  var TELEOP_PERIOD_MS = 100;      // 10 Hz while a control is held (server limit: 30 Hz)
  var TELEOP_ZERO_REPEAT_MS = 50;  // second zero after a release (the first may hit the rate limit)
  var TELEOP_IDLE_MS = 30000;      // the switch turns itself off after this long without use
  var TELEOP_MSG_THROTTLE_MS = 3000;
  var STOP_STATUS_MS = 5000;
  var JOY_KNOB_PX = 28;            // knob radius, keep in sync with style.css
  var NAV_LABELS = { idle: 'Inactivo', active: 'Navegando', succeeded: 'Llegado',
    canceled: 'Cancelado', failed: 'Fallido' };
  // User-facing Spanish message for a failed command. `status` is the HTTP status
  // (0 = no response), `code` the server's `error` field. Never exposes technical detail.
  function describeApiError(status, code, path) {
    if (status === 429 && code === 'too_many_attempts') return MSG_TOO_MANY;
    if (path === '/api/initialpose') {
      if (status === 400 && code === 'invalid_pose') return 'La pose inicial está fuera del mapa';
      if (status === 409 && code === 'goal_active') {
        return 'No se puede fijar la pose inicial mientras el robot navega o se teleopera. Detén el robot primero.';
      }
      if (status === 503 && code === 'no_map') return 'Aún no hay mapa';
    }
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

  // ---- map saving: validation and messages (mirrors the server) ----
  var MAP_NAME_RE = /^[A-Za-z0-9_][A-Za-z0-9_-]{0,39}$/;
  // The server may take ~30 s (15 s save_map + 15 s serialize_map) plus waiting for its lock.
  var SAVE_TIMEOUT_MS = 40000;
  var AGE_WARN_S = 2;              // scan/odom older than this raise a warning
  var TRAIL_MAX_POINTS = 3000;
  var TRAIL_MIN_STEP_M = 0.05;
  var LAYERS_KEY = 'mtrb_layers';
  var LAYER_IDS = { scan: 'layer-scan', plan: 'layer-plan', dock: 'layer-dock', trail: 'layer-trail' };
  var NODE_LABELS = { active: 'Activo', inactive: 'Inactivo', unconfigured: 'Sin configurar',
    finalized: 'Finalizado', unknown: 'Desconocido', unavailable: 'No disponible' };
  var MSG_INVALID_NAME = 'Nombre no válido: solo letras (sin tildes ni ñ), números, guion y guion bajo (máx. 40).';
  var MSG_NAME_DASH = 'El nombre no puede empezar por guion.';
  var MSG_NAME_RESERVED = 'Nombre reservado: elige otro nombre distinto de «map».';
  var MSG_SAVE_TIMEOUT = 'Sin respuesta del robot; comprueba en «Mapas guardados» si se guardó.';
  function isReservedMapName(n) { return typeof n === 'string' && n.toLowerCase() === 'map'; }
  function isValidMapName(n) {
    return typeof n === 'string' && MAP_NAME_RE.test(n) && !isReservedMapName(n);
  }
  // Specific Spanish reason why a name is rejected ('' when it is valid).
  function mapNameProblem(n) {
    if (typeof n !== 'string' || !n) return 'Escribe un nombre para el mapa.';
    if (n.charAt(0) === '-' && /^[A-Za-z0-9_-]{1,40}$/.test(n)) return MSG_NAME_DASH;
    if (MAP_NAME_RE.test(n) && isReservedMapName(n)) return MSG_NAME_RESERVED;
    return MAP_NAME_RE.test(n) ? '' : MSG_INVALID_NAME;
  }
  // Spanish message for a failed POST /api/map/save (status 0 = no response).
  function describeSaveError(status, code, timeout) {
    if (timeout) return MSG_SAVE_TIMEOUT;
    if (status === 401) return 'Sesión no válida: introduce el token de nuevo.';
    if (code === 'save_timeout') return 'El robot tardó demasiado en guardar el mapa; comprueba en «Mapas guardados» si se guardó.';
    if (status === 400 && code === 'invalid_name') return MSG_INVALID_NAME;
    if (status === 400 && code === 'reserved_name') return MSG_NAME_RESERVED;
    if (status === 409 && code === 'wrong_mode') return 'Solo se puede guardar el mapa en modo mapeo (mapping).';
    if (status === 409 && code === 'save_in_progress') return 'Ya hay un guardado en curso; espera a que termine.';
    if (status === 409 && code === 'backup_pending') return 'Hay un respaldo anterior sin resolver de este mapa. ' +
      'Revisa o elimina los archivos .bak en el robot.';
    if (status === 409 && code === 'map_limit') return 'Hay demasiados mapas guardados: elimina alguno en el robot antes de guardar otro.';
    if (status === 429 && code === 'save_rate_limited') return 'Espera unos segundos antes de guardar de nuevo.';
    if (status === 507 && code === 'insufficient_storage') return 'No hay espacio libre suficiente en el robot para guardar el mapa.';
    if (status === 503 && code === 'slam_unavailable') return 'SLAM no disponible: no se puede guardar el mapa ahora.';
    return describeApiError(status, code);
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
    dockConfirm: false,
    trail: [],           // flat [x0, y0, ...] of the path travelled (client side)
    saveBusy: false,     // POST /api/map/save in flight
    overwriteName: null, // name awaiting explicit overwrite confirmation
    mapsReq: 0           // sequence number of GET /api/maps (stale answers are ignored)
  };

  // Teleoperation state (disabled by default; see the teleop section).
  var teleop = {
    enabled: false,
    timer: null,         // setInterval id of the 10 Hz send loop
    running: false,
    zeroTimer: null,
    idleTimer: null,
    seq: 0,              // strictly increasing per WebSocket connection
    joyId: null,         // pointerId that owns the joystick
    joy: { x: 0, y: 0 }, // normalized joystick deflection, screen axes, |.| <= 1
    keys: {},            // pressed teleop keys
    msgText: '',
    msgAt: -1e9
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
    normalizeYaw: normalizeYaw, describeApiError: describeApiError, clampAbs: clampAbs,
    isValidMapName: isValidMapName, mapNameProblem: mapNameProblem, describeSaveError: describeSaveError
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

    if ($('layer-trail').checked && state.trail.length >= 4) {
      ctx.beginPath();
      ctx.moveTo(state.trail[0], state.trail[1]);
      for (var t = 2; t + 1 < state.trail.length; t += 2) ctx.lineTo(state.trail[t], state.trail[t + 1]);
      ctx.strokeStyle = '#ffd54f';
      ctx.lineWidth = 2 * px;
      ctx.stroke();
    }

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
    // Skip identical writes: a polite live region would re-announce them every 500 ms.
    if (b.className !== 'badge ' + cls) b.className = 'badge ' + cls;
    if ($('conn-text').textContent !== text) $('conn-text').textContent = text;
    var c = $('st-conn');
    if (c.textContent !== text) c.textContent = text;
    c.className = cls;
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
    updateRobotPanel();
    $('st-map').textContent = state.map ? state.map.meta.width + '×' + state.map.meta.height +
      ' @ ' + state.map.meta.resolution + ' m' : 'Esperando mapa…';
    updateTeleopUi();
  }

  // ---- message handling ----
  function handleJson(msg) {
    if (!msg || typeof msg.type !== 'string') return;
    switch (msg.type) {
      case 'pose':
        if (isNum(msg.x) && isNum(msg.y) && isNum(msg.yaw)) {
          state.pose = { x: msg.x, y: msg.y, yaw: msg.yaw };
          state.lastPoseAt = performance.now();
          addTrailPoint(msg.x, msg.y);
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
      case 'error': handleServerError(msg); break;
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
    updateRobotPanel();
    if ($('map-box').open) fetchMaps();
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
    if (!isValidToken(state.token)) return Promise.resolve('invalid');
    return fetch('/api/state', { headers: { Authorization: 'Bearer ' + state.token } })
      .then(function (r) { return r.status === 401 ? 'invalid' : r.status === 429 ? 'limited' : 'ok'; })
      .catch(function () { return 'ok'; });
  }

  function scheduleRetry(minMs) {
    clearTimeout(state.retryTimer);
    state.retryTimer = setTimeout(connect, Math.max(state.retryMs, minMs || 0));
    state.retryMs = Math.min(BACKOFF_MAX_MS, Math.round(state.retryMs * 1.5));
  }

  function connect() {
    if (!state.token) return;
    clearTimeout(state.retryTimer);
    var ws;
    try { ws = new WebSocket(wsUrl()); } catch (e) { scheduleRetry(); return; }
    ws.binaryType = 'arraybuffer';
    state.ws = ws;
    state.goalRejected = false; // a new connection may have missed the nav 'active'
    teleop.seq = 0;             // seq restarts with every connection
    ws.onopen = function () { /* wait for the first valid message: see markConnected */ };
    ws.onmessage = function (ev) { if (state.ws === ws) onMessage(ev); };
    ws.onclose = function (ev) {
      if (state.ws !== ws) return;
      state.connected = false;
      state.ws = null;
      teleopConnectionLost();
      var locked = !!ev && ev.code === 1008;
      state.closeNotice = locked ? MSG_TOO_MANY : (ev && ev.code === 1013) ?
        'Demasiados clientes conectados al robot. Reintentando…' : '';
      updateStatus();
      updateActions();
      updateRobotPanel();
      if (locked) {
        // Rate-limited: no probe request (it would count as another failure), long wait.
        showLogin(MSG_TOO_MANY);
        scheduleRetry(LOCKOUT_RETRY_MS);
        return;
      }
      checkTokenValid().then(function (res) {
        if (res === 'limited') {
          state.closeNotice = MSG_TOO_MANY;
          showLogin(MSG_TOO_MANY);
          updateStatus();
          scheduleRetry(LOCKOUT_RETRY_MS);
          return;
        }
        if (res === 'invalid') {
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
    teleopConnectionLost();
    if (ws) { try { ws.close(); } catch (e) { /* ignore */ } }
    updateActions();
    updateRobotPanel();
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
    // With teleoperation on, the arrow keys drive the robot instead of panning the map.
    if (teleop.enabled && teleopKeyFor(ev.key)) return;
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
  Object.keys(LAYER_IDS).forEach(function (k) {
    $(LAYER_IDS[k]).addEventListener('change', function () { state.dirty = true; saveLayers(); });
  });
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
    updateSaveUi();
  }

  // POST helper: resolves {ok, status, data, timeout?}; never rejects (network failure -> status 0).
  // A request without answer within `timeoutMs` is aborted and resolves {status: 0, timeout: true},
  // so a half-open Wi-Fi link cannot keep a button locked.
  function requestJson(method, path, body, timeoutMs) {
    var init = { method: method, headers: { Authorization: 'Bearer ' + state.token } };
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

  function postJson(path, body, timeoutMs) { return requestJson('POST', path, body, timeoutMs); }

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
          describeApiError(res.status, res.data && res.data.error, path), true);
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


  // ---- travelled path (client side), layers memory ----
  function addTrailPoint(x, y) {
    var t = state.trail, n = t.length;
    if (n >= 2 && Math.hypot(x - t[n - 2], y - t[n - 1]) < TRAIL_MIN_STEP_M) return;
    t.push(x, y);
    if (t.length > TRAIL_MAX_POINTS * 2) t.splice(0, t.length - TRAIL_MAX_POINTS * 2);
  }

  function saveLayers() {
    var o = {};
    Object.keys(LAYER_IDS).forEach(function (k) { o[k] = !!$(LAYER_IDS[k]).checked; });
    try { window.localStorage.setItem(LAYERS_KEY, JSON.stringify(o)); } catch (e) { /* ignore */ }
  }
  function loadLayers() {
    var o = null;
    try { o = JSON.parse(window.localStorage.getItem(LAYERS_KEY) || 'null'); } catch (e) { o = null; }
    if (!o || typeof o !== 'object') return;
    Object.keys(LAYER_IDS).forEach(function (k) {
      if (typeof o[k] === 'boolean') $(LAYER_IDS[k]).checked = o[k];
    });
    state.dirty = true;
  }

  // ---- robot status panel ----
  function clearChildren(el) { while (el.firstChild) el.removeChild(el.firstChild); }
  function addChild(el, child) { if (typeof el.appendChild === 'function') el.appendChild(child); }

  function fmtSeconds(v) { return v.toFixed(1).replace('.', ',') + ' s'; }

  function setAge(id, v) {
    var el = $(id);
    if (!isNum(v)) {
      el.textContent = '⚠ sin datos';
      el.className = 'warn';
    } else if (v > AGE_WARN_S) {
      el.textContent = '⚠ ' + fmtSeconds(v) + ' (datos antiguos)';
      el.className = 'warn';
    } else {
      el.textContent = fmtSeconds(v);
      el.className = '';
    }
  }

  function updateRobotPanel() {
    var st = state.status;
    $('st-battery').textContent = st && isNum(st.battery_pct) ?
      st.battery_pct.toFixed(0) + ' %' : 'No disponible';
    // Nodes: colour AND text (and a symbol) so the state never relies on colour alone.
    var list = $('st-nodes');
    clearChildren(list);
    var nodes = st && st.nodes && typeof st.nodes === 'object' ? st.nodes : null;
    var names = nodes ? Object.keys(nodes) : [];
    if (!names.length) {
      var none = document.createElement('li');
      none.textContent = 'Sin datos de nodos';
      addChild(list, none);
    }
    names.forEach(function (name) {
      var raw = nodes[name];
      var known = typeof raw === 'string' && Object.prototype.hasOwnProperty.call(NODE_LABELS, raw);
      var key = known ? raw : 'unknown';
      var li = document.createElement('li');
      li.className = 'node-' + (key === 'active' ? 'ok' : key === 'inactive' || key === 'unconfigured' ||
        key === 'unknown' ? 'warn' : 'bad');
      li.textContent = (key === 'active' ? '✔ ' : '⚠ ') + name + ': ' + NODE_LABELS[key];
      addChild(list, li);
    });
    setAge('st-scan-age', st ? st.scan_age : null);
    setAge('st-odom-age', st ? st.odom_age : null);
    $('st-stale').hidden = !st || state.connected;
    $('st-stale').textContent = st && !state.connected ?
      'Sin conexión: se muestra el último estado recibido.' : '';
  }

  // ---- save map ----
  function canSaveMap() {
    return !!state.connected && !!state.status && state.status.mode === 'mapping';
  }

  function setSaveMsg(text, kind) {
    var m = $('map-save-msg');
    m.textContent = text;
    m.className = kind === 'error' ? 'msg msg-error' : kind === 'warn' ? 'msg msg-warn' : 'msg msg-ok';
    m.setAttribute('role', kind === 'error' ? 'alert' : 'status');
    m.hidden = !text;
  }

  function setNameError(text) {
    var e = $('map-name-error');
    e.textContent = text;
    e.hidden = !text;
    $('map-name').setAttribute('aria-invalid', text ? 'true' : 'false');
  }

  function updateSaveUi() {
    var can = canSaveMap();
    // The confirmation only makes sense while saving is still possible.
    if (state.overwriteName !== null && (!can || state.saveBusy)) state.overwriteName = null;
    var asking = state.overwriteName !== null;
    $('btn-save-map').disabled = !can || state.saveBusy || asking;
    $('btn-save-map').textContent = state.saveBusy ? 'Guardando…' : 'Guardar mapa';
    $('btn-save-map').setAttribute('aria-busy', state.saveBusy ? 'true' : 'false');
    $('overwrite-confirm').hidden = !asking;
    if (asking) $('overwrite-text').textContent = 'Ya existe un mapa llamado ' + state.overwriteName +
      '. ¿Sobrescribir?';
    $('map-save-hint').textContent = !state.connected ?
      'Sin conexión con el robot: no se puede guardar el mapa.' :
      !state.status ? 'Disponible cuando se conozca el modo del robot.' :
      state.status.mode !== 'mapping' ?
        'Guardar solo está disponible en modo mapeo (mapping); ahora el robot está en navegación (map).' : '';
  }

  function handleUnauthorized() {
    disconnect();
    clearToken();
    state.token = '';
    showLogin('Token inválido. Introdúcelo de nuevo.');
  }

  function saveMap(overwrite) {
    if (state.saveBusy) return Promise.resolve(null);   // double click: one single request
    if (!canSaveMap()) { updateSaveUi(); return Promise.resolve(null); }
    var name = overwrite ? state.overwriteName : $('map-name').value;
    var problem = mapNameProblem(name);
    if (problem) { setNameError(problem); return Promise.resolve(null); }
    if (overwrite && $('map-name').value !== name) {   // the name changed after asking: never overwrite
      state.overwriteName = null; updateSaveUi(); return Promise.resolve(null);
    }
    setNameError('');
    setSaveMsg('', 'ok');
    state.saveBusy = true;
    state.overwriteName = null;
    updateSaveUi();
    var body = overwrite ? { name: name, overwrite: true } : { name: name };
    var done;
    try {
      done = postJson('/api/map/save', body, SAVE_TIMEOUT_MS);
    } catch (e) {
      done = Promise.resolve({ ok: false, status: 0, data: {} });
    }
    return done.then(function (res) {
      state.saveBusy = false;
      var code = res.data && res.data.error;
      if (res.ok) {
        var saved = res.data && typeof res.data.name === 'string' ? res.data.name : name;
        if (res.data && res.data.posegraph === false) {
          setSaveMsg('Guardado: ' + saved + '. El mapa se guardó, pero no se pudo guardar el posegraph ' +
            '(no podrás ampliarlo). Si estabas sobrescribiendo, el posegraph anterior se ha eliminado.', 'warn');
        } else {
          setSaveMsg('Guardado: ' + saved, 'ok');
        }
        fetchMaps();
      } else if (res.status === 409 && code === 'map_exists' && !overwrite) {
        state.overwriteName = name;   // explicit confirmation required before overwriting
        setSaveMsg('', 'ok');
      } else {
        setSaveMsg(describeSaveError(res.status, code, !!res.timeout), 'error');
        if (res.timeout) fetchMaps();
        if (res.status === 401) handleUnauthorized();
      }
      updateSaveUi();
      updateStatus();
      if (state.overwriteName !== null) $('btn-overwrite-no').focus();
      return res;
    });
  }

  $('btn-save-map').addEventListener('click', function () { saveMap(false); });
  $('btn-overwrite-yes').addEventListener('click', function () {
    if (state.overwriteName === null) return;
    saveMap(true);
  });
  $('btn-overwrite-no').addEventListener('click', function () {
    state.overwriteName = null;
    setSaveMsg('No se sobrescribió el mapa.', 'ok');
    updateSaveUi();
    $('map-name').focus();
  });
  $('map-name').addEventListener('input', function () {
    setNameError('');
    if (state.overwriteName !== null && $('map-name').value !== state.overwriteName) {
      state.overwriteName = null;   // a different name: the earlier confirmation no longer applies
      updateSaveUi();
    }
  });
  $('map-form').addEventListener('submit', function (ev) {
    if (ev.preventDefault) ev.preventDefault();
    saveMap(false);
  });

  // ---- saved maps list (informational only) ----
  function setMapsState(text) {
    var el = $('map-list-state');
    el.textContent = text;
    el.hidden = !text;
  }

  function renderMaps(names) {
    var ul = $('map-list');
    clearChildren(ul);
    names.forEach(function (n) {
      var li = document.createElement('li');
      li.textContent = n;   // server data: always as text, never as markup
      addChild(ul, li);
    });
  }

  function fetchMaps() {
    var req = ++state.mapsReq;
    setMapsState('Cargando mapas guardados…');
    renderMaps([]);
    return requestJson('GET', '/api/maps', undefined, REQUEST_TIMEOUT_MS).then(function (res) {
      if (req !== state.mapsReq) return res;   // a newer request owns the list
      if (res.ok && res.data && Array.isArray(res.data.maps)) {
        var names = res.data.maps.filter(function (n) { return typeof n === 'string'; });
        renderMaps(names);
        setMapsState(names.length ? '' : 'No hay mapas guardados.');
      } else {
        setMapsState(res.status === 401 ? 'Sesión no válida: introduce el token de nuevo.' :
          'No se pudo cargar la lista de mapas.');
        if (res.status === 401) handleUnauthorized();
      }
      return res;
    });
  }

  $('btn-refresh-maps').addEventListener('click', fetchMaps);
  $('map-box').addEventListener('toggle', function () { if ($('map-box').open) fetchMaps(); });

  // ---- emergency stop and teleoperation ----
  // Safety rule: when in doubt, stop. PARAR needs no confirmation, never waits for a reply and
  // does not depend on the WebSocket (the REST call is its redundant path).
  function wsOpen() { return !!state.ws && state.connected && state.ws.readyState === 1; }

  function wsSend(obj) {
    try { state.ws.send(JSON.stringify(obj)); return true; } catch (e) { return false; }
  }

  function navBlocksTeleop() {
    return !!state.nav && (state.nav.state === 'active' || state.nav.state === 'failed');
  }

  function teleopBlockReason() {
    if (!state.connected) return 'Teleoperación bloqueada: sin conexión con el robot';
    if (state.nav && state.nav.state === 'active') return 'Teleoperación bloqueada: hay una navegación activa';
    if (state.nav && state.nav.state === 'failed') {
      return 'Teleoperación bloqueada: la navegación falló y puede seguir activa; cancélala primero';
    }
    return '';
  }

  function canSendTeleop() { return teleop.enabled && wsOpen() && !navBlocksTeleop(); }

  function sendTeleop(vx, wz) {
    // seq grows even if the send fails, so it can never repeat on a connection.
    teleop.seq += 1;
    return wsSend({ type: 'teleop', vx: Math.round(clampAbs(vx, MAX_VX) * 1000) / 1000,
      wz: Math.round(clampAbs(wz, MAX_WZ) * 1000) / 1000, seq: teleop.seq });
  }

  function teleopKeyFor(key) {
    switch (key) {
      case 'w': case 'W': case 'ArrowUp': return 'fwd';
      case 's': case 'S': case 'ArrowDown': return 'back';
      case 'a': case 'A': case 'ArrowLeft': return 'left';
      case 'd': case 'D': case 'ArrowRight': return 'right';
      default: return null;
    }
  }

  // Current command from the joystick (preferred) or the keys, already saturated.
  function currentCommand() {
    var vx = 0, wz = 0;
    if (teleop.joyId !== null) {
      vx = -teleop.joy.y * MAX_VX;
      wz = -teleop.joy.x * MAX_WZ;
    } else {
      if (teleop.keys.fwd) vx += MAX_VX;
      if (teleop.keys.back) vx -= MAX_VX;
      if (teleop.keys.left) wz += MAX_WZ;
      if (teleop.keys.right) wz -= MAX_WZ;
    }
    return { vx: clampAbs(vx, MAX_VX), wz: clampAbs(wz, MAX_WZ) };
  }

  function teleopHeld() { return teleop.joyId !== null || Object.keys(teleop.keys).length > 0; }

  function showTeleopMsg(text, isError) {
    var now = performance.now();
    if (text === teleop.msgText && now - teleop.msgAt < TELEOP_MSG_THROTTLE_MS) return;  // no flooding
    teleop.msgText = text;
    teleop.msgAt = now;
    var m = $('teleop-msg');
    m.textContent = text;
    m.className = isError ? 'msg msg-error' : 'msg msg-ok';
    m.setAttribute('role', isError ? 'alert' : 'status');
    m.hidden = !text;
  }

  function moveKnob() {
    var k = $('joystick-knob');
    if (!k || !k.style) return;
    var r = joyTravel();
    k.style.transform = teleop.joyId === null ? '' :
      'translate(' + Math.round(teleop.joy.x * r) + 'px,' + Math.round(teleop.joy.y * r) + 'px)';
  }

  function joyTravel() {
    var rc = $('joystick').getBoundingClientRect();
    return Math.max(1, Math.min(rc.width, rc.height) / 2 - JOY_KNOB_PX);
  }

  function clearZeroTimer() {
    if (teleop.zeroTimer !== null) { clearTimeout(teleop.zeroTimer); teleop.zeroTimer = null; }
  }

  function stopLoop() {
    if (teleop.timer !== null) { clearInterval(teleop.timer); teleop.timer = null; }
    teleop.running = false;
  }

  // Let go of every control and end the loop. With `sendZero` an immediate zero teleop goes out
  // (twice) but only if sending is allowed; a closed WS or a busy navigation sends nothing.
  function releaseTeleop(sendZero) {
    var was = teleop.running || teleopHeld();
    stopLoop();
    teleop.joyId = null;
    teleop.joy = { x: 0, y: 0 };
    teleop.keys = {};
    moveKnob();
    if (was && sendZero && canSendTeleop()) {
      sendTeleop(0, 0);
      clearZeroTimer();
      teleop.zeroTimer = setTimeout(function () {
        teleop.zeroTimer = null;
        if (!teleop.running && canSendTeleop()) sendTeleop(0, 0);
      }, TELEOP_ZERO_REPEAT_MS);
    }
    updateTeleopUi();
  }

  function touchTeleopActivity() {
    clearTimeout(teleop.idleTimer);
    teleop.idleTimer = setTimeout(function () {
      teleop.idleTimer = null;
      setTeleopEnabled(false, 'Teleoperación desactivada por inactividad.');
    }, TELEOP_IDLE_MS);
  }

  function setTeleopEnabled(on, why) {
    if (on === teleop.enabled) { updateTeleopUi(); return; }
    if (!on) {
      releaseTeleop(true);      // zeros first (while still enabled), then switch off
      clearZeroTimer();
      clearTimeout(teleop.idleTimer);
      teleop.idleTimer = null;
    }
    teleop.enabled = on;
    if (on) touchTeleopActivity();
    $('teleop-toggle').checked = on;
    if (why) showTeleopMsg(why, false);
    else if (!on) showTeleopMsg('', false);
    updateTeleopUi();
  }

  function teleopConnectionLost() {
    teleop.seq = 0;
    if (teleop.enabled) setTeleopEnabled(false, 'Teleoperación desactivada: se perdió la conexión.');
    else releaseTeleop(false);
  }

  function teleopTick() {
    if (!canSendTeleop()) {            // never send; the reason is shown by updateTeleopUi
      releaseTeleop(false);
      return;
    }
    var c = currentCommand();
    sendTeleop(c.vx, c.wz);
    touchTeleopActivity();
  }

  // A control was pressed: start the 10 Hz loop (first message goes out at once).
  function startTeleop() {
    if (!teleop.enabled) return;
    if (!canSendTeleop()) {
      releaseTeleop(false);
      showTeleopMsg(teleopBlockReason() || 'Teleoperación no disponible.', true);
      return;
    }
    clearZeroTimer();
    if (teleop.running) return;
    teleop.running = true;
    teleopTick();
    if (teleop.running) teleop.timer = setInterval(teleopTick, TELEOP_PERIOD_MS);
    updateTeleopUi();
  }

  function handleServerError(msg) {
    switch (msg.code) {
      case 'teleop_rejected':
        releaseTeleop(false);
        showTeleopMsg('Teleoperación rechazada: otra sesión controla el robot o hay navegación activa', true);
        break;
      case 'teleop_disabled':
        releaseTeleop(false);
        showTeleopMsg('La teleoperación está desactivada en el robot', true);
        setTeleopEnabled(false, 'La teleoperación está desactivada en el robot');
        break;
      case 'invalid_teleop':
      case 'unsupported_message':
        releaseTeleop(false);
        showTeleopMsg('El robot no aceptó el comando de teleoperación.', true);
        break;
      case 'stop_unavailable':
        showStopStatus('El robot no pudo ejecutar la parada por WebSocket; pulsa de nuevo.', true);
        break;
      default: break;
    }
  }

  function updateTeleopUi() {
    if (teleop.enabled && (!state.connected || navBlocksTeleop()) && (teleop.running || teleopHeld())) {
      releaseTeleop(false);   // re-enters here once, with nothing held
      return;
    }
    var reason = teleop.enabled ? teleopBlockReason() : '';
    $('teleop-hint').textContent = reason;
    $('joystick').hidden = !teleop.enabled;
    $('st-teleop').textContent = !teleop.enabled ? 'Inactiva' :
      (reason ? 'Activa (bloqueada)' : teleop.running ? 'Activa (enviando)' : 'Activa');
    $('st-limits').textContent = '≤ ' + MAX_VX.toFixed(1).replace('.', ',') + ' m/s, ≤ ' +
      MAX_WZ.toFixed(1).replace('.', ',') + ' rad/s';
  }

  $('teleop-toggle').addEventListener('change', function () {
    setTeleopEnabled(!!$('teleop-toggle').checked, '');
  });

  // Joystick (mouse and touch via pointer events; one finger owns it).
  function joyFromEvent(ev) {
    var rc = $('joystick').getBoundingClientRect();
    var r = joyTravel();
    var x = (ev.clientX - (rc.left + rc.width / 2)) / r, y = (ev.clientY - (rc.top + rc.height / 2)) / r;
    var m = Math.hypot(x, y);
    if (!isNum(m)) { x = 0; y = 0; } else if (m > 1) { x /= m; y /= m; }
    teleop.joy = { x: x, y: y };
    moveKnob();
  }
  var joystick = $('joystick');
  joystick.addEventListener('pointerdown', function (ev) {
    if (!teleop.enabled || teleop.joyId !== null) return;
    if (ev.preventDefault) ev.preventDefault();
    try { joystick.setPointerCapture(ev.pointerId); } catch (e) { /* ignore */ }
    teleop.joyId = ev.pointerId;
    teleop.keys = {};
    joyFromEvent(ev);
    startTeleop();
  });
  joystick.addEventListener('pointermove', function (ev) {
    if (teleop.joyId === null || ev.pointerId !== teleop.joyId) return;
    joyFromEvent(ev);
  });
  function joyEnd(ev) {
    if (teleop.joyId === null || (ev && ev.pointerId !== undefined && ev.pointerId !== teleop.joyId)) return;
    releaseTeleop(true);
  }
  joystick.addEventListener('pointerup', joyEnd);
  joystick.addEventListener('pointercancel', joyEnd);
  joystick.addEventListener('lostpointercapture', joyEnd);
  joystick.addEventListener('touchcancel', function () { releaseTeleop(true); });

  // Keyboard: WASD / arrows drive; Escape is always PARAR; Space is PARAR only
  // when focus is not on an interactive control (there it keeps its native behaviour).
  function isTextEntry(t) {
    if (!t || !t.tagName) return false;
    if (t.tagName === 'TEXTAREA' || t.tagName === 'SELECT') return true;
    return t.tagName === 'INPUT' && t.type !== 'checkbox' && t.type !== 'button';
  }
  function isInteractive(t) {
    if (!t || !t.tagName) return false;
    var tag = t.tagName;
    if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || tag === 'BUTTON' ||
        tag === 'SUMMARY') return true;
    if (tag === 'A' && t.href) return true;
    if (t.isContentEditable) return true;
    var role = t.getAttribute ? t.getAttribute('role') : null;
    return role === 'button' || role === 'checkbox' || role === 'switch';
  }
  function isSpace(k) { return k === ' ' || k === 'Spacebar'; }
  window.addEventListener('keydown', function (ev) {
    var key = ev.key, t = ev.target;
    if (key === 'Escape' || (isSpace(key) && !isInteractive(t))) {
      if (ev.preventDefault) ev.preventDefault();
      if (!ev.repeat) emergencyStop();
      return;
    }
    var k = teleopKeyFor(key);
    if (!k || !teleop.enabled || isTextEntry(t) || ev.ctrlKey || ev.metaKey || ev.altKey) return;
    if (ev.preventDefault) ev.preventDefault();
    if (ev.repeat && !teleop.keys[k]) return;   // a repeat after PARAR/release must not restart
    teleop.keys[k] = true;
    teleop.joyId = null;
    startTeleop();
  });
  window.addEventListener('keyup', function (ev) {
    if (isSpace(ev.key) && ev.preventDefault && !isInteractive(ev.target)) ev.preventDefault();
    var k = teleopKeyFor(ev.key);
    if (!k || !teleop.keys[k]) return;
    delete teleop.keys[k];
    if (!teleopHeld()) releaseTeleop(true);
  });
  // Focus loss / hidden tab / touch cancel: let go at once.
  window.addEventListener('blur', function () { releaseTeleop(true); });
  window.addEventListener('touchcancel', function () { releaseTeleop(true); });
  if (document.addEventListener) {
    document.addEventListener('visibilitychange', function () {
      if (document.hidden || document.visibilityState === 'hidden') releaseTeleop(true);
    });
  }

  // PARAR: WS stop AND POST /api/stop, both fired right away and independent of each other.
  var MSG_STOP_PHYSICAL = 'USA LA PARADA FÍSICA del robot.';
  var MSG_STOP_SATURATED = 'Servidor saturado: la parada por red puede no haber llegado. USA LA PARADA FÍSICA del robot.';
  var stopStatusTimer = null;
  function showStopStatus(text, isError) {
    var el = $('stop-status');
    el.textContent = text;
    el.className = isError ? 'msg-error' : '';
    el.setAttribute('role', isError ? 'alert' : 'status');
    el.hidden = !text;
    clearTimeout(stopStatusTimer);
    stopStatusTimer = null;
    if (text && !isError) {
      stopStatusTimer = setTimeout(function () { stopStatusTimer = null; el.hidden = true; }, STOP_STATUS_MS);
    }
  }

  function emergencyStop() {
    // 1) cancel any local teleop (no zero teleop: the stop below supersedes it)
    clearZeroTimer();
    releaseTeleop(false);
    // 2) WS stop (if the link is up) and 3) REST stop, without waiting for each other
    if (wsOpen()) wsSend({ type: 'stop' });
    showStopStatus('Parada enviada', false);
    var done;
    try {
      done = postJson('/api/stop', undefined, CANCEL_TIMEOUT_MS);
    } catch (e) {
      done = Promise.resolve({ ok: false, status: 0, data: {} });
    }
    var hadWs = wsOpen();
    done.then(function (res) {
      if (res.ok) return;
      // 503 = server saturated (connection limit): the network stop may not have arrived.
      // A failed POST with no open WebSocket leaves no network path either: same warning.
      if (res.status === 503) {
        showStopStatus(MSG_STOP_SATURATED, true);
        return;
      }
      var why = res.timeout || res.status === 0 ? 'No se pudo enviar la parada por HTTP' :
        describeApiError(res.status, res.data && res.data.error).replace(/\.$/, '');
      var tail = hadWs ? '' : ' ' + MSG_STOP_PHYSICAL;
      showStopStatus('Error: ' + why + '. ¡Pulsa de nuevo!' + tail, true);
    });
  }
  $('btn-stop-all').addEventListener('click', emergencyStop);

  updateTeleopUi();

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

  loadLayers();
  resize();
  setInterval(updateStatus, 500);
  state.token = loadToken();
  if (state.token && !isValidToken(state.token)) { clearToken(); state.token = ''; }
  if (state.token) { connect(); } else { showLogin(''); }
  updateStatus();
  updatePanel();
  requestAnimationFrame(frame);
})();
