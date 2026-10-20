/* ==================== modules/camera.js ==================== */
/*
 * modules/camera.js - 双路摄像头 MJPEG 回传
 * 说明: 通过 <img src="http://<ip>:81/stream"> 直接拉取 MJPEG 流.
 *   母机 = #cameraCard / #cameraBadge  (OV3660, 10.32.7.214)
 *   子机 = #cameraCard2 / #cameraBadge2 (GC2145, 10.32.7.215)
 *   wallbot.camera.connect()    // 两路同时连接
 *   wallbot.camera.disconnect() // 两路同时断开
 */
(function (global) {
  "use strict";

  var STREAM_PORT = 81;
  var RETRY_MS = 3000;
  var RETRY_LIMIT = 3;
  var CONNECT_TIMEOUT_MS = 8000;   /* 首帧等待上限, 超时重连避免永远卡在"连接中" */
  var DETECT_TIMEOUT_MS = 20000;   /* 检测模式下的首帧等待上限: 纠偏检测会切 QVGA, 出帧变慢但不代表地址失效 */
  var STORAGE_KEY = { host: "wallbot.camHost", child: "wallbot.camChild" };

  function loadSavedIp(key) {
    try { return global.localStorage && global.localStorage.getItem(STORAGE_KEY[key]) || ""; }
    catch (e) { return ""; }
  }

  function saveIp(key, ip) {
    try {
      if (!ip) global.localStorage.removeItem(STORAGE_KEY[key]);
      else global.localStorage.setItem(STORAGE_KEY[key], ip);
    } catch (e) {}
  }

  // 每路摄像头一个通道: 独立 IP / 容器 / badge / <img> / 重连
  // ip 由 App 自动发现写入 (setIp), 为空表示尚未发现
  var CHANNELS = [
    { key: "host",  ip: "", cardSel: "#cameraCard",  badgeSel: "#cameraBadge"  },
    { key: "child", ip: "", cardSel: "#cameraCard2", badgeSel: "#cameraBadge2" }
  ];

  function Channel(cfg, app) {
    this.app = app;
    this.key = cfg.key;
    this.ip = loadSavedIp(cfg.key) || cfg.ip;
    this.cardSel = cfg.cardSel;
    this.badgeSel = cfg.badgeSel;
    this.card = null;
    this.badge = null;
    this.img = null;
    this.placeholder = null;
    this.retryTimer = null;
    this.retryCount = 0;
    this.detectMode = false;   /* true: 纠偏检测中, 首帧超时/重连不计失败, 不触发重新发现 */
  }

  /* 纯图像反馈版本: 子机与母机都走 :81/stream 普通 MJPEG
   * (检测可视化 /view 暂时停用, 用于验证基础链路) */
  Channel.prototype.streamUrl = function () {
    if (!this.ip) return "";
    var ip = this.ip.replace(/^https?:\/\//, "").replace(/\/.*$/, "");
    if (ip.indexOf(":") >= 0) return "http://" + ip + "/stream";
    return "http://" + ip + ":" + STREAM_PORT + "/stream";
  };

  Channel.prototype._setBadge = function (text, state) {
    if (!this.badge) return;
    this.badge.textContent = text;
    this.badge.className = "pill " + state;
  };

  Channel.prototype._clearRetry = function () {
    if (this.retryTimer) { clearTimeout(this.retryTimer); this.retryTimer = null; }
  };

  Channel.prototype._clearConnectTimeout = function () {
    if (this._connectTimer) { clearTimeout(this._connectTimer); this._connectTimer = null; }
  };

  Channel.prototype._removeMedia = function () {
    if (this.img) {
      this.img.onload = null;
      this.img.onerror = null;
      this.img.src = "";
      if (this.img.parentNode) this.img.parentNode.removeChild(this.img);
      this.img = null;
    }
  };

  Channel.prototype._showPlaceholder = function () {
    if (this.placeholder && !this.placeholder.parentNode && this.card) {
      this.card.appendChild(this.placeholder);
    }
  };

  Channel.prototype._hidePlaceholder = function () {
    if (this.placeholder && this.placeholder.parentNode) {
      this.placeholder.parentNode.removeChild(this.placeholder);
    }
  };

  Channel.prototype._scheduleRetry = function () {
    var self = this;
    this._clearRetry();
    /* 检测模式下流会短暂停顿(传感器切 QVGA), 这不是地址失效:
     * 重连计数不累加, 用短间隔快速恢复画面。 */
    if (this.detectMode || (this.key === "child" && this._softPause)) {
      this._softPause = false;
      this.retryCount = 0;
      this._setBadge("检测中…", "off");
      this.retryTimer = setTimeout(function () { self.connect(); }, 1500);
      return;
    }
    if (this.retryCount >= RETRY_LIMIT) {
      /* 缓存地址失效: 通知 App 触发重新发现 */
      if (this.app && typeof this.app._onCameraGiveUp === "function") this.app._onCameraGiveUp(this.key);
      return;
    }
    this.retryCount++;
    this.retryTimer = setTimeout(function () { self.connect(); }, RETRY_MS);
  };

  Channel.prototype.mount = function () {
    this.card = UI.$(this.cardSel);
    this.badge = UI.$(this.badgeSel);
    var byId = document.getElementById(this.cardSel.slice(1));
    var allCards = document.querySelectorAll('.camera-card').length;
    var dockEl = document.getElementById('cameraDock');
    UI.log("[cam] mount " + this.key + " card=" + (this.card ? "ok" : "NULL") +
           " byId=" + (byId ? "ok" : "NULL") +
           " 卡片数=" + allCards +
           " dock=" + (dockEl ? "ok" : "NULL") +
           " rs=" + document.readyState);
    if (!this.card) return;
    this.placeholder = UI.el("div", { class: "camera-placeholder" }, [
      UI.el("div", { class: "camera-icon" }),
      UI.el("div", { class: "camera-hint", text: "摄像头未接入" })
    ]);
    this.card.appendChild(this.placeholder);
    this._setBadge("未接入", "off");
  };

  /* 检测模式导致的流停顿: 标记后立即重连, 不计入失败次数 */
  Channel.prototype.softReconnect = function () {
    if (!this.card || !this.img) return;
    this._softPause = true;
    this._removeMedia();
    this._hidePlaceholder();
    this.connect();
  };

  Channel.prototype.connect = function () {
    if (!this.card) return;
    this._clearRetry();
    var self = this;
    var url = this.streamUrl();
    if (!url) {
      this._setBadge("等待发现", "off");
      return;
    }
    this._removeMedia();
    this._hidePlaceholder();
    this._setBadge("连接中…", "off");

    var img = UI.el("img", { src: url, alt: "摄像头回传" });
    UI.log("[cam] connect " + this.key + " url=" + url);
    img.onload = function () {
      self._clearConnectTimeout();
      self.retryCount = 0;
      self._setBadge("已接入", "on");
      UI.log("[cam] OK " + self.key);
    };
    img.onerror = function () {
      self._clearConnectTimeout();
      self._setBadge("连接失败", "off");
      UI.log("[cam] FAIL " + self.key + " " + url);
      self._scheduleRetry();
    };
    /* 超时仍未出帧就重连一次, 避免永远卡在"连接中" */
    this._clearConnectTimeout();
    this._connectTimer = setTimeout(function () {
      if (!self.img || self.img !== img) return;
      if (self.detectMode) {
        /* 检测模式下流仍在线, 只是出帧慢: 不重连, 避免把 retryCount 推满后清 IP */
        UI.log("[cam] " + self.key + " 检测模式首帧慢, 保持连接");
        return;
      }
      UI.log("[cam] " + self.key + " 首帧超时, 重连");
      self._setBadge("等待画面", "off");
      self.connect();
    }, this.detectMode ? DETECT_TIMEOUT_MS : CONNECT_TIMEOUT_MS);
    this.card.appendChild(img);
    this.img = img;

    this.app && this.app.transport && this.app.transport._emit &&
      this.app.transport._emit("log", "摄像头[" + this.key + "] 拉流 " + url);
  };

  Channel.prototype.disconnect = function () {
    this._clearRetry();
    this._clearConnectTimeout();
    this._removeMedia();
    this._showPlaceholder();
    this._setBadge("未接入", "off");
  };

  function CameraModule(app) {
    this.app = app;
    this.channels = CHANNELS.map(function (cfg) { return new Channel(cfg, app); });
  }

  CameraModule.prototype.mount = function () {
    // mount 无参: 各通道自解析 card/badge 选择器 (母机/子机在常驻层)
    this.channels.forEach(function (ch) { ch.mount(); });
  };

  CameraModule.prototype.connect = function () {
    this.channels.forEach(function (ch) { if (!ch.img) ch.connect(); });
  };

  /* 地址变更订阅: fn(key, ip) — 供纠偏等模块复用自动发现结果 */
  CameraModule.prototype.onIpChange = function (fn) {
    if (typeof fn !== "function") return;
    (this._ipListeners || (this._ipListeners = [])).push(fn);
  };

  CameraModule.prototype._emitIp = function (key, ip) {
    var list = this._ipListeners || [];
    for (var i = 0; i < list.length; i++) {
      try { list[i](key, ip); } catch (e) {}
    }
  };

  /* 自动发现结果写入; 已有地址变化时断开并重连 */
  CameraModule.prototype.setIp = function (key, ip) {
    var ch = this.channels.filter(function (c) { return c.key === key; })[0];
    if (!ch || !ip || ch.ip === ip) return;
    ch.ip = ip;
    ch.retryCount = 0;
    saveIp(key, ip);
    if (ch.img) { ch.disconnect(); ch.connect(); }
    this._emitIp(key, ip);
  };

  CameraModule.prototype.clearIp = function (key) {
    var ch = this.channels.filter(function (c) { return c.key === key; })[0];
    if (!ch) return;
    ch.ip = "";
    ch.retryCount = 0;
    saveIp(key, "");
    this._emitIp(key, "");
  };

  CameraModule.prototype.disconnect = function () {
    this.channels.forEach(function (ch) { ch.disconnect(); });
  };

  /* 视觉纠偏请求前调用: 主动重连子机流, 避免检测切模式时残留半帧 */
  CameraModule.prototype.softReconnect = function (key) {
    var ch = this.channels.filter(function (c) { return c.key === key; })[0];
    if (ch) ch.softReconnect();
  };

  /* 纠偏检测开关: 检测期间放宽超时与重试, 不因出帧慢判定地址失效 */
  CameraModule.prototype.setDetectMode = function (key, on) {
    var ch = this.channels.filter(function (c) { return c.key === key; })[0];
    if (!ch) return;
    ch.detectMode = !!on;
    if (on) {
      ch.retryCount = 0;
      ch._clearRetry();
    }
  };

  CameraModule.prototype.getIp = function (key) {
    var ch = this.channels.filter(function (c) { return c.key === key; })[0];
    return ch ? ch.ip : "";
  };

  CameraModule.prototype.onState = function () {};

  global.CameraModule = CameraModule;
})(typeof window !== "undefined" ? window : globalThis);
/* ==================== modules/align.js ==================== */
/*
 * modules/align.js - 子机视觉纠偏 (摄像头子机 /align/* 反馈显示)
 *
 * 本版只做【测量 + 显示】: 闭环控制由母机侧 align_bridge 负责, 暂未启用。
 * 反馈接口 (HTTP 直连摄像头子机, 明文已放行):
 *   GET /align/get         -> ok/dt_ms/id/hamming/phi_deg/psi_deg/phi_shape_deg/
 *                             axial_mm/arc_mm/perp_mm/aligned/angle_tol
 *   GET /align/ref         -> 记录当前位姿为参考位
 *   GET /align/ref?clear=1 -> 清除参考位
 * 面板为右侧抽屉, 打开时摄像头区域自动让位, 不遮挡画面。
 */
(function (global) {
  "use strict";

  var POLL_MS = 1200;
  var LOG_POLL_MS = 2000;

  var FIELDS = [
    ["phi_deg",       "\u5468\u5411 phi",   "\u00b0", 2],
    ["psi_deg",       "\u822a\u5411 psi",   "\u00b0", 2],
    ["phi_shape_deg", "\u5f62\u53d8 phi",   "\u00b0", 2],
    ["axial_mm",      "\u8f74\u5411",       "mm",  1],
    ["perp_mm",       "\u5782\u76f4",       "mm",  1],
    ["arc_mm",        "\u5f27\u957f",       "mm",  1],
    ["id",            "Tag ID",             "",    0],
    ["hamming",       "\u6c49\u660e\u8ddd", "",    0],
    ["dt_ms",         "\u8017\u65f6",       "ms",  0]
  ];

  function AlignModule(app) {
    this.app = app;
    this._drawer = null;
    this._ipInput = null;
    this._statusEl = null;
    this._valueEls = {};
    this._alignEl = null;
    this._tagEl = null;
    this._refEl = null;
    this._timer = null;
    this._busy = false;
  }

  AlignModule.prototype.mount = function () {
    var self = this;
    var entry = document.getElementById("btnAlign");
    if (entry) entry.addEventListener("click", function () { self.open(); });
    this._build();
    var cam = this._camera();
    if (cam && typeof cam.onIpChange === "function") {
      cam.onIpChange(function (key, ip) {
        if (key === "child") self._syncIp(ip);
      });
    }
  };

  AlignModule.prototype._camera = function () {
    return this.app && this.app.camera ? this.app.camera : null;
  };

  AlignModule.prototype._camIp = function () {
    var typed = this._ipInput ? this._ipInput.value.trim() : "";
    if (typed) return typed;
    var cam = this._camera();
    return cam ? (cam.getIp("child") || "") : "";
  };

  AlignModule.prototype._syncIp = function (ip) {
    if (!this._ipInput) return;
    this._ipInput.placeholder = ip || "\u672a\u53d1\u73b0 (\u81ea\u52a8\u53d1\u73b0\u4e2d)";
    if (!this._ipInput.value.trim() && ip) this._ipInput.value = ip;
  };

  AlignModule.prototype._row = function (label) {
    var row = document.createElement("div");
    row.className = "align-row";
    var k = document.createElement("span");
    k.className = "align-k";
    k.textContent = label;
    var v = document.createElement("span");
    v.className = "align-v";
    v.textContent = "--";
    row.appendChild(k); row.appendChild(v);
    return { row: row, val: v };
  };

  AlignModule.prototype._button = function (label, cls, fn) {
    var b = document.createElement("button");
    b.className = "btn " + cls + " btn-wide";
    b.style.marginTop = "8px";
    b.textContent = label;
    b.addEventListener("click", fn);
    return b;
  };

  AlignModule.prototype._build = function () {
    var self = this;
    var drawer = document.createElement("div");
    drawer.className = "align-drawer";

    var head = document.createElement("div");
    head.className = "debug-head";
    var title = document.createElement("div");
    title.className = "debug-title";
    title.textContent = "\u5b50\u673a\u89c6\u89c9\u7ea0\u504f";
    var closeBtn = document.createElement("button");
    closeBtn.className = "debug-close";
    closeBtn.textContent = "\u00d7";
    closeBtn.addEventListener("click", function () { self.close(); });
    head.appendChild(title); head.appendChild(closeBtn);

    var body = document.createElement("div");
    body.className = "align-body";

    var note = document.createElement("div");
    note.className = "align-note";
    note.textContent = "\u672c\u7248\u53ea\u505a\u6d4b\u91cf\u4e0e\u663e\u793a, \u4e0d\u4e0b\u53d1\u63a7\u5236\u6307\u4ee4\u3002\u8bf7\u624b\u52a8\u6539\u53d8\u6807\u7b7e\u4f4d\u59ff\u89c2\u5bdf\u6570\u503c\u53d8\u5316\u3002";
    body.appendChild(note);

    var ipRow = document.createElement("div");
    ipRow.className = "align-row";
    var ipLabel = document.createElement("span");
    ipLabel.className = "align-k";
    ipLabel.textContent = "\u6444\u50cf\u5934\u5b50\u673a IP";
    var ipInput = document.createElement("input");
    ipInput.className = "debug-input";
    ipInput.type = "text";
    ipInput.style.maxWidth = "190px";
    var cam = this._camera();
    var autoIp = cam ? (cam.getIp("child") || "") : "";
    ipInput.placeholder = autoIp || "\u672a\u53d1\u73b0 (\u81ea\u52a8\u53d1\u73b0\u4e2d)";
    ipInput.value = autoIp;
    this._ipInput = ipInput;
    ipRow.appendChild(ipLabel); ipRow.appendChild(ipInput);
    body.appendChild(ipRow);

    var stRow = this._row("\u72b6\u6001");
    this._statusEl = stRow.val;
    body.appendChild(stRow.row);
    var refRow = this._row("\u53c2\u8003\u4f4d");
    this._refEl = refRow.val;
    body.appendChild(refRow.row);

    var t1 = document.createElement("div");
    t1.className = "debug-section-title";
    t1.style.marginTop = "14px";
    t1.textContent = "\u6444\u50cf\u5934\u53cd\u9988 (\u5b9e\u65f6)";
    body.appendChild(t1);

    FIELDS.forEach(function (f) {
      var r = self._row(f[1]);
      self._valueEls[f[0]] = { el: r.val, unit: f[2], dec: f[3] };
      body.appendChild(r.row);
    });

    var alignRow = this._row("\u5bf9\u51c6\u5224\u636e");
    this._alignEl = alignRow.val;
    body.appendChild(alignRow.row);

    var tagRow = this._row("AprilTag");
    this._tagEl = tagRow.val;
    body.appendChild(tagRow.row);

    body.appendChild(this._button("\u8bb0\u5f55\u53c2\u8003\u4f4d", "btn-primary", function () { self._refCmd("ref"); }));
    body.appendChild(this._button("\u6e05\u9664\u53c2\u8003\u4f4d", "btn-gray", function () { self._refCmd("clear"); }));
    body.appendChild(this._button("\u5237\u65b0\u4e00\u6b21", "btn-gray", function () { self._poll(); }));

    var t2 = document.createElement("div");
    t2.className = "debug-section-title";
    t2.style.marginTop = "16px";
    t2.textContent = "\u6444\u50cf\u5934\u7aef\u65e5\u5fd7 (\u81ea\u52a8\u5237\u65b0)";
    body.appendChild(t2);
    var logBox = document.createElement("div");
    logBox.className = "align-log";
    logBox.textContent = "--";
    this._logEl = logBox;
    body.appendChild(logBox);

    drawer.appendChild(head);
    drawer.appendChild(body);
    document.body.appendChild(drawer);
    this._drawer = drawer;
  };

  AlignModule.prototype._httpJson = function (path, cb) {
    var ip = this._camIp();
    if (!ip) { cb(new Error("no_ip"), null); return; }
    var url = "http://" + ip.replace(/^https?:\/\//, "").replace(/\/.*$/, "") + path;
    var xhr = new XMLHttpRequest();
    xhr.open("GET", url, true);
    xhr.timeout = 3000;
    xhr.onload = function () {
      if (xhr.status !== 200) { cb(new Error("http_" + xhr.status), null); return; }
      try { cb(null, JSON.parse(xhr.responseText)); }
      catch (e) { cb(e, null); }
    };
    xhr.onerror = function () { cb(new Error("net"), null); };
    xhr.ontimeout = function () { cb(new Error("timeout"), null); };
    try { xhr.send(); } catch (e) { cb(e, null); }
  };

  AlignModule.prototype._setStatus = function (text, bad) {
    if (!this._statusEl) return;
    this._statusEl.textContent = text;
    this._statusEl.style.color = bad ? "#e25555" : "";
  };

  AlignModule.prototype._render = function (d) {
    var self = this;
    FIELDS.forEach(function (f) {
      var slot = self._valueEls[f[0]];
      if (!slot) return;
      var v = d[f[0]];
      slot.el.textContent = (v === undefined || v === null)
        ? "--" : (Number(v).toFixed(f[3]) + (f[2] ? " " + f[2] : ""));
    });
    if (this._alignEl) {
      this._alignEl.textContent = d.aligned ? "\u5df2\u5bf9\u51c6" : "\u672a\u5bf9\u51c6";
      this._alignEl.style.color = d.aligned ? "#2e9e4f" : "#e25555";
    }
    if (this._tagEl) {
      this._tagEl.textContent = "id=" + (d.id === undefined ? "--" : d.id) +
        " \u6c49\u660e=" + (d.hamming === undefined ? "--" : d.hamming) +
        " \u5bb9\u5dee=" + (d.angle_tol === undefined ? "--" : d.angle_tol) + "\u00b0";
    }
  };

  /* 拉取摄像头端日志 (替代串口, 直接显示在面板) */
  AlignModule.prototype._pollLog = function () {
    var self = this;
    if (!this._drawer || !this._drawer.classList.contains("show")) return;
    var ip = this._camIp();
    if (!ip || !this._logEl) return;
    var url = "http://" + ip.replace(/^https?:\/\//, "").replace(/\/.*$/, "") + "/log";
    var xhr = new XMLHttpRequest();
    xhr.open("GET", url, true);
    xhr.timeout = 4000;
    xhr.onload = function () {
      if (xhr.status !== 200) return;
      var text = xhr.responseText || "(\u7a7a)";
      if (text === self._lastLog) return;
      self._lastLog = text;
      var atBottom = self._logEl.scrollHeight - self._logEl.scrollTop - self._logEl.clientHeight < 24;
      self._logEl.textContent = text;
      if (atBottom) self._logEl.scrollTop = self._logEl.scrollHeight;
    };
    try { xhr.send(); } catch (e) {}
  };

  AlignModule.prototype._poll = function () {
    var self = this;
    if (this._busy) return;
    if (!this._drawer || !this._drawer.classList.contains("show")) return;
    this._busy = true;
    this._pollLog();
    this._httpJson("/align/get", function (err, d) {
      self._busy = false;
      if (err) { self._setStatus("\u6444\u50cf\u5934\u4e0d\u53ef\u8fbe (" + err.message + ")", true); return; }
      if (!d) { self._setStatus("\u65e0\u54cd\u5e94", true); return; }
      if (!d.ok) { self._setStatus("\u5931\u8d25: " + (d.reason || "?"), true); return; }
      self._setStatus("\u6b63\u5e38 (" + (d.dt_ms || 0) + "ms)", false);
      self._render(d);
    });
  };

  AlignModule.prototype._startPoll = function () {
    var self = this;
    this._stopPoll();
    this._poll();
    this._timer = setInterval(function () { self._poll(); }, POLL_MS);
    this._logTimer = setInterval(function () { self._pollLog(); }, LOG_POLL_MS);
  };

  AlignModule.prototype._stopPoll = function () {
    if (this._timer) { clearInterval(this._timer); this._timer = null; }
    if (this._logTimer) { clearInterval(this._logTimer); this._logTimer = null; }
  };

  AlignModule.prototype._refCmd = function (kind) {
    var self = this;
    if (this._busy) { UI.toast("\u4e0a\u4e00\u6b21\u8bf7\u6c42\u8fdb\u884c\u4e2d"); return; }
    this._busy = true;
    var path = kind === "clear" ? "/align/ref?clear=1" : "/align/ref";
    this._setStatus(kind === "clear" ? "\u6e05\u9664\u4e2d..." : "\u62cd\u6444\u8ba1\u7b97\u4e2d...", false);
    /* 子机格子已常态拉 /view (检测模式), /align/ref 不再切换模式, 无需重连 */
    this._httpJson(path, function (err, d) {
      self._busy = false;
      if (err) { self._setStatus("\u5931\u8d25: " + err.message, true); UI.toast("\u5931\u8d25: " + err.message); return; }
      if (d && d.ok) {
        if (kind === "clear") {
          self._setStatus("\u53c2\u8003\u4f4d\u5df2\u6e05\u9664", false);
          if (self._refEl) self._refEl.textContent = "\u65e0";
          UI.toast("\u53c2\u8003\u4f4d\u5df2\u6e05\u9664");
        } else {
          self._setStatus("\u53c2\u8003\u4f4d\u5df2\u8bb0\u5f55", false);
          if (self._refEl) self._refEl.textContent = "u=" + d.u + " v=" + d.v + " \u03b8=" + d.theta_deg + "\u00b0";
          UI.toast("\u53c2\u8003\u4f4d\u5df2\u8bb0\u5f55");
        }
      } else {
        var reason = (d && d.reason) || "?";
        self._setStatus("\u5931\u8d25: " + reason, true);
        UI.toast("\u5931\u8d25: " + reason);
      }
    });
  };

  /* 纠偏轮询会让子机进入检测模式(分辨率切 QVGA), 通知 camera 放宽超时,
   * 否则首帧超时会把 retryCount 推满 -> 清 IP -> 重新发现 -> 视频卡死 */
  AlignModule.prototype._setCamDetect = function (on) {
    var cam = this._camera();
    if (cam && typeof cam.setDetectMode === "function") cam.setDetectMode("child", on);
  };

  AlignModule.prototype.open = function () {
    if (!this._drawer) return;
    this._syncIp(this._camera() ? this._camera().getIp("child") : "");
    this._drawer.classList.add("show");
    document.body.classList.add("align-open");
    this._setCamDetect(true);
    this._startPoll();
  };

  AlignModule.prototype.close = function () {
    if (this._drawer) this._drawer.classList.remove("show");
    document.body.classList.remove("align-open");
    this._stopPoll();
    this._setCamDetect(false);
  };

  AlignModule.prototype.onState = function () {};

  global.AlignModule = AlignModule;
})(typeof window !== "undefined" ? window : globalThis);

/* ==================== app.js (发现/地址相关节选) ==================== */
/*
 * app.js - 应用入口
 * 职责: 组装传输层与业务模块, 绑定全局 UI, 持久化地址, 母机/子机界面切换
 * 性能: 状态帧以 rAF 合并刷新, 避免高频重排
 * 扩展: 新增模块时实现 mount()/onState(), 在 App.modules 中注册即可
 */
(function (global) {
  'use strict';

  var HOST_STORAGE_KEY = 'wallbot.host';

  function App() {
    this.transport = new Transport();
    this.view = 'host';
    this.modules = [
      new CanModule(this),
      new GaitModule(this),
      new CoopModule(this),
      new CameraModule(this),
      new JoystickModule(this),
      new AeModule(this),
      new DebugModule(this),
      new GripperModule(this),
      new CoopPadModule(this),
      new AlignModule(this),
    ];
    this._connDot = null;
    this._connText = null;
    this._pendingState = null;
    this._frameRequested = false;
    this._discovering = false;
    this._discoverTimer = null;
    this._camGiveUp = {};
  }

  App.prototype.init = function () {
    var self = this;
    this._connDot = UI.$('#connDot');
    this._connText = UI.$('#connText');

  };

  /* 公开摄像头模块: 供其他模块复用自动发现的摄像头地址 */
  Object.defineProperty(App.prototype, 'camera', {
    get: function () { return this._cameraModule() || null; }
  });

  App.prototype._loadSavedHost = function () {
    try {
      var saved = global.localStorage && global.localStorage.getItem(HOST_STORAGE_KEY);
      if (saved) this.transport.setHost(saved);
    } catch (e) {}
    this._syncHostInputs();
  };

  App.prototype._syncHostInputs = function () {
    var hostEl = UI.$('#hostInput');
    if (hostEl) hostEl.value = this.transport.getHost() || '';
    var cam = this._cameraModule();
    var camHostEl = UI.$('#camHostInput');
    var camChildEl = UI.$('#camChildInput');
    if (cam && camHostEl) camHostEl.value = cam.getIp('host') || '';
    if (cam && camChildEl) camChildEl.value = cam.getIp('child') || '';
  };

  /* 手动兜底: 用户填写地址后应用 */
  App.prototype._applyManualHost = function () {
    var hostEl = UI.$('#hostInput');
    var camHostEl = UI.$('#camHostInput');
    var camChildEl = UI.$('#camChildInput');
    var host = hostEl ? hostEl.value.trim() : '';
    var camHost = camHostEl ? camHostEl.value.trim() : '';
    var camChild = camChildEl ? camChildEl.value.trim() : '';
    this._applyDiscovery({ host: host, camHost: camHost, camChild: camChild });
    UI.log('[地址] 已应用 母机=' + (host || '-') + ' 摄像头母机=' + (camHost || '-') + ' 摄像头子机=' + (camChild || '-'));
  };

  /* 启动流程: 有缓存地址先直接用, 同时起延时兜底; 无缓存立即发现 */
  App.prototype._bootAddressFlow = function () {
    var self = this;
    var hasHost = !!this.transport.getHost();
    var cam = this._cameraModule();
    var hasCam = !!(cam && (cam.getIp('host') || cam.getIp('child')));

    if (hasHost || hasCam) {
      UI.log('[地址] 复用缓存: 母机=' + (this.transport.getHost() || '-') +
             ' 摄像头母机=' + (cam ? cam.getIp('host') || '-' : '-') +
             ' 摄像头子机=' + (cam ? cam.getIp('child') || '-' : '-'));
      this._scheduleDiscoverFallback(6000);
    } else {
      this._autoDiscover('启动无缓存');
    }
  };

  /* 缓存失效兜底: 超时仍未连上则重新发现 */
  App.prototype._scheduleDiscoverFallback = function (delay) {
    var self = this;
    if (this._discoverTimer) clearTimeout(this._discoverTimer);
    this._discoverTimer = setTimeout(function () {
      self._discoverTimer = null;
      if (self.transport.isOnline()) return;
      UI.log('[发现] 缓存地址未连通, 重新扫描');
      self._autoDiscover('缓存失效');
    }, delay);
  };

  /* WebSocket 断开: 缓存地址连不上才重新发现, 连过又断先靠重连 */
  App.prototype._onWsOffline = function (info) {
    if (this._discovering) return;
    if (this._discoverTimer) return;
    if (!this.transport.getHost()) return;

    var everOnline = info && info.everOnline;
    var retry = (info && info.retry) || 0;
    if (everOnline && retry < 3) {
      UI.log('[连接] 掉线重连中 (第 ' + (retry + 1) + ' 次)');
      return;
    }
    UI.log('[连接] 地址 ' + this.transport.getHost() + ' 连不上, 触发重新发现');
    this._scheduleDiscoverFallback(everOnline ? 3000 : 1000);
  };

  /* 某路摄像头重试耗尽: 清掉该路缓存并重新发现 */
  App.prototype._onCameraGiveUp = function (key) {
    if (this._camGiveUp[key]) return;
    this._camGiveUp[key] = true;
    var cam = this._cameraModule();
    if (cam) cam.clearIp(key);
    UI.log('[发现] 摄像头[' + key + '] 地址失效, 重新扫描');
    this._autoDiscover('摄像头地址失效');
  };

  /* 应用发现结果: 母机 WS 地址 + 两路摄像头地址 */
  App.prototype._applyDiscovery = function (res) {
    if (!res) return;
    var host = res.host || '';
    if (host && host !== this.transport.getHost()) {
      this.transport.setHost(host);
      try { global.localStorage.setItem(HOST_STORAGE_KEY, host); } catch (e) {}
      UI.log('[发现] 母机 ' + host);
      this.transport.resetForNewHost();
      this.transport.disconnect();
      this.transport.connect();
    }

    var cam = this._cameraModule();
    if (cam) {
      if (res.camHost) { cam.setIp('host', res.camHost); this._camGiveUp.host = false; }
      if (res.camChild) { cam.setIp('child', res.camChild); this._camGiveUp.child = false; }
      cam.connect();
    }
    this._syncHostInputs();
    if (!host && !res.camHost && !res.camChild) UI.log('[发现] 未收到任何设备应答');
    else if (this.transport.isOnline()) this._clearDiscoverTimer();
  };

  App.prototype._clearDiscoverTimer = function () {
    if (this._discoverTimer) { clearTimeout(this._discoverTimer); this._discoverTimer = null; }
  };

  /* 原生 UDP 广播发现; 非 Capacitor 环境降级为使用已保存地址 */
  App.prototype._autoDiscover = function (reason) {
    var self = this;
    if (this._discovering) return;
    var plugin = global.Capacitor && global.Capacitor.Plugins && global.Capacitor.Plugins.Discovery;
    if (!plugin || typeof plugin.discover !== 'function') {
      UI.log('[发现] 原生插件不可用, 使用已保存地址');
      return;
    }
    this._discovering = true;
    UI.log('[发现] 正在扫描局域网…' + (reason ? ' (' + reason + ')' : ''));
    plugin.discover().then(function (res) {
      self._discovering = false;
      self._applyDiscovery(res);
    }).catch(function (err) {
      self._discovering = false;
      UI.log('[发现] 失败: ' + (err && err.message));
    });
  };

  App.prototype._onStatus = function (status) {
    var map = {
      online:     { dot: 'dot-on',   text: '已连接' },
      connecting: { dot: 'dot-off',  text: '未连接' },
      offline:    { dot: 'dot-off',  text: '未连接' }
    };
    var cfg = map[status] || map.offline;
    if (this._connDot) this._connDot.className = 'dot ' + cfg.dot;
    if (this._connText) this._connText.textContent = cfg.text;

    if (status === 'online') {
      this._hideMask();
      this._clearDiscoverTimer();
      this._camGiveUp = {};
    }
  };

  document.addEventListener('DOMContentLoaded', function () {
    try {
      NativeAdapter.init();
      var app = new App();
      app.init();
      global.wallbot = app;
    } catch (err) {
      UI.log('[FATAL] init 崩溃: ' + (err && err.message) + ' @ ' + (err && err.stack ? err.stack.split('\n')[1] : ''), true);
    }
  });

  global.App = App;
})(typeof window !== 'undefined' ? window : globalThis);