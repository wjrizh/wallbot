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
