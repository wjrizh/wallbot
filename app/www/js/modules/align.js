/*
 * modules/align.js - 子机视觉纠偏 (摄像头子机 /align/* 反馈 + 闭环启停)
 *
 * 上位机动作:
 *   0x22 启动母机侧纠偏闭环 (payload 摄像头子机 IP)
 *   0x23 停止闭环
 *   0x20 子机直通帧 speed/spin/key=9   0x21 清空并停止
 * 反馈读取 (HTTP 直连摄像头子机, 明文已放行):
 *   GET /align/get     -> ok/dt_ms/id/hamming/phi_deg/psi_deg/phi_shape_deg/
 *                          axial_mm/arc_mm/perp_mm/aligned/angle_tol
 *   GET /align/ref     -> 记录当前位姿为参考位
 *   GET /align/ref?clear=1 -> 清除参考位
 *   GET /align/status  -> 参考位 + 配置概览 (纯文本)
 */
(function (global) {
  "use strict";

  var SPEED_MAX = 60;
  var POLL_MS = 1000;

  /* 展示字段: [键, 标签, 单位, 小数位] */
  var FIELDS = [
    ["phi_deg",       "\u5468\u5411 phi",   "\u00b0", 2],
    ["psi_deg",       "\u822a\u5411 psi",   "\u00b0", 2],
    ["phi_shape_deg", "\u5f62\u53d8 phi",   "\u00b0", 2],
    ["axial_mm",      "\u8f74\u5411",       "mm",  1],
    ["perp_mm",       "\u5782\u76f4",       "mm",  1],
    ["arc_mm",        "\u5f27\u957f",       "mm",  1],
    ["id",            "Tag ID",       "",    0],
    ["hamming",       "\u6c49\u660e\u8ddd", "",    0],
    ["dt_ms",         "\u8017\u65f6",       "ms",  0]
  ];

  function AlignModule(app) {
    this.app = app;
    this._mask = null;
    this._ipInput = null;
    this._stateEl = null;
    this._valueEls = {};
    this._tagEl = null;
    this._refEl = null;
    this._statusEl = null;
    this._timer = null;
    this._running = false;
  }

  AlignModule.prototype.mount = function () {
    var self = this;
    var entry = document.getElementById("btnAlign");
    if (entry) entry.addEventListener("click", function () { self.open(); });
    this._build();
  };

  AlignModule.prototype._camIp = function () {
    var ip = this._ipInput ? this._ipInput.value.trim() : "";
    if (!ip && this.app && this.app.camera) ip = this.app.camera.getIp("child");
    return ip;
  };

  AlignModule.prototype._row = function (label) {
    var row = document.createElement("div");
    row.className = "debug-row";
    var name = document.createElement("span");
    name.className = "debug-name";
    name.textContent = label;
    var val = document.createElement("span");
    val.className = "debug-cur";
    val.textContent = "--";
    row.appendChild(name); row.appendChild(val);
    return { row: row, val: val };
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
    var mask = document.createElement("div");
    mask.className = "debug-mask";
    var card = document.createElement("div");
    card.className = "debug-card";
    mask.appendChild(card);

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
    body.className = "debug-body";

    /* 摄像头子机地址 */
    var ipRow = document.createElement("div");
    ipRow.className = "debug-row";
    var ipLabel = document.createElement("span");
    ipLabel.className = "debug-name";
    ipLabel.textContent = "\u6444\u50cf\u5934\u5b50\u673a IP";
    var ipInput = document.createElement("input");
    ipInput.className = "debug-input";
    ipInput.type = "text";
    ipInput.placeholder = "\u81ea\u52a8\u53d1\u73b0";
    var cam = this.app && this.app.camera ? this.app.camera.getIp("child") : "";
    ipInput.value = cam || "";
    this._ipInput = ipInput;
    ipRow.appendChild(ipLabel); ipRow.appendChild(ipInput);
    body.appendChild(ipRow);

    /* 参考位 / 状态 */
    var refRow = this._row("\u53c2\u8003\u4f4d");
    this._refEl = refRow.val;
    body.appendChild(refRow.row);
    var stRow = this._row("\u72b6\u6001");
    this._statusEl = stRow.val;
    body.appendChild(stRow.row);

    /* 反馈数值 */
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

    /* 操作按钮 */
    body.appendChild(this._button("\u8bb0\u5f55\u53c2\u8003\u4f4d (/align/ref)", "btn-primary", function () { self._refCmd("ref"); }));
    body.appendChild(this._button("\u6e05\u9664\u53c2\u8003\u4f4d", "btn-gray", function () { self._refCmd("clear"); }));
    body.appendChild(this._button("\u542f\u52a8\u7ea0\u504f\u95ed\u73af", "btn-primary", function () { self._start(); }));
    body.appendChild(this._button("\u505c\u6b62\u7ea0\u504f", "btn-danger", function () { self._stop(); }));

    /* 手动微调 */
    var t2 = document.createElement("div");
    t2.className = "debug-section-title";
    t2.style.marginTop = "18px";
    t2.textContent = "\u5b50\u673a\u624b\u52a8\u5fae\u8c03";
    body.appendChild(t2);

    var pad = document.createElement("div");
    pad.className = "coop-dpad";
    var cells = ["", "up", "", "left", "stop", "right", "", "down", ""];
    cells.forEach(function (dir) {
      if (!dir) { pad.appendChild(document.createElement("span")); return; }
      var b = document.createElement("button");
      b.className = "coop-dir";
      b.textContent = dir === "up" ? "\u2191" : dir === "down" ? "\u2193"
                    : dir === "left" ? "\u2190" : dir === "right" ? "\u2192" : "\u25a0";
      if (dir === "stop") b.classList.add("coop-dir-stop");
      b.addEventListener("pointerdown", function (e) {
        e.preventDefault();
        if (dir === "stop")  self._manual(0, 0);
        if (dir === "up")    self._manual(SPEED_MAX, 0);
        if (dir === "down")  self._manual(-SPEED_MAX, 0);
        if (dir === "left")  self._manual(0, -SPEED_MAX);
        if (dir === "right") self._manual(0, SPEED_MAX);
      });
      b.addEventListener("pointerup", function () { self._manual(0, 0); });
      b.addEventListener("pointercancel", function () { self._manual(0, 0); });
      b.addEventListener("pointerleave", function () { self._manual(0, 0); });
      pad.appendChild(b);
    });
    body.appendChild(pad);

    card.appendChild(head); card.appendChild(body);
    mask.addEventListener("click", function (e) { if (e.target === mask) self.close(); });
    document.body.appendChild(mask);
    this._mask = mask;
  };

  /* ---------- 反馈轮询 ---------- */

  AlignModule.prototype._httpJson = function (path, cb) {
    var ip = this._camIp();
    if (!ip) { cb(new Error("no_ip"), null); return; }
    var url = "http://" + ip.replace(/^https?:\/\//, "").replace(/\/.*$/, "") + path;
    var xhr = new XMLHttpRequest();
    xhr.open("GET", url, true);
    xhr.timeout = 2000;
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

  AlignModule.prototype._poll = function () {
    var self = this;
    if (!this._mask || !this._mask.classList.contains("show")) return;
    this._httpJson("/align/get", function (err, d) {
      if (err) { self._setStatus("\u6444\u50cf\u5934\u4e0d\u53ef\u8fbe (" + err.message + ")", true); return; }
      if (!d) { self._setStatus("\u65e0\u54cd\u5e94", true); return; }
      if (!d.ok) {
        self._setStatus("\u5931\u8d25: " + (d.reason || "?"), true);
        return;
      }
      self._setStatus("\u6b63\u5e38 (" + (d.dt_ms || 0) + "ms)", false);
      self._render(d);
    });
  };

  AlignModule.prototype._startPoll = function () {
    var self = this;
    this._stopPoll();
    this._poll();
    this._timer = setInterval(function () { self._poll(); }, POLL_MS);
  };

  AlignModule.prototype._stopPoll = function () {
    if (this._timer) { clearInterval(this._timer); this._timer = null; }
  };

  /* ---------- 参考位 ---------- */

  AlignModule.prototype._refCmd = function (kind) {
    var self = this;
    var path = kind === "clear" ? "/align/ref?clear=1" : "/align/ref";
    this._httpJson(path, function (err, d) {
      if (err) { UI.toast("\u5931\u8d25: " + err.message); return; }
      if (d && d.ok) {
        UI.toast(kind === "clear" ? "\u53c2\u8003\u4f4d\u5df2\u6e05\u9664" : "\u53c2\u8003\u4f4d\u5df2\u8bb0\u5f55");
        if (self._refEl) self._refEl.textContent = kind === "clear" ? "\u65e0" :
          ("u=" + d.u + " v=" + d.v + " \u03b8=" + d.theta_deg + "\u00b0");
      } else {
        UI.toast("\u5931\u8d25: " + (d && d.reason || "?"));
      }
    });
  };

  /* ---------- 闭环启停 / 手动 ---------- */

  AlignModule.prototype._start = function () {
    var ip = this._camIp();
    var frame = Protocol.alignBridge(ip);
    if (!frame) { UI.toast("\u6444\u50cf\u5934\u5b50\u673a IP \u65e0\u6548"); return; }
    var ok = this.app.transport.send(frame);
    UI.log("\u7ea0\u504f\u95ed\u73af\u542f\u52a8 " + ip + (ok ? "" : " (\u672a\u53d1\u9001)"), !ok);
    if (ok) { this._running = true; this._setStatus("\u95ed\u73af\u8fd0\u884c\u4e2d", false); UI.toast("\u7ea0\u504f\u5df2\u542f\u52a8"); }
  };

  AlignModule.prototype._stop = function () {
    this._manual(0, 0);
    var ok = this.app.transport.send(Protocol.alignBridgeStop());
    this._running = false;
    this._setStatus("\u5f85\u673a", false);
    UI.log("\u7ea0\u504f\u505c\u6b62" + (ok ? "" : " (\u672a\u53d1\u9001)"), !ok);
  };

  AlignModule.prototype._manual = function (speed, spin) {
    if (this._running) this._stop();
    this.app.transport.send(Protocol.childCmd(speed, spin, 9));
    if (speed === 0 && spin === 0) this.app.transport.send(Protocol.childStop());
  };

  AlignModule.prototype.open = function () {
    if (this._mask) this._mask.classList.add("show");
    this._startPoll();
  };

  AlignModule.prototype.close = function () {
    if (this._mask) this._mask.classList.remove("show");
    this._stopPoll();
  };

  AlignModule.prototype.onState = function () {};

  global.AlignModule = AlignModule;
})(typeof window !== "undefined" ? window : globalThis);
