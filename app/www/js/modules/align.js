/*
 * modules/align.js - 视觉纠偏面板
 *
 * ① 独立纠偏测试: App 0x30 启 / 0x32 停 (母机 AlignTest: TSTA->REDY->ref/meas/ADJ->GOON->DONE->TSTE)
 * ② 协同内自动纠偏仍由 0x19 驱动 (不变)
 * ③ 姿态示意 (0x88) / 标定 0x24 / 监视+手动记参考 (直连 K230)
 *
 * 数据通道:
 *   状态/JSON  <- WS 0x88/0x89 (母机周期推, 协同与测试共用)
 *   启停/标定  -> WS 0x30/0x32/0x24
 *   监视/记参考 -> App 直连 K230 HTTP
 */
(function (global) {
  "use strict";

  function AlignModule(app) {
    this.app = app;
    this._drawer = null;
    this._logEl = null;
    this._poseCv = null;
    this._stateEl = null;
    this._roundEl = null;

    /* 当前姿态（0x88 偏差驱动动画） */
    this._cur = { phi: 0, psi: 0, ax: 0 };
    this._tgt = { phi: 0, psi: 0, ax: 0 };
    this._raf = null;

    /* 监视: App 直连 K230 轮询 /align/get (人手推车观察用) */
    this._mon = { on: false, timer: null, fails: 0, pausedByCoop: false };
  }

  /* ==================== 挂载 ==================== */

  AlignModule.prototype.mount = function () {
    var self = this;
    var entry = document.getElementById("btnAlign");
    if (entry) {
      entry.textContent = "视觉纠偏";
      entry.addEventListener("click", function () { self.open(); });
    }
    this._build();
    var cam = this.app && this.app.camera;
    if (cam && typeof cam.onIpChange === "function") {
      cam.onIpChange(function (key, ip) {
        if (key !== "child") return;
        self._onVisionIp(ip);
      });
    }
    if (this.app && this.app.transport) {
      this.app.transport.on("alignState", function (s) { self._onAlignState(s); });
      this.app.transport.on("alignJson", function (j) { self._onAlignJson(j); });
    }
    this._startAnim();
  };

  /* K230 即子机摄像头: 优先用 UDP 自动发现写入的地址, 其次上次手动填的 */
  AlignModule.prototype._resolveVisionIp = function () {
    var cam = this.app && this.app.camera;
    var ip = cam && cam.getIp ? (cam.getIp("child") || "") : "";
    if (!ip) {
      try { ip = (global.localStorage && global.localStorage.getItem("wallbot.k230Ip")) || ""; } catch (e) { ip = ""; }
    }
    return ip;
  };

  AlignModule.prototype._el = function (tag, cls, text) {
    var e = document.createElement(tag);
    if (cls) e.className = cls;
    if (text !== undefined) e.textContent = text;
    return e;
  };

  AlignModule.prototype._build = function () {
    var self = this;
    var drawer = this._el("div", "align-drawer");

    var head = this._el("div", "debug-head");
    head.appendChild(this._el("div", "debug-title", "视觉纠偏"));
    var closeBtn = this._el("button", "debug-close", "×");
    closeBtn.addEventListener("click", function () { self.close(); });
    head.appendChild(closeBtn);

    var body = this._el("div", "align-body");

    body.appendChild(this._el("div", "align-note",
      "独立测试: 下方「开始纠偏」走 TSTA→REDY→记参考→测量→ADJ→收敛→GOON→DONE→TSTE；与「启动协同」互斥。亦可只监视/标定。"));

    var stRow = this._el("div", "align-row");
    stRow.appendChild(this._el("span", "align-k", "状态"));
    this._stateEl = this._el("span", "align-v", "空闲");
    stRow.appendChild(this._stateEl);
    body.appendChild(stRow);

    var rdRow = this._el("div", "align-row");
    rdRow.appendChild(this._el("span", "align-k", "纠偏轮次"));
    this._roundEl = this._el("span", "align-v", "0 / --");
    rdRow.appendChild(this._roundEl);
    body.appendChild(rdRow);

    /* ---- ① 姿态示意 ---- */
    body.appendChild(this._el("div", "debug-section-title", "① 标志姿态（管道截面）"));
    var cv1 = document.createElement("canvas");
    cv1.className = "at-canvas";
    cv1.width = 360; cv1.height = 220;
    this._poseCv = cv1;
    body.appendChild(cv1);

    var poseLegend = this._el("div", "at-legend");
    poseLegend.appendChild(this._el("span", "at-lg-phi", "φ 周向"));
    poseLegend.appendChild(this._el("span", "at-lg-psi", "ψ 航向"));
    poseLegend.appendChild(this._el("span", "at-lg-ax", "ax 轴向"));
    body.appendChild(poseLegend);

    /* ---- ② 视觉板与标定 ---- */
    body.appendChild(this._el("div", "debug-section-title", "② 视觉板与标定"));

    var vIpRow = this._el("div", "align-row");
    vIpRow.appendChild(this._el("span", "align-k", "视觉板 IP"));
    this._vIpEl = this._el("span", "align-v", this._resolveVisionIp() || "自动发现中…");
    vIpRow.appendChild(this._vIpEl);
    body.appendChild(vIpRow);

    var vSignRow = this._el("div", "align-row");
    vSignRow.appendChild(this._el("span", "align-k", "符号"));
    var mkSign = function (label, def) {
      var s = document.createElement("select");
      s.className = "debug-input";
      s.style.maxWidth = "70px";
      ["+", "-"].forEach(function (v) {
        var o = document.createElement("option");
        o.value = v === "+" ? "1" : "-1";
        o.textContent = label + v;
        s.appendChild(o);
      });
      s.value = String(def);
      return s;
    };
    this._vSPhi = mkSign("φ", 1);
    this._vSPsi = mkSign("ψ", 1);
    this._vSAx  = mkSign("ax", 1);
    vSignRow.appendChild(this._vSPhi);
    vSignRow.appendChild(this._vSPsi);
    vSignRow.appendChild(this._vSAx);
    var btnTune = this._el("button", "av-btn av-btn-ok", "写入标定");
    btnTune.addEventListener("click", function () { self._sendTune(); });
    vSignRow.appendChild(btnTune);
    body.appendChild(vSignRow);

    var testRow = this._el("div", "av-btns");
    var mkBtn = function (label, fn, cls) {
      var b = self._el("button", "av-btn " + (cls || ""), label);
      b.addEventListener("click", fn);
      return b;
    };
    testRow.appendChild(mkBtn("开始纠偏", function () { self._testStart(); }, "av-btn-ok"));
    testRow.appendChild(mkBtn("停止", function () { self._testStop(); }, "av-btn-danger"));
    body.appendChild(testRow);

    var vCtrlRow = this._el("div", "av-btns");
    this._vMonBtn = mkBtn("监视: 关", function () { self._toggleMonitor(); });
    vCtrlRow.appendChild(this._vMonBtn);
    vCtrlRow.appendChild(mkBtn("记参考", function () { self._camRef(); }));
    body.appendChild(vCtrlRow);

    var vErrRow = this._el("div", "align-row");
    vErrRow.appendChild(this._el("span", "align-k", "实测偏差"));
    this._vErrEl = this._el("span", "align-v", "—");
    vErrRow.appendChild(this._vErrEl);
    body.appendChild(vErrRow);

    this._vJsonEl = this._el("div", "align-log");
    this._vJsonEl.textContent = "--";
    body.appendChild(this._vJsonEl);

    /* ---- 日志 ---- */
    body.appendChild(this._el("div", "debug-section-title", "日志"));
    var logBox = this._el("div", "align-log");
    logBox.textContent = "--";
    this._logEl = logBox;
    body.appendChild(logBox);

    drawer.appendChild(head);
    drawer.appendChild(body);
    document.body.appendChild(drawer);
    this._drawer = drawer;
  };

  /* ==================== 交互 ==================== */

  AlignModule.prototype._log = function (text, bad) {
    if (!this._logEl) return;
    var line = this._el("div", "align-log-line" + (bad ? " err" : ""), text);
    if (this._logEl.textContent === "--") this._logEl.textContent = "";
    this._logEl.appendChild(line);
    this._logEl.scrollTop = this._logEl.scrollHeight;
  };

  AlignModule.prototype._send = function (frame, desc) {
    if (!this.app || !this.app.transport) return false;
    var ok = this.app.transport.send(frame);
    this._log((ok ? "→ " : "✗ ") + desc, !ok);
    return ok;
  };

  AlignModule.prototype._sendTune = function () {
    var sPhi = parseInt(this._vSPhi.value, 10);
    var sPsi = parseInt(this._vSPsi.value, 10);
    var sAx  = parseInt(this._vSAx.value, 10);
    var frame = global.Protocol.alignTune(sPhi, sPsi, sAx, 20, 100);
    this._send(frame, "ALIGN_TUNE sign=" + sPhi + "/" + sPsi + "/" + sAx);
  };

  AlignModule.prototype._testStart = function () {
    var ip = this._resolveVisionIp();
    if (this._mon.on) this._stopMonitor();
    var frame = global.Protocol.testStart(ip || undefined);
    this._send(frame, "TEST_START 0x30" + (ip ? (" ip=" + ip) : " (用母机上次 IP)"));
  };

  AlignModule.prototype._testStop = function () {
    var frame = global.Protocol.testStop();
    this._send(frame, "TEST_STOP 0x32");
  };

  /* ==================== 0x88 / 0x89 ==================== */

  AlignModule.prototype._onAlignState = function (s) {
    if (this._stateEl) this._stateEl.textContent = s.stateText || ("状态" + s.state);
    if (this._roundEl) this._roundEl.textContent = s.iter + " / " + s.maxIter;
    if (typeof s.phi === "number") {
      this._setErrText(s.phi, s.psi, s.ax, s.aligned);
      this._tgt.phi = s.phi;
      this._tgt.psi = s.psi;
      this._tgt.ax = s.ax;
    }

    /* 协同视觉测量期间: K230 被母机 /align/once 共识阻塞, 子机流出帧慢/可能断,
       置检测模式防止 App 把地址判失效; 同时暂停监视避免抢 K230 单线程 */
    var busy = (s.state === 1 || s.state === 2 || s.state === 201);
    var cam = this.app && this.app.camera;
    if (cam && typeof cam.setDetectMode === "function") cam.setDetectMode("child", busy);
    if (busy && this._mon.on) {
      this._stopMonitor();
      this._log("纠偏测量中, 已暂停监视 (结束后可手动恢复)");
    }
  };

  AlignModule.prototype._onAlignJson = function (j) {
    if (this._vJsonEl) this._vJsonEl.textContent = j;
  };

  /* ==================== 视觉板直连 ==================== */

  AlignModule.prototype._vIpValue = function () {
    return this._resolveVisionIp();
  };

  /* 自动发现到子机(=K230)地址时刷新显示 */
  AlignModule.prototype._onVisionIp = function (ip) {
    if (!this._vIpEl) return;
    this._vIpEl.textContent = ip || "自动发现中…";
  };

  AlignModule.prototype._setErrText = function (phi, psi, ax, aligned) {
    if (!this._vErrEl) return;
    this._vErrEl.textContent = "φ " + Number(phi).toFixed(2) + "°  ψ " +
      Number(psi).toFixed(2) + "°  ax " + Number(ax).toFixed(0) + "mm" +
      (aligned ? "  已对齐" : "");
  };

  AlignModule.prototype._setErrHint = function (txt) {
    if (this._vErrEl) this._vErrEl.textContent = txt;
  };

  /* ---------- 监视: App 直连 K230 轮询 /align/get ---------- */

  AlignModule.prototype._monUrl = function () {
    var ip = this._vIpValue();
    return ip ? "http://" + ip + "/align/get" : "";
  };

  AlignModule.prototype._toggleMonitor = function () {
    if (this._mon.on) this._stopMonitor();
    else this._startMonitor();
  };

  AlignModule.prototype._startMonitor = function () {
    if (this._mon.on) return;
    if (!this._monUrl()) { this._log("未发现视觉板, 无法监视", true); return; }
    this._mon.on = true;
    this._mon.fails = 0;
    if (this._vMonBtn) this._vMonBtn.textContent = "监视: 开";
    this._log("开始监视 /align/get (300ms)");
    this._pollOnce();
  };

  AlignModule.prototype._stopMonitor = function () {
    this._mon.on = false;
    if (this._mon.timer) { clearTimeout(this._mon.timer); this._mon.timer = null; }
    if (this._vMonBtn) this._vMonBtn.textContent = "监视: 关";
  };

  AlignModule.prototype._pollOnce = function () {
    var self = this;
    if (!this._mon.on) return;
    var url = this._monUrl();
    if (!url) { this._stopMonitor(); return; }

    var ctrl = typeof AbortController !== "undefined" ? new AbortController() : null;
    var to = setTimeout(function () { if (ctrl) ctrl.abort(); }, 1500);

    fetch(url, { cache: "no-store", signal: ctrl ? ctrl.signal : undefined })
      .then(function (r) { return r.text(); })
      .then(function (txt) {
        clearTimeout(to);
        if (!self._mon.on) return;
        self._mon.fails = 0;
        if (self._vJsonEl) self._vJsonEl.textContent = txt;
        var j = null;
        try { j = JSON.parse(txt); } catch (e) { j = null; }
        if (j && j.ok) {
          self._setErrText(j.phi_deg, j.psi_deg, j.axial_mm, j.aligned);
          self._tgt.phi = j.phi_deg;
          self._tgt.psi = j.psi_deg;
          self._tgt.ax = j.axial_mm;
        } else if (j && j.reason) {
          if (j.reason === "no_ref") self._setErrHint("未记参考");
          else if (j.reason === "not_running") self._setErrHint("K230 摄像头未运行(先开子机视频流)");
          else self._setErrHint("无有效标签: " + j.reason);
        }
      })
      .catch(function () {
        clearTimeout(to);
        if (!self._mon.on) return;
        self._mon.fails++;
        if (self._mon.fails >= 5) {
          self._log("连续 5 次拉取失败, 停止监视", true);
          self._stopMonitor();
        }
      })
      .then(function () {
        if (self._mon.on) self._mon.timer = setTimeout(function () { self._pollOnce(); }, 300);
      });
  };

  /* 手动记参考: App 直连 K230 GET /align/ref (共识最多 5s), 不经母机。
     脱离协同单独调试用; 协同流程里母机会自己记。 */
  AlignModule.prototype._camRef = function () {
    var self = this;
    var ip = this._vIpValue();
    if (!ip) { this._log("未发现视觉板, 等自动发现或主页填子机 IP", true); return; }
    this._log("直连记参考 " + ip + "/align/ref (共识最多 5s)…");
    if (this._mon.on) this._stopMonitor();

    var ctrl = typeof AbortController !== "undefined" ? new AbortController() : null;
    var to = setTimeout(function () { if (ctrl) ctrl.abort(); }, 9000);

    fetch("http://" + ip + "/align/ref", { cache: "no-store", signal: ctrl ? ctrl.signal : undefined })
      .then(function (r) { return r.text(); })
      .then(function (txt) {
        clearTimeout(to);
        if (self._vJsonEl) self._vJsonEl.textContent = txt;
        var j = null;
        try { j = JSON.parse(txt); } catch (e) { j = null; }
        if (j && j.ok) {
          self._log("记参考成功, 开始监视");
          self._setErrText(0, 0, 0, true);
          self._tgt.phi = 0; self._tgt.psi = 0; self._tgt.ax = 0;
          self._startMonitor();
        } else {
          self._log("记参考失败: " + ((j && j.reason) || txt), true);
        }
      })
      .catch(function () {
        clearTimeout(to);
        self._log("记参考请求失败 (视觉板不在线?)", true);
      });
  };

  /* ==================== 动画 ==================== */

  AlignModule.prototype._startAnim = function () {
    var self = this;
    function tick() {
      var k = 0.12;
      self._cur.phi += (self._tgt.phi - self._cur.phi) * k;
      self._cur.psi += (self._tgt.psi - self._cur.psi) * k;
      self._cur.ax += (self._tgt.ax - self._cur.ax) * k;
      self._drawPose();
      self._raf = requestAnimationFrame(tick);
    }
    if (!this._raf) this._raf = requestAnimationFrame(tick);
  };

  /* ① 管道截面 + 标志块 */
  AlignModule.prototype._drawPose = function () {
    var cv = this._poseCv;
    if (!cv || !cv.getContext) return;
    var g = cv.getContext("2d");
    var W = cv.width, H = cv.height;
    g.clearRect(0, 0, W, H);

    var cx = W * 0.36, cy = H * 0.5, R = 74;

    /* 管道截面圆 */
    g.strokeStyle = "#8e8e93";
    g.lineWidth = 2;
    g.beginPath(); g.arc(cx, cy, R, 0, Math.PI * 2); g.stroke();

    /* 基准线 */
    g.strokeStyle = "rgba(142,142,147,.35)";
    g.setLineDash([4, 4]);
    g.beginPath(); g.moveTo(cx - R - 16, cy); g.lineTo(cx + R + 16, cy); g.stroke();
    g.beginPath(); g.moveTo(cx, cy - R - 16); g.lineTo(cx, cy + R + 16); g.stroke();
    g.setLineDash([]);

    /* 标志块位置: phi 决定圆周角 */
    var phi = this._cur.phi;
    var psi = this._cur.psi;
    var ax = this._cur.ax;
    var ang = (-90 + phi) * Math.PI / 180;
    var px = cx + Math.cos(ang) * R;
    var py = cy + Math.sin(ang) * R;

    /* 轴向偏移体现在方块距圆心的半径收缩 */
    var radScale = 1 - Math.min(Math.abs(ax) / 120, 0.35);
    px = cx + Math.cos(ang) * R * radScale;
    py = cy + Math.sin(ang) * R * radScale;

    g.save();
    g.translate(px, py);
    g.rotate((phi + psi) * Math.PI / 180);
    var sx = 1 + psi / 90;
    g.transform(sx, 0, 0, 1, 0, 0);
    g.fillStyle = Math.abs(phi) > 6 || Math.abs(psi) > 6 ? "#e25555" : "#2e9e4f";
    g.fillRect(-13, -9, 26, 18);
    g.strokeStyle = "#fff";
    g.lineWidth = 1;
    g.strokeRect(-13, -9, 26, 18);
    g.restore();

    /* 圆心 */
    g.fillStyle = "#8e8e93";
    g.beginPath(); g.arc(cx, cy, 3, 0, Math.PI * 2); g.fill();

    /* 右侧数值与箭头 */
    var bx = W - 108, by = 34;
    function arrow(y, v, color, label) {
      var len = Math.max(-44, Math.min(44, v * 2.2));
      g.strokeStyle = color;
      g.fillStyle = color;
      g.lineWidth = 2;
      g.beginPath(); g.moveTo(bx, y); g.lineTo(bx + len, y); g.stroke();
      var dir = len >= 0 ? 1 : -1;
      g.beginPath();
      g.moveTo(bx + len, y);
      g.lineTo(bx + len - 6 * dir, y - 4);
      g.lineTo(bx + len - 6 * dir, y + 4);
      g.closePath(); g.fill();
      g.font = "11px system-ui";
      g.fillStyle = "#8e8e93";
      g.fillText(label + " " + v.toFixed(1), bx - 4, y - 8);
    }
    arrow(by, phi, "#4a90d9", "φ");
    arrow(by + 50, psi, "#e08b2e", "ψ");
    arrow(by + 100, ax, "#9b59b6", "ax");
  };

  /* ==================== 抽屉开关 ==================== */

  AlignModule.prototype.open = function () {
    if (!this._drawer) return;
    this._drawer.classList.add("show");
    document.body.classList.add("align-open");
  };

  AlignModule.prototype.close = function () {
    if (this._drawer) this._drawer.classList.remove("show");
    document.body.classList.remove("align-open");
  };

  AlignModule.prototype.onState = function () {};

  global.AlignModule = AlignModule;
})(typeof window !== "undefined" ? window : globalThis);
