/*
 * modules/align.js - 视觉纠偏测试面板（无摄像头，假偏差下发）
 *
 * 链路: 本面板 -> WS(0x30/0x31/0x32) -> 母机 test_mode -> NRF -> 子机 -> 轮子运动
 *   ＜- 0x87 测试状态(round/rc) ＜- 0x86 轮子遥测('J' 帧)
 *
 * 可视化:
 *   ① 姿态示意: 管道截面 + 标志块, phi 决定圆周位置, psi 决定自身偏航, ax 决定轴向偏移
 *   ② 差速轮示意: A/B 两轮转向箭头 + 累计里程, 直行反向/自旋同向
 *   ③ 阶段时间轴: 轴向斜行 -> 自旋 -> 弧长直行, 显示进度与结果
 *
 * 真实摄像头分支 (/align/get) 折叠在底部，摄像头就绪后再启用。
 */
(function (global) {
  "use strict";

  var STAGE_NAME = { 1: "轴向斜行", 2: "自旋", 3: "弧长直行", 4: "稳定" };
  var RC_TEXT = { 0: "进行中", 1: "到位", 2: "中止", 3: "超时/打滑" };
  var STATE_TEXT = { 0: "空闲", 1: "发 TSTA", 2: "等 TSTB", 3: "就绪", 4: "发 ADJ", 5: "执行中", 6: "发 TSTE" };

  var PRESETS = [
    { label: "小", phi: 2, psi: 2, ax: 5 },
    { label: "中", phi: 5, psi: 5, ax: 10 },
    { label: "大", phi: 10, psi: 10, ax: 30 }
  ];

  function AlignModule(app) {
    this.app = app;
    this._drawer = null;
    this._logEl = null;
    this._poseCv = null;
    this._wheelCv = null;
    this._stateEl = null;
    this._roundEl = null;
    this._stageEl = null;
    this._inputs = {};
    this._rows = [];

    /* 当前姿态（目标偏差，驱动动画） */
    this._cur = { phi: 0, psi: 0, ax: 0 };
    this._tgt = { phi: 0, psi: 0, ax: 0 };
    this._raf = null;

    /* 遥测 */
    this._telem = { stage: 0, dir: 0, aMm: 0, bMm: 0, targetMm: 0, pct: 0, rc: 0 };
    this._testState = { state: 0, round: 0, lastRc: 0 };
  }

  /* ==================== 挂载 ==================== */

  AlignModule.prototype.mount = function () {
    var self = this;
    var entry = document.getElementById("btnAlign");
    if (entry) {
      entry.textContent = "视觉纠偏测试";
      entry.addEventListener("click", function () { self.open(); });
    }
    this._build();
    if (this.app && this.app.transport) {
      this.app.transport.on("testTelem", function (t) { self._onTelem(t); });
      this.app.transport.on("testState", function (s) { self._onTestState(s); });
    }
    this._startAnim();
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
    head.appendChild(this._el("div", "debug-title", "视觉纠偏测试"));
    var closeBtn = this._el("button", "debug-close", "×");
    closeBtn.addEventListener("click", function () { self.close(); });
    head.appendChild(closeBtn);

    var body = this._el("div", "align-body");

    body.appendChild(this._el("div", "align-note",
      "不下发真实图像。App 输入假偏差 → 母机 NRF → 子机按偏差驱动两轮运动。"));

    /* ---- 控制区 ---- */
    var ctrl = this._el("div", "at-ctrl");
    var btnStart = this._el("button", "btn btn-primary btn-wide", "开始测试");
    btnStart.addEventListener("click", function () { self._start(); });
    var btnStop = this._el("button", "btn btn-gray btn-wide", "结束测试");
    btnStop.addEventListener("click", function () { self._stop(); });
    ctrl.appendChild(btnStart);
    ctrl.appendChild(btnStop);
    body.appendChild(ctrl);

    var stRow = this._el("div", "align-row");
    stRow.appendChild(this._el("span", "align-k", "状态"));
    this._stateEl = this._el("span", "align-v", "空闲");
    stRow.appendChild(this._stateEl);
    body.appendChild(stRow);

    var rdRow = this._el("div", "align-row");
    rdRow.appendChild(this._el("span", "align-k", "轮次 / 上次结果"));
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

    /* ---- 输入区 ---- */
    body.appendChild(this._el("div", "debug-section-title", "② 输入假图像数据"));
    var form = this._el("div", "at-form");
    [
      { key: "phi", label: "φ 周向", unit: "°", def: 5 },
      { key: "psi", label: "ψ 航向", unit: "°", def: 5 },
      { key: "ax", label: "ax 轴向", unit: "mm", def: 10 }
    ].forEach(function (f) {
      var row = self._el("div", "at-form-row");
      row.appendChild(self._el("span", "at-form-k", f.label));
      var inp = document.createElement("input");
      inp.type = "number";
      inp.className = "debug-input at-form-in";
      inp.step = "0.5";
      inp.value = String(f.def);
      inp.addEventListener("input", function () { self._syncTarget(); });
      self._inputs[f.key] = inp;
      row.appendChild(inp);
      row.appendChild(self._el("span", "at-form-u", f.unit));
      form.appendChild(row);
    });
    body.appendChild(form);

    var presetRow = this._el("div", "at-presets");
    PRESETS.forEach(function (p) {
      var b = self._el("button", "btn btn-gray at-preset", p.label);
      b.addEventListener("click", function () {
        self._inputs.phi.value = String(p.phi);
        self._inputs.psi.value = String(p.psi);
        self._inputs.ax.value = String(p.ax);
        self._syncTarget();
      });
      presetRow.appendChild(b);
    });
    body.appendChild(presetRow);

    var btnSend = this._el("button", "btn btn-primary btn-wide", "发送模拟偏差");
    btnSend.style.marginTop = "8px";
    btnSend.addEventListener("click", function () { self._sendAdj(); });
    body.appendChild(btnSend);

    /* ---- ③ 差速轮示意 ---- */
    body.appendChild(this._el("div", "debug-section-title", "③ 子机差速轮动作"));
    var cv2 = document.createElement("canvas");
    cv2.className = "at-canvas";
    cv2.width = 360; cv2.height = 150;
    this._wheelCv = cv2;
    body.appendChild(cv2);

    /* ---- 阶段时间轴 ---- */
    body.appendChild(this._el("div", "debug-section-title", "④ 执行阶段"));
    var stageBox = this._el("div", "at-stage");
    this._stageEl = stageBox;
    body.appendChild(stageBox);

    /* ---- 日志 ---- */
    body.appendChild(this._el("div", "debug-section-title", "握手日志"));
    var logBox = this._el("div", "align-log");
    logBox.textContent = "--";
    this._logEl = logBox;
    body.appendChild(logBox);

    /* ---- 真实摄像头（折叠） ---- */
    var det = document.createElement("details");
    det.className = "at-real";
    det.appendChild(this._el("summary", null, "真实摄像头（未就绪，点开可手动填 IP）"));
    var ipRow = this._el("div", "align-row");
    ipRow.appendChild(this._el("span", "align-k", "子机 IP"));
    var ipInp = document.createElement("input");
    ipInp.type = "text";
    ipInp.className = "debug-input";
    ipInp.style.maxWidth = "170px";
    var cam = this.app && this.app.camera ? this.app.camera : null;
    ipInp.placeholder = (cam && cam.getIp("child")) || "未发现";
    ipInp.value = (cam && cam.getIp("child")) || "";
    this._ipInput = ipInp;
    ipRow.appendChild(ipInp);
    det.appendChild(ipRow);
    var btnPoll = this._el("button", "btn btn-gray btn-wide", "拉取 /align/get");
    btnPoll.addEventListener("click", function () { self._pollReal(); });
    det.appendChild(btnPoll);
    this._realOut = this._el("div", "align-log");
    this._realOut.textContent = "--";
    det.appendChild(this._realOut);
    body.appendChild(det);

    drawer.appendChild(head);
    drawer.appendChild(body);
    document.body.appendChild(drawer);
    this._drawer = drawer;

    this._syncTarget();
  };

  /* ==================== 交互 ==================== */

  AlignModule.prototype._num = function (key) {
    var v = parseFloat(this._inputs[key] ? this._inputs[key].value : "0");
    return isFinite(v) ? v : 0;
  };

  AlignModule.prototype._syncTarget = function () {
    this._tgt.phi = this._num("phi");
    this._tgt.psi = this._num("psi");
    this._tgt.ax = this._num("ax");
  };

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

  AlignModule.prototype._start = function () {
    if (!this._send(global.Protocol.testStart(), "TEST_START")) return;
    this._log("等待子机 TSTB...");
  };

  AlignModule.prototype._stop = function () {
    this._send(global.Protocol.testStop(), "TEST_STOP");
  };

  AlignModule.prototype._sendAdj = function () {
    var phi = this._num("phi"), psi = this._num("psi"), ax = this._num("ax");
    var desc = "ADJ phi=" + phi + " psi=" + psi + " ax=" + ax;
    if (!this._send(global.Protocol.testAdj(phi, psi, ax), desc)) return;
    this._rows.push({ phi: phi, psi: psi, ax: ax, rc: -1 });
    this._renderStages();
  };

  AlignModule.prototype._onTestState = function (s) {
    var prev = this._testState.round;
    this._testState = s;
    if (this._stateEl) this._stateEl.textContent = STATE_TEXT[s.state] || ("状态" + s.state);
    if (this._roundEl) {
      this._roundEl.textContent = s.round + " / " + (s.lastRc ? RC_TEXT[s.lastRc] : "--");
    }
    if (s.round !== prev) {
      this._log("第 " + s.round + " 轮完成");
      this._renderStages();
    }
    if (s.lastRc === 3) this._log("测试超时", true);
  };

  AlignModule.prototype._onTelem = function (t) {
    this._telem = t;
    if (t.rc) {
      var last = this._rows.length - 1;
      if (last >= 0) this._rows[last].rc = t.rc;
      this._renderStages();
    }
  };

  AlignModule.prototype._renderStages = function () {
    if (!this._stageEl) return;
    var t = this._telem;
    var rows = this._rows;
    var html = [];
    for (var i = 0; i < rows.length; i++) {
      var r = rows[i];
      var rc = r.rc;
      var txt = rc < 0 ? "执行中" : (RC_TEXT[rc] || ("rc=" + rc));
      html.push("<div class='at-stage-row'>" +
        "第" + (i + 1) + "轮 φ=" + r.phi + " ψ=" + r.psi + " ax=" + r.ax +
        " <b>" + txt + "</b></div>");
    }
    if (t.stage > 0) {
      var name = STAGE_NAME[t.stage] || ("阶段" + t.stage);
      var bar = "<div class='at-bar'><i style='width:" + (t.pct || 0) + "%'></i></div>";
      html.push("<div class='at-stage-row'>当前: " + name + " " +
        (t.targetMm || 0) + "mm A=" + t.aMm + " B=" + t.bMm + " " + (t.pct || 0) + "%</div>" + bar);
    }
    this._stageEl.innerHTML = html.length ? html.join("") : "<div class='at-stage-row'>暂无</div>";
  };

  AlignModule.prototype._pollReal = function () {
    var self = this;
    var ip = this._ipInput ? this._ipInput.value.trim() : "";
    if (!ip) { this._realOut.textContent = "未填 IP"; return; }
    var url = "http://" + ip.replace(/^https?:\/\//, "").replace(/\/.*$/, "") + "/align/get";
    var xhr = new XMLHttpRequest();
    xhr.open("GET", url, true);
    xhr.timeout = 3000;
    xhr.onload = function () {
      self._realOut.textContent = xhr.responseText || "(空)";
    };
    xhr.onerror = function () { self._realOut.textContent = "请求失败"; };
    xhr.ontimeout = function () { self._realOut.textContent = "超时"; };
    try { xhr.send(); } catch (e) { self._realOut.textContent = "异常 " + e.message; }
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
      self._drawWheels();
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

  /* ② 差速轮 */
  AlignModule.prototype._drawWheels = function () {
    var cv = this._wheelCv;
    if (!cv || !cv.getContext) return;
    var g = cv.getContext("2d");
    var W = cv.width, H = cv.height;
    g.clearRect(0, 0, W, H);

    var t = this._telem;
    var a = t.aMm || 0, b = t.bMm || 0;

    var cxL = W * 0.28, cxR = W * 0.72, cy = H * 0.42, RW = 46, RH = 26;

    function wheel(cx, name, mm) {
      var moving = Math.abs(mm) > 0.5;
      g.fillStyle = moving ? "#2e9e4f" : "#c7c7cc";
      g.fillRect(cx - RW / 2, cy - RH / 2, RW, RH);
      g.strokeStyle = "#8e8e93";
      g.lineWidth = 2;
      g.strokeRect(cx - RW / 2, cy - RH / 2, RW, RH);

      g.fillStyle = "#1c1c1e";
      g.font = "bold 13px system-ui";
      g.textAlign = "center";
      g.fillText(name, cx, cy + 5);

      if (moving) {
        var dir = mm >= 0 ? 1 : -1;
        g.strokeStyle = "#2e9e4f";
        g.lineWidth = 3;
        g.beginPath();
        g.moveTo(cx, cy - RH / 2 - 10);
        g.lineTo(cx + 22 * dir, cy - RH / 2 - 10);
        g.stroke();
        g.beginPath();
        g.moveTo(cx + 22 * dir, cy - RH / 2 - 10);
        g.lineTo(cx + 15 * dir, cy - RH / 2 - 15);
        g.lineTo(cx + 15 * dir, cy - RH / 2 - 5);
        g.closePath();
        g.fillStyle = "#2e9e4f";
        g.fill();
      }

      g.fillStyle = "#1c1c1e";
      g.font = "12px ui-monospace,monospace";
      g.textAlign = "center";
      g.fillText(mm.toFixed(1) + " mm", cx, cy + RH / 2 + 22);
    }

    wheel(cxL, "A", a);
    wheel(cxR, "B", b);

    /* 差速关系判定 */
    var sameDir = (a >= 0 && b >= 0) || (a <= 0 && b <= 0);
    var modeText = "--";
    if (Math.abs(a) > 0.5 || Math.abs(b) > 0.5) {
      modeText = sameDir ? "自旋（两轮同向）" : "直行（两轮反向）";
    }
    g.fillStyle = "#8e8e93";
    g.font = "11px system-ui";
    g.textAlign = "center";
    g.fillText(modeText, W / 2, H - 8);
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
