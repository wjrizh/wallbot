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

  /* 迭代纠偏: 必须与固件 main.c 的常数一致, 否则扣减的剩余量算错 */
  var ADJ_PIPE_R_MM = 250.0;   /* 实测管半径 2026-10, 与子机固件一致 */
  var ADJ_TRACK_MM  = 145.0;   /* 实测轮距 2026-10, 与子机固件一致 */
  var ADJ_TOL_PHI   = 0.5;     /* ADJ_TOL_PHI   */
  var ADJ_TOL_PSI   = 0.5;     /* ADJ_TOL_PSI   */
  var ADJ_TOL_AX    = 2.0;     /* ADJ_TOL_AX    */
  var ADJ_MAX_ITER  = 6;

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

    /* 迭代纠偏: on=开关, active=进行中, phi/psi/ax=当前剩余偏差, n=已完成轮次 */
    this._iter = { on: false, active: false, phi: 0, psi: 0, ax: 0, n: 0 };

    /* 监视: App 直连 K230 轮询 /align/get 刷新实测偏差 (人手推车观察用) */
    this._mon = { on: false, timer: null, fails: 0 };
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
    var cam = this.app && this.app.camera;
    if (cam && typeof cam.onIpChange === "function") {
      cam.onIpChange(function (key, ip) {
        if (key !== "child") return;
        self._onVisionIp(ip);
      });
    }
    if (this.app && this.app.transport) {
      this.app.transport.on("testTelem", function (t) { self._onTelem(t); });
      this.app.transport.on("testState", function (s) { self._onTestState(s); });
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

    var fanRow = this._el("div", "av-btns");
    var mkFan = function (label, key, cls) {
      var b = self._el("button", "av-btn " + (cls || ""), label);
      b.addEventListener("click", function () { self._sendFanKey(key); });
      return b;
    };
    fanRow.appendChild(mkFan("风机+", 3, "av-btn-ok"));
    fanRow.appendChild(mkFan("风机−", 4, ""));
    fanRow.appendChild(mkFan("停风机", 6, "av-btn-danger"));
    body.appendChild(fanRow);

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
      { key: "phi", label: "φ 周向", unit: "°", def: 0 },
      { key: "psi", label: "ψ 航向", unit: "°", def: 0 },
      { key: "ax", label: "ax 轴向", unit: "mm", def: 0 }
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

    var btnIter = this._el("button", "btn btn-gray btn-wide", "迭代纠偏: 关");
    btnIter.style.marginTop = "6px";
    btnIter.addEventListener("click", function () {
      self._iter.on = !self._iter.on;
      if (!self._iter.on) self._iter.active = false;
      btnIter.textContent = "迭代纠偏: " + (self._iter.on ? "开" : "关");
      self._log("迭代纠偏已" + (self._iter.on ? "开启" : "关闭"));
    });
    body.appendChild(btnIter);

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

    /* ---- ⑤ 真实视觉闭环（K230 = 子机摄像头, IP 取自自动发现） ---- */
    body.appendChild(this._el("div", "debug-section-title", "⑤ 真实视觉闭环（K230）"));

    var vIpRow = this._el("div", "align-row");
    vIpRow.appendChild(this._el("span", "align-k", "视觉板 IP"));
    this._vIpEl = this._el("span", "align-v", this._resolveVisionIp() || "自动发现中…");
    vIpRow.appendChild(this._vIpEl);
    body.appendChild(vIpRow);

    var vGainRow = this._el("div", "align-row");
    vGainRow.appendChild(this._el("span", "align-k", "增益%"));
    var vGain = document.createElement("input");
    vGain.type = "number"; vGain.className = "debug-input"; vGain.value = "100";
    vGain.style.maxWidth = "70px";
    this._vGain = vGain;
    vGainRow.appendChild(vGain);
    vGainRow.appendChild(this._el("span", "align-k", " 轮次上限"));
    var vIter = document.createElement("input");
    vIter.type = "number"; vIter.className = "debug-input"; vIter.value = "30";
    vIter.style.maxWidth = "60px";
    this._vIter = vIter;
    vGainRow.appendChild(vIter);
    body.appendChild(vGainRow);

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
    this._vSPsi = mkSign("ψ", -1);
    this._vSAx  = mkSign("ax", 1);
    vSignRow.appendChild(this._vSPhi);
    vSignRow.appendChild(this._vSPsi);
    vSignRow.appendChild(this._vSAx);
    var btnTune = this._el("button", "av-btn av-btn-ok", "写入标定");
    btnTune.addEventListener("click", function () { self._sendTune(); });
    vSignRow.appendChild(btnTune);
    body.appendChild(vSignRow);

    var vCtrlRow = this._el("div", "av-btns");
    var mkBtn = function (label, fn, cls) {
      var b = self._el("button", "av-btn " + (cls || ""), label);
      b.addEventListener("click", fn);
      return b;
    };
    vCtrlRow.appendChild(mkBtn("开始闭环", function () { self._startVision(); }, "av-btn-primary"));
    vCtrlRow.appendChild(mkBtn("记参考", function () { self._camRef(); }));
    this._vMonBtn = mkBtn("监视: 关", function () { self._toggleMonitor(); });
    vCtrlRow.appendChild(this._vMonBtn);
    vCtrlRow.appendChild(mkBtn("停止", function () { self._stopVision(); }, "av-btn-danger"));
    body.appendChild(vCtrlRow);

    var vStatusRow = this._el("div", "align-row");
    vStatusRow.appendChild(this._el("span", "align-k", "闭环状态"));
    this._vStateEl = this._el("span", "align-v", "空闲");
    vStatusRow.appendChild(this._vStateEl);
    body.appendChild(vStatusRow);

    var vErrRow = this._el("div", "align-row");
    vErrRow.appendChild(this._el("span", "align-k", "实测偏差"));
    this._vErrEl = this._el("span", "align-v", "—");
    vErrRow.appendChild(this._vErrEl);
    body.appendChild(vErrRow);

    this._vJsonEl = this._el("div", "align-log");
    this._vJsonEl.textContent = "--";
    body.appendChild(this._vJsonEl);

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

  AlignModule.prototype._sendFanKey = function (key) {
    if (!this.app || !this.app.transport || typeof Protocol === "undefined" || !Protocol.childCmd) {
      this._log("风机: Protocol.childCmd 不可用", true);
      return;
    }
    var ok = this.app.transport.send(Protocol.childCmd(0, 0, key & 0xFF));
    var name = key === 3 ? "风机+" : (key === 4 ? "风机−" : (key === 6 ? "停风机" : ("KEY=" + key)));
    this._log("风机 " + name + (ok ? "" : " (未发送)"), !ok);
  };

  AlignModule.prototype._start = function () {
    if (!this._send(global.Protocol.testStart(), "TEST_START")) return;
    this._log("等待子机 TSTB...");
  };

  AlignModule.prototype._stop = function () {
    this._iter.active = false;
    this._send(global.Protocol.testStop(), "TEST_STOP");
  };

  /* 发一轮 ADJ. 迭代模式下由 _nextIter 续发, 手动模式只发这一次 */
  AlignModule.prototype._sendRound = function (phi, psi, ax) {
    var desc = "ADJ phi=" + phi.toFixed(1) + " psi=" + psi.toFixed(1) + " ax=" + ax.toFixed(1);
    if (!this._send(global.Protocol.testAdj(phi, psi, ax), desc)) return false;
    this._rows.push({ phi: phi, psi: psi, ax: ax, rc: -1 });
    this._renderStages();
    return true;
  };

  AlignModule.prototype._sendAdj = function () {
    /* 母机 TM_IDLE 会直接丢弃 ADJ 帧, 且不回 DONE, App 会永远显示"执行中".
       只允许在 state=3(TM_READY) 下发 */
    var st = this._testState.state;
    if (st !== 3) {
      this._log("子机未就绪 (state=" + st + "), 请先点「开始测试」并等待 TSTB", true);
      if (st === 0) this._log("提示: 先点左侧「开始测试」按钮", true);
      return;
    }
    var phi = this._num("phi"), psi = this._num("psi"), ax = this._num("ax");
    if (phi === 0 && psi === 0 && ax === 0) {
      this._log("三个偏差全为 0, 无需纠偏", true);
      return;
    }
    if (this._iter.on) {
      this._iter.phi = phi; this._iter.psi = psi; this._iter.ax = ax;
      this._iter.n = 0; this._iter.active = true;
      this._log("迭代纠偏开始: φ=" + phi + " ψ=" + psi + " ax=" + ax);
    }
    this._sendRound(phi, psi, ax);
  };

  /* 阶段完成(rc=1)时, 用实测里程扣减该阶段对应的剩余偏差 */
  AlignModule.prototype._applyStageResult = function (t) {
    var it = this._iter;
    var done = Math.min(Math.abs(t.aMm), Math.abs(t.bMm));   /* 与固件 min 判据一致 */
    var cmdMm = Math.abs(t.targetMm);
    var resid = cmdMm - done;
    if (resid < 0) resid = 0;
    if (t.stage === 3) {
      var deg = resid / (Math.PI / 180 * ADJ_PIPE_R_MM);
      it.phi = (it.phi < 0 ? -1 : 1) * deg;
      if (deg < ADJ_TOL_PHI) it.phi = 0;
      this._log("φ 剩余: 目标" + cmdMm + "mm 实走" + done + "mm -> " + deg.toFixed(2) + "°");
    } else if (t.stage === 2) {
      var th = resid / (Math.PI / 180 * (ADJ_TRACK_MM * 0.5));
      it.psi = (it.psi < 0 ? -1 : 1) * th;
      if (th < ADJ_TOL_PSI) it.psi = 0;
      this._log("ψ 剩余: 目标" + cmdMm + "mm 实走" + done + "mm -> " + th.toFixed(2) + "°");
    }
  };

  /* 一轮结束: 判断是否继续下一轮 */
  AlignModule.prototype._nextIter = function () {
    var it = this._iter;
    it.n++;
    if (it.n >= ADJ_MAX_ITER) {
      it.active = false;
      this._log("迭代纠偏: 达最大轮次 " + ADJ_MAX_ITER + ", 停止", true);
      this._syncInputs();
      return;
    }
    if (Math.abs(it.phi) < ADJ_TOL_PHI) it.phi = 0;
    if (Math.abs(it.psi) < ADJ_TOL_PSI) it.psi = 0;
    if (Math.abs(it.ax)  < ADJ_TOL_AX)  it.ax  = 0;
    if (it.phi === 0 && it.psi === 0 && it.ax === 0) {
      it.active = false;
      this._log("迭代纠偏: 已进入容差, 共 " + it.n + " 轮");
      this._syncInputs();
      return;
    }
    this._log("迭代第 " + (it.n + 1) + " 轮: φ=" + it.phi.toFixed(2) +
              " ψ=" + it.psi.toFixed(2) + " ax=" + it.ax.toFixed(2));
    this._syncInputs();
    this._sendRound(it.phi, it.psi, it.ax);
  };

  AlignModule.prototype._syncInputs = function () {
    if (this._inputs.phi) this._inputs.phi.value = this._iter.phi.toFixed(1);
    if (this._inputs.psi) this._inputs.psi.value = this._iter.psi.toFixed(1);
    if (this._inputs.ax)  this._inputs.ax.value  = this._iter.ax.toFixed(1);
    this._syncTarget();
  };

  AlignModule.prototype._onTestState = function (s) {
    var prev = this._testState.round;
    this._testState = s;
    if (this._stateEl) this._stateEl.textContent = STATE_TEXT[s.state] || ("状态" + s.state);
    if (this._roundEl) {
      this._roundEl.textContent = s.round + " / " + (s.lastRc ? RC_TEXT[s.lastRc] : "--");
    }
    if (s.state === 3 && prev === 0) this._log("子机就绪, 可以发送模拟偏差");
    if (s.state === 0 && this._iter.active) {
      this._iter.active = false;
      this._log("测试已退出, 迭代中止", true);
    }
    if (s.round !== prev) {
      this._log("第 " + s.round + " 轮完成");
      this._renderStages();
      if (this._iter.active && s.lastRc === 0) this._nextIter();
      else if (this._iter.active) { this._iter.active = false; this._syncInputs(); }
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
    /* 阶段到位(rc=1)才扣减; 中止/超时不改剩余量 */
    if (this._iter.active && t.rc === 1) this._applyStageResult(t);
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

  /* ==================== 真实视觉闭环 ==================== */

  AlignModule.prototype._vIpValue = function () {
    return this._resolveVisionIp();
  };

  /* 自动发现到子机(=K230)地址时刷新显示 */
  AlignModule.prototype._onVisionIp = function (ip) {
    if (!this._vIpEl) return;
    this._vIpEl.textContent = ip || "自动发现中…";
  };

  AlignModule.prototype._startVision = function () {
    var ip = this._vIpValue();
    if (!ip) { this._log("未发现视觉板, 等自动发现或主页填子机 IP", true); return; }
    /* 闭环结束后自动恢复监视 (闭环期间必须停, 避免与 /align/once 抢 K230) */
    this._mon.resumeAfterVision = this._mon.on;
    if (this._mon.on) {
      this._stopMonitor();
      this._log("已停止监视, 闭环结束后自动恢复");
    }
    var gain = parseInt(this._vGain.value, 10);
    if (isNaN(gain)) gain = 100;
    var iter = parseInt(this._vIter.value, 10);
    if (isNaN(iter)) iter = 30;
    if (iter < 1) iter = 1;
    /* 只带 0x02 开摄像头, 不带 0x04 结束关摄像头: 关流后 WebView 对 MJPEG
       正常断开不一定触发 img.onerror, 画面会永久卡死只能重启 App */
    var frame = global.Protocol.alignVisionStart(ip, gain, iter, 0x02);
    if (!frame) { this._log("IP 格式错误", true); return; }
    this._send(frame, "ALIGN_VISION_START ip=" + ip + " gain=" + gain + "% 轮次上限=" + iter);
    /* 闭环期间子机流出帧慢是正常的, 不计失败不清 IP */
    var cam = this.app && this.app.camera;
    if (cam && typeof cam.setDetectMode === "function") cam.setDetectMode("child", true);
  };

  AlignModule.prototype._stopVision = function () {
    var cam = this.app && this.app.camera;
    if (cam && typeof cam.setDetectMode === "function") cam.setDetectMode("child", false);
    this._send(global.Protocol.alignVisionStop(), "ALIGN_VISION_STOP");
  };

  /* 记参考位: 0x26 action=3, 需要视觉板 IP */
  AlignModule.prototype._camRef = function () {
    var ip = this._vIpValue();
    if (!ip) { this._log("未发现视觉板, 等自动发现或主页填子机 IP", true); return; }
    var frame = global.Protocol.alignCamCtl(3, ip);
    if (!frame) { this._log("IP 格式错误", true); return; }
    if (!this._send(frame, "ALIGN_CAM_CTL 记参考 ip=" + ip)) return;
    /* 母机同步 GET /align/ref (K230 共识最多 5s): 先本地清零,
       1.5s 后开监视取真值; 若参考未落地, 轮询先显示未记参考/旧值, 落地后自愈 */
    this._setErrText(0, 0, 0, true);
    this._tgt.phi = 0; this._tgt.psi = 0; this._tgt.ax = 0;
    var self = this;
    if (this._mon.on) this._stopMonitor();
    this._log("记参考已下发, 1.5s 后开始监视");
    setTimeout(function () { self._startMonitor(); }, 1500);
  };

  /* ---------- 监视: App 直连 K230 轮询 /align/get ---------- */

  AlignModule.prototype._monUrl = function () {
    var ip = this._vIpValue();
    return ip ? "http://" + ip + "/align/get" : "";
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
    if (this._vStateEl) this._vStateEl.textContent = "监视中";
    this._log("开始监视 /align/get (300ms)");
    this._pollOnce();
  };

  AlignModule.prototype._stopMonitor = function () {
    this._mon.on = false;
    if (this._mon.timer) { clearTimeout(this._mon.timer); this._mon.timer = null; }
    if (this._vMonBtn) this._vMonBtn.textContent = "监视: 关";
    if (this._vStateEl) this._vStateEl.textContent = "空闲";
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

  AlignModule.prototype._sendTune = function () {
    var sPhi = parseInt(this._vSPhi.value, 10);
    var sPsi = parseInt(this._vSPsi.value, 10);
    var sAx  = parseInt(this._vSAx.value, 10);
    var frame = global.Protocol.alignTune(sPhi, sPsi, sAx, 20, 100);
    this._send(frame, "ALIGN_TUNE sign=" + sPhi + "/" + sPsi + "/" + sAx);
  };

  AlignModule.prototype._onAlignState = function (s) {
    if (!this._vStateEl) return;
    var txt = s.stateText + (s.rcText ? " · " + s.rcText : "");
    this._vStateEl.textContent = txt + "  (" + s.iter + "/" + s.maxIter + ")";
    this._vErrEl.textContent = "φ " + s.phi.toFixed(2) + "°  ψ " + s.psi.toFixed(2) +
                               "°  ax " + s.ax + "mm" + (s.aligned ? "  已对齐" : "");
    /* 同步可视化 */
    this._tgt.phi = s.phi;
    this._tgt.psi = s.psi;
    this._tgt.ax  = s.ax;
    /* 终态(7收敛/8超时/9视觉失败/10子机失败/11用户停止):
       解除子机流检测模式保护; 闭环前在监视的话自动恢复 */
    if (s.state >= 7) {
      var cam = this.app && this.app.camera;
      if (cam && typeof cam.setDetectMode === "function") cam.setDetectMode("child", false);
      if (this._mon.resumeAfterVision) {
        this._mon.resumeAfterVision = false;
        var self = this;
        setTimeout(function () { self._startMonitor(); }, 1000);
      }
    }
  };

  AlignModule.prototype._onAlignJson = function (j) {
    if (this._vJsonEl) this._vJsonEl.textContent = j;
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
