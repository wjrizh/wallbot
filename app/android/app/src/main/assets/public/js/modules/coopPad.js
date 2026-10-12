/*
 * modules/coopPad.js - 协同控制悬浮窗 (母机界面)
 * 组1 子机手动(0x18): 前后左右 / 风机+ 风机- 停风机(key=6)
 * 组2 母机快捷(0x01舵机/0x1B夹持器): 下压PB9->95 / 预夹紧40 / 夹紧0 / 上抬PB9->160
 * 所有按钮必须先"人工介入"(0x1D 中止协同)解锁后才生效
 */
(function (global) {
  "use strict";

  var AXIS_MAX = 100;
  var REPEAT_MS = 80;
  var MOVE_DUR = 2.0;

  var DIR = {
    up:    { lh:  AXIS_MAX, lv: 0 },
    down:  { lh: -AXIS_MAX, lv: 0 },
    left:  { lh: 0, lv: -AXIS_MAX },
    right: { lh: 0, lv:  AXIS_MAX }
  };

  function CoopPadModule(app) {
    this.app = app;
    this._mask = null;
    this._lh = 0;
    this._lv = 0;
    this._timer = null;
    this._activeDir = null;
    this._step2 = null;      /* 组2 容器, 默认 disabled */
    this._unlocked = false;
  }

  CoopPadModule.prototype.mount = function () {
    var self = this;
    var entry = document.getElementById("btnCoopPad");
    if (entry) entry.addEventListener("click", function () { self.open(); });
    this._build();
  };

  CoopPadModule.prototype._build = function () {
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
    title.textContent = "协同控制";
    var closeBtn = document.createElement("button");
    closeBtn.className = "debug-close";
    closeBtn.textContent = "\u00d7";
    closeBtn.addEventListener("click", function () { self.close(); });
    head.appendChild(title); head.appendChild(closeBtn);

    var body = document.createElement("div");
    body.className = "debug-body";

    /* ---- 组1 子机手动 ---- */
    var t1 = document.createElement("div");
    t1.className = "debug-section-title";
    t1.textContent = "子机手动 (前后左右 / 风机)";
    body.appendChild(t1);

    /* 方向键 3x3 网格 */
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
        if (!self._unlocked) return;
        if (dir === "stop") { self._stop(true, false); return; }
        self._start(dir);
      });
      b.addEventListener("pointerup", function () { if (self._unlocked) self._stop(); });
      b.addEventListener("pointercancel", function () { if (self._unlocked) self._stop(); });
      b.addEventListener("pointerleave", function () { if (self._activeDir === dir) self._stop(); });
      pad.appendChild(b);
    });
    body.appendChild(pad);

    /* 风机 / 人工介入 */
    var fanRow = document.createElement("div");
    fanRow.className = "debug-row";
    fanRow.style.marginTop = "12px";
    [
      { label: "\u98ce\u673a +", key: 3 },
      { label: "\u98ce\u673a \u2212", key: 4 },
      { label: "\u505c\u98ce\u673a", key: 6 }
    ].forEach(function (it) {
      var b = document.createElement("button");
      b.className = "debug-send";
      b.style.flex = "1 1 0";
      b.textContent = it.label;
      b.addEventListener("click", function () {
        if (!self._unlocked) return;
        self._sendKey(it.key);
      });
      fanRow.appendChild(b);
    });
    body.appendChild(fanRow);

    var abortRow = document.createElement("div");
    abortRow.className = "debug-foot";
    var abortBtn = document.createElement("button");
    abortBtn.className = "btn btn-danger";
    abortBtn.textContent = "\u4eba\u5de5\u4ecb\u5165 (\u4e2d\u6b62\u534f\u540c)";
    abortBtn.addEventListener("click", function () { self._abort(); });
    abortRow.appendChild(abortBtn);
    body.appendChild(abortRow);

    /* ---- 组2 母机快捷 ---- */
    var t2 = document.createElement("div");
    t2.className = "debug-section-title";
    t2.textContent = "母机快捷 (人工介入后可用)";
    t2.style.marginTop = "18px";
    body.appendChild(t2);

    var step2 = document.createElement("div");
    step2.className = "debug-grid coop-step2 disabled";
    [
      { label: "\u4e0b\u538b", fn: function () { self._sendOne(9, 95); } },
      { label: "\u9884\u5939\u7d27", fn: function () { self._sendGrip(40); } },
      { label: "\u5939\u7d27", fn: function () { self._sendGrip(0); } },
      { label: "\u4e0a\u62ac", fn: function () { self._sendOne(9, 160); } }
    ].forEach(function (it) {
      var b = document.createElement("button");
      b.className = "debug-send coop-step2-btn";
      b.textContent = it.label;
      b.addEventListener("click", function () { if (self._unlocked) it.fn(); });
      step2.appendChild(b);
    });
    body.appendChild(step2);
    this._step2 = step2;

    card.appendChild(head); card.appendChild(body);
    mask.addEventListener("click", function (e) { if (e.target === mask) self.close(); });
    document.body.appendChild(mask);
    this._mask = mask;
  };

  CoopPadModule.prototype.open = function () { if (this._mask) this._mask.classList.add("show"); };
  CoopPadModule.prototype.close = function () { if (this._mask) this._mask.classList.remove("show"); };

  CoopPadModule.prototype._start = function (dir) {
    var self = this;
    this._stop(false);
    this._activeDir = dir;
    var d = DIR[dir];
    if (!d) return;
    this._lh = d.lh; this._lv = d.lv;
    this._flush();
    this._timer = setInterval(function () { self._flush(); }, REPEAT_MS);
  };

  CoopPadModule.prototype._stop = function (sendZero, abortCoop) {
    if (this._timer) { clearInterval(this._timer); this._timer = null; }
    this._activeDir = null;
    this._lh = 0; this._lv = 0;
    if (sendZero !== false) this._flush();
    if (abortCoop) this._abort();
  };

  CoopPadModule.prototype._flush = function () {
    this.app.transport.send(Protocol.joypad(this._lh, this._lv, 0, 0, 0));
  };

  CoopPadModule.prototype._sendKey = function (key) {
    var ok = this.app.transport.send(Protocol.joypad(0, 0, 0, 0, key));
    UI.log("\u534f\u540c\u952e KEY=" + key + (ok ? "" : " (未发送)"), !ok);
  };

  CoopPadModule.prototype._abort = function () {
    var ok = this.app.transport.send(Protocol.coopAbort());
    UI.log("\u4eba\u5de5\u4ecb\u5165: \u534f\u540c\u4e2d\u6b62" + (ok ? "" : " (未发送)"), !ok);
    if (ok) {
      UI.toast("\u5df2\u4eba\u5de5\u4ecb\u5165, \u53ef\u624b\u52a8\u63a7\u5236");
      this._unlock();
    }
  };

  CoopPadModule.prototype._unlock = function () {
    this._unlocked = true;
    if (this._step2) this._step2.classList.remove("disabled");
  };

  CoopPadModule.prototype._sendOne = function (id, deg) {
    var ok = this.app.transport.send(Protocol.servoOne(id, deg, MOVE_DUR));
    UI.log("\u534f\u540c \u8235\u673a" + id + " -> " + deg + "\u00b0" + (ok ? "" : " (未发送)"), !ok);
  };

  CoopPadModule.prototype._sendGrip = function (deg) {
    var ok = this.app.transport.send(Protocol.gripper(deg));
    UI.log("\u534f\u540c \u5939\u6301\u5668 -> " + deg + "\u00b0" + (ok ? "" : " (未发送)"), !ok);
  };

  CoopPadModule.prototype.onState = function () {};

  global.CoopPadModule = CoopPadModule;
})(typeof window !== "undefined" ? window : globalThis);