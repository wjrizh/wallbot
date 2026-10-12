/*
 * modules/debug.js - 舵机调试浮窗
 * 功能: 实时显示 M1/M2/M3/PB9 角度(输入框自动跟随), 输入目标角度+旋转时间单独下发 (0x01)
 *       前/后真空泵开关 (0x03)
 */
(function (global) {
  "use strict";
  var SERVOS = [
    { id: 1, name: "M1 前髋" },
    { id: 2, name: "M2 中膝" },
    { id: 3, name: "M3 后髋" },
    { id: 9, name: "PB9 后垂" }
  ];
  var DEF_DUR = 1.0;

  function DebugModule(app) {
    this.app = app;
    this._els = {};
    this._mask = null;
    this._pumpEls = {};
  }

  DebugModule.prototype.mount = function () {
    var self = this;
    var entry = document.getElementById("btnDebug");
    if (entry) entry.addEventListener("click", function () { self.open(); });
    this._build();
  };

  DebugModule.prototype._build = function () {
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
    title.textContent = "调试";
    var closeBtn = document.createElement("button");
    closeBtn.className = "debug-close";
    closeBtn.textContent = "\u00d7";
    closeBtn.addEventListener("click", function () { self.close(); });
    head.appendChild(title); head.appendChild(closeBtn);

    var body = document.createElement("div");
    body.className = "debug-body";

    /* ---- 舵机区 ---- */
    var servoTitle = document.createElement("div");
    servoTitle.className = "debug-section-title";
    servoTitle.textContent = "舵机角度 / 旋转时间";
    body.appendChild(servoTitle);

    var grid = document.createElement("div");
    grid.className = "debug-grid";
    SERVOS.forEach(function (servo) {
      var item = document.createElement("div");
      item.className = "debug-item";

      var itemHead = document.createElement("div");
      itemHead.className = "debug-item-head";
      var nameEl = document.createElement("span");
      nameEl.className = "debug-name";
      nameEl.textContent = servo.name;
      var curEl = document.createElement("span");
      curEl.className = "debug-cur";
      curEl.textContent = "--";
      itemHead.appendChild(nameEl); itemHead.appendChild(curEl);

      var row = document.createElement("div");
      row.className = "debug-row";
      var input = document.createElement("input");
      input.className = "debug-input";
      input.type = "number";
      input.min = "0"; input.max = "180"; input.step = "1";
      input.placeholder = "角度";
      var durInput = document.createElement("input");
      durInput.className = "debug-input debug-dur";
      durInput.type = "number";
      durInput.min = "0.1"; durInput.max = "10"; durInput.step = "0.1";
      durInput.placeholder = "秒";
      durInput.value = DEF_DUR;
      var sendBtn = document.createElement("button");
      sendBtn.className = "debug-send";
      sendBtn.textContent = "发送";
      sendBtn.addEventListener("click", function () { self._sendOne(servo, input, durInput); });
      input.addEventListener("keydown", function (e) { if (e.key === "Enter") self._sendOne(servo, input, durInput); });
      durInput.addEventListener("keydown", function (e) { if (e.key === "Enter") self._sendOne(servo, input, durInput); });
      row.appendChild(input); row.appendChild(durInput); row.appendChild(sendBtn);

      item.appendChild(itemHead); item.appendChild(row);
      grid.appendChild(item);
      self._els[servo.id] = { cur: curEl, input: input, dur: durInput };
    });
    body.appendChild(grid);

    /* ---- 真空泵区 ---- */
    var pumpTitle = document.createElement("div");
    pumpTitle.className = "debug-section-title";
    pumpTitle.textContent = "真空泵";
    body.appendChild(pumpTitle);

    var pumpGrid = document.createElement("div");
    pumpGrid.className = "debug-grid";
    [{ id: 1, name: "前泵" }, { id: 2, name: "后泵" }].forEach(function (pump) {
      var item = document.createElement("div");
      item.className = "debug-item";
      var itemHead = document.createElement("div");
      itemHead.className = "debug-item-head";
      var nameEl = document.createElement("span");
      nameEl.className = "debug-name";
      nameEl.textContent = pump.name;
      var stEl = document.createElement("span");
      stEl.className = "debug-cur debug-pump-state";
      stEl.textContent = "关";
      itemHead.appendChild(nameEl); itemHead.appendChild(stEl);

      var row = document.createElement("div");
      row.className = "debug-row";
      var onBtn = document.createElement("button");
      onBtn.className = "debug-send";
      onBtn.textContent = "吸";
      onBtn.addEventListener("click", function () { self._sendPump(pump.id, 1); });
      var offBtn = document.createElement("button");
      offBtn.className = "debug-send debug-send-off";
      offBtn.textContent = "松";
      offBtn.addEventListener("click", function () { self._sendPump(pump.id, 0); });
      row.appendChild(onBtn); row.appendChild(offBtn);

      item.appendChild(itemHead); item.appendChild(row);
      pumpGrid.appendChild(item);
      self._pumpEls[pump.id] = stEl;
    });
    body.appendChild(pumpGrid);

    card.appendChild(head); card.appendChild(body);
    mask.addEventListener("click", function (e) { if (e.target === mask) self.close(); });
    document.body.appendChild(mask);
    this._mask = mask;
  };

  DebugModule.prototype.open = function () {
    if (this._mask) this._mask.classList.add("show");
  };

  DebugModule.prototype.close = function () {
    if (this._mask) this._mask.classList.remove("show");
  };

  DebugModule.prototype._sendOne = function (servo, input, durInput) {
    var v = parseFloat(input.value);
    if (isNaN(v)) { UI.toast("请输入角度"); return; }
    v = Math.max(0, Math.min(180, v));

    var dur = parseFloat(durInput.value);
    if (isNaN(dur) || dur <= 0) dur = DEF_DUR;
    dur = Math.max(0.1, Math.min(10, dur));

    var sent = this.app.transport.send(Protocol.servoOne(servo.id, v, dur));
    UI.log("调试 " + servo.name + " → " + v + "\u00b0 / " + dur + "s" + (sent ? "" : " (未发送)"), !sent);
    if (sent) UI.toast(servo.name + " → " + v + "\u00b0");
  };

  DebugModule.prototype._sendPump = function (id, state) {
    var sent = this.app.transport.send(Protocol.pump(id, state));
    var name = id === 1 ? "前泵" : "后泵";
    UI.log("调试 " + name + (state ? " 吸" : " 松") + (sent ? "" : " (未发送)"), !sent);
  };

  /* 每帧状态刷新: 只更新实时实际角度大字, 输入框纯作目标输入不被覆盖 */
  DebugModule.prototype.onState = function (state) {
    var self = this;
    SERVOS.forEach(function (servo) {
      var cache = self._els[servo.id];
      if (!cache) return;
      var key = servo.id === 9 ? "pb9" : ("m" + servo.id);
      var raw = state.servo ? state.servo[key] : null;
      if (raw == null) return;
      var deg = Math.round(raw);
      cache.cur.textContent = deg;
    });

    if (state.pump) {
      if (this._pumpEls[1]) this._pumpEls[1].textContent = state.pump.front ? "吸" : "关";
      if (this._pumpEls[2]) this._pumpEls[2].textContent = state.pump.rear ? "吸" : "关";
    }
  };

  global.DebugModule = DebugModule;
})(typeof window !== "undefined" ? window : globalThis);