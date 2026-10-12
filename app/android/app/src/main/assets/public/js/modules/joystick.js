/*
 * modules/joystick.js - 子机手柄控制 (四键十字方向键)
 * 方向: ↑↓=前后(LH)  ←→=转向(LV)  → CMD 0x18
 * 按键: 3=风机+ 4=风机- 1=夹持+ 2=夹持- → CMD 0x18 KEY
 * 母机 joypad 帧: LH LV RH RV KEY (int8 -100~100 / key)
 * 空中协议(母机→子机): buf[0]=0x00, buf[1]=LH=前后, buf[2]=LV=转向, buf[3]=RH, buf[4]=RV, buf[5]=KEY
 */
(function (global) {
  "use strict";

  var AXIS_MAX = 100;
  var REPEAT_MS = 80;

  var DIR = {
    up:    { lh:  AXIS_MAX, lv: 0 },
    down:  { lh: -AXIS_MAX, lv: 0 },
    left:  { lh: 0, lv: -AXIS_MAX },
    right: { lh: 0, lv:  AXIS_MAX }
  };

  function JoystickModule(app) {
    this.app = app;
    this._axisX = null;
    this._axisY = null;
    this._sendState = null;
    this._lh = 0;
    this._lv = 0;
    this._timer = null;
    this._activeDir = null;
    this._keyTimer = null;
  }

  JoystickModule.prototype.mount = function () {
    var self = this;
    this._axisX = UI.$("#axisX");
    this._axisY = UI.$("#axisY");
    this._sendState = UI.$("#childSendState");

    var dpad = UI.$("#dpad");
    if (dpad) {
      Array.prototype.forEach.call(dpad.querySelectorAll("[data-dpad]"), function (btn) {
        var dir = btn.getAttribute("data-dpad");
        btn.addEventListener("pointerdown", function (e) {
          e.preventDefault();
          if (dir === "stop") { self._stop(true, true); return; }
          self._start(dir, btn);
        });
        btn.addEventListener("pointerup", function () { self._stop(); });
        btn.addEventListener("pointercancel", function () { self._stop(); });
        btn.addEventListener("pointerleave", function () { if (self._activeDir === dir) self._stop(); });
      });
    }

    Array.prototype.forEach.call(
      document.querySelectorAll("[data-child-key]"),
      function (btn) {
        btn.addEventListener("pointerdown", function (e) {
          e.preventDefault();
          self._pressKey(parseInt(btn.getAttribute("data-child-key"), 10), btn);
        });
      }
    );
  };

  JoystickModule.prototype._start = function (dir, btn) {
    var self = this;
    this._stop(false);
    this._activeDir = dir;
    var d = DIR[dir];
    if (!d) return;
    if (btn) btn.classList.add("held");
    this._lh = d.lh;
    this._lv = d.lv;
    this._render();
    this._flushAxes();
    this._timer = setInterval(function () { self._flushAxes(); }, REPEAT_MS);
  };

  JoystickModule.prototype._stop = function (sendZero, abortCoop) {
    if (this._timer) { clearInterval(this._timer); this._timer = null; }
    this._activeDir = null;
    this._lh = 0;
    this._lv = 0;
    this._render();
    var held = document.querySelectorAll(".dpad-btn.held");
    for (var i = 0; i < held.length; i++) held[i].classList.remove("held");
    if (sendZero !== false) this._flushAxes();
    if (abortCoop) this._abortCoop();
  };

  /* 停按钮: 先发协同中止(0x1D)打断子母协同, 让操作者手动接管 */
  JoystickModule.prototype._abortCoop = function () {
    if (this.app && this.app.view === "child") {
      var frame = Protocol.coopAbort();
      var ok = this.app.transport.send(frame);
      UI.log("协同中止" + (ok ? "" : " (未发送)"), !ok);
      if (ok) UI.toast("已中止协同, 可手动接管");
    }
  };

  JoystickModule.prototype._render = function () {
    if (this._axisX) this._axisX.textContent = this._lh;
    if (this._axisY) this._axisY.textContent = this._lv;
  };

  JoystickModule.prototype._flushAxes = function () {
    if (this.app && this.app.view === "child") {
      this._send(this._lh, this._lv, 0);
    }
  };

  JoystickModule.prototype._send = function (lh, lv, key) {
    var frame = Protocol.joypad(lh, lv, 0, 0, key);
    var ok = this.app.transport.send(frame);
    if (this._sendState) {
      this._sendState.textContent = ok ? "已发送" : "未连接";
      this._sendState.style.color = ok ? "rgba(52,199,89,.9)" : "rgba(255,69,58,.9)";
    }
    return ok;
  };

  JoystickModule.prototype._pressKey = function (key, btn) {
    this._send(this._lh, this._lv, key);
    if (btn) {
      btn.classList.add("held");
      if (this._keyTimer) clearTimeout(this._keyTimer);
      this._keyTimer = setTimeout(function () { btn.classList.remove("held"); }, 120);
    }
    UI.log("子机按键 KEY=" + key);
  };

  JoystickModule.prototype.refresh = function () {};

  JoystickModule.prototype.onState = function () {};

  global.JoystickModule = JoystickModule;
})(typeof window !== "undefined" ? window : globalThis);