/*
 * modules/gait.js - 步态模块
 * 功能: 步距前进 / 越障 / 掉头 / 启动 / 急停
 */
(function (global) {
  'use strict';

  var STEP_MIN = 10;
  var STEP_MAX = 200;

  function GaitModule(app) {
    this.app = app;
    this._stepInput = null;
  }

  GaitModule.prototype.mount = function () {
    var self = this;
    this._stepInput = UI.$('#stepInput');

    var stepper = UI.$('#stepStepper');
    if (stepper) {
      Array.prototype.forEach.call(stepper.querySelectorAll('[data-step-delta]'), function (btn) {
        btn.addEventListener('click', function () {
          self._bumpStep(parseInt(btn.getAttribute('data-step-delta'), 10));
        });
      });
    }

    if (this._stepInput) {
      this._stepInput.addEventListener('change', function () { self._clampStep(); });
      this._stepInput.addEventListener('blur', function () { self._clampStep(); });
    }

    UI.$('#btnForward').addEventListener('click', function () { self.forward(); });
    UI.$('#btnObstacle').addEventListener('click', function () { self.obstacle(); });
    UI.$('#btnUTurn').addEventListener('click', function () { self.uTurn(); });
    UI.$('#btnStart').addEventListener('click', function () { self.gaitStart(); });
    UI.$('#btnStop').addEventListener('click', function () { self.estop(); });
  };

  GaitModule.prototype._stepValue = function () {
    var v = parseInt(this._stepInput ? this._stepInput.value : '80', 10);
    return isNaN(v) ? 80 : v;
  };

  GaitModule.prototype._bumpStep = function (delta) {
    if (!this._stepInput) return;
    var v = this._stepValue() + delta;
    v = Math.max(STEP_MIN, Math.min(STEP_MAX, v));
    this._stepInput.value = v;
  };

  GaitModule.prototype._clampStep = function () {
    if (!this._stepInput) return;
    var v = this._stepValue();
    v = Math.max(STEP_MIN, Math.min(STEP_MAX, v));
    this._stepInput.value = v;
  };

  GaitModule.prototype.forward = function () {
    var step = this._stepValue();
    var sent = this.app.transport.send(Protocol.step(step));
    UI.log('前进 ' + step + 'mm' + (sent ? '' : ' (未发送)'), !sent);
    if (sent) UI.toast('前进 ' + step + 'mm');
  };

  GaitModule.prototype.obstacle = function () {
    var sent = this.app.transport.send(Protocol.obstacle());
    UI.log('越障' + (sent ? '' : ' (未发送)'), !sent);
    if (sent) UI.toast('越障');
  };

  GaitModule.prototype.estop = function () {
    /* 先停避障自动运行(EXEC 期会急停), 再发步态急停兜底 */
    var s1 = this.app.transport.send(Protocol.avoidRun(0));
    var s2 = this.app.transport.send(Protocol.gaitRun(0));
    var sent = s1 || s2;
    UI.log('停止' + (sent ? '' : ' (未发送)'), !sent);
    UI.toast('已停止');
  };

  GaitModule.prototype.uTurn = function () {
    var sent = this.app.transport.send(Protocol.uTurn());
    UI.log('掉头' + (sent ? '' : ' (未发送)'), !sent);
    if (sent) UI.toast('掉头');
  };

  GaitModule.prototype.gaitStart = function () {
    /* 开启激光自主避障行走 */
    var sent = this.app.transport.send(Protocol.avoidRun(1));
    UI.log('自动运行' + (sent ? '' : ' (未发送)'), !sent);
    if (sent) UI.toast('自动运行');
  };

  global.GaitModule = GaitModule;
})(typeof window !== 'undefined' ? window : globalThis);