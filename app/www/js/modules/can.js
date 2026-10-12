/*
 * modules/can.js - CAN RS02 电机控制模块
 * 功能: 使能 / 失能 / 设零, 实时角度反馈
 * 扩展: 在 MOTORS 中增减电机即可, UI 自动生成
 */
(function (global) {
  'use strict';

  var MOTORS = [
    { id: 1, name: '电机 1' },
    { id: 2, name: '电机 2' }
  ];

  function CanModule(app) {
    this.app = app;
    this._els = {};
  }

  CanModule.prototype.mount = function (container) {
    var self = this;
    container.innerHTML = '';

    MOTORS.forEach(function (motor) {
      var feedback = UI.el('span', { class: 'mrow-value', text: '—' });

      container.appendChild(UI.el('div', { class: 'mrow' }, [
        UI.el('label', { class: 'mrow-label', text: motor.name }),
        feedback
      ]));

      container.appendChild(UI.el('div', { class: 'mrow-actions' }, [
        UI.el('button', {
          class: 'btn btn-green',
          text: '使能',
          onclick: function () { self.command(motor.id, Protocol.CAN_SUB.ENABLE, '使能'); }
        }),
        UI.el('button', {
          class: 'btn btn-gray',
          text: '失能',
          onclick: function () { self.command(motor.id, Protocol.CAN_SUB.DISABLE, '失能'); }
        }),
        UI.el('button', {
          class: 'btn btn-gray',
          text: '设零',
          onclick: function () { self.command(motor.id, Protocol.CAN_SUB.SET_ZERO, '设零'); }
        })
      ]));

      self._els[motor.id] = { feedback: feedback };
    });
  };

  CanModule.prototype.command = function (motorId, subCmd, label) {
    var sent = this.app.transport.send(Protocol.canCmd(motorId, subCmd));
    UI.log('CAN ' + motorId + ' ' + label + (sent ? '' : ' (未发送)'), !sent);
    if (sent) UI.toast('电机 ' + motorId + ' ' + label);
  };

  CanModule.prototype.onState = function (state) {
    var self = this;
    MOTORS.forEach(function (motor) {
      var cached = self._els[motor.id];
      if (!cached) return;
      cached.feedback.textContent = UI.fmt(UI.radToDeg(state.can['m' + motor.id]), 1) + '°';
    });
  };

  global.CanModule = CanModule;
})(typeof window !== 'undefined' ? window : globalThis);
