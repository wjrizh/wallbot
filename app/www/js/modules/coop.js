/*
 * modules/coop.js - 子母协同模块
 * 功能: 启动协同 (CMD 0x19) 
 * 显示: 母机滚转与 IMU 在线状态 (状态帧不含子机 IMU)
 * 扩展: ACTIONS 中追加动作即可, UI 自动生成
 */
(function (global) {
  'use strict';

  var ACTIONS = [
    { method: 'coopStart',  label: '启动协同', style: 'btn-primary', toast: '子母协同已启动' },
  ];

  function CoopModule(app) {
    this.app = app;
    this._els = {};
  }

  CoopModule.prototype.mount = function (container) {
    var self = this;
    container.innerHTML = '';

    container.appendChild(UI.el('div', { class: 'mrow-actions' },
      ACTIONS.map(function (action) {
        return UI.el('button', {
          class: 'btn ' + action.style,
          text: action.label,
          onclick: function () { self.run(action); }
        });
      })
    ));

    var rollValue = UI.el('span', { class: 'mrow-value', text: '—' });
    var rollPill = UI.el('span', { class: 'pill', text: '离线' });
    container.appendChild(UI.el('div', { class: 'mrow' }, [
      UI.el('label', { class: 'mrow-label', text: '母机滚转' }),
      rollValue,
      rollPill
    ]));

    var gndValue = UI.el('span', { class: 'mrow-value', text: '—' });
    container.appendChild(UI.el('div', { class: 'mrow' }, [
      UI.el('label', { class: 'mrow-label', text: '地面基准' }),
      gndValue
    ]));

    var laserValue = UI.el('span', { class: 'mrow-value', text: '—' });
    var laserPill = UI.el('span', { class: 'pill', text: '离线' });
    container.appendChild(UI.el('div', { class: 'mrow' }, [
      UI.el('label', { class: 'mrow-label', text: '前方障碍' }),
      laserValue,
      laserPill
    ]));

    this._els.roll = rollValue;
    this._els.rollPill = rollPill;
    this._els.gnd = gndValue;
    this._els.laser = laserValue;
    this._els.laserPill = laserPill;
  };

  CoopModule.prototype.run = function (action) {
    var build = Protocol[action.method];
    var frame = typeof build === 'function' ? build() : null;
    if (!frame) { UI.log('未知协同动作: ' + action.method, true); return; }
    var sent = this.app.transport.send(frame);
    UI.log(action.label + (sent ? '' : ' (未发送)'), !sent);
    if (sent) UI.toast(action.toast);
  };

  CoopModule.prototype.onState = function (state) {
    if (this._els.roll) this._els.roll.textContent = UI.fmt(state.imu.roll, 1) + '°';
    if (this._els.gnd) this._els.gnd.textContent = UI.fmt(state.imu.ground, 1) + '°';
    if (this._els.rollPill) {
      this._els.rollPill.textContent = state.imu.online ? '在线' : '离线';
      this._els.rollPill.className = 'pill ' + (state.imu.online ? 'on' : 'off');
    }
    if (state.laser) {
      if (this._els.laser) {
        var mm = state.laser.front;
        this._els.laser.textContent = (state.laser.online && mm > 0) ? (mm + ' mm') : '—';
      }
      if (this._els.laserPill) {
        this._els.laserPill.textContent = state.laser.online ? '在线' : '离线';
        this._els.laserPill.className = 'pill ' + (state.laser.online ? 'on' : 'off');
      }
    }
  };

  global.CoopModule = CoopModule;
})(typeof window !== 'undefined' ? window : globalThis);