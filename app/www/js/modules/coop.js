/*
 * modules/coop.js - 子母协同模块
 * 功能: 启动协同 (CMD 0x19)
 * 显示: 当前协同阶段 / 问题告警 (0x88) + 母机滚转 / 激光
 */
(function (global) {
  'use strict';

  var ACTIONS = [
    { method: 'coopStart',  label: '启动协同', style: 'btn-primary', toast: '子母协同已启动' },
  ];

  function CoopModule(app) {
    this.app = app;
    this._els = {};
    this._lastStage = '';
    this._lastAlert = '';
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

    var stageValue = UI.el('span', { class: 'mrow-value', text: '空闲' });
    container.appendChild(UI.el('div', { class: 'mrow' }, [
      UI.el('label', { class: 'mrow-label', text: '协同阶段' }),
      stageValue
    ]));

    var alertValue = UI.el('span', { class: 'mrow-value', text: '—' });
    container.appendChild(UI.el('div', { class: 'mrow' }, [
      UI.el('label', { class: 'mrow-label', text: '问题反馈' }),
      alertValue
    ]));

    var roundValue = UI.el('span', { class: 'mrow-value', text: '—' });
    container.appendChild(UI.el('div', { class: 'mrow' }, [
      UI.el('label', { class: 'mrow-label', text: '纠偏轮次' }),
      roundValue
    ]));

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

    this._els.stage = stageValue;
    this._els.alert = alertValue;
    this._els.round = roundValue;
    this._els.roll = rollValue;
    this._els.rollPill = rollPill;
    this._els.gnd = gndValue;
    this._els.laser = laserValue;
    this._els.laserPill = laserPill;

    if (this.app && this.app.transport) {
      this.app.transport.on('alignState', function (s) { self._onAlignState(s); });
    }
  };

  CoopModule.prototype._onAlignState = function (s) {
    if (!s) return;
    var stage = s.stateText || ('状态' + s.state);
    if (this._els.stage) this._els.stage.textContent = stage;

    if (this._els.round) {
      if (s.state === 0) this._els.round.textContent = '—';
      else this._els.round.textContent = s.iter + ' / ' + s.maxIter;
    }

    var alertTxt = s.alertText || '';
    if (this._els.alert) {
      this._els.alert.textContent = alertTxt || '—';
      this._els.alert.style.color = alertTxt ? '#c0392b' : '';
    }

    /* 阶段变化记一条日志, 便于对照问题 */
    if (stage !== this._lastStage && s.state !== 0) {
      this._lastStage = stage;
      UI.log('协同阶段: ' + stage);
    }
    if (alertTxt && alertTxt !== this._lastAlert) {
      this._lastAlert = alertTxt;
      UI.log('协同问题: ' + alertTxt, true);
      if (typeof UI.toast === 'function') UI.toast(alertTxt);
    }
    if (!alertTxt) this._lastAlert = '';
  };

  CoopModule.prototype.run = function (action) {
    var build = Protocol[action.method];
    if (typeof build !== 'function') { UI.log('未知协同动作: ' + action.method, true); return; }
    var ip = '';
    try { ip = (this.app.camera && this.app.camera.getIp('child')) || ''; } catch (e) {}
    var frame = action.method === 'coopStart' ? build(ip) : build();
    var sent = this.app.transport.send(frame);
    UI.log(action.label + (ip ? ' (视觉板 ' + ip + ')' : ' (无视觉板地址, 用母机缓存)'), !sent);
    if (sent) {
      UI.toast(action.toast);
      this._lastStage = '';
      this._lastAlert = '';
      if (this._els.stage) this._els.stage.textContent = '启动中…';
      if (this._els.alert) { this._els.alert.textContent = '—'; this._els.alert.style.color = ''; }
    }
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
