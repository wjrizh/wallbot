/*
 * app.js - 应用入口
 * 职责: 组装传输层与业务模块, 绑定全局 UI, 持久化地址, 母机/子机界面切换
 * 性能: 状态帧以 rAF 合并刷新, 避免高频重排
 * 扩展: 新增模块时实现 mount()/onState(), 在 App.modules 中注册即可
 */
(function (global) {
  'use strict';

  var HOST_STORAGE_KEY = 'wallbot.host';

  function App() {
    this.transport = new Transport();
    this.view = 'host';
    this.modules = [
      new CanModule(this),
      new GaitModule(this),
      new CoopModule(this),
      new CameraModule(this),
      new JoystickModule(this),
      new AeModule(this),
      new DebugModule(this),
      new GripperModule(this),
      new CoopPadModule(this),
    ];
    this._connDot = null;
    this._connText = null;
    this._pendingState = null;
    this._frameRequested = false;
  }

  App.prototype.init = function () {
    var self = this;
    this._connDot = UI.$('#connDot');
    this._connText = UI.$('#connText');

    this.modules.forEach(function (module) {
      if (typeof module.mount !== 'function') return;
      try {
        module.mount(self._containerFor(module));
      } catch (err) {
        UI.log('[FATAL] ' + (module.constructor && module.constructor.name) + '.mount 崩溃: ' + (err && err.message), true);
      }
    });

    this._attachCamera(this.view);
    this._bindCameraZoom();

    this.transport.on('status', function (status) { self._onStatus(status); });
    this.transport.on('state', function (state) { self._queueState(state); });
    this.transport.on('ack', function (ack) { self._onAck(ack); });
    this.transport.on('log', function (msg) { UI.log(msg); });

    this._bindControls();
    this._loadSavedHost();
    this.transport.connect();
    this._onStatus(Transport.STATUS.OFFLINE);
    this._autoDiscover();

    var sw = document.getElementById('viewSwitch');
    if (sw) {
      this._moveThumb(sw);
      window.addEventListener('resize', function () { self._moveThumb(sw); });
    }

    this._hideMask();
  };

  /* 命令回执: 在日志打印一条闭环结果, 便于逐条核对 */
  var CMD_NAME = {
    0x01: '单舵机', 0x02: '全舵机', 0x03: '真空泵', 0x04: 'CAN角度', 0x05: 'CAN命令',
    0x09: '自动纠偏', 0x0A: '纠偏急停', 0x11: '越障', 0x12: '启停',
    0x13: '迈步', 0x14: '掉头', 0x18: '手柄', 0x19: '启动协同', 0x1B: '夹持器',
    0x1C: '避障行走'
  };

  App.prototype._onAck = function (ack) {
    var hex = '0x' + (ack.cmd < 16 ? '0' : '') + ack.cmd.toString(16).toUpperCase();
    var name = CMD_NAME[ack.cmd] || '未知';
    UI.log((ack.ok ? '✓ ' : '✗ ') + name + ' ' + hex + ' ' + ack.text, !ack.ok);
  };

  App.prototype._hideMask = function () {
    var mask = document.getElementById('connectMask');
    if (mask && !mask.classList.contains('hidden')) {
      mask.classList.add('hidden');
    }
  };

  /* 返回模块挂载容器; 界面为静态 HTML 的模块返回 null */
  App.prototype._containerFor = function (module) {
    if (module instanceof CanModule) return UI.$('#canCard');
    if (module instanceof CoopModule) return UI.$('#coopCard');
    if (module instanceof CameraModule) return null;
    return null;
  };

  /* 双击摄像头块: 放大独占 / 再双击恢复双区域 */
  App.prototype._bindCameraZoom = function () {
    var duo = document.querySelector('#cameraDock .camera-duo');
    if (!duo) return;
    var cells = duo.querySelectorAll('.camera-cell');
    duo.addEventListener('dblclick', function (e) {
      var cell = e.target.closest ? e.target.closest('.camera-cell') : null;
      if (!cell || !duo.contains(cell)) return;
      if (duo.classList.contains('zoomed') && duo.querySelector('.zoomed-cell') === cell) {
        duo.classList.remove('zoomed', 'zoom-right');
        cell.classList.remove('zoomed-cell');
      } else {
        var prev = duo.querySelector('.zoomed-cell');
        if (prev) prev.classList.remove('zoomed-cell');
        var isRight = cells.length > 1 && cells[cells.length - 1] === cell;
        cell.classList.add('zoomed-cell');
        duo.classList.toggle('zoom-right', isRight);
        duo.classList.add('zoomed');
      }
    });
  };


  /* 把常驻摄像头层搬运到当前界面 slot; appendChild 为移动语义, <img> 不销毁, 流不断 */
  App.prototype._attachCamera = function (view) {
    var dock = document.getElementById('cameraDock');
    var slot = document.getElementById(view === 'child' ? 'cameraSlotChild' : 'cameraSlotHost');
    if (dock && slot && dock.parentNode !== slot) slot.appendChild(dock);
  };

  /* 界面切换: host=母机(白) / child=子机(黑) */
  App.prototype._setView = function (view) {
    if (this.view === view) return;

    var hostEl = UI.$('#viewHost');
    var childEl = UI.$('#viewChild');
    var sw = document.getElementById('viewSwitch');

    this._attachCamera(view);

    if (view === 'host') {
      if (hostEl) hostEl.classList.remove('leaving');
      if (childEl) childEl.classList.remove('active');
    } else {
      if (hostEl) hostEl.classList.add('leaving');
      if (childEl) childEl.classList.add('active');
    }

    var segItems = document.querySelectorAll('.seg-item');
    for (var i = 0; i < segItems.length; i++) {
      segItems[i].classList.toggle('active', segItems[i].getAttribute('data-view') === view);
    }

    if (sw) {
      sw.setAttribute('data-active', view);
      sw.classList.toggle('seg-dark', view === 'child');
      this._moveThumb(sw);
    }

    this.view = view;
    UI.log(view === 'child' ? '已切换到子机控制' : '已切换到母机控制');

    if (view === 'child') {
      var self = this;
      var settle = function () {
        var joy = self.modules.filter(function (m) { return m instanceof JoystickModule; })[0];
        if (joy && typeof joy.refresh === 'function') joy.refresh();
        var ae = self.modules.filter(function (m) { return m instanceof AeModule; })[0];
        if (ae && typeof ae.refresh === 'function') ae.refresh();
      };
      if (childEl) childEl.addEventListener('transitionend', settle, { once: true });
      setTimeout(settle, 320);
    }
  };

  /* 滑块定位: 按激活按钮实测位置与宽度 */
  App.prototype._moveThumb = function (sw) {
    var thumb = sw.querySelector('.seg-thumb');
    var btn = sw.querySelector('.seg-item.active');
    if (!thumb || !btn) return;
    thumb.style.width = btn.offsetWidth + 'px';
    thumb.style.transform = 'translateX(' + (btn.offsetLeft - 3) + 'px)';
  };

  /* 状态帧 5Hz 到达: 合并到下一帧统一刷新, 避免逐帧布局抖动 */
  App.prototype._queueState = function (state) {
    var self = this;
    this._pendingState = state;
    if (this._frameRequested) return;
    this._frameRequested = true;
    requestAnimationFrame(function () {
      self._frameRequested = false;
      var latest = self._pendingState;
      self._pendingState = null;
      if (latest) self._applyState(latest);
    });
  };

  App.prototype._applyState = function (state) {
    this.modules.forEach(function (module) {
      if (typeof module.onState === 'function') module.onState(state);
    });
  };

  App.prototype._bindControls = function () {
    var self = this;

    UI.$('#btnClearLog').addEventListener('click', function () { UI.clearLog(); });

    var applyBtn = UI.$('#btnApplyHost');
    if (applyBtn) applyBtn.addEventListener('click', function () { self._applyManualHost(); });

    var segItems = document.querySelectorAll('.seg-item');
    for (var i = 0; i < segItems.length; i++) {
      segItems[i].addEventListener('click', function () { self._setView(this.getAttribute('data-view')); });
    }

    this._autoCamera();
  };

  App.prototype._autoCamera = function () {
    var cam = this._cameraModule();
    if (cam) cam.connect();
  };

  App.prototype._cameraModule = function () {
    return this.modules.filter(function (m) { return m instanceof CameraModule; })[0];
  };

  App.prototype._loadSavedHost = function () {
    try {
      var saved = global.localStorage && global.localStorage.getItem(HOST_STORAGE_KEY);
      if (saved) this.transport.setHost(saved);
    } catch (e) {}
    this._syncHostInputs();
  };

  App.prototype._syncHostInputs = function () {
    var hostEl = UI.$('#hostInput');
    if (hostEl) hostEl.value = this.transport.getHost() || '';
    var cam = this._cameraModule();
    var camHostEl = UI.$('#camHostInput');
    var camChildEl = UI.$('#camChildInput');
    if (cam && camHostEl) camHostEl.value = cam.getIp('host') || '';
    if (cam && camChildEl) camChildEl.value = cam.getIp('child') || '';
  };

  /* 手动兜底: 用户填写地址后应用 */
  App.prototype._applyManualHost = function () {
    var hostEl = UI.$('#hostInput');
    var camHostEl = UI.$('#camHostInput');
    var camChildEl = UI.$('#camChildInput');
    var host = hostEl ? hostEl.value.trim() : '';
    var camHost = camHostEl ? camHostEl.value.trim() : '';
    var camChild = camChildEl ? camChildEl.value.trim() : '';
    this._applyDiscovery({ host: host, camHost: camHost, camChild: camChild });
    UI.log('[地址] 已应用 母机=' + (host || '-') + ' 摄像头母机=' + (camHost || '-') + ' 摄像头子机=' + (camChild || '-'));
  };

  /* 应用发现结果: 母机 WS 地址 + 两路摄像头地址 */
  App.prototype._applyDiscovery = function (res) {
    if (!res) return;
    var host = res.host || '';
    if (host && host !== this.transport.getHost()) {
      this.transport.setHost(host);
      try { global.localStorage.setItem(HOST_STORAGE_KEY, host); } catch (e) {}
      UI.log('[发现] 母机 ' + host);
      this.transport.disconnect();
      this.transport.connect();
    }

    var cam = this._cameraModule();
    if (cam) {
      if (res.camHost) cam.setIp('host', res.camHost);
      if (res.camChild) cam.setIp('child', res.camChild);
      cam.connect();
    }
    this._syncHostInputs();
    if (!host && !res.camHost && !res.camChild) UI.log('[发现] 未收到任何设备应答');
  };

  /* 原生 UDP 广播发现; 非 Capacitor 环境降级为使用已保存地址 */
  App.prototype._autoDiscover = function () {
    var self = this;
    var plugin = global.Capacitor && global.Capacitor.Plugins && global.Capacitor.Plugins.Discovery;
    if (!plugin || typeof plugin.discover !== 'function') {
      UI.log('[发现] 原生插件不可用, 使用已保存地址');
      return;
    }
    UI.log('[发现] 正在扫描局域网…');
    plugin.discover().then(function (res) { self._applyDiscovery(res); })
      .catch(function (err) { UI.log('[发现] 失败: ' + (err && err.message)); });
  };

  App.prototype._onStatus = function (status) {
    var map = {
      online:     { dot: 'dot-on',   text: '已连接' },
      connecting: { dot: 'dot-off',  text: '未连接' },
      offline:    { dot: 'dot-off',  text: '未连接' }
    };
    var cfg = map[status] || map.offline;
    if (this._connDot) this._connDot.className = 'dot ' + cfg.dot;
    if (this._connText) this._connText.textContent = cfg.text;

    if (status === 'online') this._hideMask();
  };

  document.addEventListener('DOMContentLoaded', function () {
    try {
      NativeAdapter.init();
      var app = new App();
      app.init();
      global.wallbot = app;
    } catch (err) {
      UI.log('[FATAL] init 崩溃: ' + (err && err.message) + ' @ ' + (err && err.stack ? err.stack.split('\n')[1] : ''), true);
    }
  });

  global.App = App;
})(typeof window !== 'undefined' ? window : globalThis);