/*
 * transport.js - 母机 WebSocket 传输层
 * 负责: 连接管理、自动重连、二进制帧收发、状态回调
 * 扩展: 通过 on/off 订阅事件, 不侵入业务模块
 */
(function (global) {
  'use strict';

  var STATUS = { OFFLINE: 'offline', CONNECTING: 'connecting', ONLINE: 'online' };

  function Transport() {
    this._ws = null;
    this._host = '';
    this._port = 80;
    this._path = '/ws';
    this._status = STATUS.OFFLINE;
    this._retry = 0;
    this._retryTimer = null;
    this._manualClose = false;
    this._listeners = {};
    this._stateHandler = null;
    this._everOnline = false;
    this._lastError = '';
  }

  Transport.prototype.on = function (event, handler) {
    (this._listeners[event] || (this._listeners[event] = [])).push(handler);
    return this;
  };

  Transport.prototype.off = function (event, handler) {
    var list = this._listeners[event];
    if (!list) return this;
    this._listeners[event] = list.filter(function (h) { return h !== handler; });
    return this;
  };

  Transport.prototype._emit = function (event, data) {
    var list = this._listeners[event];
    if (!list) return;
    for (var i = 0; i < list.length; i++) list[i](data);
  };

  Transport.prototype.setHost = function (host) {
    this._host = (host || '').trim().replace(/^wss?:\/\//, '').replace(/\/.*$/, '');
  };

  Transport.prototype.getHost = function () { return this._host; };
  Transport.prototype.getStatus = function () { return this._status; };

  Transport.prototype._setStatus = function (status) {
    if (this._status === status) return;
    this._status = status;
    this._emit('status', status);
  };

  Transport.prototype.connect = function () {
    if (this._ws && (this._ws.readyState === 0 || this._ws.readyState === 1)) return;
    this._manualClose = false;
    this._clearRetry();
    this._open();
  };

  Transport.prototype._open = function () {
    var self = this;
    if (!this._host) {
      this._setStatus(STATUS.OFFLINE);
      this._emit('log', '等待设备地址…');
      return;
    }
    var url = 'ws://' + this._host + (this._port === 80 ? '' : ':' + this._port) + this._path;
    this._setStatus(STATUS.CONNECTING);
    this._emit('log', '连接 ' + url);

    var ws;
    try { ws = new WebSocket(url); }
    catch (e) { this._emit('log', '地址无效: ' + e.message); this._scheduleRetry(); return; }

    ws.binaryType = 'arraybuffer';
    this._ws = ws;

    ws.onopen = function () {
      self._retry = 0;
      self._everOnline = true;
      self._setStatus(STATUS.ONLINE);
      self._emit('log', 'WebSocket 已连接');
    };

    ws.onmessage = function (ev) {
      if (!(ev.data instanceof ArrayBuffer)) return;
      var bytes = new Uint8Array(ev.data);
      var state = global.Protocol.parseState(bytes);
      if (state) { self._emit('state', state); return; }
      var ack = global.Protocol.parseAck(bytes);
      if (ack) { self._emit('ack', ack); return; }
      var logMsg = global.Protocol.parseLog(bytes);
      if (logMsg) { self._emit('log', '[固件] ' + logMsg); return; }
    };

    ws.onclose = function (ev) {
      if (ws.__superseded) return;
      if (self._ws === ws) self._ws = null;
      self._setStatus(STATUS.OFFLINE);
      if (self._manualClose) { self._manualClose = false; return; }
      self._emit('log', '连接断开 code=' + (ev && ev.code) + ' reason=' + (ev && ev.reason) + ', 重连中…');
      self._emit('offline', { host: self._host, everOnline: self._everOnline, retry: self._retry, error: self._lastError });
      self._scheduleRetry();
    };

    ws.onerror = function (ev) {
      var msg = '';
      try { msg = ev && ev.message ? ev.message : (ev && ev.type) || ''; } catch (e) {}
      self._emit('log', '连接错误 ' + msg + ' url=' + url);
    };
  };

  Transport.prototype._scheduleRetry = function () {
    var self = this;
    this._clearRetry();
    this._retry = Math.min(this._retry + 1, 6);
    var delay = Math.min(1000 * Math.pow(1.6, this._retry - 1), 8000);
    this._retryTimer = setTimeout(function () { self._open(); }, delay);
  };

  Transport.prototype._clearRetry = function () {
    if (this._retryTimer) { clearTimeout(this._retryTimer); this._retryTimer = null; }
  };

  Transport.prototype.disconnect = function () {
    this._manualClose = true;
    this._clearRetry();
    if (this._ws) {
      var old = this._ws;
      old.__superseded = true;
      old.onopen = null;
      old.onmessage = null;
      old.onerror = null;
      old.onclose = null;
      try { old.close(); } catch (e) {}
      this._ws = null;
    }
    this._setStatus(STATUS.OFFLINE);
    this._emit('log', '已断开');
  };

  Transport.prototype.isOnline = function () { return this._status === STATUS.ONLINE; };
  Transport.prototype.hasEverConnected = function () { return this._everOnline; };

  /* 换地址重连: 重置历史状态, 不触发离线发现 */
  Transport.prototype.resetForNewHost = function () {
    this._everOnline = false;
    this._retry = 0;
  };

  Transport.prototype.send = function (frame) {
    if (!this.isOnline() || !this._ws) {
      this._emit('log', '发送失败: 未连接');
      return false;
    }
    try { this._ws.send(frame); return true; }
    catch (e) { this._emit('log', '发送异常: ' + e.message); return false; }
  };

  Transport.STATUS = STATUS;
  global.Transport = Transport;
  if (typeof module !== 'undefined' && module.exports) module.exports = Transport;
})(typeof window !== 'undefined' ? window : globalThis);