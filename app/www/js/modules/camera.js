/*
 * modules/camera.js - 双路摄像头 MJPEG 回传
 * 说明: 通过 <img src="http://<ip>:81/stream"> 直接拉取 MJPEG 流.
 *   母机 = #cameraCard / #cameraBadge  (OV3660, 10.32.7.214)
 *   子机 = #cameraCard2 / #cameraBadge2 (GC2145, 10.32.7.215)
 *   wallbot.camera.connect()    // 两路同时连接
 *   wallbot.camera.disconnect() // 两路同时断开
 */
(function (global) {
  "use strict";

  var STREAM_PORT = 81;
  var RETRY_MS = 3000;
  var RETRY_LIMIT = 3;
  var STORAGE_KEY = { host: "wallbot.camHost", child: "wallbot.camChild" };

  function loadSavedIp(key) {
    try { return global.localStorage && global.localStorage.getItem(STORAGE_KEY[key]) || ""; }
    catch (e) { return ""; }
  }

  function saveIp(key, ip) {
    try {
      if (!ip) global.localStorage.removeItem(STORAGE_KEY[key]);
      else global.localStorage.setItem(STORAGE_KEY[key], ip);
    } catch (e) {}
  }

  // 每路摄像头一个通道: 独立 IP / 容器 / badge / <img> / 重连
  // ip 由 App 自动发现写入 (setIp), 为空表示尚未发现
  var CHANNELS = [
    { key: "host",  ip: "", cardSel: "#cameraCard",  badgeSel: "#cameraBadge"  },
    { key: "child", ip: "", cardSel: "#cameraCard2", badgeSel: "#cameraBadge2" }
  ];

  function Channel(cfg, app) {
    this.app = app;
    this.key = cfg.key;
    this.ip = loadSavedIp(cfg.key) || cfg.ip;
    this.cardSel = cfg.cardSel;
    this.badgeSel = cfg.badgeSel;
    this.card = null;
    this.badge = null;
    this.img = null;
    this.placeholder = null;
    this.retryTimer = null;
    this.retryCount = 0;
  }

  Channel.prototype.streamUrl = function () {
    if (!this.ip) return "";
    var ip = this.ip.replace(/^https?:\/\//, "").replace(/\/.*$/, "");
    if (ip.indexOf(":") >= 0) return "http://" + ip + "/stream";
    return "http://" + ip + ":" + STREAM_PORT + "/stream";
  };

  Channel.prototype._setBadge = function (text, state) {
    if (!this.badge) return;
    this.badge.textContent = text;
    this.badge.className = "pill " + state;
  };

  Channel.prototype._clearRetry = function () {
    if (this.retryTimer) { clearTimeout(this.retryTimer); this.retryTimer = null; }
  };

  Channel.prototype._removeMedia = function () {
    if (this.img) {
      this.img.onload = null;
      this.img.onerror = null;
      this.img.src = "";
      if (this.img.parentNode) this.img.parentNode.removeChild(this.img);
      this.img = null;
    }
  };

  Channel.prototype._showPlaceholder = function () {
    if (this.placeholder && !this.placeholder.parentNode && this.card) {
      this.card.appendChild(this.placeholder);
    }
  };

  Channel.prototype._hidePlaceholder = function () {
    if (this.placeholder && this.placeholder.parentNode) {
      this.placeholder.parentNode.removeChild(this.placeholder);
    }
  };

  Channel.prototype._scheduleRetry = function () {
    var self = this;
    this._clearRetry();
    if (this.retryCount >= RETRY_LIMIT) {
      /* 缓存地址失效: 通知 App 触发重新发现 */
      if (this.app && typeof this.app._onCameraGiveUp === "function") this.app._onCameraGiveUp(this.key);
      return;
    }
    this.retryCount++;
    this.retryTimer = setTimeout(function () { self.connect(); }, RETRY_MS);
  };

  Channel.prototype.mount = function () {
    this.card = UI.$(this.cardSel);
    this.badge = UI.$(this.badgeSel);
    var byId = document.getElementById(this.cardSel.slice(1));
    var allCards = document.querySelectorAll('.camera-card').length;
    var dockEl = document.getElementById('cameraDock');
    UI.log("[cam] mount " + this.key + " card=" + (this.card ? "ok" : "NULL") +
           " byId=" + (byId ? "ok" : "NULL") +
           " 卡片数=" + allCards +
           " dock=" + (dockEl ? "ok" : "NULL") +
           " rs=" + document.readyState);
    if (!this.card) return;
    this.placeholder = UI.el("div", { class: "camera-placeholder" }, [
      UI.el("div", { class: "camera-icon" }),
      UI.el("div", { class: "camera-hint", text: "摄像头未接入" })
    ]);
    this.card.appendChild(this.placeholder);
    this._setBadge("未接入", "off");
  };

  Channel.prototype.connect = function () {
    if (!this.card) return;
    this._clearRetry();
    var self = this;
    var url = this.streamUrl();
    if (!url) {
      this._setBadge("等待发现", "off");
      return;
    }
    this._removeMedia();
    this._hidePlaceholder();
    this._setBadge("连接中…", "off");

    var img = UI.el("img", { src: url, alt: "摄像头回传" });
    UI.log("[cam] connect " + this.key + " url=" + url);
    img.onload = function () {
      self.retryCount = 0;
      self._setBadge("已接入", "on");
      UI.log("[cam] OK " + self.key);
    };
    img.onerror = function () {
      self._setBadge("连接失败", "off");
      UI.log("[cam] FAIL " + self.key + " " + url);
      self._scheduleRetry();
    };
    this.card.appendChild(img);
    this.img = img;

    this.app && this.app.transport && this.app.transport._emit &&
      this.app.transport._emit("log", "摄像头[" + this.key + "] 拉流 " + url);
  };

  Channel.prototype.disconnect = function () {
    this._clearRetry();
    this._removeMedia();
    this._showPlaceholder();
    this._setBadge("未接入", "off");
  };

  function CameraModule(app) {
    this.app = app;
    this.channels = CHANNELS.map(function (cfg) { return new Channel(cfg, app); });
  }

  CameraModule.prototype.mount = function () {
    // mount 无参: 各通道自解析 card/badge 选择器 (母机/子机在常驻层)
    this.channels.forEach(function (ch) { ch.mount(); });
  };

  CameraModule.prototype.connect = function () {
    this.channels.forEach(function (ch) { if (!ch.img) ch.connect(); });
  };

  /* 自动发现结果写入; 已有地址变化时断开并重连 */
  CameraModule.prototype.setIp = function (key, ip) {
    var ch = this.channels.filter(function (c) { return c.key === key; })[0];
    if (!ch || !ip || ch.ip === ip) return;
    ch.ip = ip;
    ch.retryCount = 0;
    saveIp(key, ip);
    if (ch.img) { ch.disconnect(); ch.connect(); }
  };

  CameraModule.prototype.clearIp = function (key) {
    var ch = this.channels.filter(function (c) { return c.key === key; })[0];
    if (!ch) return;
    ch.ip = "";
    ch.retryCount = 0;
    saveIp(key, "");
  };

  CameraModule.prototype.disconnect = function () {
    this.channels.forEach(function (ch) { ch.disconnect(); });
  };

  CameraModule.prototype.getIp = function (key) {
    var ch = this.channels.filter(function (c) { return c.key === key; })[0];
    return ch ? ch.ip : "";
  };

  CameraModule.prototype.onState = function () {};

  global.CameraModule = CameraModule;
})(typeof window !== "undefined" ? window : globalThis);