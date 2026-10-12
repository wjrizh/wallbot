/*
 * modules/ae.js - 声发射监测 (与子机同板的 Linux 服务, 端口 5000)
 * 职责: 采集参数设置 / 启停采集 / 状态与日志轮询 / 数据文件管理(下载/删除)
 * 说明: 服务地址固定为母机 192.168.4.1, 端口固定 5000
 */
(function (global) {
  "use strict";

  var AE_PORT = 5000;
  function AeModule(app) {
    this.app = app;
    this._els = {};
  }

  AeModule.prototype.mount = function () {
    var self = this;
    var e = this._els;
    e.dot = UI["$"]("#aeStatusDot");
    e.text = UI["$"]("#aeStatusText");
    e.log = UI["$"]("#aeLog");
    e.files = UI["$"]("#aeFiles");
    e.sampleRate = UI["$"]("#aeSampleRate");
    e.threshold = UI["$"]("#aeThreshold");
    e.hp = UI["$"]("#aeHp");
    e.lp = UI["$"]("#aeLp");
    e.ddt = UI["$"]("#aeDdt");
    if (!e.dot) return;

    UI["$"]("#btnAeRefresh").addEventListener("click", function () { self.refresh(); });
    UI["$"]("#btnAeFiles").addEventListener("click", function () { self.loadFiles(); });
    UI["$"]("#btnAeStart").addEventListener("click", function () { self.start(); });
    UI["$"]("#btnAeStop").addEventListener("click", function () { self.stop(); });
  };

  AeModule.prototype._host = function () {
    return "192.168.4.1";
  };

  AeModule.prototype._url = function (path) {
    return "http://" + this._host() + ":" + AE_PORT + path;
  };

  AeModule.prototype._log = function (msg, isError) {
    var box = this._els.log;
    if (!box) return;
    var time = new Date().toLocaleTimeString("zh-CN", { hour12: false });
    var line = UI.el("div", { class: isError ? "err" : "" }, [
      UI.el("span", { class: "t", text: "[" + time + "] " }),
      document.createTextNode(msg)
    ]);
    box.appendChild(line);
    box.scrollTop = box.scrollHeight;
    while (box.children.length > 200) box.removeChild(box.firstChild);
  };

  AeModule.prototype._http = function () {
    if (global.CapacitorHttp) return global.CapacitorHttp;
    if (global.Capacitor && global.Capacitor.Plugins) return global.Capacitor.Plugins.CapacitorHttp || null;
    return null;
  };

  AeModule.prototype._get = function (path) {
    var url = this._url(path);
    var http = this._http();
    if (http) {
      return http.get({ url: url }).then(function (res) {
        return typeof res.data === "string" ? JSON.parse(res.data) : res.data;
      });
    }
    return fetch(url).then(function (r) {
      if (!r.ok) throw new Error("HTTP " + r.status);
      return r.json();
    });
  };

  AeModule.prototype._post = function (path, body) {
    var url = this._url(path);
    var http = this._http();
    if (http) {
      return http.post({ url: url, headers: { "Content-Type": "application/json" }, data: body || {} }).then(function (res) {
        return typeof res.data === "string" ? JSON.parse(res.data) : res.data;
      });
    }
    return fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {})
    }).then(function (r) { return r.json(); });
  };

  AeModule.prototype.refresh = function () {
    var self = this;
    return this._get("/api/info").then(function (info) {
      self._setStatus(info.status);
      if (typeof info.logs === "string") self._renderLogs(info.logs);
      return self.loadFiles();
    }).catch(function () {
      self._setStatus("unreachable");
      self._log("无法连接声发射服务 " + self._url(""), true);
    });
  };

  AeModule.prototype._setStatus = function (status) {
    var e = this._els;
    var map = { running: { cls: "on", text: "采集中" }, stopped: { cls: "off", text: "已停止" }, unreachable: { cls: "err", text: "不可达" } };
    var cfg = map[status] || map.unreachable;
    if (e.dot) e.dot.className = "ae-dot " + cfg.cls;
    if (e.text) e.text.textContent = cfg.text;
  };

  AeModule.prototype._renderLogs = function (text) {
    var box = this._els.log;
    if (!box) return;
    var lines = text.split("\n");
    box.innerHTML = "";
    for (var i = 0; i < lines.length; i++) {
      if (!lines[i]) continue;
      box.appendChild(UI.el("div", { text: lines[i] }));
    }
    box.scrollTop = box.scrollHeight;
  };

  AeModule.prototype.loadFiles = function () {
    var self = this;
    if (!this._els.files) return Promise.resolve();
    return this._get("/api/files").then(function (list) {
      self._renderFiles(list || []);
    }).catch(function () {});
  };

  AeModule.prototype._renderFiles = function (list) {
    var box = this._els.files;
    if (!box) return;
    box.innerHTML = "";
    if (!list.length) {
      box.appendChild(UI.el("div", { class: "ae-empty", text: "暂无数据文件" }));
      return;
    }
    var self = this;
    list.forEach(function (f) {
      var row = UI.el("div", { class: "ae-file" }, [
        UI.el("div", { class: "ae-file-name", text: f.name }),
        UI.el("div", { class: "ae-file-size", text: f.size })
      ]);
      var dl = UI.el("a", { class: "ae-file-btn", text: "下载" });
      dl.href = self._url("/download/" + encodeURIComponent(f.name));
      dl.setAttribute("download", f.name);
      row.appendChild(dl);
      var del = UI.el("button", { class: "ae-file-btn ae-file-del", text: "删除" });
      del.addEventListener("click", function () { self.remove(f.name); });
      row.appendChild(del);
      box.appendChild(row);
    });
  };

  AeModule.prototype.remove = function (name) {
    var self = this;
    if (!global.confirm("确定删除 " + name + " ?")) return;
    this._post("/api/delete/" + encodeURIComponent(name), {}).then(function (res) {
      self._log(res && res.message ? res.message : "已删除");
      self.loadFiles();
    }).catch(function () { self._log("删除失败", true); });
  };

  AeModule.prototype.start = function () {
    var self = this;
    var e = this._els;
    var cfg = {
      sample_rate: parseInt(e.sampleRate.value, 10),
      threshold_db: parseFloat(e.threshold.value),
      filter_hp_khz: parseFloat(e.hp.value),
      filter_lp_khz: parseFloat(e.lp.value),
      ddt_us: parseInt(e.ddt.value, 10)
    };
    this._log("启动采集: " + JSON.stringify(cfg));
    this._post("/api/start", cfg).then(function (res) {
      self._log(res && res.message ? res.message : "已启动");
      self.refresh();
    }).catch(function () { self._log("启动失败", true); });
  };

  AeModule.prototype.stop = function () {
    var self = this;
    this._post("/api/stop", {}).then(function (res) {
      self._log(res && res.message ? res.message : "已停止");
      self.refresh();
    }).catch(function () { self._log("停止失败", true); });
  };

  global.AeModule = AeModule;
})(typeof window !== "undefined" ? window : globalThis);
