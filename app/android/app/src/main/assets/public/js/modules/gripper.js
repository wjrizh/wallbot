/*
 * modules/gripper.js - 夹持器控制浮窗
 * 下发: 0x1B 夹持器角度; 回传: 状态帧 payload[27-30]
 */
(function (global) {
  "use strict";

  function GripperModule(app) {
    this.app = app;
    this._mask = null;
    this._curEl = null;
    this._input = null;
  }

  GripperModule.prototype.mount = function () {
    var self = this;
    var entry = document.getElementById("btnGripper");
    if (entry) entry.addEventListener("click", function () { self.open(); });
    this._build();
  };

  GripperModule.prototype._build = function () {
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
    title.textContent = "夹持器";
    var closeBtn = document.createElement("button");
    closeBtn.className = "debug-close";
    closeBtn.textContent = "\u00d7";
    closeBtn.addEventListener("click", function () { self.close(); });
    head.appendChild(title); head.appendChild(closeBtn);

    var body = document.createElement("div");
    body.className = "debug-body";

    var item = document.createElement("div");
    item.className = "debug-item";
    var itemHead = document.createElement("div");
    itemHead.className = "debug-item-head";
    var nameEl = document.createElement("span");
    nameEl.className = "debug-name";
    nameEl.textContent = "夹持器角度";

    var linkEl = document.createElement("span");
    linkEl.className = "grip-link";
    linkEl.textContent = "离线";
    nameEl.appendChild(linkEl);
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
    input.placeholder = "0-180";
    var sendBtn = document.createElement("button");
    sendBtn.className = "debug-send";
    sendBtn.textContent = "发送";
    sendBtn.addEventListener("click", function () { self._send(); });
    input.addEventListener("keydown", function (e) { if (e.key === "Enter") self._send(); });
    row.appendChild(input); row.appendChild(sendBtn);

    item.appendChild(itemHead); item.appendChild(row);
    body.appendChild(item);

    card.appendChild(head); card.appendChild(body);
    mask.addEventListener("click", function (e) { if (e.target === mask) self.close(); });
    document.body.appendChild(mask);

    this._mask = mask;
    this._curEl = curEl;
    this._input = input;
    this._linkEl = linkEl;
  };

  GripperModule.prototype.open = function () {
    if (this._mask) this._mask.classList.add("show");
  };

  GripperModule.prototype.close = function () {
    if (this._mask) this._mask.classList.remove("show");
  };

  GripperModule.prototype._send = function () {
    var v = parseFloat(this._input.value);
    if (isNaN(v)) { UI.toast("请输入角度"); return; }
    v = Math.max(0, Math.min(180, v));
    var sent = this.app.transport.send(Protocol.gripper(v));
    UI.log("夹持器 → " + v + "\u00b0" + (sent ? "" : " (未发送)"), !sent);
    if (sent) UI.toast("夹持器 → " + v + "\u00b0");
  };

  GripperModule.prototype.onState = function (state) {
    if (this._curEl && state.gripper != null) {
      this._curEl.textContent = Math.round(state.gripper);
    }
    if (this._linkEl) {
      var on = !!state.gripperOnline;
      this._linkEl.textContent = on ? "已连接" : "离线";
      this._linkEl.className = "grip-link " + (on ? "on" : "off");
    }
  };

  global.GripperModule = GripperModule;
})(typeof window !== "undefined" ? window : globalThis);