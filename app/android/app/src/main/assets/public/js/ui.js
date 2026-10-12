/*
 * ui.js - 通用 UI 工具 (toast / 日志 / 元素构建)
 */
(function (global) {
  'use strict';

  function $(sel, root) { return (root || document).querySelector(sel); }

  function el(tag, attrs, children) {
    var node = document.createElement(tag);
    if (attrs) {
      Object.keys(attrs).forEach(function (k) {
        if (k === 'class') node.className = attrs[k];
        else if (k === 'text') node.textContent = attrs[k];
        else if (k === 'html') node.innerHTML = attrs[k];
        else if (k.slice(0, 2) === 'on') node.addEventListener(k.slice(2), attrs[k]);
        else if (attrs[k] != null) node.setAttribute(k, attrs[k]);
      });
    }
    if (children) {
      (Array.isArray(children) ? children : [children]).forEach(function (c) {
        if (c == null) return;
        node.appendChild(typeof c === 'string' ? document.createTextNode(c) : c);
      });
    }
    return node;
  }

  var toastTimer = null;
  function toast(message, ms) {
    var node = $('#toast');
    if (!node) return;
    node.textContent = message;
    node.classList.add('show');
    if (toastTimer) clearTimeout(toastTimer);
    toastTimer = setTimeout(function () { node.classList.remove('show'); }, ms || 1800);
  }

  /* 日志提加到多个容器 (仅子机界面也有日志) */
  function log(message, isError) {
    var time = new Date().toLocaleTimeString('zh-CN', { hour12: false });
    var boxes = [$('#log'), $('#childLog')];
    boxes.forEach(function (box) {
      if (!box) return;
      var line = el('div', { class: isError ? 'err' : '' }, [
        el('span', { class: 't', text: '[' + time + '] ' }),
        document.createTextNode(message)
      ]);
      box.appendChild(line);
      box.scrollTop = box.scrollHeight;
      while (box.children.length > 200) box.removeChild(box.firstChild);
    });
  }

  function clearLog() {
    [$('#log'), $('#childLog')].forEach(function (box) {
      if (box) box.innerHTML = '';
    });
  }
  function fmt(value, digits) {
    if (value == null || isNaN(value)) return '—';
    return Number(value).toFixed(digits == null ? 1 : digits);
  }

  function radToDeg(rad) { return rad * 180 / Math.PI; }

  global.UI = { $: $, el: el, toast: toast, log: log, clearLog: clearLog, fmt: fmt, radToDeg: radToDeg };
})(typeof window !== 'undefined' ? window : globalThis);
