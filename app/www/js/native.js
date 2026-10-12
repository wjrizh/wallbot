/*
 * native.js - 原生能力适配 (Capacitor)
 * 职责: Android 返回键、防止双击缩放与橡皮筋滚动
 * 说明: 无 Capacitor 时静默降级为纯浏览器行为
 */
(function (global) {
  'use strict';

  function isCapacitor() {
    return !!(global.Capacitor && global.Capacitor.isNativePlatform && global.Capacitor.isNativePlatform());
  }

  function bindBackButton() {
    if (!isCapacitor()) return;
    var App = global.Capacitor.Plugins && global.Capacitor.Plugins.App;
    if (!App || !App.addListener) return;
    App.addListener('backButton', function (event) {
      if (event && event.canGoBack) global.history.back();
      else if (App.exitApp) App.exitApp();
    });
  }

  function blockGestures() {
    document.addEventListener('gesturestart', function (e) { e.preventDefault(); });
    document.addEventListener('dblclick', function (e) { e.preventDefault(); }, { passive: false });
    document.addEventListener('touchmove', function (e) {
      if (e.touches.length > 1) e.preventDefault();
    }, { passive: false });
  }

  global.NativeAdapter = {
    init: function () {
      bindBackButton();
      blockGestures();
    }
  };
})(typeof window !== 'undefined' ? window : globalThis);
