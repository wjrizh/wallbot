/* log_page.cpp - 内置网页: 控制页 (GET /) + 日志页 (GET /logs) + 状态 (GET /status)
 *
 * 用途: 不依赖 App。子机开热点后, 手机/平板/电脑连上热点,
 *       浏览器打开 http://192.168.4.1/ 即可同屏看到实时画面 + 纠偏数据 + 日志。
 */
#include <Arduino.h>
#include <WiFi.h>
#include "esp_http_server.h"
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "cam_module.h"
#include "log_page.h"
#include "stream_server.h"

/* ==================== 控制页 / ==================== */
static const char *PAGE_CTRL =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>WallBot Camera</title>"
    "<style>"
    "*{box-sizing:border-box}"
    "body{margin:0;background:#0d0d0f;color:#e8e8ea;font:14px/1.5 -apple-system,system-ui,sans-serif}"
    "header{padding:10px 14px;background:#1a1a1d;border-bottom:1px solid #2c2c30;display:flex;"
    "align-items:center;gap:12px;position:sticky;top:0;z-index:10}"
    "header b{font-size:15px}"
    "#st{color:#8e8e93;font-size:12px;margin-left:auto;text-align:right}"
    ".wrap{display:flex;flex-wrap:wrap}"
    ".pane{flex:1 1 480px;min-width:320px;padding:10px}"
    ".pane h3{margin:0 0 8px;font-size:12px;color:#8e8e93;font-weight:600;"
    "letter-spacing:.5px;text-transform:uppercase}"
    "#pic{width:100%;background:#000;border-radius:8px;display:block;min-height:200px}"
    "table{width:100%;border-collapse:collapse;font:13px ui-monospace,Menlo,monospace}"
    "td{padding:5px 6px;border-bottom:1px solid #232326}"
    "td.k{color:#8e8e93;width:38%}"
    "td.v{text-align:right}"
    ".bad{color:#ff6b6b}.good{color:#4ade80}"
    "#log{margin:0;padding:10px;background:#000;border-radius:8px;color:#9ca3af;"
    "font:12px/1.45 ui-monospace,Menlo,monospace;white-space:pre-wrap;word-break:break-all;"
    "height:260px;overflow:auto}"
    "button{background:#2c2c30;color:#e8e8ea;border:1px solid #3a3a40;border-radius:6px;"
    "padding:6px 12px;font-size:13px;cursor:pointer;margin-right:6px}"
    "button:active{background:#3a3a40}button:disabled{opacity:.45;cursor:wait}"
    "</style></head><body>"
    "<header><b>摄像头子机</b>"
    "<span id='st'>连接中…</span></header>"
    "<div class='wrap'>"
    "<div class='pane'><h3>实时画面 :81/stream</h3>"
    "<img id='pic' alt='stream'>"
    "</div>"
    "<div class='pane'><h3>纠偏 (双码 tag36h11)</h3>"
    "<table id='t'>"
    "<tr><td class='k'>状态</td><td class='v' id='f_st'>--</td></tr>"
    "<tr><td class='k'>周向 phi</td><td class='v' id='f_phi'>--</td></tr>"
    "<tr><td class='k'>航向 psi</td><td class='v' id='f_psi'>--</td></tr>"
    "<tr><td class='k'>轴向 mm</td><td class='v' id='f_ax'>--</td></tr>"
    "<tr><td class='k'>弧长 mm</td><td class='v' id='f_arc'>--</td></tr>"
    "<tr><td class='k'>双码距 px</td><td class='v' id='f_id'>--</td></tr>"
    "<tr><td class='k'>耗时</td><td class='v' id='f_dt'>--</td></tr>"
    "</table>"
    "<div style='margin-top:10px'>"
    "<button id='btnOnce' onclick=\"measureOnce()\">测一次</button>"
    "<button onclick=\"refCmd('')\">记参考位</button>"
    "<button onclick=\"refCmd('clear=1')\">清除参考位</button>"
    "</div>"
    "<h3 style='margin-top:14px'>设备日志 /log</h3>"
    "<pre id='log'>…</pre>"
    "</div></div>"
    "<script>"
    "var BASE=location.protocol+'//'+location.hostname,bSt=0,bAg=0,bLg=0;"
    "function st(){if(bSt)return;bSt=1;fetch(BASE+'/status',{cache:'no-store'})"
    ".then(function(r){return r.text()})"
    ".then(function(t){bSt=0;document.getElementById('st').textContent=t})"
    "['catch'](function(e){bSt=0;document.getElementById('st').textContent='状态失败: '+e});}"
    "function num(v,d){return (v===undefined||v===null)?'--':Number(v).toFixed(d);}"
    "function applyAlign(d){var s=document.getElementById('f_st');"
    "if(!d||!d.ok){s.textContent='失败: '+((d&&d.reason)||'?');s.className='v bad';"
    "document.getElementById('f_phi').textContent='--';"
    "document.getElementById('f_psi').textContent='--';"
    "document.getElementById('f_ax').textContent='--';"
    "document.getElementById('f_arc').textContent='--';"
    "document.getElementById('f_id').textContent=(d&&d.edge_px)?num(d.edge_px,1)+' px':'--';"
    "document.getElementById('f_dt').textContent=(d&&d.dt_ms!=null)?num(d.dt_ms,0)+' ms':'--';return;}"
    "s.textContent='正常';s.className='v good';"
    "document.getElementById('f_phi').textContent=num(d.phi_deg,2)+' °';"
    "document.getElementById('f_psi').textContent=num(d.psi_deg,2)+' °';"
    "document.getElementById('f_ax').textContent=num(d.axial_mm,1)+' mm';"
    "document.getElementById('f_arc').textContent=num(d.arc_mm,1)+' mm';"
    "document.getElementById('f_id').textContent=num(d.edge_px,1)+' px';"
    "document.getElementById('f_dt').textContent=num(d.dt_ms,0)+' ms';}"
    "function ag(){if(bAg)return;bAg=1;fetch(BASE+'/align/get',{cache:'no-store'})"
    ".then(function(r){return r.json()})"
    ".then(function(d){bAg=0;applyAlign(d);})"
    "['catch'](function(e){bAg=0;var s=document.getElementById('f_st');"
    "s.textContent='不可达';s.className='v bad';});}"
    "function measureOnce(){if(bAg)return;bAg=1;"
    "var btn=document.getElementById('btnOnce');if(btn)btn.disabled=true;"
    "var s=document.getElementById('f_st');s.textContent='测量中…';s.className='v';"
    "fetch(BASE+'/align/once',{cache:'no-store'})"
    ".then(function(r){return r.json()})"
    ".then(function(d){bAg=0;if(btn)btn.disabled=false;applyAlign(d);})"
    "['catch'](function(e){bAg=0;if(btn)btn.disabled=false;"
    "s.textContent='请求失败';s.className='v bad';});}"
    "function lg(){if(bLg)return;bLg=1;fetch(BASE+'/log',{cache:'no-store'})"
    ".then(function(r){return r.text()})"
    ".then(function(t){bLg=0;var d=document.getElementById('log');"
    "var b=d.scrollHeight-d.scrollTop-d.clientHeight<24;"
    "d.textContent=t||'(空)';if(b)d.scrollTop=d.scrollHeight;})"
    "['catch'](function(e){bLg=0;document.getElementById('log').textContent='日志失败: '+e});}"
    "function refCmd(q){var clearing=q&&q.indexOf('clear=1')>=0;"
    "var s=document.getElementById('f_st');"
    "if(!clearing){s.textContent='记参考中…';s.className='v';}"
    "fetch(BASE+'/align/ref'+(q?'?'+q:''),{cache:'no-store'})"
    ".then(function(r){return r.json()})"
    ".then(function(d){alert(d.ok?'成功':'失败: '+(d.reason||'?'));"
    "if(clearing&&d.ok){s.textContent='参考已清除';s.className='v';"
    "document.getElementById('f_phi').textContent='--';"
    "document.getElementById('f_psi').textContent='--';"
    "document.getElementById('f_ax').textContent='--';"
    "document.getElementById('f_arc').textContent='--';}"
    "else{ag();}})"
    "['catch'](function(e){alert('请求失败: '+e)});}"
    "setTimeout(function(){document.getElementById('pic').src=BASE+':81/stream';},300);"
    "st();lg();"
    "document.getElementById('f_st').textContent='待命 (点测一次)';"
    "setInterval(st,2000);setInterval(lg,2000);"
    "</script></body></html>";

static esp_err_t ctrl_page_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, PAGE_CTRL, HTTPD_RESP_USE_STRLEN);
}

/* ==================== 日志页 /logs ==================== */
static const char *PAGE_HEAD =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>WallBot Camera Log</title>"
    "<style>body{margin:0;background:#111;color:#ddd;font:13px/1.5 ui-monospace,Menlo,monospace}"
    "#h{padding:10px 14px;background:#1c1c1e;border-bottom:1px solid #333;position:sticky;top:0}"
    "#s{color:#8e8e93;font-size:12px;margin-top:4px}"
    "#l{padding:10px 14px;white-space:pre-wrap;word-break:break-all}"
    "</style></head><body>"
    "<div id='h'><b>摄像头子机日志</b><div id='s'>加载中…</div></div>"
    "<div id='l'>…</div>"
    "<script>"
    "function st(){fetch('/status',{cache:'no-store'}).then(function(r){return r.text()})"
    ".then(function(t){document.getElementById('s').textContent=t;})['catch'](function(e){document.getElementById('s').textContent='状态获取失败: '+e;});}"
    "function lg(){fetch('/log',{cache:'no-store'}).then(function(r){return r.text()})"
    ".then(function(t){var d=document.getElementById('l');d.textContent=t||'(空)';})['catch'](function(e){"
    "document.getElementById('l').textContent='日志获取失败: '+e;});}"
    "st();lg();setInterval(st,2000);setInterval(lg,1000);"
    "</script></body></html>";

static esp_err_t logs_page_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, PAGE_HEAD, HTTPD_RESP_USE_STRLEN);
}

/* /status: 板子当前状态, 一眼看出是 WiFi 问题还是摄像头问题 */
static esp_err_t status_handler(httpd_req_t *req) {
    char buf[320];
    int n = snprintf(buf, sizeof(buf),
        "mode=%s ip=%s rssi=%d ch=%d | cam ok=%d err=0x%x pid=0x%x detect=%d | heap=%u psram=%u | up=%us",
#if CAM_AP_MODE
        "AP", WiFi.softAPIP().toString().c_str(), 0, (int)WiFi.channel(),
#else
        "STA", WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(), (int)WiFi.channel(),
#endif
        Cam_IsOk() ? 1 : 0, Cam_Err(), Cam_SensorPid(),
        Cam_InDetectMode() ? 1 : 0,
        (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getPsramSize(),
        (unsigned)(millis() / 1000));
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, n);
}

void LogPage_Register(httpd_handle_t httpd) {
    if (!httpd) return;
    httpd_uri_t ctrl_uri = {.uri = "/", .method = HTTP_GET,
                            .handler = ctrl_page_handler, .user_ctx = NULL};
    httpd_uri_t logs_uri = {.uri = "/logs", .method = HTTP_GET,
                            .handler = logs_page_handler, .user_ctx = NULL};
    httpd_uri_t status_uri = {.uri = "/status", .method = HTTP_GET,
                              .handler = status_handler, .user_ctx = NULL};
    httpd_register_uri_handler(httpd, &ctrl_uri);
    httpd_register_uri_handler(httpd, &logs_uri);
    httpd_register_uri_handler(httpd, &status_uri);
    Stream_Log("pages ready: / /logs /status");
}
