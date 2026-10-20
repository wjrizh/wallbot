/* ===== 文件 1/4: camchild.ino ===== */
/* 摄像头子机 - 主入口
 * ESP32-S3-CAM (Hiwonder), 俯视安装于母机夹爪处。
 *
 * 职责 (实现按模块分文件, 参数集中在 config.h):
 *   cam_module     摄像头初始化 + 检测模式切换
 *   discovery      UDP 自动发现 (:48899, 角色 camchild)
 *   stream_server  MJPEG 流 :81/stream + :80 /jpg /fps / 诊断
 *   tag_detector   板端 AprilTag 36h11 检测
 *   align_measure  视觉纠偏测量 (参考位 + 偏差解算)
 *   align_server   纠偏 API: /align/ref /align/get /align/status
 *
 * 纠偏闭环 (执行文档 6.3): 母机松爪前调 /align/ref 记参考位;
 * 子机回程到预对接点停稳后轮询 /align/get, 按 phi 发 MV、按 psi 发 SPIN,
 * 三角全部 <=4° 判据达标再夹取。
 */
#include <WiFi.h>
#include <string.h>
#include "config.h"
#include "cam_module.h"
#include "discovery.h"
#include "stream_server.h"
#include "align_measure.h"
#include "align_server.h"
#include "stream_server.h"
#include "log_page.h"

#if CAM_AP_MODE
/* AP 模式: 子机自己开热点, 地址固定, 不依赖外部路由器与 DHCP */
static void wifi_connect() {
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    IPAddress ip, gw, mask;
    ip.fromString(AP_IP);
    gw.fromString(AP_GW);
    mask.fromString(AP_MASK);
    bool ok = WiFi.softAPConfig(ip, gw, mask);
    /* 显式指定 OPEN 认证, 不传密码串 (见 config.h 中关于 wpa_ap_get_wpa_ie 崩溃的说明) */
    bool up;
    if (strlen(AP_PASS) >= 8) {
        up = WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 4);
    } else {
        up = WiFi.softAP(AP_SSID, NULL, AP_CHANNEL, 0, 1);
    }
    delay(300);
    Stream_Log("AP %s cfg=%d up=%d ip=%s", AP_SSID, ok ? 1 : 0, up ? 1 : 0,
               WiFi.softAPIP().toString().c_str());
    if (!up) Stream_Log("ERR softAP failed");
}
#else
static void wifi_connect() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("Connecting to %s", WIFI_SSID);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Stream_Log("WiFi ok ip=%s", WiFi.localIP().toString().c_str());
    } else {
        Stream_Log("ERR WiFi timeout, retry in loop");
    }
}
#endif

void setup() {
    Serial.begin(115200);
    Serial.println();

    Stream_Log("=== boot ===");
    Stream_Log("chip=%s cores=%d flash=%uMB psram=%uKB",
               ESP.getChipModel(), (int)ESP.getChipCores(),
               (unsigned)(ESP.getFlashChipSize() / (1024 * 1024)),
               (unsigned)(ESP.getPsramSize() / 1024));
    Stream_Log("ssid=%s build=%s %s", WIFI_SSID, __DATE__, __TIME__);

    AlignMeasure_Init();          /* 先恢复参考位 (NVS) */

    Cam_Init();
    if (!Cam_IsOk()) {
        Stream_Log("ERR Camera init failed err=0x%x", Cam_Err());
    } else {
        Stream_Log("Cam ok pid=0x%x %ums jpeg=%d",
                   Cam_SensorPid(), (unsigned)Cam_InitMs(), Cam_JpegOk() ? 1 : 0);
    }

    wifi_connect();

    if (StreamServer_Start()) {
        Stream_Log("HTTP ready :80 /logs /status /view /log /align/*, :81 /stream");
    } else {
        Stream_Log("ERR HTTP server start failed");
    }
}

void loop() {
    static uint32_t last_check = 0;
    uint32_t now = millis();

#if !CAM_AP_MODE
    // STA 模式: WiFi 掉线重连
    if (now - last_check > 3000) {
        last_check = now;
        if (WiFi.status() != WL_CONNECTED) {
            Stream_Log("WiFi lost, reconnecting...");
            WiFi.disconnect();
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            Discovery_Reset();
        }
    }
#else
    (void)last_check;
#endif
    Discovery_Poll();
    delay(20);
}

/* ===== 文件 2/4: config.h ===== */
/* config.h - 摄像头子机全部可调参数 (集中一处, 上机调参只改这里)
 *
 * 模块划分:
 *   摄像头子机.ino  主入口: 初始化/看门狗/模块编排
 *   cam_module      摄像头初始化 + 测检模式切换 + 诊断计数
 *   discovery       UDP 自动发现 (端口 48899)
 *   stream_server   MJPEG 流 (:81/stream) + :80 的 /jpg /fps / 诊断
 *   tag_detector    板端 AprilTag 36h11 检测 (灰度帧 -> 角点/ID)
 *   align_measure   视觉纠偏测量 (参考位 + 偏差解算, 移植 tag_utils.py)
 *   align_server    纠偏 HTTP API (/align/*), 母机/App 闭环调用的接口
 */
#ifndef WALLBOT_CAM_CONFIG_H
#define WALLBOT_CAM_CONFIG_H

#include "esp_camera.h"   /* FRAMESIZE_*/

/* ---- WiFi: 子机自开热点 (AP 模式) ----
 * 不再依赖外部热点与 DHCP: 地址固定, 平板/手机直连后打开 http://192.168.4.1/ 即可。
 * 若需回到 STA 模式 (与母机同一热点), 把 CAM_AP_MODE 设为 0。 */
#define CAM_AP_MODE   1
#define WIFI_SSID     "robot"          /* STA 模式的上级热点 (CAM_AP_MODE=0 时生效) */
#define WIFI_PASS     "12345678"
#define AP_SSID       "wallbot-cam"    /* 子机自己开的热点名 */
/* 开放热点 (无密码): ESP32 Arduino 3.3.11 的 AP+WPA2 在 wpa_ap_get_wpa_ie
 * 有已知崩溃 (LoadProhibited/LoadStoreAlignment), 客户端一探测就重启。
 * 调试场景无需加密, 置空即走 WIFI_AUTH_OPEN, 彻底绕开该缺陷。 */
#define AP_PASS       ""
#define AP_IP         "192.168.4.1"
#define AP_GW         "192.168.4.1"
#define AP_MASK       "255.255.255.0"
#define AP_CHANNEL    6

/* ---- UDP 自动发现 ---- */
#define DISCOVERY_PORT        48899
#define DISCOVERY_REQ         "WALLBOT_DISCOVER"
#define DEVICE_ROLE           "camchild"

/* ---- 摄像头 ---- */
/* 流媒体分辨率/画质 (MJPEG 硬编码)
 * 固定 VGA 不再运行时切换: 切 framesize 会重置 GC2145 寄存器与 GDMA 链,
 * 正在推流的帧被截断(丢 EOI), WebView 的 MJPEG 解码器会定格,
 * 且画面尺寸跳变本身就不满足"检测时画面不变"的要求。
 * 检测侧用 esp_jpeg_decode 的 1/2 缩放直接从 VGA 解出 320x240。 */
#define STREAM_FRAMESIZE      FRAMESIZE_VGA
#define STREAM_JPEG_QUALITY   12
#define STREAM_FB_COUNT       3      /* 3 缓冲: 推流与后台检测并发取帧时不至于 DMA 停摆 */
/* ---- 检测分辨率 (固定, 不再是运行时的另一种相机模式) ----
 * VGA 帧经 JPEG_IMAGE_SCALE_1_2 解码后正好是 320x240,
 * tag_detector 的像素参数(EDGE_PX_MIN/ADAPT_WIN)全部沿用不变。 */
#define ALIGN_DETECT_W          320
#define ALIGN_DETECT_H          240

/* ---- AprilTag / 视觉纠偏几何参数 (与 code/tag_utils.py 保持一致) ---- */
#define TAG_REAL_MM       50.0f  /* 标签黑框真实边长(mm), 打印后实测填入 */
#define R_TAG_MM          60.0f  /* 标签黑框中心平面到管道轴线的垂直距离(mm), 贴好后必须实测 */
#define PIPE_RADIUS_MM    60.0f  /* 管道内半径(mm), 仅用于把 phi 换算成沿管壁弧长 */

#define TAG_ID            0      /* 使用的标签 ID */

/* ---- 检测器参数 ---- */
#define DET_EDGE_PX_MIN   24     /* 标签黑框边长像素下限 (QVGA 下的预期范围) */
#define DET_EDGE_PX_MAX   200    /* 标签黑框边长像素上限 */
#define DET_MAX_HAMMING   1      /* 解码允许的最大汉明距离 */
#define DET_CONTRAST_MIN  25.0f  /* 黑白电平最小对比度 (抗全黑/全白误检) */
#define DET_ADAPT_WIN     45     /* 自适应阈值窗口(奇数, 像素)。
                                  * 须大于最大标签的黑框厚度(edge_max/8*1=25px):
                                  * 窗口大->厚黑框核心也判暗(或成闭合厚环带), 不断裂 */
#define DET_ADAPT_BIAS    25     /* 判黑偏置: gray < mean - bias 视为暗。
                                  * 取 ~2.5x 噪声sigma: 噪声误判率<1%, 背景不絮凝;
                                  * 黑框会变细环带(连续等值线,不断裂), 由填洞补实心 */

/* ---- 纠偏闭环参数 (执行文档 6.3 节: ANGLE_TOL=4°, 3 帧一致才采信) ---- */
#define ALIGN_FRAMES        3      /* 每次测量连拍帧数 (识别要连续一致) */
#define ALIGN_THETA_SPREAD  1.5f   /* 多帧 theta 一致性阈值 (deg) */
#define ALIGN_UV_SPREAD     3.0f   /* 多帧中心一致性阈值 (px) */
#define ALIGN_ANGLE_TOL     4.0f   /* 夹取判据: |phi|,|psi| <= 4° (夹爪 5° 留 1° 裕量) */
#define ALIGN_AXIAL_TOL_MM  20.0f  /* 轴向进/退偏差容差(mm), 夹爪开口容差相关 */

/* ---- NVS 存储命名空间 (参考位掉电保存) ---- */
#define ALIGN_NVS_NS        "wallbot-cam"

#endif /* WALLBOT_CAM_CONFIG_H */

/* ===== 文件 3/4: log_page.cpp ===== */
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
    "button:active{background:#3a3a40}"
    "</style></head><body>"
    "<header><b>摄像头子机</b>"
    "<span id='st'>连接中…</span></header>"
    "<div class='wrap'>"
    "<div class='pane'><h3>实时画面 :81/stream</h3>"
    "<img id='pic' src='http://192.168.4.1:81/stream' alt='stream'>"
    "</div>"
    "<div class='pane'><h3>纠偏数据 /align/get</h3>"
    "<table id='t'>"
    "<tr><td class='k'>状态</td><td class='v' id='f_st'>--</td></tr>"
    "<tr><td class='k'>周向 phi</td><td class='v' id='f_phi'>--</td></tr>"
    "<tr><td class='k'>航向 psi</td><td class='v' id='f_psi'>--</td></tr>"
    "<tr><td class='k'>轴向 mm</td><td class='v' id='f_ax'>--</td></tr>"
    "<tr><td class='k'>弧长 mm</td><td class='v' id='f_arc'>--</td></tr>"
    "<tr><td class='k'>Tag ID</td><td class='v' id='f_id'>--</td></tr>"
    "<tr><td class='k'>耗时</td><td class='v' id='f_dt'>--</td></tr>"
    "</table>"
    "<div style='margin-top:10px'>"
    "<button onclick=\"refCmd('')\">记参考位</button>"
    "<button onclick=\"refCmd('clear=1')\">清除参考位</button>"
    "</div>"
    "<h3 style='margin-top:14px'>设备日志 /log</h3>"
    "<pre id='log'>…</pre>"
    "</div></div>"
    "<script>"
    "var BASE='http://192.168.4.1';"
    "function st(){fetch(BASE+'/status',{cache:'no-store'})"
    ".then(function(r){return r.text()})"
    ".then(function(t){document.getElementById('st').textContent=t})"
    "['catch'](function(e){document.getElementById('st').textContent='状态失败: '+e});}"
    "function num(v,d){return (v===undefined||v===null)?'--':Number(v).toFixed(d);}"
    "function ag(){fetch(BASE+'/align/get',{cache:'no-store'})"
    ".then(function(r){return r.json()})"
    ".then(function(d){"
    "var s=document.getElementById('f_st');"
    "if(!d.ok){s.textContent='失败: '+(d.reason||'?');s.className='v bad';return;}"
    "s.textContent='正常';s.className='v good';"
    "document.getElementById('f_phi').textContent=num(d.phi_deg,2)+' °';"
    "document.getElementById('f_psi').textContent=num(d.psi_deg,2)+' °';"
    "document.getElementById('f_ax').textContent=num(d.axial_mm,1)+' mm';"
    "document.getElementById('f_arc').textContent=num(d.arc_mm,1)+' mm';"
    "document.getElementById('f_id').textContent=(d.id===undefined?'--':d.id);"
    "document.getElementById('f_dt').textContent=num(d.dt_ms,0)+' ms';"
    "})['catch'](function(e){"
    "var s=document.getElementById('f_st');"
    "s.textContent='不可达';s.className='v bad';});}"
    "function lg(){fetch(BASE+'/log',{cache:'no-store'})"
    ".then(function(r){return r.text()})"
    ".then(function(t){var d=document.getElementById('log');"
    "var b=d.scrollHeight-d.scrollTop-d.clientHeight<24;"
    "d.textContent=t||'(空)';if(b)d.scrollTop=d.scrollHeight;})"
    "['catch'](function(e){document.getElementById('log').textContent='日志失败: '+e});}"
    "function refCmd(q){fetch(BASE+'/align/ref'+(q?'?'+q:''),{cache:'no-store'})"
    ".then(function(r){return r.json()})"
    ".then(function(d){alert(d.ok?'成功':'失败: '+(d.reason||'?'));ag();})"
    "['catch'](function(e){alert('请求失败: '+e)});}"
    "st();ag();lg();"
    "setInterval(st,2000);setInterval(ag,1000);setInterval(lg,2000);"
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

/* ===== 文件 4/4: stream_server.cpp ===== */
/* stream_server.cpp - MJPEG 流服务 + 截图/帧率/诊断接口 */
#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"
#include "cam_module.h"
#include "align_server.h"
#include "align_view.h"
#include "log_page.h"
#include "stream_server.h"

static const char *PART_BOUNDARY = "123456789000000000000987654321";
static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" "123456789000000000000987654321";
static const char *STREAM_BOUNDARY =
    "\r\n--" "123456789000000000000987654321" "\r\n";
static const char *STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t s_stream_httpd = NULL;
static httpd_handle_t s_capture_httpd = NULL;
static httpd_handle_t s_view_httpd = NULL;

static volatile float g_fps = 0.0f;
static volatile uint32_t g_softenc_fail = 0;   /* 检测模式下软编码失败帧计数 */

/* ===== 日志环形缓冲 (供 App 通过 /log 拉取, 替代串口) ===== */
#define LOG_LINES      24
#define LOG_LINE_LEN   96
static char     g_log[LOG_LINES][LOG_LINE_LEN];
static uint32_t g_log_seq[LOG_LINES];
static volatile uint32_t g_log_head = 0;   /* 下一个写入槽 */
static volatile uint32_t g_log_total = 0;  /* 累计写入条数 */

void Stream_Log(const char *fmt, ...) {
    char line[LOG_LINE_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    uint32_t slot = g_log_head % LOG_LINES;
    uint32_t ms = millis();
    snprintf(g_log[slot], LOG_LINE_LEN, "%7u %s", (unsigned)ms, line);
    g_log_seq[slot] = ++g_log_total;
    g_log_head = (g_log_head + 1) % LOG_LINES;
}

/* GET /log -> 按时间顺序输出最近的日志行 */
static esp_err_t log_handler(httpd_req_t *req) {
    static char out[LOG_LINES * LOG_LINE_LEN + 32];
    size_t n = 0;
    uint32_t total = g_log_total;
    uint32_t first = (total > LOG_LINES) ? (total - LOG_LINES + 1) : 1;
    for (uint32_t seq = first; seq <= total; seq++) {
        uint32_t slot = (seq - 1) % LOG_LINES;
        if (g_log_seq[slot] != seq) continue;
        int k = snprintf(out + n, sizeof(out) - n, "%s\n", g_log[slot]);
        if (k <= 0) break;
        n += (size_t)k;
        if (n + LOG_LINE_LEN + 2 >= sizeof(out)) break;
    }
    if (n == 0) n = (size_t)snprintf(out, sizeof(out), "(no log)\n");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, out, n);
}

static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[64];

    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "30");

    uint32_t fps_frames = 0;
    uint32_t fps_t0 = millis();

    /* 发送缓冲预分配一次复用: 避免每帧 malloc/free 造成 PSRAM 碎片 */
    size_t cap = 128 * 1024;
    char *frame_buf = (char *)malloc(cap);
    if (!frame_buf) {
        frame_buf = (char *)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!frame_buf) return ESP_FAIL;
    }

    while (true) {
        /* 分辨率固定 VGA, 本流全程出帧; 检测不再切 framesize, 画面尺寸恒定。 */
        fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* 顺手把当前 JPEG 帧投喂给后台检测任务 (仅当它正在等帧时才 memcpy, <0.2ms) */
        if (fb->format == PIXFORMAT_JPEG) {
            AlignServer_FeedFrame(fb->buf, fb->len, fb->width, fb->height);
        }

        fps_frames++;
        uint32_t now = millis();
        if (now - fps_t0 >= 1000) {
            g_fps = fps_frames * 1000.0f / (now - fps_t0);
            fps_frames = 0;
            fps_t0 = now;
        }

        uint8_t *jpg_buf = NULL;
        size_t jpg_len = 0;
        bool soft = false;
        if (fb->format == PIXFORMAT_JPEG) {
            jpg_buf = fb->buf;
            jpg_len = fb->len;
        } else {
            /* 灰度模式防御分支 (固定 JPEG 后正常不会走到): 软编码失败只丢这一帧,
             * 不能断开流, 否则 /align/* 期间画面直接黑屏 */
            if (!frame2jpg(fb, 12, &jpg_buf, &jpg_len)) {
                g_softenc_fail++;
                esp_camera_fb_return(fb);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            soft = true;
        }

        size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, (unsigned)jpg_len);
        /* boundary + part header + jpeg body 合并为单次发送, 避免 TCP 拆段抖动 */
        size_t blen = strlen(STREAM_BOUNDARY);
        size_t total = blen + hlen + jpg_len;
        if (total > cap) {
            /* 帧异常大: 扩容一次, 仍失败则丢弃该帧 */
            char *nb = (char *)realloc(frame_buf, total);
            if (!nb) {
                if (soft && jpg_buf) free(jpg_buf);
                esp_camera_fb_return(fb);
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
            frame_buf = nb;
            cap = total;
        }
        memcpy(frame_buf, STREAM_BOUNDARY, blen);
        memcpy(frame_buf + blen, part_buf, hlen);
        memcpy(frame_buf + blen + hlen, jpg_buf, jpg_len);

        /* 关键: 已拷贝进 frame_buf, 立即归还相机 fb, 再做阻塞的网络发送。
         * fb 占用时间从数十 ms 降到 <0.2ms, DMA 缓冲立刻可用于采集下一帧。 */
        if (soft && jpg_buf) free(jpg_buf);
        esp_camera_fb_return(fb);
        fb = NULL;

        res = httpd_resp_send_chunk(req, frame_buf, total);
        if (res != ESP_OK) break;

        /* 让出 2ms 给低优先级任务 (loop 中的 UDP 发现) 与 IDLE 看门狗 */
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (frame_buf) free(frame_buf);
    return res;
}

static esp_err_t root_handler(httpd_req_t *req) {
    char buf[384];
    camera_fb_t *fb = esp_camera_fb_get();
    int fb_ok = fb ? 1 : 0;
    size_t fb_len = fb ? fb->len : 0;
    int fb_fmt = fb ? (int)fb->format : -1;
    if (fb) esp_camera_fb_return(fb);
    int n = snprintf(buf, sizeof(buf),
        "cam_init=%d cam_err=0x%x jpeg_ok=%d pid=0x%x err2=0x%x init_ms=%u "
        "detect_mode=%d fb_get=%d fb_len=%u fb_fmt=%d psram=%u heap=%u\n",
        Cam_IsOk() ? 1 : 0, Cam_Err(), Cam_JpegOk() ? 1 : 0,
        Cam_SensorPid(), Cam_Err2(), (unsigned)Cam_InitMs(),
        Cam_InDetectMode() ? 1 : 0,
        fb_ok, (unsigned)fb_len, fb_fmt,
        (unsigned)ESP.getPsramSize(), (unsigned)ESP.getFreeHeap());
    n += snprintf(buf + n, sizeof(buf) - n, "softenc_fail=%u\n", (unsigned)g_softenc_fail);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, n);
}

static esp_err_t fps_handler(httpd_req_t *req) {
    char buf[32];
    int n = snprintf(buf, sizeof(buf), "%.1f\n", (double)g_fps);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, n);
}

static esp_err_t jpg_handler(httpd_req_t *req) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "fb_get failed", HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t res;
    if (fb->format == PIXFORMAT_JPEG) {
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    } else {
        uint8_t *jpg_buf = NULL;
        size_t jpg_len = 0;
        if (frame2jpg(fb, 12, &jpg_buf, &jpg_len)) {
            res = httpd_resp_send(req, (const char *)jpg_buf, jpg_len);
            free(jpg_buf);
        } else {
            res = httpd_resp_send(req, "encode failed", HTTPD_RESP_USE_STRLEN);
        }
    }
    esp_camera_fb_return(fb);
    return res;
}

bool StreamServer_Start(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 16384;   /* /align/get 同步做检测, 局部数组较大, 默认4K不够 */
    /* 默认 max_uri_handlers=8, :80 实际注册 9 个(不含 /view),
     * 不调大会让后注册的路由静默注册失败(返回 ESP_ERR_HTTPD_HANDLERS_FULL 但没人检查) */
    config.max_uri_handlers = 16;

    httpd_uri_t stream_uri = {.uri = "/stream", .method = HTTP_GET,
                              .handler = stream_handler, .user_ctx = NULL};
    httpd_uri_t jpg_uri = {.uri = "/jpg", .method = HTTP_GET,
                           .handler = jpg_handler, .user_ctx = NULL};
    httpd_uri_t diag_uri = {.uri = "/diag", .method = HTTP_GET,
                            .handler = root_handler, .user_ctx = NULL};
    httpd_uri_t fps_uri = {.uri = "/fps", .method = HTTP_GET,
                           .handler = fps_handler, .user_ctx = NULL};
    httpd_uri_t log_uri = {.uri = "/log", .method = HTTP_GET,
                           .handler = log_handler, .user_ctx = NULL};

    // Control server on :80
    if (httpd_start(&s_capture_httpd, &config) != ESP_OK) return false;
    httpd_register_uri_handler(s_capture_httpd, &jpg_uri);
    httpd_register_uri_handler(s_capture_httpd, &diag_uri);
    httpd_register_uri_handler(s_capture_httpd, &fps_uri);
    httpd_register_uri_handler(s_capture_httpd, &log_uri);

    // 视觉纠偏 API (/align/*) 挂在 :80 控制服务上
    AlignServer_Register(s_capture_httpd);
    // 内置网页日志页 (:80/logs) 与状态 (:80/status)
    LogPage_Register(s_capture_httpd);
    /* 注意: /view 是 while(true) 推流 handler, 不能挂在 :80。
     * esp_http_server 每个实例是单线程, 推流会永久占用该任务,
     * 导致同实例上的 /align/* /log /logs 全部无响应。放到 :81 流实例上。 */

    // Stream server on :81 (专用流媒体实例, 与控制接口物理隔离)
    config.server_port = 81;
    config.ctrl_port = 32769;
    if (httpd_start(&s_stream_httpd, &config) != ESP_OK) return false;
    httpd_register_uri_handler(s_stream_httpd, &stream_uri);
    /* 注意: /view 也是 while(true) 推流 handler。esp_http_server 每个实例单线程,
     * 与 /stream 同挂 :81 会互相阻塞 (一个在推流时另一个永远排不上队),
     * 因此 /view 必须用独立实例。 */
    config.server_port = 82;
    config.ctrl_port = 32770;
    if (httpd_start(&s_view_httpd, &config) == ESP_OK) {
        AlignView_Register(s_view_httpd);
    } else {
        Stream_Log(":82 start failed, /view disabled");
    }
    return true;
}
