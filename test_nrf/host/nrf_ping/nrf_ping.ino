/* 母机 NRF24L01 PING 测试
 * 网页 http://<母机IP>/  点按钮发 PING:n，显示回包 PONG:n 与 RTT
 * 串口 115200 同步打印
 */
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <RF24.h>
#include <stdarg.h>
#include <string.h>

#define CE_PIN   47
#define CSN_PIN  18
#define SCK_PIN  15
#define MOSI_PIN 13
#define MISO_PIN 14
#define RF_CH    40

static const uint8_t ADDR[5] = {0x11, 0x22, 0x33, 0x44, 0x55};

RF24 radio(CE_PIN, CSN_PIN);
WebServer server(80);

static uint32_t s_seq = 0;
static uint32_t s_sent = 0;
static uint32_t s_recv = 0;
static uint32_t s_lost = 0;
static uint32_t s_lastRtt = 0;
static uint32_t s_writeMs = 0;
static uint32_t s_rxMs = 0;
static uint32_t s_iters = 0;
static uint32_t s_maxAvailMs = 0;
static uint32_t s_maxDelayMs = 0;
static bool     s_wbOk = false;
static bool     s_sbOk = false;
static bool     s_radioOk = false;
static char     s_lastTx[33] = {0};
static char     s_lastRx[33] = {0};
static char     s_initInfo[220] = {0};   /* NRF 初始化详情, 页面启动时展示 */
static uint8_t  s_alignState = 0;        /* 0=IDLE 1=WAIT_TSTB 2=READY 3=WAIT_DONE */
static uint32_t s_alignMs = 0;
static uint32_t s_alignFrames = 0;
static uint32_t s_alignDeadline = 0;
static uint8_t  s_alignDone = 0;
static char     s_alignMsg[48] = {0};
static char     s_telemLast[80] = {0};
static float    s_adjPhi = 0, s_adjPsi = 0, s_adjAx = 0;

static bool radioSetup() {
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, -1);
  if (!radio.begin()) return false;
  radio.setChannel(RF_CH);
  radio.setDataRate(RF24_2MBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);
  radio.setRetries(0, 3);
  radio.setPayloadSize(32);
  radio.openWritingPipe(ADDR);
  radio.openReadingPipe(0, ADDR);   /* 用 pipe0: 子机只开 EN_RXADDR pipe0 */
  radio.stopListening();

  /* 用公开 API 读回实际生效配置 */
  snprintf(s_initInfo, sizeof(s_initInfo),
    "chip=%d ch=%u rate=%u pa=%u payload=%u crc=%u status=%02X",
    radio.isChipConnected() ? 1 : 0,
    (unsigned)radio.getChannel(),
    (unsigned)radio.getDataRate(),
    (unsigned)radio.getPALevel(),
    (unsigned)radio.getPayloadSize(),
    (unsigned)radio.getCRCLength(),
    (unsigned)radio.getStatusFlags());
  return true;
}

static void doPing() {
  if (!s_radioOk) { Serial.println("[PING] radio not ok"); return; }

  uint8_t tx[32];
  memset(tx, 0, 32);
  snprintf((char*)tx, 32, "PING:%lu", (unsigned long)++s_seq);
  memcpy(s_lastTx, tx, 32);

  /* 清残留标志 */
  radio.stopListening();
  radio.flush_tx();
  radio.clearStatusFlags();
  delay(2);

  uint32_t t0 = millis();
  /* 原协同时序: 非阻塞 write() 后立刻 startListening() 开接收窗口.
   * 之前的 writeBlocking+txStandBy 让母机停在 PTX 等 ACK 200ms,
   * 期间子机回包全被丢, 导致 sb=0 / ARC_CNT=10. */
  bool wbOk = radio.write(tx, 32);
  uint32_t t1 = millis();
  s_writeMs = t1 - t0;
  radio.flush_rx();
  radio.startListening();
  bool sbOk = wbOk;
  s_wbOk = wbOk;
  s_sbOk = sbOk;
  s_sent++;

  Serial.printf("[TX] %s -> wb=%d\n", s_lastTx, wbOk ? 1 : 0);
  if (!wbOk) { s_lost++; return; }

  /* 等回包, 最多 300ms */
  uint32_t deadline = millis() + 300;
  bool got = false;
  uint32_t iters = 0, maxAvail = 0, maxDelay = 0;
  while (millis() < deadline) {
    iters++;
    uint32_t ta = millis();
    bool av = radio.available();
    uint32_t dav = millis() - ta;
    if (dav > maxAvail) maxAvail = dav;
    if (av) {
      uint8_t rx[32];
      uint8_t n = radio.getPayloadSize();
      if (n == 0 || n > 32) n = 32;
      radio.read(rx, n);
      memcpy(s_lastRx, rx, 32);
      s_recv++;
      s_lastRtt = millis() - t0;
      s_rxMs = s_lastRtt - s_writeMs;
      got = true;
      Serial.printf("[RX] %s (rtt=%lums)\n", s_lastRx, (unsigned long)s_lastRtt);
      break;
    }
    uint32_t td = millis();
    delay(1);
    uint32_t dd = millis() - td;
    if (dd > maxDelay) maxDelay = dd;
  }
  s_iters = iters;
  s_maxAvailMs = maxAvail;
  s_maxDelayMs = maxDelay;
  if (!got) {
    s_lost++;
    Serial.println("[RX] timeout");
  }
}

/* ===== 视觉纠偏测试 (虚拟数据, 不驱动真电机) ===== */

/* 全过程 trace, 网页显示 */
static char s_trace[1400] = {0};
static void trace(const char* fmt, ...) {
  char line[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  size_t cur = strlen(s_trace);
  snprintf(s_trace + cur, sizeof(s_trace) - cur, "[%lu] %s\n",
           (unsigned long)millis(), line);
}

static bool sendRaw(const uint8_t* buf) {
  /* 子机回 TSTB / 发遥测时会短暂离开 RX 切到 TX, 单次 write() 撞上该窗口
   * 就收不到 ACK 返回 false. 重试多次, 每次失败清 FIFO+标志, 间隔 25ms
   * 让子机回到 RX 再发. */
  for (int attempt = 0; attempt < 6; attempt++) {
    radio.stopListening();
    radio.flush_tx();
    radio.clearStatusFlags();
    delay(2);
    uint32_t t0 = millis();
    bool ok = radio.write(buf, 32);
    uint32_t dt = millis() - t0;
    radio.flush_rx();
    radio.startListening();
    trace("TX att%d ok=%d %lums st=%02X",
          attempt, ok ? 1 : 0, (unsigned long)dt, radio.getStatusFlags());
    if (ok) return true;
    delay(25);
  }
  return false;
}

static void alignStart(void) {
  uint8_t tx[32];
  memset(tx, 0, 32);
  memcpy(tx, "TSTA", 4);
  s_alignDone = 0;
  s_alignFrames = 0;
  s_telemLast[0] = 0;
  s_trace[0] = 0;
  trace("== START TSTA ==");
  if (!sendRaw(tx)) { s_alignState = 0; snprintf(s_alignMsg, sizeof(s_alignMsg), "TSTA 发送失败"); trace("TSTA FAIL"); return; }
  s_alignState = 1;
  s_alignDeadline = millis() + 3000;
  snprintf(s_alignMsg, sizeof(s_alignMsg), "等待 TSTB");
}

static void alignSendAdj(float phi, float psi, float ax) {
  uint8_t tx[32];
  memset(tx, 0, 32);
  snprintf((char*)tx, 32, "ADJ,%.2f,%.2f,%.2f", phi, psi, ax);
  s_adjPhi = phi; s_adjPsi = psi; s_adjAx = ax;
  s_alignFrames = 0;
  s_alignDone = 0;
  s_telemLast[0] = 0;
  s_alignMs = millis();
  trace("== ADJ %s ==", (char*)tx);
  if (!sendRaw(tx)) { s_alignState = 2; snprintf(s_alignMsg, sizeof(s_alignMsg), "ADJ 发送失败"); trace("ADJ FAIL"); return; }
  s_alignState = 3;
  s_alignDeadline = millis() + 8000;
  snprintf(s_alignMsg, sizeof(s_alignMsg), "执行中");
  trace("ADJ sent, wait DONE");
}

static void alignStop(void) {
  uint8_t tx[32];
  memset(tx, 0, 32);
  memcpy(tx, "TSTE", 4);
  trace("== STOP TSTE ==");
  sendRaw(tx);
  s_alignState = 0;
  snprintf(s_alignMsg, sizeof(s_alignMsg), "已退出测试");
  trace("state=IDLE");
}

/* 每 loop 调用: 推进状态机, 收 TSTB/遥测/DONE */
static void alignTick(void) {
  if (s_alignState == 0) return;

  uint8_t rx[32];
  while (radio.available()) {
    uint8_t n = radio.getPayloadSize();
    if (n == 0 || n > 32) n = 32;
    memset(rx, 0, 32);
    radio.read(rx, n);

    trace("RX %c%c%c%c len", rx[0], rx[1]?rx[1]:'.', rx[2]?rx[2]:'.', rx[3]?rx[3]:'.');
    if (memcmp(rx, "TSTB", 4) == 0) {
      s_alignState = 2;
      snprintf(s_alignMsg, sizeof(s_alignMsg), "就绪, 可发偏差");
    } else if (memcmp(rx, "DONE", 4) == 0) {
      s_alignDone = 1;
      s_alignState = 2;
      snprintf(s_alignMsg, sizeof(s_alignMsg), "完成 %lums %lu帧",
               (unsigned long)(millis() - s_alignMs), (unsigned long)s_alignFrames);
    } else if (rx[0] == 'J') {
      int16_t a_mm, b_mm, target;
      int8_t dir;
      memcpy(&a_mm, &rx[3], 2);
      memcpy(&b_mm, &rx[5], 2);
      memcpy(&target, &rx[7], 2);
      dir = (int8_t)rx[2];
      s_alignFrames++;
      snprintf(s_telemLast, sizeof(s_telemLast),
               "stage=%u dir=%d a=%d b=%d tgt=%d pct=%u rc=%u",
               rx[1], dir, a_mm, b_mm, target, rx[9], rx[10]);
    }
  }

  if ((s_alignState == 1 || s_alignState == 3) &&
      (int32_t)(millis() - s_alignDeadline) > 0) {
    snprintf(s_alignMsg, sizeof(s_alignMsg),
             s_alignState == 1 ? "超时: 无 TSTB" : "超时: 无 DONE");
    trace(s_alignState == 1 ? "TIMEOUT no TSTB" : "TIMEOUT no DONE");
    s_alignState = 0;
  }
}

static const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>NRF Ping Test</title>
<style>
body{font-family:system-ui;margin:0;padding:16px;background:#111;color:#eee}
h2{margin:0 0 12px;font-size:18px}
.card{background:#1c1c1e;border-radius:10px;padding:14px;margin-bottom:12px}
.k{color:#8e8e93;font-size:13px}
.v{font-size:22px;font-weight:600;font-variant-numeric:tabular-nums}
.row{display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid #2c2c2e}
.row:last-child{border:none}
button{width:100%;padding:14px;font-size:16px;border:0;border-radius:10px;
  background:#0a84ff;color:#fff;font-weight:600;margin-top:8px}
button:active{background:#0060df}
.mono{font-family:ui-monospace,monospace;font-size:12px;word-break:break-all;color:#d1d1d6}
#log{max-height:220px;overflow-y:auto;font-family:ui-monospace,monospace;
  font-size:11px;line-height:1.5;color:#aaa;white-space:pre-wrap}
</style></head><body>
<h2>NRF24L01 子母机信号测试</h2>

<div class="card">
  <div class="row"><span class="k">模块状态</span><span class="v" id="st">--</span></div>
  <div class="row"><span class="k">已发送</span><span class="v" id="sent">0</span></div>
  <div class="row"><span class="k">已收到</span><span class="v" id="recv">0</span></div>
  <div class="row"><span class="k">丢包</span><span class="v" id="lost">0</span></div>
  <div class="row"><span class="k">RTT</span><span class="v" id="rtt">--</span></div>
</div>

<div class="card">
  <div class="k">最近发送</div><div class="mono" id="tx">--</div>
  <div class="k" style="margin-top:8px">最近收到</div><div class="mono" id="rx">--</div>
</div>

<button id="btn">发送一次 PING</button>
<button id="btn10" style="background:#30d158">连发 10 次</button>

<h2 style="margin-top:18px">视觉纠偏测试 (虚拟数据)</h2>
<div class="card">
  <div class="row"><span class="k">状态</span><span class="v" id="aSt">IDLE</span></div>
  <div class="row"><span class="k">已收遥测帧</span><span class="v" id="aFn">0</span></div>
  <div class="k" style="margin-top:6px">最近遥测</div><div class="mono" id="aTm">--</div>
  <div class="k" style="margin-top:8px">全过程 trace</div>
  <div id="aTr" style="max-height:200px;overflow-y:auto;font-family:ui-monospace,monospace;font-size:11px;line-height:1.5;color:#9a9a9f;white-space:pre-wrap;background:#000;padding:6px;border-radius:6px">--</div>
</div>
<div class="card">
  <div class="row"><span class="k">phi (环向)</span><input id="phi" type="number" step="0.1" value="5" style="width:100px;background:#2c2c2e;color:#fff;border:0;border-radius:6px;padding:6px;text-align:right"></div>
  <div class="row"><span class="k">psi (偏航)</span><input id="psi" type="number" step="0.1" value="3" style="width:100px;background:#2c2c2e;color:#fff;border:0;border-radius:6px;padding:6px;text-align:right"></div>
  <div class="row"><span class="k">ax (径向)</span><input id="ax" type="number" step="0.1" value="0" style="width:100px;background:#2c2c2e;color:#fff;border:0;border-radius:6px;padding:6px;text-align:right"></div>
</div>
<button id="aStart">① 开始测试 (TSTA)</button>
<button id="aAdj" style="background:#ff9f0a">② 发偏差 (ADJ)</button>
<button id="aStop" style="background:#ff453a">③ 结束 (TSTE)</button>

<div class="card" style="margin-top:12px">
  <div class="k" style="margin-bottom:6px">日志</div>
  <div id="log"></div>
</div>

<script>
var logEl = document.getElementById('log');
function log(s){ logEl.textContent += s + '\n'; logEl.scrollTop = logEl.scrollHeight; }

var initShown = false;
function refresh(){
  fetch('/stat').then(function(r){return r.json();}).then(function(d){
    document.getElementById('st').textContent = d.radioOk ? '就绪' : '未找到';
    document.getElementById('st').style.color = d.radioOk ? '#30d158' : '#ff453a';
    document.getElementById('sent').textContent = d.sent;
    document.getElementById('recv').textContent = d.recv;
    document.getElementById('lost').textContent = d.lost;
    document.getElementById('rtt').textContent = d.rtt ? (d.rtt + ' ms') : '--';
    document.getElementById('tx').textContent = d.lastTx || '--';
    document.getElementById('rx').textContent = d.lastRx || '--';
    if (!initShown && d.init) {
      initShown = true;
      log('=== 母机 NRF 初始化 ===');
      log(d.init);
      if (!d.radioOk) log('!! 模块未找到, 检查 CE=47 CSN=18 SCK=15 MOSI=13 MISO=14 与供电');
      log('=== 等待 PING ===');
    }
  }).catch(function(){});
}

var seq = 0;
function ping(){
  seq++;
  log('--- PING #' + seq + ' 发出 ---');
  fetch('/ping').then(function(r){return r.json();}).then(function(d){
    log('  TX: ' + (d.tx || '(空)'));
    if (d.rx) {
      log('  RX: ' + d.rx);
      log('  RTT: ' + d.rtt + ' ms (write ' + d.wms + ' + rx ' + d.rms + ') it=' + d.it + ' maxAvail=' + d.mav + ' maxDelay=' + d.mdl);
      log('  ✓ 双向通信成功');
    } else {
      log('  ✗ wb=' + (d.wb?'1':'0') + ' sb=' + (d.sb?'1':'0'));
      if (!d.wb) log('  → TX FIFO 满, 发送未发起');
      else if (!d.sb) log('  → 已发但未收到硬件ACK (子机未收 或 信道被占)');
      else log('  → 已收ACK 但 300ms 内无 PONG 回包');
    }
    refresh();
  }).catch(function(e){ log('请求失败 ' + e); });
}

document.getElementById('btn').onclick = ping;
document.getElementById('btn10').onclick = function(){
  var n = 0;
  (function next(){ if (n++ >= 10) return; ping(); setTimeout(next, 400); })();
};

var aStateName = ['IDLE','等待TSTB','就绪','执行中'];
function aRefresh(){
  fetch('/align/stat').then(function(r){return r.json();}).then(function(d){
    var el = document.getElementById('aSt');
    el.textContent = d.msg || aStateName[d.st] || 'IDLE';
    el.style.color = d.st===3 ? '#ff9f0a' : (d.st===2 ? '#30d158' : '#eee');
    document.getElementById('aFn').textContent = d.frames;
    document.getElementById('aTm').textContent = d.telem || '--';
    var tr = document.getElementById('aTr');
    if (d.trace) { tr.textContent = d.trace; tr.scrollTop = tr.scrollHeight; }
  }).catch(function(){});
}
function aPost(url, body, okMsg){
  fetch(url, {method:'POST', headers:{'Content-Type':'application/json'}, body: body||'{}'})
    .then(function(r){return r.json();}).then(function(d){
      log(okMsg + (d.ok ? ' ✓' : ' ✗ ' + (d.msg||'')));
      aRefresh();
    }).catch(function(e){ log('请求失败 ' + e); });
}
document.getElementById('aStart').onclick = function(){
  log('--- 开始测试 TSTA ---'); aPost('/align/start', '{}', 'TSTA');
};
document.getElementById('aAdj').onclick = function(){
  var phi = parseFloat(document.getElementById('phi').value)||0;
  var psi = parseFloat(document.getElementById('psi').value)||0;
  var ax  = parseFloat(document.getElementById('ax').value)||0;
  log('--- 发偏差 ADJ,' + phi + ',' + psi + ',' + ax + ' ---');
  fetch('/align/adj?phi='+phi+'&psi='+psi+'&ax='+ax, {method:'POST'})
    .then(function(r){return r.json();}).then(function(d){ log('ADJ ' + (d.ok?'✓':'✗ '+(d.msg||''))); aRefresh(); })
    .catch(function(e){ log('请求失败 ' + e); });
};
document.getElementById('aStop').onclick = function(){
  log('--- 结束 TSTE ---'); aPost('/align/stop', '{}', 'TSTE');
};

refresh();
setInterval(refresh, 1000);
aRefresh();
setInterval(aRefresh, 300);
log('页面就绪, 正在获取母机状态...');
</script></body></html>
)HTML";

static void noCache() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
}
static void handleRoot() { noCache(); server.send_P(200, "text/html", PAGE); }

static void handleStat() {
  noCache();
  char buf[320];
  snprintf(buf, sizeof(buf),
    "{\"radioOk\":%s,\"sent\":%lu,\"recv\":%lu,\"lost\":%lu,\"rtt\":%lu,"
    "\"lastTx\":\"%s\",\"lastRx\":\"%s\",\"init\":\"%s\"}",
    s_radioOk ? "true" : "false",
    (unsigned long)s_sent, (unsigned long)s_recv, (unsigned long)s_lost,
    (unsigned long)s_lastRtt, s_lastTx, s_lastRx, s_initInfo);
  server.send(200, "application/json", buf);
}

static void handlePing() {
  noCache();
  uint32_t before = s_recv;
  doPing();
  bool got = (s_recv > before);
  char buf[320];
  snprintf(buf, sizeof(buf),
    "{\"ok\":%s,\"wb\":%s,\"sb\":%s,\"tx\":\"%s\",\"rx\":\"%s\",\"rtt\":%lu,\"wms\":%lu,\"rms\":%lu,\"it\":%lu,\"mav\":%lu,\"mdl\":%lu}",
    got ? "true" : "false", s_wbOk ? "true" : "false", s_sbOk ? "true" : "false",
    s_lastTx, got ? s_lastRx : "", (unsigned long)s_lastRtt,
    (unsigned long)s_writeMs, (unsigned long)s_rxMs,
    (unsigned long)s_iters, (unsigned long)s_maxAvailMs, (unsigned long)s_maxDelayMs);
  server.send(200, "application/json", buf);
}

static String jsonEscape(const char* s) {
  String o;
  o.reserve(strlen(s) + 64);
  for (const char* p = s; *p; p++) {
    char c = *p;
    if (c == '"') o += "\\\"";
    else if (c == '\\') o += "\\\\";
    else if (c == '\n') o += "\\n";
    else if (c == '\r') o += "\\r";
    else if (c == '\t') o += "\\t";
    else if ((unsigned char)c < 0x20) { char t[8]; snprintf(t, sizeof(t), "\\u%04X", c); o += t; }
    else o += c;
  }
  return o;
}

static void handleAlignStat() {
  noCache();
  String buf = "{\"st\":" + String(s_alignState) +
               ",\"msg\":\"" + jsonEscape(s_alignMsg) +
               "\",\"frames\":" + String(s_alignFrames) +
               ",\"telem\":\"" + jsonEscape(s_telemLast) +
               "\",\"trace\":\"" + jsonEscape(s_trace) + "\"}";
  server.send(200, "application/json", buf);
}

static void handleAlignStart() {
  noCache();
  alignStart();
  server.send(200, "application/json",
    s_alignState == 1 ? "{\"ok\":true}" : "{\"ok\":false,\"msg\":\"send fail\"}");
}

static void handleAlignAdj() {
  noCache();
  float phi = server.hasArg("phi") ? server.arg("phi").toFloat() : 0;
  float psi = server.hasArg("psi") ? server.arg("psi").toFloat() : 0;
  float ax  = server.hasArg("ax")  ? server.arg("ax").toFloat()  : 0;
  alignSendAdj(phi, psi, ax);
  server.send(200, "application/json",
    s_alignState == 3 ? "{\"ok\":true}" : "{\"ok\":false,\"msg\":\"send fail\"}");
}

static void handleAlignStop() {
  noCache();
  alignStop();
  server.send(200, "application/json", "{\"ok\":true}");
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);   /* USB CDC 无主机读时写不阻塞(否则卡~2s) */
  delay(300);
  Serial.println("\n=== NRF24L01 HOST PING TEST ===");

  s_radioOk = radioSetup();
  Serial.printf("[NRF] %s\n", s_radioOk ? "ok" : "NOT FOUND");
  if (s_radioOk) {
    uint8_t cfg = radio.isChipConnected() ? 1 : 0;
    Serial.printf("[NRF] chipConnected=%u\n", cfg);
  }

  /* AP 模式: ESP32-S3 + WPA2 有已知崩溃(wpa_ap_get_wpa_ie), 用开放热点规避 */
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  delay(200);
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  bool apOk = WiFi.softAP("robot");          /* 开放热点, 无密码 */
  Serial.printf("[WiFi] softAP=%d ssid=robot open\n", apOk ? 1 : 0);
  Serial.printf("[WiFi] http://%s/\n", WiFi.softAPIP().toString().c_str());
  if (apOk) {
    WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
    Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());
  }

  server.on("/", handleRoot);
  server.on("/stat", handleStat);
  server.on("/ping", handlePing);
  server.on("/align/stat", handleAlignStat);
  server.on("/align/start", HTTP_POST, handleAlignStart);
  server.on("/align/adj", HTTP_POST, handleAlignAdj);
  server.on("/align/stop", HTTP_POST, handleAlignStop);
  server.begin();
  Serial.println("[HTTP] started");
}

void loop() {
  server.handleClient();
  alignTick();
}
