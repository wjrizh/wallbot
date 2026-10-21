/* 母机 NRF24L01 PING 测试
 * 网页 http://<母机IP>/  点按钮发 PING:n，显示回包 PONG:n 与 RTT
 * 串口 115200 同步打印
 */
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <RF24.h>

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

refresh();
setInterval(refresh, 1000);
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
  server.begin();
  Serial.println("[HTTP] started");
}

void loop() {
  server.handleClient();
}
