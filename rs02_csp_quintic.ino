/*
 * rs02_csp_quintic.ino - RS02 5次多项式 CSP 轨迹规划控制器
 * 特性:
 *  1. 严格使用 uint8 写入 run_mode=5，杜绝 float 写入 bug
 *  2. 5次多项式加减速 (端点加速度与速度为 0，启停完全无抖动冲击)
 *  3. 四重安全保护上电自动回零 (电子围栏 + 1.8A 限制电流 + 路径收敛 + 低速)
 *  4. 0° / 180° 双工位平滑切换 + 网页实时状态显示
 */

#include <Arduino.h>
#include <driver/twai.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <math.h>

#define CAN_TX_PIN       6
#define CAN_RX_PIN       7
#define HOST_ID          0xFD

#define AP_SSID          "RS02-CSP-CONTROLLER"
#define AP_PASS          "12345678"

#define P_MAX            (4.0f * M_PI)
#define V_MAX            44.0f
#define T_MAX            17.0f

#define SAFE_MIN_DEG     -10.0f
#define SAFE_MAX_DEG     190.0f
#define HOME_SAFE_CUR    1.8f
#define HOME_DURATION    4.5f

#define WORK_LIMIT_CUR   16.0f
#define WORK_SPEED_LIMIT 8.0f
#define WORK_DURATION    2.0f

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");
static Preferences prefs;

static volatile uint8_t g_motor_id = 0x01;
static volatile float   g_zero_rad = 0.0f;
static volatile float   g_cur_rad = 0.0f;
static volatile float   g_cur_tq = 0.0f;
static volatile bool    g_have_fb = false;
static volatile bool    g_safety_locked = false;

struct QuinticTrajectory {
  float start_pos;
  float end_pos;
  float total_time;
  uint32_t start_ms;
  bool active;
  float current_cmd;
};

static QuinticTrajectory g_traj = {0, 0, 0, 0, false, 0};

static volatile int     g_cmd = 0;
static volatile float   g_cmd_deg = 0.0f;
static volatile bool    g_busy = false;
static volatile uint32_t g_old_id = 0, g_new_id = 0;

static String g_pending_log;
static uint32_t g_last_ws_flush = 0;

// ---------------- 日志系统 ----------------
static void logln(const String& str){
  Serial.println(str);
  g_pending_log += str + "\n";
}

static void logf(const char* fmt, ...){
  char buf[256]; va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  logln(String(buf));
}

static void flush_ws(){
  if (g_pending_log.length() == 0) return;
  uint32_t now = millis();
  if (now - g_last_ws_flush < 30) return;
  g_last_ws_flush = now;
  if (ws.count() > 0) ws.textAll(g_pending_log);
  g_pending_log = "";
}

// ---------------- 底层 CAN 通信 ----------------
static bool send_ext(uint32_t id, const uint8_t* data, uint8_t len){
  twai_message_t msg;
  memset(&msg, 0, sizeof(msg));
  msg.identifier = id;
  msg.extd = 1;
  msg.data_length_code = len;
  if (len) memcpy(msg.data, data, len);
  return twai_transmit(&msg, pdMS_TO_TICKS(50)) == ESP_OK;
}

static void ensure_can_ok(){
  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK) return;
  if (st.state == TWAI_STATE_BUS_OFF){
    twai_initiate_recovery();
    delay(100);
  } else if (st.state == TWAI_STATE_STOPPED){
    twai_start();
  }
}

static void write_u8_param(uint16_t index, uint8_t val){
  ensure_can_ok();
  uint32_t cid = ((uint32_t)0x12 << 24) | ((uint32_t)HOST_ID << 8) | g_motor_id;
  uint8_t d[8] = {0};
  memcpy(&d[0], &index, 2);
  d[4] = val;
  send_ext(cid, d, 8);
}

static void write_float_param(uint16_t index, float val){
  ensure_can_ok();
  uint32_t cid = ((uint32_t)0x12 << 24) | ((uint32_t)HOST_ID << 8) | g_motor_id;
  uint8_t d[8] = {0};
  memcpy(&d[0], &index, 2);
  memcpy(&d[4], &val, 4);
  send_ext(cid, d, 8);
}

static void write_float_fast(uint16_t index, float val){
  uint32_t cid = ((uint32_t)0x12 << 24) | ((uint32_t)HOST_ID << 8) | g_motor_id;
  uint8_t d[8] = {0};
  memcpy(&d[0], &index, 2);
  memcpy(&d[4], &val, 4);
  twai_message_t msg;
  memset(&msg, 0, sizeof(msg));
  msg.identifier = cid;
  msg.extd = 1;
  msg.data_length_code = 8;
  memcpy(msg.data, d, 8);
  twai_transmit(&msg, pdMS_TO_TICKS(2));
}

static void enable_motor(){
  ensure_can_ok();
  uint32_t cid = ((uint32_t)3 << 24) | ((uint32_t)HOST_ID << 8) | g_motor_id;
  uint8_t d[8] = {0};
  send_ext(cid, d, 8);
  logln("[CMD] 电机使能");
}

static void stop_motor(){
  ensure_can_ok();
  g_traj.active = false;
  uint32_t cid = ((uint32_t)4 << 24) | ((uint32_t)HOST_ID << 8) | g_motor_id;
  uint8_t d[8] = {0};
  send_ext(cid, d, 8);
  logln("[CMD] 电机停止 / 失能");
}

static void start_report(){
  ensure_can_ok();
  uint32_t cid = ((uint32_t)0x18 << 24) | ((uint32_t)HOST_ID << 8) | g_motor_id;
  uint8_t d[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x01, 0x00};
  send_ext(cid, d, 8);
}

static void check_rx(){
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK){
    if (!rx.extd) continue;
    uint8_t type = (rx.identifier >> 24) & 0x1F;
    if (type == 2){
      uint16_t pu = (rx.data[0] << 8) | rx.data[1];
      uint16_t tu = (rx.data[4] << 8) | rx.data[5];
      g_cur_rad = ((float)pu / 65535.0f * 2.0f - 1.0f) * P_MAX;
      g_cur_tq  = ((float)tu / 65535.0f * 2.0f - 1.0f) * T_MAX;
      g_have_fb = true;
    }
  }
}

static float get_rel_deg(){
  return (g_cur_rad - g_zero_rad) * 180.0f / (float)M_PI;
}

// ---------------- CSP 模式配置与 5 次多项式插补 ----------------
static void setup_csp_mode(float current_limit, float speed_limit){
  stop_motor();
  delay(30);
  write_u8_param(0x7005, 5);
  delay(30);
  enable_motor();
  delay(30);
  write_float_param(0x7017, speed_limit);
  delay(20);
  write_float_param(0x7018, current_limit);
  delay(20);
  logf("[CSP] 模式配置就绪: 电流限制=%.1f A, 速度限制=%.1f rad/s", current_limit, speed_limit);
}

static void start_quintic_traj(float target_rad, float duration_sec){
  g_traj.start_pos = g_cur_rad;
  g_traj.end_pos = target_rad;
  g_traj.total_time = duration_sec;
  g_traj.start_ms = millis();
  g_traj.current_cmd = g_traj.start_pos;
  g_traj.active = true;
  logf("[TRAJ] 5次多项式规划启动: %.3f -> %.3f rad, 耗时: %.1f s",
       g_traj.start_pos, g_traj.end_pos, duration_sec);
}

static void update_quintic_traj(){
  if (!g_traj.active) {
    if (g_have_fb && g_traj.total_time > 0.1f){
      write_float_fast(0x7016, g_traj.current_cmd);
    }
    return;
  }

  float elapsed = (millis() - g_traj.start_ms) / 1000.0f;
  float tau = elapsed / g_traj.total_time;

  if (tau >= 1.0f){
    tau = 1.0f;
    g_traj.active = false;
    g_traj.current_cmd = g_traj.end_pos;
    logln("[TRAJ] 5次多项式轨迹执行完毕，平滑就位保持");
  } else {
    float tau2 = tau * tau;
    float tau3 = tau2 * tau;
    float s = tau3 * (10.0f - 15.0f * tau + 6.0f * tau2);
    g_traj.current_cmd = g_traj.start_pos + (g_traj.end_pos - g_traj.start_pos) * s;
  }

  write_float_fast(0x7016, g_traj.current_cmd);
}

// ---------------- ID 管理 ----------------
static void probe_ids(){
  ensure_can_ok();
  uint8_t d[8] = {0};
  logln("[PROBE] scan ID 1..32 ...");
  bool found[33]; memset(found, 0, sizeof(found));
  for (uint8_t id = 1; id <= 32; id++){
    uint32_t cid = ((uint32_t)0 << 24) | ((uint32_t)HOST_ID << 8) | id;
    send_ext(cid, d, 8);
    uint32_t t0 = millis();
    while (millis() - t0 < 30){
      twai_message_t rx;
      if (twai_receive(&rx, pdMS_TO_TICKS(2)) == ESP_OK && rx.extd){
        uint8_t t = (rx.identifier >> 24) & 0x1F;
        if (t == 0){
          uint8_t mid = (rx.identifier >> 8) & 0xFF;
          if (mid >= 1 && mid <= 32) found[mid] = true;
        }
      }
    }
  }
  bool any = false;
  for (uint8_t id = 1; id <= 32; id++){
    if (found[id]){ any = true; logf("[PROBE] found ID=0x%02X", id); g_motor_id = id; prefs.putUChar("motor_id", id); }
  }
  if (!any) logln("[PROBE] no motor");
  else logf("[PROBE] using motor_id=0x%02X", g_motor_id);
}

static void set_id(uint32_t old_id, uint32_t new_id){
  ensure_can_ok();
  stop_motor();
  uint32_t cid = ((uint32_t)7 << 24) | ((new_id & 0xFF) << 16) | ((uint32_t)HOST_ID << 8) | (old_id & 0xFF);
  uint8_t d[8] = {0};
  logf("[TX] SETID old=0x%02lX new=0x%02lX", (unsigned long)old_id, (unsigned long)new_id);
  send_ext(cid, d, 8); delay(300);

  uint32_t scid = ((uint32_t)0x18 << 24) | ((uint32_t)HOST_ID << 8) | (new_id & 0xFF);
  uint8_t sd[8] = {0}; sd[0] = 1;
  send_ext(scid, sd, 8); delay(300);

  uint32_t pcid = ((uint32_t)0 << 24) | ((uint32_t)HOST_ID << 8) | (new_id & 0xFF);
  send_ext(pcid, d, 8);
  bool ok = false;
  uint32_t t1 = millis();
  while (millis() - t1 < 400){
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(10)) == ESP_OK && rx.extd){
      uint8_t t = (rx.identifier >> 24) & 0x1F;
      uint8_t mid = (rx.identifier >> 8) & 0xFF;
      if (t == 0 && mid == (new_id & 0xFF)) ok = true;
    }
  }
  if (ok){ g_motor_id = new_id & 0xFF; prefs.putUChar("motor_id", g_motor_id); logf("=== SETID OK motor_id=0x%02X ===", g_motor_id); }
  else logln("=== SETID FAIL ===");
}

// ---------------- 四重安全上电回零 ----------------
static void safe_auto_home(){
  start_report();
  logln("[SAFE_HOME] 正在检测电机物理绝对位置...");

  uint32_t t0 = millis();
  while (millis() - t0 < 800){
    check_rx();
    delay(10);
  }

  if (!g_have_fb){
    logln("[FATAL] 未收到电机状态反馈！回零流程终止，请检查供电与接线。");
    return;
  }

  float cur_deg = get_rel_deg();
  logf("[SAFE_HOME] 读取相对 0 度偏差: %.1f°", cur_deg);

  if (cur_deg < SAFE_MIN_DEG || cur_deg > SAFE_MAX_DEG){
    g_safety_locked = true;
    logf("[ALARM] 当前角度 %.1f° 越出安全区间 [%.1f°, %.1f°]！",
         cur_deg, SAFE_MIN_DEG, SAFE_MAX_DEG);
    logln("[ALARM] 存在绞线或反向碰撞风险！系统闭锁，严禁自动使能！");
    logln("[ALARM] 请断电人工复位或在网页端重新标定零位。");
    return;
  }

  logln("[SAFE_HOME] 配置 CSP 保护电流 (1.8A) ...");
  setup_csp_mode(HOME_SAFE_CUR, 4.0f);

  float target_rad = g_zero_rad;

  start_quintic_traj(target_rad, HOME_DURATION);

  uint32_t wait_start = millis();
  while (millis() - wait_start < (uint32_t)(HOME_DURATION * 1000.0f + 2000.0f)){
    check_rx();
    update_quintic_traj();
    flush_ws();
    delay(20);
    if (!g_traj.active) break;
  }

  if (!g_traj.active){
    logln("[SAFE_HOME] 成功安全归位至 0°！恢复正常工作电流限制 (16A)。");
    write_float_param(0x7018, WORK_LIMIT_CUR);
    delay(30);
  } else {
    logln("[WARN] 回零超时，停止电机！");
    stop_motor();
  }
}

static void move_to_deg(float target_deg){
  if (g_safety_locked){
    logln("[DENIED] 系统已报警闭锁，拒绝执行运动！");
    return;
  }
  if (target_deg < 0.0f) target_deg = 0.0f;
  if (target_deg > 180.0f) target_deg = 180.0f;

  stop_motor(); delay(50);
  write_u8_param(0x7005, 5);
  delay(30);
  enable_motor(); delay(30);

  float target_rad = g_zero_rad + (target_deg * (float)M_PI / 180.0f);
  logf(">>> MOVE %.1f deg -> %.3f rad", target_deg, target_rad);

  write_float_param(0x7018, WORK_LIMIT_CUR);
  write_float_param(0x7017, WORK_SPEED_LIMIT);

  start_quintic_traj(target_rad, WORK_DURATION);
}

// ---------------- 网页界面 ----------------
static const char PAGE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RS02 CSP 5次多项式控制</title>
<style>
body{margin:0;background:#121212;color:#eee;font:14px -apple-system,Consolas,sans-serif}
#header{position:sticky;top:0;background:#1e1e1e;padding:12px;display:flex;justify-content:space-between;align-items:center;border-bottom:1px solid #333}
.pos-box{font-size:24px;font-weight:bold;color:#00e5ff}
.card{background:#1a1a1a;margin:12px;padding:14px;border-radius:10px;border:1px solid #2d2d2d}
h3{margin:0 0 10px 0;font-size:13px;color:#888;text-transform:uppercase}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
button{padding:14px;font-size:16px;font-weight:bold;border-radius:8px;border:0;cursor:pointer;transition:0.15s}
button:active{transform:scale(0.98)}
.btn-0{background:#00897b;color:#fff}
.btn-180{background:#1e88e5;color:#fff}
.btn-zero{background:#d35400;color:#fff;width:100%;padding:11px}
.btn-home{background:#4b6584;color:#fff;width:100%;padding:11px;margin-top:8px}
.btn-stop{background:#c0392b;color:#fff;width:100%;padding:11px;margin-top:8px}
#log{padding:12px;background:#0a0a0a;margin:12px;border-radius:8px;font-family:monospace;font-size:12px;height:240px;overflow-y:auto;white-space:pre-wrap;color:#7bed9f}
.alarm{color:#ff4757;font-weight:bold}
</style></head><body>

<div id="header">
  <div>相对角度: <span id="pos" class="pos-box">--</span> °</div>
  <div id="status" style="color:#ffa502">连接中...</div>
</div>

<div class="card">
  <h3>5次多项式工位平滑切换 (CSP)</h3>
  <div class="grid">
    <button class="btn-0" onclick="go(0)">平滑转到 0° (原点)</button>
    <button class="btn-180" onclick="go(180)">平滑转到 180°</button>
  </div>
</div>

<div class="card">
  <h3>零位标定与安全管理</h3>
  <button class="btn-zero" onclick="setZero()">将当前位置设为 0° 原点 (存NVS)</button>
  <button class="btn-home" onclick="triggerHome()">手动触发安全回零</button>
  <button class="btn-stop" onclick="stopMotor()">急停 / 失能电机</button>
</div>

<div id="log"></div>

<div class="card">
  <h3>编号管理</h3>
  旧ID <input id="oid" value="1" style="width:60px"> 新ID <input id="nid" value="2" style="width:60px">
  <button class="btn-zero" onclick="setid()">修改并回读</button>
  <button class="btn-home" onclick="c('n=7')">读取ID</button>
  <button class="btn-home" onclick="c('n=1')">使能</button>
</div>

<script>
var p=document.getElementById('pos'), s=document.getElementById('status'), l=document.getElementById('log');
function go(d){ fetch('/cmd?n=4&deg='+d); }
function setZero(){ if(confirm('确认将当前实际位置标定为 0° 绝对原点？')) fetch('/cmd?n=3'); }
function triggerHome(){ fetch('/cmd?n=9'); }
function stopMotor(){ fetch('/cmd?n=2'); }
function c(q){ fetch('/cmd?'+q); }
function setid(){var o=document.getElementById('oid').value,n=document.getElementById('nid').value;c('n=5&old='+o+'&new='+n);}

var ws=new WebSocket('ws://'+location.host+'/ws');
ws.onopen=function(){ s.textContent='系统在线 (CSP 5次多项式)'; s.style.color='#00e5ff'; };
ws.onclose=function(){ s.textContent='连接断开'; s.style.color='#ff4757'; };
ws.onmessage=function(e){
  var lines=e.data.split('\n');
  for(var i=0;i<lines.length;i++){
    if(!lines[i]) continue;
    var d=document.createElement('div');
    if(lines[i].indexOf('ALARM')>=0||lines[i].indexOf('FATAL')>=0) d.className='alarm';
    d.textContent=lines[i];
    l.appendChild(d);
  }
  l.scrollTop=l.scrollHeight;
};

setInterval(function(){
  fetch('/pos').then(r=>r.json()).then(data=>{
    p.textContent = data.deg.toFixed(1);
    if(data.alarm){
      s.textContent='电子围栏报警锁定!';
      s.style.color='#ff4757';
    }
  }).catch(()=>{});
}, 100);
</script></body></html>
)HTML";

// ---------------- 主流程 ----------------
void setup(){
  Serial.begin(115200);
  delay(300);
  logln("=== RS02 CSP 5次多项式控制器启动 ===");

  prefs.begin("rs02", false);
  g_zero_rad = prefs.getFloat("zero", 0.0f);
  g_motor_id = prefs.getUChar("motor_id", 0x01);
  logf("[NVS] 载入配置 -> 电机ID: 0x%02X, 零点: %.3f rad", g_motor_id, g_zero_rad);

  twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_cfg = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_cfg, &t_cfg, &f_cfg) == ESP_OK && twai_start() == ESP_OK){
    logln("[CAN] 1Mbps 总线初始化成功");
  }

  WiFi.softAP(AP_SSID, AP_PASS);
  logf("[WEB] 控制页面: http://%s/", WiFi.softAPIP().toString().c_str());

  ws.onEvent([](AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* arg, uint8_t* d, size_t l){
    if (t == WS_EVT_CONNECT) logf("[WS] 网页端已连接");
  });
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){ r->send_P(200, "text/html", PAGE_HTML); });

  server.on("/pos", HTTP_GET, [](AsyncWebServerRequest* r){
    float cur_d = get_rel_deg();
    char resp[120];
    snprintf(resp, sizeof(resp), "{\"deg\":%.2f,\"tq\":%.2f,\"alarm\":%s}",
             cur_d, g_cur_tq, g_safety_locked ? "true" : "false");
    r->send(200, "application/json", resp);
  });

  server.on("/cmd", HTTP_GET, [](AsyncWebServerRequest* r){
    if (r->hasParam("n")){
      int n = r->getParam("n")->value().toInt();
      if (!g_busy || n == 2 || n == 7) g_cmd = n;
      if (n == 4 && r->hasParam("deg")) g_cmd_deg = r->getParam("deg")->value().toFloat();
      if (n == 5){
        if (r->hasParam("old")) g_old_id = strtoul(r->getParam("old")->value().c_str(), NULL, 0);
        if (r->hasParam("new")) g_new_id = strtoul(r->getParam("new")->value().c_str(), NULL, 0);
      }
    }
    r->send(200, "text/plain", "ok");
  });
  server.begin();

  enable_motor(); delay(30);
  setup_csp_mode(WORK_LIMIT_CUR, WORK_SPEED_LIMIT);

  safe_auto_home();
}

void loop(){
  ws.cleanupClients();
  check_rx();
  flush_ws();

  static uint32_t last_cycle = 0;
  uint32_t now = millis();
  if (now - last_cycle >= 20){
    last_cycle = now;
    update_quintic_traj();
  }

  if (g_busy || g_cmd == 0){
    delay(5);
    return;
  }

  int cmd = g_cmd;
  g_busy = true;

  if (cmd == 1){
    enable_motor();
  } else if (cmd == 2){
    stop_motor();
  } else if (cmd == 3){
    g_zero_rad = g_cur_rad;
    prefs.putFloat("zero", g_zero_rad);
    g_safety_locked = false;
    logf("[ZERO] 标定完成！当前绝对位置记录为 0°: %.3f rad", g_zero_rad);
  } else if (cmd == 4){
    move_to_deg(g_cmd_deg);
  } else if (cmd == 5){
    set_id(g_old_id, g_new_id);
  } else if (cmd == 7){
    probe_ids();
  } else if (cmd == 9){
    g_safety_locked = false;
    safe_auto_home();
  }

  g_busy = false;
  g_cmd = 0;
}