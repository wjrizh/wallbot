/* T1: 最小 AP + 静态页面. 无摄像头, 无 httpd 额外实例, 无检测算法. */
#include <WiFi.h>
#include <esp_http_server.h>

static httpd_handle_t s_httpd = NULL;

static esp_err_t root_handler(httpd_req_t *req) {
    const char *page =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>T1</title></head><body style='background:#111;color:#eee;font:16px sans-serif;padding:20px'>"
        "<h2>T1: AP + 静态页面</h2>"
        "<p id='s'>加载中…</p>"
        "<script>"
        "fetch('/status',{cache:'no-store'}).then(r=>r.text()).then(t=>{"
        "document.getElementById('s').textContent='OK: '+t;"
        "}).catch(e=>{document.getElementById('s').textContent='FAIL: '+e;});"
        "setInterval(()=>{fetch('/status',{cache:'no-store'}).then(r=>r.text())"
        ".then(t=>{document.getElementById('s').textContent='OK: '+t;});},2000);"
        "</script></body></html>";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req) {
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "up=%us heap=%u",
                     (unsigned)(millis() / 1000), (unsigned)ESP.getFreeHeap());
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, buf, n);
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.printf("T1 boot rst=%d\n", (int)esp_reset_reason());

    WiFi.persistent(false);
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(100);

    WiFi.mode(WIFI_AP);
    WiFi.setTxPower(WIFI_POWER_11dBm);
    WiFi.setSleep(false);
    bool up = WiFi.softAP("wallbot-t1", NULL, 6, 0, 4);
    delay(100);
    IPAddress ip(192, 168, 4, 1);
    WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
    delay(200);
    Serial.printf("T1 AP up=%d ip=%s\n", up ? 1 : 0,
                  WiFi.softAPIP().toString().c_str());

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.stack_size = 8192;
    if (httpd_start(&s_httpd, &cfg) == ESP_OK) {
        httpd_uri_t r = {.uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = NULL};
        httpd_uri_t s = {.uri = "/status", .method = HTTP_GET, .handler = status_handler, .user_ctx = NULL};
        httpd_register_uri_handler(s_httpd, &r);
        httpd_register_uri_handler(s_httpd, &s);
        Serial.println("T1 httpd ok");
    } else {
        Serial.println("T1 httpd FAIL");
    }
}

void loop() {
    delay(100);
}
