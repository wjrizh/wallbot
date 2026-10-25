/* align_vision.cpp - 真实视觉纠偏闭环
 *
 * 链路: App 0x22 -> 本模块 -> HTTP /align/once (K230) -> TestMode -> NRF -> 子机
 *       子机 Adj_Execute(phi,psi,ax) -> DONE -> TestMode -> 本模块
 *
 * 设计要点:
 *   1) 每轮都调 /align/once 触发新的共识测量 (不是 /align/get 读缓存)
 *   2) NRF 收发只经 TestMode, 时序与虚拟纠偏完全一致
 *   3) 符号/增益/单步 全部可在 App 里改, 实机标定不用重烧固件
 *   4) 全过程打印到母机串口, 前缀 [AV]
 */
#include "align_vision.h"
#include "config.h"
#include "test_mode.h"
#include "auto_mode.h"
#include "web_server.h"
#include <Arduino.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* TestMode 状态值 (与 test_mode.cpp 的 enum 对应, 只用到 READY) */
#define TM_STATE_IDLE   0
#define TM_STATE_READY  3

#define AV_JSON_CAP     224

static volatile uint8_t s_state = AV_ST_IDLE;
static volatile uint8_t s_iter  = 0;
static volatile uint8_t s_rc    = AV_RC_NONE;
static volatile uint8_t s_aligned = 0;
static float s_lastPhi = 0.0f, s_lastPsi = 0.0f, s_lastAx = 0.0f;
static char  s_json[AV_JSON_CAP] = "\0";

/* 启动参数 */
static char    s_base[48] = "";
static uint8_t s_gainPct  = AV_DEFAULT_GAIN_PCT;
static uint8_t s_maxIter  = AV_MAX_ITER;
static uint8_t s_flags    = 0;

/* 标定参数 (NVS 持久化) */
static int8_t  s_signPhi = 1, s_signPsi = -1, s_signAx = 1;
static uint8_t s_betaDeg = 20, s_speedPct = 100;

/* 单步 */
static uint8_t s_stepWhich = AV_STEP_ALL;
static float   s_stepPhi = 0.0f, s_stepPsi = 0.0f, s_stepAx = 0.0f;

/* 运行时 */
static uint32_t s_tTotal = 0, s_tState = 0, s_tPoll = 0;
static uint8_t  s_roundBase = 0;
static uint8_t  s_httpFail = 0;
static uint8_t  s_alignedHits = 0;
static uint8_t  s_measureRetry = 0;
static bool     s_camStarted = false;

static Preferences s_prefs;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* ==================== 日志 ==================== */

#define AV_LOG(fmt, ...)  do { Serial.printf("[AV] " fmt "\n", ##__VA_ARGS__); } while (0)

/* 同时推一条到 App 日志通道, 便于面板同步 */
static void avLogBoth(const char* line) {
    Serial.printf("[AV] %s\n", line);
    WebServer_SendLog(line);
}

/* ==================== 参数持久化 ==================== */

static void loadTune(void) {
    if (!s_prefs.begin("wallbot-av", false)) {
        AV_LOG("nvs open fail, use defaults");
        return;
    }
    s_signPhi  = (int8_t)s_prefs.getChar("sPhi", 1);
    s_signPsi  = (int8_t)s_prefs.getChar("sPsi", -1);
    s_signAx   = (int8_t)s_prefs.getChar("sAx", 1);
    s_betaDeg  = (uint8_t)s_prefs.getUChar("beta", 20);
    s_speedPct = (uint8_t)s_prefs.getUChar("spd", 100);
    s_prefs.end();
    if (s_signPhi != -1) s_signPhi = 1;
    if (s_signPsi != -1) s_signPsi = 1;
    if (s_signAx  != -1) s_signAx  = 1;
    if (s_betaDeg < 10 || s_betaDeg > 40) s_betaDeg = 20;
    if (s_speedPct < 20 || s_speedPct > 100) s_speedPct = 100;
}

static void saveTune(void) {
    if (!s_prefs.begin("wallbot-av", false)) return;
    s_prefs.putChar("sPhi", s_signPhi);
    s_prefs.putChar("sPsi", s_signPsi);
    s_prefs.putChar("sAx",  s_signAx);
    s_prefs.putUChar("beta", s_betaDeg);
    s_prefs.putUChar("spd",  s_speedPct);
    s_prefs.end();
}

/* ==================== JSON 解析 ==================== */

/* 找 "key" 后的数值; 成功返回 true */
static bool jsonNum(const char* body, const char* key, float* out) {
    const char* p = strstr(body, key);
    if (!p) return false;
    p += strlen(key);
    while (*p == ' ' || *p == ':' || *p == '\"') p++;
    char* end = NULL;
    double v = strtod(p, &end);
    if (end == p) return false;
    *out = (float)v;
    return true;
}

static bool jsonBool(const char* body, const char* key, bool* out) {
    const char* p = strstr(body, key);
    if (!p) return false;
    p += strlen(key);
    while (*p == ' ' || *p == ':' || *p == '\"') p++;
    if (strncmp(p, "true", 4) == 0)  { *out = true;  return true; }
    if (strncmp(p, "false", 5) == 0) { *out = false; return true; }
    return false;
}

/* 取 "reason":"xxx" 的值到 out (最多 cap-1) */
static bool jsonStr(const char* body, const char* key, char* out, size_t cap) {
    const char* p = strstr(body, key);
    if (!p) return false;
    p += strlen(key);
    while (*p == ' ' || *p == ':') p++;
    if (*p != '\"') return false;
    p++;
    const char* q = strchr(p, '\"');
    if (!q) return false;
    size_t n = (size_t)(q - p);
    if (n >= cap) n = cap - 1;
    memcpy(out, p, n);
    out[n] = '\0';
    return true;
}

/* ==================== HTTP ==================== */

static void buildBase(const uint8_t ip[4], char* out, size_t cap) {
    snprintf(out, cap, "http://%u.%u.%u.%u", (unsigned)ip[0], (unsigned)ip[1],
             (unsigned)ip[2], (unsigned)ip[3]);
}

/* GET <base><path>, 结果写入 out; 返回 true 表示拿到 200 */
static bool httpGet(const char* base, const char* path, char* out, size_t cap,
                    int* codeOut, uint32_t* dtMsOut) {
    char url[96];
    snprintf(url, sizeof(url), "%s%s", base, path);
    HTTPClient http;
    http.setTimeout(AV_HTTP_TIMEOUT_MS);
    http.setConnectTimeout(AV_HTTP_TIMEOUT_MS);
    if (!http.begin(url)) {
        if (codeOut) *codeOut = -1;
        return false;
    }
    uint32_t t0 = millis();
    int code = http.GET();
    uint32_t dt = millis() - t0;
    if (dtMsOut) *dtMsOut = dt;
    if (codeOut) *codeOut = code;
    if (code != 200) {
        http.end();
        return false;
    }
    String body = http.getString();
    http.end();
    size_t n = body.length();
    if (n >= cap) n = cap - 1;
    memcpy(out, body.c_str(), n);
    out[n] = '\0';
    return true;
}

/* ==================== 状态迁移辅助 ==================== */

static void enterState(uint8_t st) {
    s_state = st;
    s_tState = millis();
}

static void finish(uint8_t st, uint8_t rc) {
    s_rc = rc;
    s_state = st;
    s_tState = millis();
    AV_LOG("finish state=%u rc=%u iter=%u", (unsigned)st, (unsigned)rc, (unsigned)s_iter);
    WebServer_SendLog("[AV] 闭环结束");
    if ((s_flags & AV_FLAG_STOP_CAM) && s_camStarted) {
        char ignored[8];
        httpGet(s_base, "/align/stop", ignored, sizeof(ignored), NULL, NULL);
        s_camStarted = false;
        AV_LOG("cam stop sent");
    }
    /* 测试模式仍活跃则退出, 让子机回手动 */
    if (TestMode_IsActive()) TestMode_Stop();
}

static bool isTerminal(uint8_t st) {
    return st == AV_ST_CONVERGED || st == AV_ST_TIMEOUT || st == AV_ST_FAIL_CAM ||
           st == AV_ST_FAIL_TEST || st == AV_ST_STOPPED;
}

/* 把视觉失败 reason 映射成 AV_RC_* 并重试或终止 */
static void handleMeasureFail(const char* reason) {
    if (strcmp(reason, "no_ref") == 0) {
        avLogBoth("视觉未记参考位, 请在 K230 页面点「记参考位」");
        finish(AV_ST_FAIL_CAM, AV_RC_NO_REF);
    } else if (strcmp(reason, "not_running") == 0) {
        avLogBoth("K230 摄像头未运行, 请在页面点「开始」");
        finish(AV_ST_FAIL_CAM, AV_RC_NOT_RUNNING);
    } else if (strcmp(reason, "no_tag") == 0 || strcmp(reason, "need_two_tags") == 0) {
        s_measureRetry++;
        AV_LOG("measure fail reason=%s retry=%u", reason, (unsigned)s_measureRetry);
        if (s_measureRetry >= 3) {
            avLogBoth("连续未识别到双码, 放弃");
            finish(AV_ST_FAIL_CAM, strcmp(reason, "no_tag") == 0 ? AV_RC_NO_TAG : AV_RC_NEED_TWO_TAGS);
        }
    } else {
        s_httpFail++;
        AV_LOG("measure fail reason=%s httpFail=%u", reason, (unsigned)s_httpFail);
        if (s_httpFail >= AV_HTTP_FAIL_MAX) finish(AV_ST_FAIL_CAM, AV_RC_HTTP_FAIL);
    }
}

/* ==================== 单轮命令下发 ==================== */

static float clampf(float v, float lim) {
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

static void sendAdjRound(void) {
    float phi = s_lastPhi, psi = s_lastPsi, ax = s_lastAx;

    /* 单步: 未选中分量清零 */
    if (s_flags & AV_FLAG_SINGLE_STEP) {
        if (s_stepWhich != AV_STEP_PHI) phi = 0.0f;
        if (s_stepWhich != AV_STEP_PSI) psi = 0.0f;
        if (s_stepWhich != AV_STEP_AX)  ax  = 0.0f;
        if (s_stepWhich == AV_STEP_PHI) { phi = s_stepPhi; psi = 0.0f; ax = 0.0f; }
        if (s_stepWhich == AV_STEP_PSI) { psi = s_stepPsi; phi = 0.0f; ax = 0.0f; }
        if (s_stepWhich == AV_STEP_AX)  { ax  = s_stepAx;  phi = 0.0f; psi = 0.0f; }
    }

    /* 收敛兜底: 原始偏差(未缩放)全部进容差, 阈值不随增益变化。
       正常由 MEASURE 的 K230 aligned(4°/20mm) 连击先收敛, 这里是兜底 */
    if (!(s_flags & AV_FLAG_SINGLE_STEP) &&
        fabsf(phi) <= AV_RAW_TOL_PHI &&
        fabsf(psi) <= AV_RAW_TOL_PSI &&
        fabsf(ax)  <= AV_RAW_TOL_AX) {
        avLogBoth("原始偏差已进容差, 收敛");
        finish(AV_ST_CONVERGED, AV_RC_DONE);
        return;
    }

    float gain = (float)s_gainPct / 100.0f;
    float cPsi = clampf(psi * (float)s_signPsi * gain, AV_CMD_MAX_PSI);
    float cPhi = clampf(phi * (float)s_signPhi * gain, AV_CMD_MAX_PHI);
    float cAx  = clampf(ax  * (float)s_signAx  * gain, AV_CMD_MAX_AX);

    if (!(s_flags & AV_FLAG_SINGLE_STEP)) {
        /* 只纠超出容差的轴: 进容差的轴清零, 防止小误差被最小命令放大 */
        if (fabsf(phi) <= AV_RAW_TOL_PHI) cPhi = 0.0f;
        if (fabsf(psi) <= AV_RAW_TOL_PSI) cPsi = 0.0f;
        if (fabsf(ax)  <= AV_RAW_TOL_AX)  cAx  = 0.0f;
        /* 增益过小导致命令低于子机最小可动量: 抬到最小量, 防止空转烧轮次 */
        if (cPhi != 0.0f && fabsf(cPhi) < AV_CMD_MIN_PHI)
            cPhi = (cPhi > 0.0f) ? AV_CMD_MIN_PHI : -AV_CMD_MIN_PHI;
        if (cPsi != 0.0f && fabsf(cPsi) < AV_CMD_MIN_PSI)
            cPsi = (cPsi > 0.0f) ? AV_CMD_MIN_PSI : -AV_CMD_MIN_PSI;
        if (cAx != 0.0f && fabsf(cAx) < AV_CMD_MIN_AX)
            cAx  = (cAx  > 0.0f) ? AV_CMD_MIN_AX  : -AV_CMD_MIN_AX;
    }

    AV_LOG("it=%u cmd phi=%.2f psi=%.2f ax=%.1f (raw %.2f/%.2f/%.1f gain=%.2f)",
           (unsigned)(s_iter + 1), (double)cPhi, (double)cPsi, (double)cAx,
           (double)phi, (double)psi, (double)ax, (double)gain);

    if (cPhi == 0.0f && cPsi == 0.0f && cAx == 0.0f) {
        avLogBoth("命令全在死区, 视为已对齐");
        finish(AV_ST_CONVERGED, AV_RC_DONE);
        return;
    }

    if (s_flags & AV_FLAG_DRY_RUN) {
        char lb[80];
        snprintf(lb, sizeof(lb), "[AV] 干跑: 本应下发 ADJ,%.1f,%.1f,%.1f",
                 (double)cPhi, (double)cPsi, (double)cAx);
        avLogBoth(lb);
        finish(AV_ST_CONVERGED, AV_RC_DONE);
        return;
    }

    if (TestMode_GetState() != TM_STATE_READY) {
        AV_LOG("SendAdj rejected, test state=%u", (unsigned)TestMode_GetState());
        finish(AV_ST_FAIL_TEST, AV_RC_FAIL_TEST);
        return;
    }

    s_roundBase = TestMode_GetRound();
    TestMode_SendAdj(cPhi, cPsi, cAx);
    enterState(AV_ST_WAIT_DONE);
}

/* ==================== 状态机 ==================== */

static void avStep(void) {
    uint32_t now = millis();

    if (s_state == AV_ST_IDLE || isTerminal(s_state)) return;

    if (now - s_tTotal > AV_TOTAL_TIMEOUT_MS) {
        avLogBoth("闭环总超时");
        finish(AV_ST_TIMEOUT, AV_RC_TOTAL_TIMEOUT);
        return;
    }

    switch (s_state) {
    case AV_ST_CAM_START: {
        if (!(s_flags & AV_FLAG_START_CAM)) {
            AV_LOG("skip cam start (flag off)");
            TestMode_Start();
            enterState(AV_ST_TEST_START);
            break;
        }
        if (now - s_tState > AV_CAM_START_WAIT_MS) {
            avLogBoth("K230 /align/start 超时");
            finish(AV_ST_FAIL_CAM, AV_RC_HTTP_FAIL);
            break;
        }
        char body[96];
        int code = 0;
        if (httpGet(s_base, "/align/start", body, sizeof(body), &code, NULL)) {
            bool ok = false;
            jsonBool(body, "\"ok\"", &ok);
            AV_LOG("cam start -> %s", body);
            if (ok) {
                s_camStarted = true;
                TestMode_Start();
                enterState(AV_ST_TEST_START);
            } else {
                finish(AV_ST_FAIL_CAM, AV_RC_HTTP_FAIL);
            }
        } else {
            AV_LOG("cam start http code=%d, retry", code);
        }
        break;
    }

    case AV_ST_TEST_START: {
        uint8_t st = TestMode_GetState();
        if (st == TM_STATE_READY) {
            AV_LOG("child ready (round=%u)", (unsigned)TestMode_GetRound());
            avLogBoth("子机就绪, 开始视觉测量");
            s_tPoll = 0;
            enterState(AV_ST_MEASURE);
        } else if (now - s_tState > AV_TEST_READY_WAIT_MS) {
            avLogBoth("子机 TSTB 超时");
            finish(AV_ST_FAIL_TEST, AV_RC_FAIL_TEST);
        }
        break;
    }

    case AV_ST_MEASURE: {
        if (s_tPoll != 0 && (now - s_tPoll) < AV_POLL_MS) break;
        s_tPoll = now;

        char body[AV_JSON_CAP];
        int code = 0;
        uint32_t dt = 0;
        if (!httpGet(s_base, "/align/once", body, sizeof(body), &code, &dt)) {
            AV_LOG("once http code=%d", code);
            handleMeasureFail("http");
            break;
        }
        strncpy(s_json, body, sizeof(s_json) - 1);
        s_json[sizeof(s_json) - 1] = '\0';
        AV_LOG("once %ums %s", (unsigned)dt, body);
        WebServer_PushAlignJson(body);

        bool ok = false;
        if (!jsonBool(body, "\"ok\"", &ok) || !ok) {
            char reason[24] = "unknown";
            jsonStr(body, "\"reason\"", reason, sizeof(reason));
            handleMeasureFail(reason);
            break;
        }

        float phi = 0, psi = 0, ax = 0;
        bool aligned = false;
        jsonNum(body, "\"phi_deg\"", &phi);
        jsonNum(body, "\"psi_deg\"", &psi);
        jsonNum(body, "\"axial_mm\"", &ax);
        jsonBool(body, "\"aligned\"", &aligned);

        s_lastPhi = phi;
        s_lastPsi = psi;
        s_lastAx  = ax;
        s_aligned = aligned ? 1 : 0;
        s_httpFail = 0;
        s_measureRetry = 0;

        AV_LOG("it=%u meas phi=%.2f psi=%.2f ax=%.1f aligned=%d",
               (unsigned)(s_iter + 1), (double)phi, (double)psi, (double)ax,
               aligned ? 1 : 0);

        if (aligned) {
            s_alignedHits++;
            if (s_alignedHits >= AV_ALIGNED_HITS) {
                avLogBoth("视觉判定已对齐, 闭环完成");
                finish(AV_ST_CONVERGED, AV_RC_DONE);
                break;
            }
            AV_LOG("aligned hit %u/%u, 复测", (unsigned)s_alignedHits, (unsigned)AV_ALIGNED_HITS);
            break;   /* 保持 MEASURE, 下个轮询再测 */
        }
        s_alignedHits = 0;

        if (s_flags & AV_FLAG_SINGLE_STEP) {
            sendAdjRound();
            break;
        }
        sendAdjRound();
        break;
    }

    case AV_ST_WAIT_DONE: {
        uint8_t r = TestMode_GetRound();
        if (r != s_roundBase) {
            AV_LOG("round %u done (%.1fs)", (unsigned)r, (double)(now - s_tState) / 1000.0);
            s_iter++;
            if (s_flags & AV_FLAG_SINGLE_STEP) {
                avLogBoth("单步完成");
                finish(AV_ST_CONVERGED, AV_RC_DONE);
                break;
            }
            /* 轮次上限是安全阀(防振荡/不收敛死循环), 不是正常终止条件;
               正常终止靠 MEASURE 的收敛判据 */
            if (s_iter >= s_maxIter) {
                avLogBoth("达轮次安全上限, 强制结束");
                finish(AV_ST_TIMEOUT, AV_RC_ITER_MAX);
                break;
            }
            s_tPoll = 0;
            enterState(AV_ST_MEASURE);
            break;
        }
        if (!TestMode_IsActive()) {
            avLogBoth("子机测试状态丢失");
            finish(AV_ST_FAIL_TEST, AV_RC_FAIL_TEST);
            break;
        }
        if (now - s_tState > AV_DONE_TIMEOUT_MS) {
            avLogBoth("等 DONE 超时");
            finish(AV_ST_TIMEOUT, AV_RC_TOTAL_TIMEOUT);
        }
        break;
    }

    default:
        break;
    }
}

/* ==================== 对外接口 ==================== */

void AlignVision_Init(void) {
    s_state = AV_ST_IDLE;
    s_iter = 0;
    s_rc = AV_RC_NONE;
    s_json[0] = '\0';
    loadTune();
    Serial.printf("[AV] init tune sign=%+d/%+d/%+d beta=%u spd=%u%%\n",
                  s_signPhi, s_signPsi, s_signAx, s_betaDeg, s_speedPct);
}

bool AlignVision_Start(const uint8_t ip[4], uint8_t gainPct, uint8_t maxIter, uint8_t flags) {
    if (AutoMode_IsActive()) {
        AV_LOG("start rejected: AutoMode active");
        return false;
    }
    if (s_state != AV_ST_IDLE && !isTerminal(s_state)) {
        AV_LOG("start rejected: already active state=%u", (unsigned)s_state);
        return false;
    }
    buildBase(ip, s_base, sizeof(s_base));
    s_gainPct = (gainPct == 0) ? AV_DEFAULT_GAIN_PCT : gainPct;
    if (s_gainPct < 5)   s_gainPct = 5;
    if (s_gainPct > 100) s_gainPct = 100;
    s_maxIter = (maxIter == 0) ? AV_MAX_ITER : maxIter;
    if (s_maxIter < 1) s_maxIter = 1;
    if (s_maxIter > AV_SAFETY_MAX_ITER) s_maxIter = AV_SAFETY_MAX_ITER;
    s_flags = flags;
    if (flags & AV_FLAG_SINGLE_STEP) s_flags &= ~AV_FLAG_DRY_RUN;   /* 单步不用干跑 */

    s_iter = 0;
    s_rc = AV_RC_NONE;
    s_alignedHits = 0;
    s_httpFail = 0;
    s_measureRetry = 0;
    s_camStarted = false;
    s_lastPhi = s_lastPsi = s_lastAx = 0.0f;
    s_tTotal = millis();

    AV_LOG("start %s gain=%u%% iter<=%u flags=0x%02X sign=%+d/%+d/%+d",
           s_base, (unsigned)s_gainPct, (unsigned)s_maxIter, (unsigned)s_flags,
           s_signPhi, s_signPsi, s_signAx);
    WebServer_SendLog("[AV] 真实视觉闭环启动");

    /* 若测试模式残留, 先退出 */
    if (TestMode_IsActive()) TestMode_Stop();

    enterState(AV_ST_CAM_START);
    return true;
}

void AlignVision_Stop(void) {
    if (s_state == AV_ST_IDLE || isTerminal(s_state)) {
        s_state = AV_ST_IDLE;
        return;
    }
    AV_LOG("user stop");
    finish(AV_ST_STOPPED, AV_RC_USER_STOP);
}

bool AlignVision_IsActive(void) {
    return !(s_state == AV_ST_IDLE || isTerminal(s_state));
}

void AlignVision_SetTune(int8_t signPhi, int8_t signPsi, int8_t signAx,
                         uint8_t betaDeg, uint8_t speedPct) {
    s_signPhi = (signPhi < 0) ? -1 : 1;
    s_signPsi = (signPsi < 0) ? -1 : 1;
    s_signAx  = (signAx  < 0) ? -1 : 1;
    if (betaDeg >= 10 && betaDeg <= 40) s_betaDeg = betaDeg;
    if (speedPct >= 20 && speedPct <= 100) s_speedPct = speedPct;
    saveTune();
    AV_LOG("tune sign=%+d/%+d/%+d beta=%u spd=%u%% (saved)",
           s_signPhi, s_signPsi, s_signAx, s_betaDeg, s_speedPct);
}

void AlignVision_GetTune(int8_t* signPhi, int8_t* signPsi, int8_t* signAx,
                         uint8_t* betaDeg, uint8_t* speedPct) {
    if (signPhi) *signPhi = s_signPhi;
    if (signPsi) *signPsi = s_signPsi;
    if (signAx)  *signAx  = s_signAx;
    if (betaDeg) *betaDeg = s_betaDeg;
    if (speedPct) *speedPct = s_speedPct;
}

bool AlignVision_Step(uint8_t which, float phi, float psi, float ax) {
    if (which < AV_STEP_PSI || which > AV_STEP_ALL) return false;
    s_stepWhich = which;
    s_stepPhi = phi;
    s_stepPsi = psi;
    s_stepAx  = ax;
    AV_LOG("step which=%u phi=%.2f psi=%.2f ax=%.1f",
           (unsigned)which, (double)phi, (double)psi, (double)ax);
    return true;
}

bool AlignVision_CamCtl(uint8_t action, const uint8_t ip[4]) {
    char base[48];
    buildBase(ip, base, sizeof(base));
    const char* path = NULL;
    switch (action) {
    case AV_CAM_STATUS: path = "/align/status"; break;
    case AV_CAM_START:  path = "/align/start";  break;
    case AV_CAM_STOP:   path = "/align/stop";   break;
    case AV_CAM_REF:    path = "/align/ref";    break;
    case AV_CAM_ONCE:   path = "/align/once";   break;
    case AV_CAM_CLRREF: path = "/align/ref?clear=1"; break;
    default: return false;
    }
    char body[AV_JSON_CAP];
    int code = 0;
    uint32_t dt = 0;
    bool ok = httpGet(base, path, body, sizeof(body), &code, &dt);
    AV_LOG("camctl %s -> code=%d %ums %s", path, code, (unsigned)dt, ok ? body : "(fail)");
    if (ok) {
        char lb[180];
        snprintf(lb, sizeof(lb), "[AV] %s %s", path, body);
        WebServer_SendLog(lb);
        if (action == AV_CAM_ONCE) {
            strncpy(s_json, body, sizeof(s_json) - 1);
            s_json[sizeof(s_json) - 1] = '\0';
            WebServer_PushAlignJson(body);
        }
    }
    return ok;
}

void AlignVision_GetStatus(uint8_t out[AV_STATUS_LEN]) {
    out[0] = s_state;
    out[1] = s_iter;
    out[2] = s_rc;
    int16_t p = (int16_t)lroundf(s_lastPhi * 100.0f);
    int16_t q = (int16_t)lroundf(s_lastPsi * 100.0f);
    int16_t a = (int16_t)lroundf(s_lastAx);
    memcpy(&out[3], &p, 2);
    memcpy(&out[5], &q, 2);
    memcpy(&out[7], &a, 2);
    out[9] = s_aligned;
    out[10] = s_maxIter;
}

const char* AlignVision_GetLastJson(void) {
    return s_json;
}

/* ==================== FreeRTOS 任务 ==================== */

void AlignVision_Task(void* pvParameters) {
    (void)pvParameters;
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(AV_TASK_PERIOD_MS);
    /* 0x88 只在状态/轮次/结果变化时推送 (App 端 _onAlignState 的触发源) */
    uint8_t lastSt = 0xFF, lastIter = 0xFF, lastRc = 0xFF;
    AV_LOG("task started (period=%dms)", AV_TASK_PERIOD_MS);
    while (1) {
        avStep();
        if (s_state != lastSt || s_iter != lastIter || s_rc != lastRc) {
            lastSt = s_state;
            lastIter = s_iter;
            lastRc = s_rc;
            WebServer_PushAlignState();
        }
        vTaskDelayUntil(&last, period);
    }
}
