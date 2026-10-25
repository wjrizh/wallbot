# dual_tag_align_web.py — CanMV-K230 双码纠偏（WiFi + 浏览器画面 + 手动标参考）
# 对齐视觉烧录固件 camchild_fw：
#   WiFi → 打印 IP → :80 控制页/align API → :81 MJPEG 预览
#   上电默认不跑摄像头；网页「开始」才工作，「结束」停采省电
#   手动「记参考位 / 清除 / 测一次」，无自动设参考
#
# ★ 使用前改顶部 WIFI_* / AP_* 占位符

print("[ALIGN-WEB] script start")  # cold-boot AP retry v2

import time
import math
import os
import gc
import sys
import socket

print("[ALIGN-WEB] import media...")
from media.sensor import *
from media.display import *
from media.media import *
print("[ALIGN-WEB] media imported")

try:
    import network
except Exception as e:
    network = None
    print("[ALIGN-WEB] import network failed:", e)

# ========== 几何常数（视觉 config.h / tag_utils）==========
DUAL_TAG_SEP_MM = 40.0
TAG_REAL_MM = 22.0
R_TAG_MM = 395.0
PIPE_RADIUS_MM = 250.0
TAG_ID_A = 0
TAG_ID_B = 1
ALIGN_ANGLE_TOL_DEG = 4.0
ALIGN_AXIAL_TOL_MM = 20.0
ALIGN_FRAMES = 2

# ========== WiFi（亚博板载 WiFi）==========
# "ap"  = 板子开热点（手机连板子，地址固定 192.168.4.1）
# "sta" = 板子连平板开的 robot 热点，与母机/App 同网段（默认）
WIFI_MODE = "sta"

# STA 模式：平板热点 robot（与母机 firmware/config.h 的 WIFI_SSID/PASSWORD 一致）
WIFI_SSID = "robot"
WIFI_PASS = "12345678"
# 平板热点若不发 DHCP 或要固定地址，填 ("10.32.7.230", "255.255.255.0", "10.32.7.1")，否则留 None 走 DHCP
STA_STATIC_IP = None

# AP 模式（视觉默认 wallbot-cam / 192.168.4.1）
AP_SSID = "k230-align"
# 本机 CanMV 固件不支持开放热点（空密码会起不来、搜不到 WiFi）
AP_PASS = "12345678"  # WPA，至少 8 位
AP_CHANNEL = 6

WIFI_TIMEOUT_S = 25
# UDP 自动发现（与母机/ESP32 子机同协议）
DISCOVERY_PORT = 48899
# 平板热点看不到已连设备的 IP, 光靠「App 问→板子答」可能因时机错过;
# 板子每 DISCOVERY_ANNOUNCE_MS 主动广播自己的 role:ip, App 常驻扫描必然收到
DISCOVERY_ANNOUNCE_MS = 2000
DISCOVERY_ANNOUNCE_ADDRS = ("255.255.255.255", "10.32.7.255")
DISCOVERY_REQ = "WALLBOT_DISCOVER"
DEVICE_ROLE = "camchild"   # App 侧映射到「子机摄像头」

HTTP_CTRL_PORT = 80
HTTP_STREAM_PORT = 81
JPEG_QUALITY = 30  # 预览流畅优先（检测仍用原分辨率）

# ========== 板端 ==========
DETECT_WIDTH = 640
DETECT_HEIGHT = 480
TAG_FAMILIES = image.TAG36H11
# IDE 虚拟屏会占 CPU；网页预览用 MJPEG，默认关
USE_IDE_PREVIEW = False
CONSENSUS_TIMEOUT_MS = 5000
# 预览目标约 10fps，减轻 JPEG+AP 单线程压力
PREVIEW_INTERVAL_MS = 100
# 预览检码节流: 有 g_ref 时每 PREVIEW_DETECT_INTERVAL_MS 检一次双码,
# 结果写入 g_last_dev/g_last_json, 供 App 轮询 /align/get 实时刷新
PREVIEW_DETECT_INTERVAL_MS = 300
# /align/stop 之后这段时间内, 有新客户端拉 :81/stream 也不自动开摄像头
# (App「结束关摄像头」后会自动重连流, 不设窗口会立刻把摄像头又点亮)
STREAM_DENY_AFTER_STOP_MS = 8000

# 叠图颜色（RGB565 彩色预览）
COL_TAG = (0, 255, 64)
COL_FUSE = (255, 220, 0)
COL_REF = (0, 180, 255)
COL_TEXT = (255, 255, 255)

g_ref = None
g_sensor = None
g_last_obs = None
g_last_dev = None
g_last_json = '{"ok":false,"reason":"never_run"}'
g_last_reason = "never_run"
g_last_detect_ms = 0  # 上次预览检码时刻（节流用）
g_win = []
g_jpeg = None
g_ip = "0.0.0.0"
g_wifi = None
g_boot_ms = 0
g_udp_disc = None
g_disc_recv = 0
g_disc_reply = 0
g_disc_ready = False
g_disc_announce = 0
g_disc_last_ms = 0
g_disc_factory = "none"
g_disc_bind = "none"
g_running = False  # 默认未运行：仅 WiFi+控制页，省电
g_close_stream = False  # 结束时断开 MJPEG 客户端
g_media_inited = False  # MediaManager 只 init 一次
g_start_detail = ""  # 最近一次 start_fail 详情
# 推流 socket（主循环与测一次间隙共用，避免检测时流饿死）
g_stream_client = None
g_stream_hdr_sent = False
# /align/stop 后短暂拒绝「拉流自动开采」, 避免 App 重连立刻把摄像头又打开
g_stream_deny_until_ms = 0


def wrap_deg(d):
    return (d + 180.0) % 360.0 - 180.0


def arc_mm_for(deg):
    return math.radians(deg) * PIPE_RADIUS_MM


def _hypot(dx, dy):
    return math.sqrt(dx * dx + dy * dy)


def delta_report(tag, ref):
    th = ref["theta"]
    dir_ax_x = math.cos(th)
    dir_ax_y = math.sin(th)
    dir_pp_x = -math.sin(th)
    dir_pp_y = math.cos(th)
    landmark_mm = tag["landmark_mm"] if "landmark_mm" in tag else TAG_REAL_MM
    ea = tag["edge_a"]
    if ea < 1e-9:
        ea = 1e-9
    scale = landmark_mm / ea
    du = tag["u"] - ref["u"]
    dv = tag["v"] - ref["v"]
    axial_mm = (du * dir_ax_x + dv * dir_ax_y) * scale
    perp_mm = (du * dir_pp_x + dv * dir_pp_y) * scale
    ratio_p = perp_mm / R_TAG_MM
    if ratio_p > 1.0:
        ratio_p = 1.0
    if ratio_p < -1.0:
        ratio_p = -1.0
    phi_deg = math.degrees(math.asin(ratio_p))
    psi_deg = wrap_deg(math.degrees(tag["theta"] - ref["theta"]))
    ep = tag["edge_p"] if "edge_p" in tag else ea
    ratio = ep / ea
    if ratio > 1.0:
        ratio = 1.0
    if ratio < -1.0:
        ratio = -1.0
    phi_shape_deg = math.degrees(math.acos(ratio))
    arc_mm = arc_mm_for(phi_deg)
    aligned = (
        abs(phi_deg) <= ALIGN_ANGLE_TOL_DEG
        and abs(psi_deg) <= ALIGN_ANGLE_TOL_DEG
        and abs(axial_mm) <= ALIGN_AXIAL_TOL_MM
    )
    return {
        "phi_deg": phi_deg,
        "psi_deg": psi_deg,
        "phi_shape_deg": phi_shape_deg,
        "axial_mm": axial_mm,
        "perp_mm": perp_mm,
        "arc_mm": arc_mm,
        "mm_per_px": scale,
        "aligned": aligned,
    }


def fuse_dual_obs(ta, tb, landmark_mm=None):
    lo, hi = (ta, tb) if ta["id"] <= tb["id"] else (tb, ta)
    dx = hi["u"] - lo["u"]
    dy = hi["v"] - lo["v"]
    dist = _hypot(dx, dy)
    if dist < 1e-6:
        dist = 1e-6
    lm = DUAL_TAG_SEP_MM if landmark_mm is None else landmark_mm
    ha = ta["hamming"] if "hamming" in ta else 0
    hb = tb["hamming"] if "hamming" in tb else 0
    return {
        "id": lo["id"],
        "id_b": hi["id"],
        "u": 0.5 * (lo["u"] + hi["u"]),
        "v": 0.5 * (lo["v"] + hi["v"]),
        "theta": math.atan2(dy, dx),
        "edge_a": dist,
        "edge_p": dist,
        "edge_px": dist,
        "landmark_mm": lm,
        "hamming": max(ha, hb),
    }


def obs_from_apriltag(tag):
    tid = int(tag.id())
    u = float(tag.cx())
    v = float(tag.cy())
    hamming = 0
    try:
        hamming = int(tag.hamming())
    except Exception:
        pass
    corners = None
    try:
        corners = tag.corners()
    except Exception:
        corners = None
    if corners is not None and len(corners) >= 4:
        c0x, c0y = float(corners[0][0]), float(corners[0][1])
        c1x, c1y = float(corners[1][0]), float(corners[1][1])
        c2x, c2y = float(corners[2][0]), float(corners[2][1])
        c3x, c3y = float(corners[3][0]), float(corners[3][1])
        theta = math.atan2(c1y - c0y, c1x - c0x)
        e01 = _hypot(c1x - c0x, c1y - c0y)
        e12 = _hypot(c2x - c1x, c2y - c1y)
        e23 = _hypot(c3x - c2x, c3y - c2y)
        e30 = _hypot(c0x - c3x, c0y - c3y)
        edge_a = 0.5 * (e01 + e23)
        edge_p = 0.5 * (e12 + e30)
    else:
        theta = float(tag.rotation())
        try:
            r = tag.rect()
            edge_a = float(min(r[2], r[3]))
            edge_p = edge_a
        except Exception:
            edge_a = 1.0
            edge_p = 1.0
    return {
        "id": tid,
        "u": u,
        "v": v,
        "theta": theta,
        "edge_a": edge_a,
        "edge_p": edge_p,
        "edge_px": edge_a,
        "hamming": hamming,
    }


def detect_dual_from_tags(tag_list):
    by_id = {}
    for raw in tag_list:
        obs = obs_from_apriltag(raw)
        prev = by_id[obs["id"]] if obs["id"] in by_id else None
        if prev is None or obs["edge_a"] > prev["edge_a"]:
            by_id[obs["id"]] = obs
    have_a = TAG_ID_A in by_id
    have_b = TAG_ID_B in by_id
    n = int(have_a) + int(have_b)
    if n == 0:
        return None, "no_tag", tag_list
    if n == 1:
        return None, "need_two_tags", tag_list
    return fuse_dual_obs(by_id[TAG_ID_A], by_id[TAG_ID_B]), "ok", tag_list


def align_json_ok(dev, obs, dt_ms=0):
    aligned_s = "true" if dev["aligned"] else "false"
    oid = obs["id"] if "id" in obs else 0
    ham = obs["hamming"] if "hamming" in obs else 0
    ea = obs["edge_a"] if "edge_a" in obs else 0.0
    return (
        '{"ok":true,"dt_ms":%d,"id":%d,"hamming":%d,'
        '"phi_deg":%.2f,"psi_deg":%.2f,"phi_shape_deg":%.2f,'
        '"axial_mm":%.1f,"arc_mm":%.1f,"perp_mm":%.1f,'
        '"edge_px":%.1f,"aligned":%s,"angle_tol":%.1f}'
    ) % (
        int(dt_ms), int(oid), int(ham),
        dev["phi_deg"], dev["psi_deg"], dev["phi_shape_deg"],
        dev["axial_mm"], dev["arc_mm"], dev["perp_mm"],
        ea, aligned_s, ALIGN_ANGLE_TOL_DEG,
    )


def align_json_fail(reason, dt_ms=0, edge_px=0.0):
    return '{"ok":false,"reason":"%s","dt_ms":%d,"edge_px":%.1f}' % (
        reason, int(dt_ms), float(edge_px)
    )


def reset_consensus_window():
    global g_win
    g_win = []


def push_consensus(obs):
    global g_win
    g_win.append(obs)
    if len(g_win) > ALIGN_FRAMES:
        g_win = g_win[-ALIGN_FRAMES:]
    if len(g_win) < ALIGN_FRAMES:
        return None
    n = float(len(g_win))
    su = sv = st = se = 0.0
    ham = 0
    oid = g_win[0]["id"]
    oid_b = g_win[0]["id_b"] if "id_b" in g_win[0] else TAG_ID_B
    lm = g_win[0]["landmark_mm"] if "landmark_mm" in g_win[0] else DUAL_TAG_SEP_MM
    for o in g_win:
        su += o["u"]
        sv += o["v"]
        st += o["theta"]
        se += o["edge_a"]
        h = o["hamming"] if "hamming" in o else 0
        if h > ham:
            ham = h
    return {
        "id": oid,
        "id_b": oid_b,
        "u": su / n,
        "v": sv / n,
        "theta": st / n,
        "edge_a": se / n,
        "edge_p": se / n,
        "edge_px": se / n,
        "landmark_mm": lm,
        "hamming": ham,
    }


def set_ref_from_obs(obs):
    global g_ref, g_last_reason
    g_ref = {
        "u": obs["u"],
        "v": obs["v"],
        "theta": obs["theta"],
        "edge_a": obs["edge_a"],
    }
    g_last_reason = "ok"
    print("[ALIGN-WEB] ref set u=%.1f v=%.1f theta=%.2fdeg edge=%.1f" % (
        g_ref["u"], g_ref["v"], math.degrees(g_ref["theta"]), g_ref["edge_a"]))


def clear_ref():
    global g_ref, g_last_reason, g_last_dev, g_last_json
    g_ref = None
    g_last_dev = None
    g_last_reason = "ref_cleared"
    g_last_json = '{"ok":false,"reason":"ref_cleared"}'
    print("[ALIGN-WEB] ref cleared")


# ---------- WiFi ----------
def _ap_force_active(ap, attempt):
    """Cold-boot safe: cycle inactive->active; never swallow errors silently."""
    try:
        if hasattr(ap, "active"):
            try:
                ap.active(False)
            except Exception as e:
                print("[ALIGN-WEB] AP active(0) warn:", e)
            time.sleep(0.4)
            ap.active(True)
            time.sleep(0.6)
            print("[ALIGN-WEB] AP active=%s (try %d)" % (ap.active(), attempt))
            return True
    except Exception as e:
        print("[ALIGN-WEB] AP active fail try=%d:" % attempt, type(e).__name__, e)
    return False


def _ap_try_config(ap):
    """Try several CanMV/MicroPython AP config shapes (WPA, with channel)."""
    configured = False
    last_err = None
    ch = AP_CHANNEL
    if AP_PASS and len(AP_PASS) >= 8:
        # Prefer WPA/WPA2 with key; include channel (was defined but unused).
        candidates = [
            {"ssid": AP_SSID, "key": AP_PASS, "channel": ch},
            {"ssid": AP_SSID, "password": AP_PASS, "channel": ch},
            {"ssid": AP_SSID, "key": AP_PASS},
            {"ssid": AP_SSID, "password": AP_PASS},
        ]
        # security constants differ by firmware
        for sec_name in ("AUTH_WPA2_PSK", "SECURITY_WPA2_PSK", "AUTH_WPA_WPA2_PSK"):
            sec = getattr(ap, sec_name, None)
            if sec is None:
                sec = getattr(network, sec_name, None)
            if sec is not None:
                candidates.insert(0, {"ssid": AP_SSID, "key": AP_PASS, "security": sec, "channel": ch})
                candidates.insert(1, {"ssid": AP_SSID, "password": AP_PASS, "security": sec, "channel": ch})
                break
        for kwargs in candidates:
            try:
                ap.config(**kwargs)
                configured = True
                print("[ALIGN-WEB] AP WPA ssid=%s cfg=%s" % (AP_SSID, kwargs))
                break
            except Exception as e:
                last_err = e
                print("[ALIGN-WEB] AP cfg skip:", kwargs, "->", e)
    else:
        # 空密码：先试开放热点；失败则回退固定 WPA（部分固件不支持 OPEN）
        try:
            if hasattr(ap, "SECURITY_OPEN"):
                ap.config(ssid=AP_SSID, security=ap.SECURITY_OPEN, channel=ch)
            else:
                ap.config(ssid=AP_SSID, channel=ch)
            configured = True
            print("[ALIGN-WEB] AP OPEN ssid=%s" % AP_SSID)
        except Exception as e:
            last_err = e
            print("[ALIGN-WEB] AP OPEN fail, fallback WPA:", e)
            fb = "12345678"
            for kwargs in (
                {"ssid": AP_SSID, "key": fb, "channel": ch},
                {"ssid": AP_SSID, "password": fb, "channel": ch},
                {"ssid": AP_SSID, "key": fb},
            ):
                try:
                    ap.config(**kwargs)
                    configured = True
                    print("[ALIGN-WEB] AP WPA fallback ssid=%s pass=%s" % (AP_SSID, fb))
                    break
                except Exception as e2:
                    last_err = e2
    return configured, last_err


def wifi_connect():
    global g_wifi, g_ip
    if network is None:
        raise RuntimeError("network module missing")
    mode = WIFI_MODE.lower().strip()
    print("[ALIGN-WEB] WiFi mode=%s" % mode)
    # Give WiFi chip time after cold power-on (soft reboot usually already warm).
    time.sleep(1.0)
    if mode == "ap":
        try:
            ap_if = network.AP_IF
        except Exception:
            ap_if = 1
        last_err = None
        for attempt in range(1, 4):
            ap = network.WLAN(ap_if)
            print("[ALIGN-WEB] AP bring-up attempt %d/3" % attempt)
            _ap_force_active(ap, attempt)
            configured, last_err = _ap_try_config(ap)
            if not configured:
                print("[ALIGN-WEB] AP config failed try=%d: %s" % (attempt, last_err))
                time.sleep(1.0)
                continue
            # Some firmwares need active AFTER config.
            try:
                if hasattr(ap, "active") and not ap.active():
                    ap.active(True)
                    print("[ALIGN-WEB] AP re-active after config")
            except Exception as e:
                print("[ALIGN-WEB] AP re-active warn:", e)
            # Wait for IP / interface ready
            ok_ip = False
            for _ in range(10):
                time.sleep(0.5)
                try:
                    cfg = ap.ifconfig()
                    ip = cfg[0]
                except Exception as e:
                    print("[ALIGN-WEB] ifconfig wait:", e)
                    continue
                act = True
                try:
                    act = ap.active()
                except Exception:
                    pass
                print("[ALIGN-WEB] AP wait ip=%s active=%s" % (ip, act))
                if act and ip and ip != "0.0.0.0":
                    g_ip = ip
                    g_wifi = ap
                    print("[ALIGN-WEB] AP ssid=%s ip=%s ifconfig=%s" % (AP_SSID, g_ip, cfg))
                    return g_ip
            print("[ALIGN-WEB] AP no usable IP on try %d, retry" % attempt)
            time.sleep(1.0)
        raise RuntimeError("AP bring-up failed after retries: %s" % last_err)
    # STA
    try:
        sta_if = network.STA_IF
    except Exception:
        sta_if = 0
    sta = network.WLAN(sta_if)
    try:
        if hasattr(sta, "active") and not sta.active():
            sta.active(True)
    except Exception:
        pass
    print("[ALIGN-WEB] Connecting STA ssid=%s ..." % WIFI_SSID)
    try:
        sta.connect(WIFI_SSID, WIFI_PASS)
    except TypeError:
        sta.connect(ssid=WIFI_SSID, key=WIFI_PASS)
    if STA_STATIC_IP:
        try:
            sta.ifconfig(STA_STATIC_IP)
            print("[ALIGN-WEB] STA static %s" % (STA_STATIC_IP,))
        except Exception as e:
            print("[ALIGN-WEB] STA static fail, keep DHCP:", e)
    t0 = time.time()
    while True:
        try:
            os.exitpoint()
        except Exception:
            pass
        ip = sta.ifconfig()[0]
        connected = False
        try:
            connected = sta.isconnected()
        except Exception:
            connected = ip != "0.0.0.0"
        if connected and ip != "0.0.0.0":
            g_ip = ip
            g_wifi = sta
            print("[ALIGN-WEB] STA ok ifconfig=%s" % (sta.ifconfig(),))
            return g_ip
        if time.time() - t0 > WIFI_TIMEOUT_S:
            print("")
            print("[ALIGN-WEB] STA timeout; 确认平板已开 %s 热点且密码 %s" % (WIFI_SSID, WIFI_PASS))
            print("[ALIGN-WEB] 若只想直连板子调试, 把文件顶部 WIFI_MODE 改回 \"ap\" 后重启")
            raise RuntimeError("WiFi STA timeout")
        time.sleep(0.5)
        print(".", end="")
    return g_ip


# ---------- UDP 自动发现 (与母机/子机同协议) ----------
# App 广播 "WALLBOT_DISCOVER" 到 :48899，本机回 "WALLBOT:camchild:<ip>"
# 角色用 camchild: K230 就是子机摄像头, App 自动把地址填进子机流与视觉闭环
def make_udp_discovery():
    global g_udp_disc, g_disc_ready, g_disc_factory, g_disc_bind
    s = None
    last_err = None
    factories = (
        ("AF_INET+SOCK_DGRAM",
         lambda: socket.socket(socket.AF_INET, socket.SOCK_DGRAM)),
        ("AF_INET+SOCK_DGRAM+proto",
         lambda: socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)),
        ("no args",
         lambda: socket.socket()),
    )
    for name, factory in factories:
        try:
            s = factory()
            g_disc_factory = name
            print("[ALIGN-WEB] discovery socket via", name)
            break
        except Exception as e:
            last_err = e
            s = None
            print("[ALIGN-WEB] discovery socket try", name, "->",
                  type(e).__name__, e)
    if s is None:
        print("[ALIGN-WEB] discovery socket FAILED:", last_err)
        g_disc_ready = False
        return None

    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except Exception as e:
        print("[ALIGN-WEB] discovery setsockopt warn:", e)
    # 主动广播必须有 SO_BROADCAST, 否则 sendto 255.255.255.255 会被内核拒掉
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except Exception as e:
        print("[ALIGN-WEB] discovery SO_BROADCAST warn:", e)

    bound = False
    bind_targets = []
    try:
        bind_targets.append(("getaddrinfo", socket.getaddrinfo(
            "0.0.0.0", DISCOVERY_PORT)[0][-1]))
    except Exception as e:
        print("[ALIGN-WEB] discovery getaddrinfo warn:", e)
    bind_targets.append(("tuple", ("0.0.0.0", DISCOVERY_PORT)))

    for name, target in bind_targets:
        try:
            s.bind(target)
            g_disc_bind = name + ":" + str(target)
            bound = True
            print("[ALIGN-WEB] discovery bind", target, "via", name)
            break
        except Exception as e:
            print("[ALIGN-WEB] discovery bind try", name, target, "->",
                  type(e).__name__, e)
    if not bound:
        print("[ALIGN-WEB] discovery bind FAILED all targets")
        try:
            s.close()
        except Exception:
            pass
        g_disc_ready = False
        return None

    try:
        s.setblocking(False)
    except Exception as e:
        print("[ALIGN-WEB] discovery setblocking warn:", e)

    g_udp_disc = s
    g_disc_ready = True
    print("[ALIGN-WEB] discovery READY :%d role=%s ip=%s" % (
        DISCOVERY_PORT, DEVICE_ROLE, g_ip))
    return s


def announce_discovery(s):
    """周期性主动广播 role:ip, 不依赖 App 的请求窗口。"""
    global g_disc_announce, g_disc_last_ms
    if not g_disc_ready or s is None:
        return
    now = time.ticks_ms()
    if g_disc_last_ms and time.ticks_diff(now, g_disc_last_ms) < DISCOVERY_ANNOUNCE_MS:
        return
    g_disc_last_ms = now
    reply = "WALLBOT:%s:%s" % (DEVICE_ROLE, g_ip)
    payload = reply.encode("utf-8")
    for dst in DISCOVERY_ANNOUNCE_ADDRS:
        try:
            s.sendto(payload, (dst, DISCOVERY_PORT))
        except Exception as e:
            print("[ALIGN-WEB] announce %s err: %s" % (dst, e))
    g_disc_announce += 1
    if g_disc_announce <= 3 or (g_disc_announce % 30) == 0:
        print("[ALIGN-WEB] announce %s x%d" % (reply, g_disc_announce))


def poll_discovery(s):
    global g_disc_recv, g_disc_reply, g_disc_ready
    if not g_disc_ready or s is None:
        return
    for _ in range(4):
        try:
            data, addr = s.recvfrom(64)
        except Exception:
            return
        g_disc_recv += 1
        try:
            msg = data.decode("utf-8", "ignore").strip()
        except Exception:
            continue
        if not msg.startswith(DISCOVERY_REQ):
            continue
        reply = "WALLBOT:%s:%s" % (DEVICE_ROLE, g_ip)
        try:
            s.sendto(reply.encode("utf-8"), addr)
            g_disc_reply += 1
            if g_disc_reply <= 5 or (g_disc_reply % 10) == 0:
                print("[ALIGN-WEB] discovery reply %s -> %s recv=%d reply=%d" % (
                    reply, addr[0], g_disc_recv, g_disc_reply))
        except Exception as e:
            print("[ALIGN-WEB] discovery reply err:", e)


# ---------- Camera ----------
def json_escape(s):
    t = str(s) if s is not None else ""
    return t.replace("\\", "\\\\").replace('"', '\\"').replace("\r", " ").replace("\n", " ")


def _sensor_apply_format(sensor):
    """reset 之后必须重新 set framesize/pixformat（官方生命周期）。"""
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    try:
        sensor.set_pixformat(Sensor.RGB565)
        print("[ALIGN-WEB] pixformat RGB565")
    except Exception as e:
        print("[ALIGN-WEB] RGB565 fail, fallback GRAYSCALE:", e)
        sensor.set_pixformat(Sensor.GRAYSCALE)


def _sensor_new(force=False):
    if force:
        try:
            return Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT, force=True)
        except TypeError:
            pass
        except Exception as e:
            print("[ALIGN-WEB] Sensor(force=True) fail:", e)
    return Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)


def camera_init(force=False):
    """首次或 force 重建：Sensor +（一次）MediaManager + run。"""
    global g_sensor, g_media_inited
    if force and g_sensor is not None:
        try:
            g_sensor.stop()
        except Exception:
            pass
        g_sensor = None
        time.sleep_ms(80)
    print("[ALIGN-WEB] Sensor(force=%s)..." % force)
    g_sensor = _sensor_new(force=force)
    g_sensor.reset()
    _sensor_apply_format(g_sensor)
    if USE_IDE_PREVIEW:
        Display.init(Display.VIRT, width=DETECT_WIDTH, height=DETECT_HEIGHT, fps=60)
    if not g_media_inited:
        try:
            MediaManager.init()
        except Exception as e:
            # 新固件可能已自动 init / 弃用，失败不阻断
            print("[ALIGN-WEB] MediaManager.init warn:", e)
        g_media_inited = True
    g_sensor.run()
    time.sleep_ms(80)
    print("[ALIGN-WEB] sensor ready %dx%d" % (DETECT_WIDTH, DETECT_HEIGHT))


def camera_pause():
    """结束：只 stop 采流，不拆 MediaManager（二次 deinit/init 易失败）。"""
    global g_sensor
    if g_sensor is not None:
        try:
            g_sensor.stop()
            print("[ALIGN-WEB] sensor stopped (soft)")
        except Exception as e:
            print("[ALIGN-WEB] sensor.stop err:", e)


def camera_resume():
    """再开始：已有 Sensor 则 stop→reset→配置→run；失败则 force 重建。"""
    global g_sensor
    if g_sensor is None:
        camera_init(force=False)
        return
    try:
        try:
            g_sensor.stop()
        except Exception:
            pass
        time.sleep_ms(50)
        g_sensor.reset()
        _sensor_apply_format(g_sensor)
        g_sensor.run()
        time.sleep_ms(80)
        print("[ALIGN-WEB] sensor resumed (reset+cfg+run)")
        return
    except Exception as e:
        print("[ALIGN-WEB] resume fail, force rebuild:", type(e).__name__, e)
        try:
            sys.print_exception(e)
        except Exception:
            pass
        camera_init(force=True)


def camera_deinit():
    """进程退出时才完整拆除。"""
    global g_sensor, g_media_inited
    if g_sensor is not None:
        try:
            g_sensor.stop()
        except Exception:
            pass
    if USE_IDE_PREVIEW:
        try:
            Display.deinit()
        except Exception:
            pass
    try:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    except Exception:
        pass
    time.sleep_ms(100)
    try:
        MediaManager.deinit()
    except Exception:
        pass
    g_sensor = None
    g_media_inited = False


def vision_start():
    """网页「开始」：开采/恢复摄像头，只推实时画面（不检码）。
    返回 (ok, reason, detail)。
    """
    global g_running, g_last_reason, g_jpeg, g_start_detail
    g_start_detail = ""
    if g_running and g_sensor is not None:
        g_last_reason = "already_running"
        print("[ALIGN-WEB] already running")
        return True, "already_running", ""
    try:
        camera_resume()
        # 单帧预热，失败只 warn，不判 start_fail
        try:
            capture_preview(encode_jpeg=False)
        except Exception as e:
            print("[ALIGN-WEB] warmup warn:", type(e).__name__, e)
        g_running = True
        g_last_reason = "running"
        print("[ALIGN-WEB] vision STARTED (preview only; detect on ref/once)")
        return True, "started", ""
    except Exception as e:
        detail = "%s: %s" % (type(e).__name__, e)
        g_start_detail = detail
        print("[ALIGN-WEB] vision start fail:", detail)
        try:
            sys.print_exception(e)
        except Exception:
            pass
        try:
            camera_pause()
        except Exception:
            pass
        g_running = False
        g_last_reason = "start_fail"
        return False, "start_fail", detail


def vision_stop():
    """网页「结束」：先停推流标志，再 soft stop；保留 Sensor 以便再开始。"""
    global g_running, g_last_reason, g_jpeg, g_close_stream, g_last_obs, g_last_dev
    global g_stream_deny_until_ms
    g_stream_deny_until_ms = time.ticks_add(time.ticks_ms(), STREAM_DENY_AFTER_STOP_MS)
    if not g_running and g_sensor is None:
        g_last_reason = "already_stopped"
        print("[ALIGN-WEB] already stopped")
        return True, "already_stopped"
    if not g_running:
        g_last_reason = "already_stopped"
        print("[ALIGN-WEB] already stopped (sensor kept)")
        return True, "already_stopped"
    g_running = False
    g_close_stream = True
    g_jpeg = None
    g_last_obs = None
    # 保留 g_ref / g_sensor，方便下次开始继续测
    g_last_dev = None
    try:
        camera_pause()
    except Exception as e:
        print("[ALIGN-WEB] pause err:", e)
    gc.collect()
    g_last_reason = "stopped"
    print("[ALIGN-WEB] vision STOPPED (soft pause, power save)")
    return True, "stopped"


def img_for_detect(img):
    """彩色帧转灰度再检码；已是灰度则原样返回。"""
    try:
        return img.to_grayscale(copy=True)
    except Exception:
        pass
    try:
        g = img.copy()
        g.to_grayscale()
        return g
    except Exception:
        return img


def img_to_jpeg_bytes(img, quality=JPEG_QUALITY):
    # 兼容亚博/CanMV 多种 JPEG API
    try:
        j = img.to_jpeg(quality=quality)
        try:
            return bytes(j.bytearray())
        except Exception:
            pass
        try:
            return bytes(j)
        except Exception:
            pass
    except Exception:
        pass
    try:
        c = img.compressed(quality=quality)
        return bytes(c)
    except Exception:
        pass
    try:
        img.compress(quality=quality)
        try:
            return bytes(img.bytearray())
        except Exception:
            return bytes(img)
    except Exception:
        pass
    return None


def draw_overlays(img, fused, tags, reason):
    # 仅在「记参考/测一次」检测帧上叠加码框与偏差
    if tags:
        for tag in tags:
            try:
                tid = tag.id()
                if tid == TAG_ID_A or tid == TAG_ID_B:
                    img.draw_rectangle([v for v in tag.rect()], color=COL_TAG)
                    img.draw_cross(tag.cx(), tag.cy(), color=COL_TAG)
            except Exception:
                pass
    if fused is not None:
        try:
            cu = int(fused["u"])
            cv = int(fused["v"])
            img.draw_cross(cu, cv, color=COL_FUSE, size=12)
            dx = int(28 * math.cos(fused["theta"]))
            dy = int(28 * math.sin(fused["theta"]))
            img.draw_line(cu, cv, cu + dx, cv + dy, color=COL_FUSE)
        except Exception:
            pass
    if g_ref is not None:
        try:
            img.draw_cross(int(g_ref["u"]), int(g_ref["v"]), color=COL_REF, size=10)
            img.draw_circle(int(g_ref["u"]), int(g_ref["v"]), 8, color=COL_REF)
        except Exception:
            pass
    line1 = "REF YES" if g_ref else "REF NO (press Set Ref)"
    line2 = reason
    if g_last_dev is not None and g_ref is not None and fused is not None:
        d = g_last_dev
        line2 = "phi%+.1f psi%+.1f ax%+.1f %s" % (
            d["phi_deg"], d["psi_deg"], d["axial_mm"],
            "OK" if d["aligned"] else "NO")
    try:
        img.draw_string(4, 4, line1, color=COL_TEXT, scale=1)
        img.draw_string(4, 18, line2, color=COL_TEXT, scale=1)
    except Exception:
        pass


def capture_preview(encode_jpeg=True):
    """平时推流：采帧 + 节流检码（有 g_ref 时），叠图后 JPEG（省 CPU）。
    检码结果写入 g_last_dev/g_last_json，供 App 轮询 /align/get 实时刷新。
    """
    global g_jpeg, g_last_detect_ms, g_last_obs, g_last_dev, g_last_json, g_last_reason
    img = g_sensor.snapshot()

    detected = False
    fused = None
    tag_list = []
    reason = g_last_reason

    if g_ref is not None:
        now = time.ticks_ms()
        if time.ticks_diff(now, g_last_detect_ms) >= PREVIEW_DETECT_INTERVAL_MS:
            g_last_detect_ms = now
            detected = True
            try:
                gray = img_for_detect(img)
                tags = gray.find_apriltags(families=TAG_FAMILIES)
                fused, reason, tag_list = detect_dual_from_tags(tags)
                if fused is not None:
                    g_last_obs = fused
                    g_last_dev = delta_report(fused, g_ref)
                    g_last_reason = "ok"
                    g_last_json = align_json_ok(g_last_dev, fused, 0)
                else:
                    g_last_reason = reason
                    g_last_json = align_json_fail(reason, 0, 0.0)
            except Exception as e:
                print("[ALIGN-WEB] preview detect err:", type(e).__name__, e)
            try:
                gc.collect()
            except Exception:
                pass

    if detected:
        # 本次检码才画 tag 框/融合点/偏差文字；未检帧只画参考十字，
        # 避免把上一帧结果擦成 no_tag
        draw_overlays(img, fused, tag_list, reason)
    else:
        if g_ref is not None:
            try:
                img.draw_cross(int(g_ref["u"]), int(g_ref["v"]), color=COL_REF, size=10)
                img.draw_circle(int(g_ref["u"]), int(g_ref["v"]), 8, color=COL_REF)
            except Exception:
                pass
        try:
            tip = "LIVE REF" if g_ref else "LIVE"
            img.draw_string(4, 4, tip, color=COL_TEXT, scale=1)
        except Exception:
            pass
    if USE_IDE_PREVIEW:
        try:
            Display.show_image(img)
        except Exception:
            pass
    if encode_jpeg:
        jb = img_to_jpeg_bytes(img)
        if jb:
            g_jpeg = jb
    return img


def capture_detect_update(update_dev=True):
    """记参考/测一次：取帧→灰度检码→叠图→JPEG。返回 (fused, reason, dt_ms, tags)。"""
    global g_last_obs, g_last_dev, g_last_json, g_last_reason, g_jpeg
    t0 = time.ticks_ms()
    img = g_sensor.snapshot()
    gray = img_for_detect(img)
    tags = gray.find_apriltags(families=TAG_FAMILIES)
    fused, reason, tag_list = detect_dual_from_tags(tags)
    dt = time.ticks_diff(time.ticks_ms(), t0)
    if fused is not None:
        g_last_obs = fused
        if update_dev and g_ref is not None:
            g_last_dev = delta_report(fused, g_ref)
            g_last_reason = "ok"
            g_last_json = align_json_ok(g_last_dev, fused, dt)
        else:
            g_last_reason = reason
    else:
        g_last_reason = reason
        if update_dev:
            g_last_json = align_json_fail(reason, dt, 0.0)
    draw_overlays(img, fused, tag_list, reason)
    if USE_IDE_PREVIEW:
        try:
            Display.show_image(img)
        except Exception:
            pass
    jb = img_to_jpeg_bytes(img)
    if jb:
        g_jpeg = jb
    gc.collect()
    return fused, reason, dt, tag_list


def stream_drop():
    """发送失败或结束时丢掉推流客户端。"""
    global g_stream_client, g_stream_hdr_sent
    if g_stream_client is not None:
        try:
            g_stream_client.close()
        except Exception:
            pass
    g_stream_client = None
    g_stream_hdr_sent = False


def pump_stream_frame():
    """把当前 g_jpeg 推一帧（测一次/记参考间隙调用，减轻黑屏）。"""
    global g_stream_client, g_stream_hdr_sent
    if g_stream_client is None or not g_jpeg:
        return
    if not g_stream_hdr_sent:
        try:
            send_mjpeg_headers(g_stream_client)
            g_stream_hdr_sent = True
        except Exception:
            stream_drop()
            return
    if not send_mjpeg_part(g_stream_client, g_jpeg):
        print("[ALIGN-WEB] stream send fail during pump")
        stream_drop()


def sync_consensus(timeout_ms=CONSENSUS_TIMEOUT_MS):
    """对齐视觉 /align/ref|/align/once：累计 ALIGN_FRAMES 帧共识（此处才检码）。
    每检一帧后顺手推 MJPEG，避免检测阻塞把流饿死。
    """
    reset_consensus_window()
    t0 = time.ticks_ms()
    last_reason = "measuring"
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        try:
            os.exitpoint()
        except Exception:
            pass
        fused, reason, dt, _ = capture_detect_update(update_dev=False)
        last_reason = reason
        # 检测帧已写入 g_jpeg，间隙推给浏览器
        try:
            pump_stream_frame()
        except Exception:
            pass
        if fused is not None:
            avg = push_consensus(fused)
            if avg is not None:
                return avg, "ok", time.ticks_diff(time.ticks_ms(), t0)
        else:
            pass
    return None, last_reason, time.ticks_diff(time.ticks_ms(), t0)


# ---------- HTTP ----------
BOUNDARY = "k230alignboundary"
STREAM_CT = "multipart/x-mixed-replace;boundary=" + BOUNDARY


def http_send(sock, status, content_type, body, extra_headers=None):
    if body is None:
        body = b""
    if isinstance(body, str):
        body = body.encode("utf-8")
    hdr = "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %d\r\nConnection: close\r\nCache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\n" % (
        status, content_type, len(body))
    if extra_headers:
        hdr += extra_headers
    hdr += "\r\n"
    try:
        sock.send(hdr.encode("utf-8"))
        if body:
            # 分块发送，避免 RT-Smart 发送超时
            mv = memoryview(body)
            off = 0
            n = len(body)
            while off < n:
                chunk = mv[off:off + 4096]
                sock.send(chunk)
                off += 4096
    except Exception as e:
        print("[ALIGN-WEB] http send err:", e)


def make_listen(port):
    s = socket.socket()
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except Exception:
        pass
    ai = socket.getaddrinfo("0.0.0.0", port)[0][-1]
    s.bind(ai)
    s.listen(3)
    try:
        s.setblocking(False)
    except Exception:
        pass
    print("[ALIGN-WEB] listen :%d" % port)
    return s


def read_http_request(client, timeout_ms=2000):
    client.setblocking(False)
    buf = b""
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        try:
            chunk = client.recv(256)
        except Exception:
            chunk = None
        if chunk:
            buf += chunk
            if b"\r\n\r\n" in buf:
                break
        else:
            time.sleep_ms(10)
        try:
            os.exitpoint()
        except Exception:
            pass
    return buf


def parse_path(req_bytes):
    try:
        line = req_bytes.split(b"\r\n", 1)[0].decode("utf-8", "ignore")
    except Exception:
        return "/", ""
    parts = line.split(" ")
    if len(parts) < 2:
        return "/", ""
    raw = parts[1]
    if "?" in raw:
        path, query = raw.split("?", 1)
    else:
        path, query = raw, ""
    return path, query


PAGE_HTML = """<!DOCTYPE html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>K230 Dual Tag Align</title>
<style>
*{box-sizing:border-box}
body{margin:0;background:#0d0d0f;color:#e8e8ea;font:14px/1.5 system-ui,sans-serif}
header{padding:10px 14px;background:#1a1a1d;border-bottom:1px solid #2c2c30;display:flex;gap:12px;align-items:center;flex-wrap:wrap}
#st{color:#8e8e93;font-size:12px;margin-left:auto}
.wrap{display:flex;flex-wrap:wrap}
.pane{flex:1 1 420px;min-width:280px;padding:10px}
.pane h3{margin:0 0 8px;font-size:12px;color:#8e8e93;text-transform:uppercase}
#pic{width:100%;background:#000;border-radius:8px;display:block;min-height:180px}
table{width:100%;border-collapse:collapse;font:13px monospace}
td{padding:5px 6px;border-bottom:1px solid #232326}
td.k{color:#8e8e93;width:40%}td.v{text-align:right}
.bad{color:#ff6b6b}.good{color:#4ade80}
button{background:#2c2c30;color:#e8e8ea;border:1px solid #3a3a40;border-radius:6px;padding:8px 12px;margin:0 6px 6px 0;cursor:pointer}
button:disabled{opacity:.45}
button.go{background:#166534;border-color:#22c55e}
button.stop{background:#7f1d1d;border-color:#ef4444}
#runHint{margin:0 0 8px;font-size:13px}
</style></head><body>
<header><b>K230 双码纠偏</b><span id=st>连接中…</span></header>
<div class=wrap>
<div class=pane><h3>实时画面 :81/stream</h3>
<img id=pic alt=stream>
</div>
<div class=pane><h3>纠偏 (tag36h11 ID0+1)</h3>
<p id=runHint class=bad>未运行 — 点「开始」后才开摄像头/推流（省电）</p>
<table>
<tr><td class=k>运行</td><td class=v id=f_run>--</td></tr>
<tr><td class=k>状态</td><td class=v id=f_st>--</td></tr>
<tr><td class=k>周向 phi</td><td class=v id=f_phi>--</td></tr>
<tr><td class=k>航向 psi</td><td class=v id=f_psi>--</td></tr>
<tr><td class=k>轴向 mm</td><td class=v id=f_ax>--</td></tr>
<tr><td class=k>弧长 mm</td><td class=v id=f_arc>--</td></tr>
<tr><td class=k>双码距 px</td><td class=v id=f_id>--</td></tr>
<tr><td class=k>耗时</td><td class=v id=f_dt>--</td></tr>
</table>
<div style="margin-top:10px">
<button class=go id=btnStart onclick="doStart()">开始</button>
<button class=stop id=btnStop onclick="doStop()">结束</button>
<button id=btnOnce onclick="measureOnce()">测一次</button>
<button id=btnRef onclick="refCmd('')">记参考位</button>
<button id=btnClr onclick="refCmd('clear=1')">清除参考位</button>
</div>
<p style="color:#8e8e93;font-size:12px">上电默认休眠。点「开始」只看实时画面；摆好码后「记参考位」→「测一次」。测完点「结束」关摄像头省电。</p>
</div></div>
<script>
var BASE=location.protocol+'//'+location.hostname,bAg=0,running=false;
var streamGen=0,reconnectT=null;
function num(v,d){return (v===undefined||v===null)?'--':Number(v).toFixed(d);}
function clearStreamTimers(){if(reconnectT){clearTimeout(reconnectT);reconnectT=null;}}
function refreshStream(delay){
  if(!running)return;
  clearStreamTimers();
  var gen=++streamGen;
  reconnectT=setTimeout(function(){
    if(!running||gen!==streamGen)return;
    var pic=document.getElementById('pic');
    pic.onerror=function(){if(running&&gen===streamGen)refreshStream(700);};
    try{pic.removeAttribute('src');}catch(e){}
    setTimeout(function(){
      if(!running||gen!==streamGen)return;
      pic.src=BASE+':81/stream?'+Date.now();
    },80);
  }, delay==null?400:delay);
}
function setWorkEnabled(on){
  running=!!on;
  ['btnOnce','btnRef','btnClr'].forEach(function(id){var el=document.getElementById(id);if(el)el.disabled=!on;});
  var h=document.getElementById('runHint');
  var fr=document.getElementById('f_run');
  if(on){h.textContent='运行中 — 实时预览；点「记参考位/测一次」才检码';h.className='good';fr.textContent='运行中';fr.className='v good';
    refreshStream(400);}
  else{clearStreamTimers();streamGen++;h.textContent='未运行 — 点「开始」后才开摄像头/推流（省电）';h.className='bad';fr.textContent='未运行';fr.className='v bad';
    var pic=document.getElementById('pic');pic.onerror=null;try{pic.removeAttribute('src');}catch(e){}}
}
function applyAlign(d){var s=document.getElementById('f_st');
if(!d||!d.ok){s.textContent='失败: '+((d&&d.reason)||'?');s.className='v bad';
document.getElementById('f_phi').textContent='--';
document.getElementById('f_psi').textContent='--';
document.getElementById('f_ax').textContent='--';
document.getElementById('f_arc').textContent='--';
document.getElementById('f_id').textContent=(d&&d.edge_px)?num(d.edge_px,1)+' px':'--';
document.getElementById('f_dt').textContent=(d&&d.dt_ms!=null)?num(d.dt_ms,0)+' ms':'--';return;}
s.textContent=d.aligned?'已对准':'偏差中';s.className=d.aligned?'v good':'v bad';
document.getElementById('f_phi').textContent=num(d.phi_deg,2)+' °';
document.getElementById('f_psi').textContent=num(d.psi_deg,2)+' °';
document.getElementById('f_ax').textContent=num(d.axial_mm,1)+' mm';
document.getElementById('f_arc').textContent=num(d.arc_mm,1)+' mm';
document.getElementById('f_id').textContent=num(d.edge_px,1)+' px';
document.getElementById('f_dt').textContent=num(d.dt_ms,0)+' ms';}
function ag(){if(bAg||!running)return;bAg=1;fetch(BASE+'/align/get',{cache:'no-store'})
.then(function(r){return r.json()}).then(function(d){bAg=0;applyAlign(d);})
.catch(function(){bAg=0;});}
function measureOnce(){if(bAg)return;if(!running){alert('请先点「开始」');return;}
bAg=1;var btn=document.getElementById('btnOnce');
if(btn)btn.disabled=true;var s=document.getElementById('f_st');s.textContent='测量中…';s.className='v';
fetch(BASE+'/align/once',{cache:'no-store'}).then(function(r){return r.json()})
.then(function(d){bAg=0;if(btn)btn.disabled=false;applyAlign(d);refreshStream(300);})
.catch(function(){bAg=0;if(btn)btn.disabled=false;s.textContent='请求失败';s.className='v bad';refreshStream(300);});}
function refCmd(q){if(!running){alert('请先点「开始」');return;}
var clearing=q&&q.indexOf('clear=1')>=0;var s=document.getElementById('f_st');
if(!clearing){s.textContent='记参考中…';s.className='v';}
fetch(BASE+'/align/ref'+(q?'?'+q:''),{cache:'no-store'}).then(function(r){return r.json()})
.then(function(d){alert(d.ok?'成功':'失败: '+(d.reason||'?'));
if(clearing&&d.ok){s.textContent='参考已清除';s.className='v';}
else{ag();}refreshStream(300);})
.catch(function(e){alert('请求失败: '+e);refreshStream(300);});}
function doStart(){var s=document.getElementById('f_st');s.textContent='启动中…';s.className='v';
fetch(BASE+'/align/start',{cache:'no-store'}).then(function(r){return r.json()})
.then(function(d){if(d&&d.ok){setWorkEnabled(true);s.textContent='已开始';s.className='v good';st();}
else{var msg=(d&&d.reason)||'?';if(d&&d.detail)msg+=' | '+d.detail;
alert('开始失败: '+msg);s.textContent='开始失败';s.className='v bad';}})
.catch(function(e){alert('请求失败: '+e);});}
function doStop(){fetch(BASE+'/align/stop',{cache:'no-store'}).then(function(r){return r.json()})
.then(function(d){setWorkEnabled(false);document.getElementById('f_st').textContent='已结束(省电)';
document.getElementById('f_st').className='v';st();})
.catch(function(e){alert('请求失败: '+e);});}
function st(){fetch(BASE+'/status',{cache:'no-store'}).then(function(r){return r.text()})
.then(function(t){document.getElementById('st').textContent=t;
var on=t.indexOf('running=yes')>=0;if(on!==running)setWorkEnabled(on);}).catch(function(){});}
setWorkEnabled(false);
st();document.getElementById('f_st').textContent='未运行';
setInterval(st,3000);
</script></body></html>
"""


def status_text():
    up = 0
    try:
        up = int(time.ticks_diff(time.ticks_ms(), g_boot_ms) / 1000)
    except Exception:
        pass
    mode = WIFI_MODE
    ref_s = "yes" if g_ref else "no"
    run_s = "yes" if g_running else "no"
    tip = "运行中" if g_running else "未运行(省电)"
    disc_s = "ready" if g_disc_ready else "off"
    return (
        "mode=%s ip=%s running=%s ref=%s reason=%s up=%us | %s"
        "\ndiscovery: %s recv=%d reply=%d factory=%s bind=%s"
    ) % (
        mode, g_ip, run_s, ref_s, g_last_reason, up, tip,
        disc_s, g_disc_recv, g_disc_reply, g_disc_factory, g_disc_bind,
    )


def handle_control_client(client):
    global g_last_json, g_last_reason, g_last_dev
    req = read_http_request(client)
    if not req:
        try:
            client.close()
        except Exception:
            pass
        return
    path, query = parse_path(req)
    q = query.lower()
    print("[ALIGN-WEB] HTTP", path, query)

    if path == "/" or path == "/index.html":
        http_send(client, "200 OK", "text/html; charset=utf-8", PAGE_HTML)
    elif path == "/status":
        http_send(client, "200 OK", "text/plain; charset=utf-8", status_text())
    elif path == "/align/start":
        ok, reason, detail = vision_start()
        body = '{"ok":%s,"running":%s,"reason":"%s","detail":"%s"}' % (
            "true" if ok else "false",
            "true" if g_running else "false",
            json_escape(reason),
            json_escape(detail),
        )
        http_send(client, "200 OK", "application/json", body)
    elif path == "/align/stop":
        ok, reason = vision_stop()
        body = '{"ok":%s,"running":false,"reason":"%s"}' % (
            "true" if ok else "false", reason)
        http_send(client, "200 OK", "application/json", body)
    elif path == "/align/status":
        run_line = "running: %s (%s)\n" % (
            "yes" if g_running else "no",
            "摄像头已开" if g_running else "未运行-省电",
        )
        if g_ref is not None:
            body = (
                run_line +
                "ref: u=%.1f v=%.1f theta=%.2fdeg edge=%.1fpx\n"
                "last(%s): phi=%.2fdeg psi=%.2fdeg axial=%.1fmm arc=%.1fmm aligned=%d\n"
                "cfg: landmark=%.0fmm R_TAG=%.0fmm pipe_R=%.0fmm tol=%.1fdeg axial_tol=%.0fmm frames=%d\n"
            ) % (
                g_ref["u"], g_ref["v"], math.degrees(g_ref["theta"]), g_ref["edge_a"],
                g_last_reason,
                g_last_dev["phi_deg"] if g_last_dev else 0.0,
                g_last_dev["psi_deg"] if g_last_dev else 0.0,
                g_last_dev["axial_mm"] if g_last_dev else 0.0,
                g_last_dev["arc_mm"] if g_last_dev else 0.0,
                1 if (g_last_dev and g_last_dev["aligned"]) else 0,
                DUAL_TAG_SEP_MM, R_TAG_MM, PIPE_RADIUS_MM,
                ALIGN_ANGLE_TOL_DEG, ALIGN_AXIAL_TOL_MM, ALIGN_FRAMES,
            )
        else:
            body = (
                run_line +
                "ref: none (last=%s)\n"
                "cfg: landmark=%.0fmm R_TAG=%.0fmm pipe_R=%.0fmm tol=%.1fdeg axial_tol=%.0fmm frames=%d\n"
            ) % (
                g_last_reason,
                DUAL_TAG_SEP_MM, R_TAG_MM, PIPE_RADIUS_MM,
                ALIGN_ANGLE_TOL_DEG, ALIGN_AXIAL_TOL_MM, ALIGN_FRAMES,
            )
        http_send(client, "200 OK", "text/plain; charset=utf-8", body)
    elif path == "/align/get":
        if not g_running:
            body = '{"ok":false,"reason":"not_running"}'
        elif g_ref is None:
            body = '{"ok":false,"reason":"no_ref"}'
        else:
            body = g_last_json
        http_send(client, "200 OK", "application/json", body)
    elif path == "/align/ref":
        if not g_running:
            vision_start()
        if not g_running:
            http_send(client, "200 OK", "application/json",
                      '{"ok":false,"reason":"not_running"}')
        elif "clear=1" in q:
            clear_ref()
            http_send(client, "200 OK", "application/json", '{"ok":true,"ref":false}')
        else:
            avg, reason, dt = sync_consensus()
            if avg is None:
                body = '{"ok":false,"reason":"%s"}' % reason
                http_send(client, "200 OK", "application/json", body)
            else:
                set_ref_from_obs(avg)
                # 记参考后偏差必为 0: 立即写入缓存,
                # App 轮询 /align/get 先拿到零偏差, 再被预览检码刷新
                g_last_dev = delta_report(avg, g_ref)
                g_last_reason = "ok"
                g_last_json = align_json_ok(g_last_dev, avg, dt)
                body = (
                    '{"ok":true,"ref":true,"id":%d,"u":%.1f,"v":%.1f,'
                    '"theta_deg":%.2f,"edge_px":%.1f}'
                ) % (
                    int(avg["id"]), avg["u"], avg["v"],
                    math.degrees(avg["theta"]), avg["edge_a"],
                )
                http_send(client, "200 OK", "application/json", body)
    elif path == "/align/once":
        if not g_running:
            vision_start()
        if not g_running:
            http_send(client, "200 OK", "application/json",
                      '{"ok":false,"reason":"not_running"}')
        elif g_ref is None:
            http_send(client, "200 OK", "application/json", '{"ok":false,"reason":"no_ref"}')
        else:
            avg, reason, dt = sync_consensus()
            if avg is None:
                body = align_json_fail(reason, dt, 0.0)
                g_last_json = body
                g_last_reason = reason
                http_send(client, "200 OK", "application/json", body)
            else:
                g_last_dev = delta_report(avg, g_ref)
                g_last_reason = "ok"
                body = align_json_ok(g_last_dev, avg, dt)
                g_last_json = body
                http_send(client, "200 OK", "application/json", body)
    elif path == "/jpg" or path == "/snapshot.jpg":
        if not g_running:
            http_send(client, "503 Service Unavailable", "text/plain", "not_running")
        elif g_jpeg:
            http_send(client, "200 OK", "image/jpeg", g_jpeg)
        else:
            http_send(client, "503 Service Unavailable", "text/plain", "no frame")
    elif path == "/stream":
        # 若误打到 :80/stream，给提示
        http_send(client, "200 OK", "text/plain", "use :%d/stream for MJPEG" % HTTP_STREAM_PORT)
    else:
        http_send(client, "404 Not Found", "text/plain", "not found: " + path)
    try:
        client.close()
    except Exception:
        pass


def send_mjpeg_headers(client):
    hdr = (
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: %s\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n"
    ) % STREAM_CT
    client.send(hdr.encode("utf-8"))


def send_mjpeg_part(client, jpeg_bytes):
    if not jpeg_bytes:
        return False
    part = (
        "\r\n--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
        % (BOUNDARY, len(jpeg_bytes))
    )
    try:
        client.send(part.encode("utf-8"))
        mv = memoryview(jpeg_bytes)
        off = 0
        n = len(jpeg_bytes)
        while off < n:
            client.send(mv[off:off + 4096])
            off += 4096
        return True
    except Exception:
        return False


def try_accept(sock):
    try:
        return sock.accept()
    except Exception:
        return None


def run_servers():
    global g_close_stream, g_stream_client, g_stream_hdr_sent
    ctrl = make_listen(HTTP_CTRL_PORT)
    stream = make_listen(HTTP_STREAM_PORT)
    udp_disc = make_udp_discovery()
    g_stream_client = None
    g_stream_hdr_sent = False
    print("[ALIGN-WEB] Open http://%s:%d/ in browser" % (g_ip, HTTP_CTRL_PORT))
    print("[ALIGN-WEB] MJPEG http://%s:%d/stream (after Start)" % (g_ip, HTTP_STREAM_PORT))
    if AP_PASS and len(AP_PASS) >= 8:
        print("[ALIGN-WEB] AP 时请手机/电脑先连热点 %s，密码 %s，再打开上面地址" % (AP_SSID, AP_PASS))
    else:
        print("[ALIGN-WEB] AP 时请手机/电脑先连热点 %s（无密码），再打开上面地址" % AP_SSID)
    print("[ALIGN-WEB] idle: camera OFF — press Start on web page")
    frame_n = 0
    idle_n = 0
    last_preview_ms = 0
    while True:
        try:
            os.exitpoint()
        except Exception:
            pass

        # 0) UDP 自动发现: 收 App 询问并应答 + 主动广播自己的 role:ip
        poll_discovery(udp_disc)
        announce_discovery(udp_disc)

        # 结束时断开推流客户端
        if g_close_stream:
            g_close_stream = False
            stream_drop()
            print("[ALIGN-WEB] stream closed by stop")

        # 2) 控制口短请求（优先；测一次期间也会 pump 推流）
        acc = try_accept(ctrl)
        if acc:
            c, addr = acc
            print("[ALIGN-WEB] ctrl client", addr)
            try:
                c.settimeout(8)
            except Exception:
                pass
            try:
                handle_control_client(c)
            except Exception as e:
                print("[ALIGN-WEB] ctrl handler err:", type(e).__name__, e)
                try:
                    c.close()
                except Exception:
                    pass

        # 3) 流媒体：先 accept 客户端，再决定是否 JPEG
        if not g_running:
            if g_stream_client is not None:
                stream_drop()
            acc2 = try_accept(stream)
            if acc2:
                sc, addr2 = acc2
                try:
                    sc.settimeout(5)
                except Exception:
                    pass
                try:
                    read_http_request(sc, 1500)
                except Exception:
                    pass
                # App 直接拉 :81/stream 时自动开采, 不需要先调 /align/start
                denied = time.ticks_diff(time.ticks_ms(), g_stream_deny_until_ms) < 0
                if denied:
                    ok, reason, detail = False, "denied_after_stop", ""
                else:
                    ok, reason, detail = vision_start()
                if ok:
                    g_stream_client = sc
                    g_stream_hdr_sent = False
                    print("[ALIGN-WEB] auto start by stream client %s" % (addr2,))
                else:
                    try:
                        http_send(sc, "503 Service Unavailable", "text/plain", "start_fail")
                    except Exception:
                        pass
                    try:
                        sc.close()
                    except Exception:
                        pass
        else:
            if g_stream_client is None:
                acc2 = try_accept(stream)
                if acc2:
                    g_stream_client, addr2 = acc2
                    g_stream_hdr_sent = False
                    print("[ALIGN-WEB] stream client", addr2)
                    try:
                        g_stream_client.settimeout(5)
                    except Exception:
                        pass
                    try:
                        read_http_request(g_stream_client, 1500)
                    except Exception:
                        pass

        # 1) running 时限帧预览；无推流客户端则不编码 JPEG
        new_jpeg = False
        if g_running and g_sensor is not None:
            now = time.ticks_ms()
            due = time.ticks_diff(now, last_preview_ms) >= PREVIEW_INTERVAL_MS
            force = g_stream_client is not None and g_jpeg is None
            if due or force:
                try:
                    need_jpeg = g_stream_client is not None
                    capture_preview(encode_jpeg=need_jpeg)
                    last_preview_ms = now
                    frame_n += 1
                    new_jpeg = need_jpeg and (g_jpeg is not None)
                except Exception as e:
                    print("[ALIGN-WEB] frame err:", type(e).__name__, e)
                    time.sleep_ms(50)
            else:
                time.sleep_ms(10)
        else:
            idle_n += 1
            time.sleep_ms(80)

        # 推 MJPEG：仅在新帧编码后发送；失败则 drop，等浏览器自动重连
        if g_running and g_stream_client is not None and new_jpeg:
            if not g_stream_hdr_sent:
                try:
                    send_mjpeg_headers(g_stream_client)
                    g_stream_hdr_sent = True
                except Exception:
                    stream_drop()
                    continue
            if not send_mjpeg_part(g_stream_client, g_jpeg):
                print("[ALIGN-WEB] stream closed (send fail, await browser reconnect)")
                stream_drop()

        if g_running and (frame_n % 60) == 1:
            print("[ALIGN-WEB] alive frames=%d reason=%s ref=%s jpeg=%s" % (
                frame_n, g_last_reason, "yes" if g_ref else "no",
                str(len(g_jpeg)) if g_jpeg else "0"))
        elif (not g_running) and (idle_n % 50) == 1:
            print("[ALIGN-WEB] idle (power save) reason=%s" % g_last_reason)


def main():
    global g_boot_ms, g_running, g_last_reason
    print("[ALIGN-WEB] main()")
    g_boot_ms = time.ticks_ms()
    g_running = False
    g_last_reason = "idle"
    try:
        os.exitpoint(os.EXITPOINT_ENABLE)
    except Exception:
        pass
    try:
        wifi_connect()
        print("[ALIGN-WEB] WiFi up; camera deferred until /align/start")
        run_servers()
    except KeyboardInterrupt as e:
        print("[ALIGN-WEB] user stop:", e)
    except Exception as e:
        print("[ALIGN-WEB] FATAL:", type(e).__name__, e)
        try:
            sys.print_exception(e)
        except Exception:
            pass
    finally:
        print("[ALIGN-WEB] deinit")
        try:
            vision_stop()
        except Exception:
            try:
                camera_deinit()
            except Exception:
                pass
    print("[ALIGN-WEB] exit")


main()