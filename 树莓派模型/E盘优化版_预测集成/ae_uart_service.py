#!/usr/bin/env python3
"""常驻 AE 串口服务: 等子机 CAP → 采集 20s → 预测 → 回传 R 行.

接线(机载直连, 无 USB-TTL):
  子机 PA9(TX)  → Pi Pin10 / GPIO15 (RX)
  子机 PA10(RX) ← Pi Pin8  / GPIO14 (TX)
  共地 + 子机 5V 给 Pi 供电

协议(ASCII, \\n 结尾):
  子机→Pi: CAP
  Pi→子机: R,cm=<f|,>,near=<n|,>,pd=<0|1|,>,st=<code>
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

HERE = Path(__file__).resolve().parent
PREDICTOR_DIR = HERE / "predictor_10_20"
PREDICT_JS = PREDICTOR_DIR / "live_prediction_10.20.js"
DATA_DIR = Path(os.environ.get("AE_DATA_DIR", HERE / "data_storage"))
MAIN_PY = HERE / "main.py"
SERIAL_PORT = os.environ.get("AE_UART_PORT", "/dev/serial0")
BAUD = int(os.environ.get("AE_UART_BAUD", "115200"))
NODE_BIN = os.environ.get("NODE_BIN") or (
    str(PREDICTOR_DIR / "node24" / "bin" / "node")
    if (PREDICTOR_DIR / "node24" / "bin" / "node").is_file()
    else (shutil.which("node") or "node")
)

_busy = threading.Lock()
_ser_lock = threading.Lock()


def _log(msg: str) -> None:
    print(f"[AE-UART] {msg}", flush=True)


def _open_serial():
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "需要 pyserial: pip3 install pyserial"
        ) from exc
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.2)
    # 丢掉上电噪声
    time.sleep(0.1)
    ser.reset_input_buffer()
    return ser


def _write_line(ser, line: str) -> None:
    payload = (line if line.endswith("\n") else line + "\n").encode("ascii", "ignore")
    with _ser_lock:
        ser.write(payload)
        ser.flush()
    _log(f"TX {line.strip()}")


def _latest_pridb(after_mtime: float) -> Optional[Path]:
    if not DATA_DIR.is_dir():
        return None
    best = None
    best_m = after_mtime
    for p in DATA_DIR.glob("*.pridb"):
        try:
            m = p.stat().st_mtime
        except OSError:
            continue
        if m >= best_m and (best is None or m > best_m):
            best, best_m = p, m
    return best


def _status_code(status: str, has_pd) -> str:
    if status in (
        "near_source_point",
        "near_10cm_or_near_field",
        "near_10cm_boundary",
        "within_calibration_range",
        "near_50cm_boundary",
        "beyond_50cm_outside_calibration",
    ):
        return "ok"
    if status in ("insufficient_events", "collecting", "waiting_for_pridb", "waiting_for_tradb"):
        return "few"
    if status in ("incompatible_acquisition",):
        return "bad"
    if status in ("below_noise_or_beyond_50cm",) or has_pd is False:
        return "nopd"
    if status in ("temporarily_unavailable",):
        return "busy"
    return "err"


def _format_result(obj: dict) -> str:
    res = obj.get("result") or {}
    cm = res.get("distanceCm")
    near = res.get("nearestReferenceDistanceCm")
    pd = res.get("hasPartialDischarge")
    st = _status_code(str(obj.get("status") or ""), pd)
    cm_s = "" if cm is None else f"{float(cm):.1f}"
    near_s = "" if near is None else str(int(near))
    if pd is True:
        pd_s = "1"
    elif pd is False:
        pd_s = "0"
    else:
        pd_s = ""
    return f"R,cm={cm_s},near={near_s},pd={pd_s},st={st}"


def _run_capture() -> bool:
    if not MAIN_PY.is_file():
        _log(f"找不到 {MAIN_PY}")
        return False
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["AE_DATA_DIR"] = str(DATA_DIR)
    _log(f"开始采集 main.py … data={DATA_DIR}")
    # 与网页启动一致: -u 无缓冲, 日志进 journal; 绝对数据目录避免写丢
    proc = subprocess.run(
        [sys.executable, "-u", str(MAIN_PY)],
        cwd=str(HERE),
        env=env,
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
    )
    if proc.stdout:
        for line in proc.stdout.strip().splitlines()[-40:]:
            _log(f"main| {line}")
    if proc.stderr:
        for line in proc.stderr.strip().splitlines()[-20:]:
            _log(f"main! {line}")
    _log(f"采集结束 code={proc.returncode}")
    try:
        files = sorted(DATA_DIR.glob("AE_Data_*.pridb"), key=lambda p: p.stat().st_mtime, reverse=True)
        if files:
            newest = files[0]
            _log(f"最新 pridb={newest.name} size={newest.stat().st_size}")
        else:
            _log("data_storage 内仍无 AE_Data_*.pridb")
    except OSError as exc:
        _log(f"列举数据目录失败: {exc}")
    try:
        os.sync()
    except Exception:
        pass
    return proc.returncode == 0


def _run_predict(pridb: Path) -> dict:
    if not PREDICT_JS.is_file():
        return {
            "status": "prediction_unavailable",
            "result": {"distanceCm": None, "nearestReferenceDistanceCm": None, "hasPartialDischarge": None},
        }
    proc = subprocess.run(
        [NODE_BIN, str(PREDICT_JS), str(pridb)],
        cwd=str(PREDICTOR_DIR),
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        timeout=30,
        check=False,
    )
    if proc.returncode != 0:
        _log(f"预测失败: {proc.stderr[-300:]}")
        return {
            "status": "prediction_read_error",
            "result": {"distanceCm": None, "nearestReferenceDistanceCm": None, "hasPartialDischarge": None},
        }
    try:
        return json.loads(proc.stdout)
    except json.JSONDecodeError:
        _log(f"预测 JSON 无效: {proc.stdout[:200]}")
        return {
            "status": "prediction_read_error",
            "result": {"distanceCm": None, "nearestReferenceDistanceCm": None, "hasPartialDischarge": None},
        }


def _handle_cap(ser) -> None:
    if not _busy.acquire(blocking=False):
        _write_line(ser, "R,cm=,near=,pd=,st=busy")
        return
    try:
        _write_line(ser, "OK CAP")
        t0 = time.time()
        ok = _run_capture()
        if not ok:
            _write_line(ser, "R,cm=,near=,pd=,st=err")
            return
        pridb = _latest_pridb(t0 - 1.0)
        if pridb is None:
            _write_line(ser, "R,cm=,near=,pd=,st=few")
            return
        # 等 tradb 落盘片刻
        tradb = pridb.with_suffix(".tradb")
        for _ in range(20):
            if tradb.is_file() and tradb.stat().st_size > 0:
                break
            time.sleep(0.1)
        pred = _run_predict(pridb)
        line = _format_result(pred)
        # 多发两次, 提高子机在抬升/视觉阶段抽到的概率
        _write_line(ser, line)
        time.sleep(0.05)
        _write_line(ser, line)
    finally:
        _busy.release()


def main() -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    _log(f"常驻启动 port={SERIAL_PORT} baud={BAUD}")
    _log(f"predictor={PREDICT_JS} node={NODE_BIN}")
    ser = _open_serial()
    buf = ""
    while True:
        try:
            chunk = ser.read(64)
            if chunk:
                buf += chunk.decode("ascii", "ignore")
                while True:
                    nl = buf.find("\n")
                    if nl < 0:
                        # 也接受裸 \r
                        cr = buf.find("\r")
                        if cr < 0:
                            break
                        nl = cr
                    line = buf[:nl].strip()
                    buf = buf[nl + 1 :]
                    if not line:
                        continue
                    _log(f"RX {line}")
                    if line.upper() == "CAP":
                        threading.Thread(target=_handle_cap, args=(ser,), daemon=True).start()
                    elif line.upper() in ("PING", "?"):
                        _write_line(ser, "OK PING")
            else:
                time.sleep(0.02)
        except KeyboardInterrupt:
            _log("退出")
            break
        except Exception as exc:
            _log(f"串口异常: {exc}")
            time.sleep(1.0)
            try:
                ser.close()
            except Exception:
                pass
            try:
                ser = _open_serial()
                buf = ""
            except Exception as exc2:
                _log(f"重开串口失败: {exc2}")
                time.sleep(2.0)


if __name__ == "__main__":
    main()
