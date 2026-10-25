#!/home/z/toolchain/venv-mpy/bin/python3
"""烧录 K230 CanMV: 本地 py -> 板端 /sdcard/main.py (不依赖 CanMV IDE)

用法:
    python3 k230/flash_k230.py [本地文件] [板端路径]
默认:
    python3 k230/flash_k230.py k230/dual_tag_align_web.py /sdcard/main.py

原理: 串口 Ctrl-C 打断 main.py -> Ctrl-A 进 raw REPL -> 分块 base64 写入 -> 软复位
"""
import base64
import glob
import os
import sys
import time

import serial

CHUNK = 384            # 每块原始字节数 (base64 后约 512 字符)
BAUD = 115200


def find_port():
    for pat in ('/dev/serial/by-id/*Kendryte*', '/dev/serial/by-id/*CanMV*'):
        hits = glob.glob(pat)
        if hits:
            return os.path.realpath(hits[0])
    for p in ('/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1'):
        if os.path.exists(p):
            return p
    raise SystemExit('未找到 K230 串口 (检查 USB 线/是否已枚举)')


def read_until(ser, token, timeout=8.0):
    t0 = time.time()
    buf = b''
    while time.time() - t0 < timeout:
        data = ser.read(4096)
        if data:
            buf += data
            if token in buf:
                return buf
        else:
            time.sleep(0.01)
    return buf


def enter_raw_repl(ser):
    for i in range(40):
        ser.write(b'\x03')
        ser.flush()
        time.sleep(0.08)
        data = ser.read(8192)
        if data and b'>>>' in data:
            break
    else:
        raise SystemExit('无法中断到 REPL (Ctrl-C 无效, 或串口被占用)')
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b'\x01')          # Ctrl-A: raw REPL
    ser.flush()
    data = read_until(ser, b'raw REPL; CTRL-B to exit', 5.0)
    if b'raw REPL' not in data:
        raise SystemExit('进入 raw REPL 失败: %r' % data[-200:])


def exec_raw(ser, code, timeout=10.0):
    ser.write(code.encode('utf-8') + b'\x04')
    ser.flush()
    data = read_until(ser, b'\x04>', timeout)
    if not data.startswith(b'OK'):
        raise SystemExit('执行失败: %r' % data[:300])
    body = data[2:]
    idx = body.find(b'\x04')
    stdout = body[:idx] if idx >= 0 else body
    if b'Traceback' in body or b'Error:' in body:
        raise SystemExit('板端报错: %r' % body[:500])
    return stdout


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else 'k230/dual_tag_align_web.py'
    dst = sys.argv[2] if len(sys.argv) > 2 else '/sdcard/main.py'
    data = open(src, 'rb').read()
    port = find_port()
    print('端口 %s' % port)
    print('烧录 %s (%d B) -> %s' % (src, len(data), dst))

    ser = serial.Serial(port, BAUD, timeout=0.2)
    # DTR/RTS 不要拉低: 拉低会让 CanMV 卡住, 连 Ctrl-C 都不响应
    try:
        ser.dtr = True
        ser.rts = True
    except Exception:
        pass
    try:
        enter_raw_repl(ser)
        exec_raw(ser, 'import os, ubinascii')
        exec_raw(ser, "try:\n os.remove('%s.bak')\nexcept Exception:\n pass" % dst)
        exec_raw(ser, "try:\n os.rename('%s', '%s.bak')\nexcept Exception:\n pass" % (dst, dst))
        exec_raw(ser, "f=open('%s','wb')" % dst)

        done = 0
        for off in range(0, len(data), CHUNK):
            chunk = data[off:off + CHUNK]
            b64 = base64.b64encode(chunk).decode('ascii')
            exec_raw(ser, "f.write(ubinascii.a2b_base64('%s'))" % b64, timeout=8.0)
            done += len(chunk)
            if done % (CHUNK * 20) < CHUNK or done == len(data):
                print('  %d / %d (%.0f%%)' % (done, len(data), done * 100.0 / len(data)))

        exec_raw(ser, 'f.close()')
        out = exec_raw(ser, "print(os.stat('%s')[6])" % dst)
        print('板端文件大小: %s' % out.decode('utf-8', 'ignore').strip())

        ser.write(b'\x02')      # Ctrl-B: 退出 raw REPL
        ser.flush()
        time.sleep(0.3)
        ser.reset_input_buffer()
        ser.write(b'\x04')      # Ctrl-D: 软复位, 跑新的 main.py
        ser.flush()
        print('已软复位, 观察串口日志:')
        t0 = time.time()
        while time.time() - t0 < 12.0:
            data = ser.read(4096)
            if data:
                sys.stdout.write(data.decode('utf-8', 'ignore'))
                sys.stdout.flush()
    finally:
        ser.close()


if __name__ == '__main__':
    main()
