/*
 * protocol.js - 母机二进制协议编解码
 * 帧格式: AA 55 CMD LEN [PAYLOAD] CRC16_LE
 * CRC16: Modbus 多项式 (初值 0xFFFF, 多项式 0xA001)
 */
(function (global) {
  'use strict';

  var CMD = {
    SERVO_ONE:     0x01,
    SERVO_ALL:     0x02,
    PUMP:          0x03,
    CAN_ANGLE:     0x04,
    CAN_CMD:       0x05,
    AUTO_ALIGN:    0x09,
    ALIGN_ESTOP:   0x0A,
    OBSTACLE:      0x11,
    GAIT_RUN:      0x12,
    STEP:          0x13,
    U_TURN:        0x14,
    JOYPAD:        0x18,
    COOP_START:    0x19,
    COOP_SIMPLE:   0x1A,
    GRIPPER:       0x1B,
    AVOID_RUN:     0x1C,
    COOP_ABORT:    0x1D,
    TEST_START:    0x30,
    TEST_ADJ:      0x31,
    TEST_STOP:     0x32
  };

  var CAN_SUB = { ENABLE: 0, DISABLE: 1, SET_ZERO: 2 };

  var STATE_CMD = 0x81;
  var STATE_LEN = 32;

  /* 回执帧 0x82: payload = [原CMD][CODE]  CODE: 0=OK 1=参数错 2=CRC错 3=未知命令 */
  var ACK_CMD = 0x82;
  var ACK_CODE_TEXT = { 0: '成功', 1: '参数错误', 2: 'CRC错误', 3: '未知命令' };

  /* 固件日志帧 0x83: payload = 文本字符串 (len 字节) */
  var LOG_CMD = 0x83;
  var TEST_TELEM_CMD = 0x86;
  var TEST_STATE_CMD = 0x87;

  function crc16(bytes) {
    var crc = 0xFFFF;
    for (var i = 0; i < bytes.length; i++) {
      crc ^= bytes[i];
      for (var j = 0; j < 8; j++) {
        crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
      }
    }
    return crc & 0xFFFF;
  }

  function buildFrame(cmd, payload) {
    var len = payload ? payload.length : 0;
    var buf = new Uint8Array(6 + len);
    buf[0] = 0xAA;
    buf[1] = 0x55;
    buf[2] = cmd & 0xFF;
    buf[3] = len & 0xFF;
    if (len) buf.set(payload, 4);
    var crc = crc16(buf.subarray(0, 4 + len));
    buf[4 + len] = crc & 0xFF;
    buf[5 + len] = (crc >> 8) & 0xFF;
    return buf;
  }

  function u16le(value) {
    var v = value & 0xFFFF;
    return [v & 0xFF, (v >> 8) & 0xFF];
  }

  function parseFrame(bytes) {
    if (!bytes || bytes.length <6) return null;
    if (bytes[0] !== 0xAA || bytes[1] !== 0x55) return null;
    var cmd = bytes[2];
    var len = bytes[3];
    if (bytes.length < len + 6) return null;
    var crcRecv = bytes[4 + len] | (bytes[5 + len] << 8);
    if (crc16(bytes.subarray(0, 4 + len)) !== crcRecv) return null;
    return { cmd: cmd, payload: bytes.subarray(4, 4 + len) };
  }

  /* 状态帧 0x81: 32 字节 payload (见固件 web_server.cpp:WebServer_PushLaser)
     0-3 舵机 m1/m2/m3/pb9, 4 泵位, 5-8 CAN1 弧度, 9-12 CAN2 弧度,
     13-16 IMU roll, 17 IMU 在线, 18-21 子机 IP, 22-25 地面标定滚转, 26 纠偏模式, 27-30 夹持器角度, 31 夹持器在线 */
  function parseState(frame) {
    var f = parseFrame(frame);
    if (!f || f.cmd !== STATE_CMD || f.payload.length < STATE_LEN) return null;
    var p = f.payload;
    var dv = new DataView(p.buffer, p.byteOffset, p.byteLength);
    return {
      servo:   { m1: p[0], m2: p[1], m3: p[2], pb9: p[3] },
      pump:    { front: !!(p[4] & 1), rear: !!(p[4] & 2) },
      can:     { m1: dv.getFloat32(5, true), m2: dv.getFloat32(9, true) },
      imu:     { roll: dv.getFloat32(13, true), online: !!p[17], ground: dv.getFloat32(22, true) },
      laser:   { front: dv.getUint16(18, true), online: !!p[20] },
      corrMode: p[26],
      gripper: dv.getFloat32(27, true),
      gripperOnline: !!p[31]
    };
  }

  /* 解析回执帧 0x82: 返回 { cmd, code, ok, text } */
  function parseAck(frame) {
    var f = parseFrame(frame);
    if (!f || f.cmd !== ACK_CMD || f.payload.length < 2) return null;
    var cmd = f.payload[0];
    var code = f.payload[1];
    return {
      cmd: cmd,
      code: code,
      ok: code === 0,
      text: ACK_CODE_TEXT[code] || ('未知(' + code + ')')
    };
  }

  /* 解析固件日志帧 0x83: 返回字符串, 不是则返回 null */
  function parseLog(frame) {
    var f = parseFrame(frame);
    if (!f || f.cmd !== LOG_CMD) return null;
    var text = '';
    for (var i = 0; i < f.payload.length; i++) text += String.fromCharCode(f.payload[i]);
    return text;
  }

  /* 0x86: 子机轮子运动遥测 'J' */
  function parseTestTelem(frame) {
    var f = parseFrame(frame);
    if (!f || f.cmd !== TEST_TELEM_CMD || f.payload.length < 11) return null;
    var p = f.payload;
    var dv = new DataView(p.buffer, p.byteOffset, p.byteLength);
    return {
      stage: p[1],
      dir: (p[2] & 0x80) ? (p[2] - 256) : p[2],
      aMm: dv.getInt16(3, true),
      bMm: dv.getInt16(5, true),
      targetMm: dv.getInt16(7, true),
      pct: p[9],
      rc: p[10]
    };
  }

  /* 0x87: 测试状态 [state][round][last_rc] */
  function parseTestState(frame) {
    var f = parseFrame(frame);
    if (!f || f.cmd !== TEST_STATE_CMD || f.payload.length < 3) return null;
    return { state: f.payload[0], round: f.payload[1], lastRc: f.payload[2] };
  }

  function uint8ArrayToHex(bytes) {
    var out = [];
    for (var i = 0; i < bytes.length; i++) {
      out.push((bytes[i] < 16 ? '0' : '') + bytes[i].toString(16));
    }
    return out.join(' ');
  }

  var Protocol = {
    CMD: CMD,
    CAN_SUB: CAN_SUB,
    STATE_CMD: STATE_CMD,
    STATE_LEN: STATE_LEN,
    ACK_CMD: ACK_CMD,
    ACK_CODE_TEXT: ACK_CODE_TEXT,
    LOG_CMD: LOG_CMD,
    parseAck: parseAck,
    parseLog: parseLog,
    crc16: crc16,
    buildFrame: buildFrame,
    parseFrame: parseFrame,
    parseState: parseState,
    TEST_TELEM_CMD: TEST_TELEM_CMD,
    TEST_STATE_CMD: TEST_STATE_CMD,
    parseTestTelem: parseTestTelem,
    parseTestState: parseTestState,
    uint8ArrayToHex: uint8ArrayToHex,

    canCmd: function (motorId, subCmd) {
      return buildFrame(CMD.CAN_CMD, new Uint8Array([motorId & 0xFF, subCmd & 0xFF]));
    },
    step: function (stepMm) {
      var p = new Uint8Array(4);
      new DataView(p.buffer).setFloat32(0, stepMm, true);
      return buildFrame(CMD.STEP, p);
    },
    obstacle: function () { return buildFrame(CMD.OBSTACLE, null); },
    gaitRun: function (val) { return buildFrame(CMD.GAIT_RUN, new Uint8Array([val & 0xFF])); },
    uTurn: function () { return buildFrame(CMD.U_TURN, null); },
    coopStart: function () { return buildFrame(CMD.COOP_START, null); },
    coopAbort: function () { return buildFrame(CMD.COOP_ABORT, null); },
    testStart: function () { return buildFrame(CMD.TEST_START, null); },
    testStop: function () { return buildFrame(CMD.TEST_STOP, null); },
    testAdj: function (phi, psi, ax) {
      var p = new Uint8Array(12);
      var dv = new DataView(p.buffer);
      dv.setFloat32(0, phi, true);
      dv.setFloat32(4, psi, true);
      dv.setFloat32(8, ax, true);
      return buildFrame(CMD.TEST_ADJ, p);
    },
    coopSimple: function () { return buildFrame(CMD.COOP_SIMPLE, null); },
    gripper: function (angleDeg) {
      var a = Math.round(angleDeg * 10);
      return buildFrame(CMD.GRIPPER, new Uint8Array(u16le(a)));
    },
    joypad: function (lh, lv, rh, rv, key) {
      return buildFrame(CMD.JOYPAD, new Uint8Array([
        lh & 0xFF, lv & 0xFF, rh & 0xFF, rv & 0xFF, key & 0xFF
      ]));
    },
    pump: function (id, state) { return buildFrame(CMD.PUMP, new Uint8Array([id & 0xFF, state & 0xFF])); },
    servoOne: function (id, angleDeg, durSec) {
      var a = Math.round(angleDeg * 10), d = Math.round(durSec * 10);
      return buildFrame(CMD.SERVO_ONE, new Uint8Array([id & 0xFF].concat(u16le(a), u16le(d))));
    },
    servoAll: function (m1, m2, m3, durSec) {
      var d = Math.round(durSec * 10);
      return buildFrame(CMD.SERVO_ALL, new Uint8Array(
        u16le(Math.round(m1 * 10)).concat(u16le(Math.round(m2 * 10)), u16le(Math.round(m3 * 10)), u16le(d))
      ));
    },
    canAngle: function (id, angleDeg, durSec) {
      var a = Math.round(angleDeg * 10), d = Math.round(durSec * 10);
      return buildFrame(CMD.CAN_ANGLE, new Uint8Array([id & 0xFF].concat(u16le(a), u16le(d))));
    },
    avoidRun: function (val) { return buildFrame(CMD.AVOID_RUN, new Uint8Array([val & 0xFF])); },
    autoAlign: function () { return buildFrame(CMD.AUTO_ALIGN, null); },
    alignEstop: function () { return buildFrame(CMD.ALIGN_ESTOP, null); }
  };

  global.Protocol = Protocol;
  if (typeof module !== 'undefined' && module.exports) module.exports = Protocol;
})(typeof window !== 'undefined' ? window : globalThis);