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
    CHILD_CMD:     0x20,
    COOP_START:    0x19,
    COOP_SIMPLE:   0x1A,
    GRIPPER:       0x1B,
    AVOID_RUN:     0x1C,
    COOP_ABORT:    0x1D,
    ALIGN_TUNE:    0x24,
    TEST_START:    0x30,
    TEST_STOP:     0x32
  };

  /* 协同视觉纠偏: 状态帧 0x88 / 视觉 JSON 0x89
   * [0] 阶段: 0-6=CV; 210-227=协同细阶段; 200=协同中; 201=纠偏测试
   * [2] 告警: 0正常 1=NRF补发中 2=超时 3=记参考失败 4=测量失败 5=轮次上限 6=已中止 */
  var ALIGN_STATE_CMD = 0x88;
  var ALIGN_JSON_CMD  = 0x89;
  var ALIGN_STATE_LEN = 11;
  /* AE 距离 0x8A: [cm×10 i16][near u8][pd u8][st u8][seq u8] */
  var AE_DIST_CMD = 0x8A;
  var AE_DIST_LEN = 6;
  /* 与 Pi st= / 子机 Ae_StCode / 模型 status 话术对齐 */
  var AE_ST_TEXT = {
    0: '标定范围内',
    1: '有效事件不足',
    2: '采样条件不匹配',
    3: '预测失败',
    4: '服务忙',
    5: '未检出局放',
    6: '近场约0–10cm',
    7: '超出可显示范围'
  };
  var ALIGN_STATE_TEXT = {
    0: '空闲', 1: '记参考中', 2: '测量中', 3: '参考已就绪',
    4: '参考失败', 5: '测量完成', 6: '测量失败',
    200: '协同进行中', 201: '纠偏测试中',
    210: 'PB9下压', 211: '打开夹持器', 212: '发送COOP',
    213: '等REDY①(风机)', 214: 'PB9抬起', 215: '记参考位',
    216: '发送GOON①', 217: '子机作业中', 218: '视觉纠偏',
    219: '等子机纠偏步', 220: '等DONE', 221: 'PB9下压夹紧',
    222: '预夹紧', 223: '发送LOCK', 224: '等ACKD',
    225: '夹持器夹紧', 226: '抬起完成', 227: '发送STOP'
  };
  var ALIGN_ALERT_TEXT = {
    0: '', 1: 'NRF发送失败, 正在补发…', 2: '等待超时, 已中止',
    3: '记参考失败', 4: '视觉测量失败', 5: '纠偏轮次达上限',
    6: '协同已中止'
  };

  var CAN_SUB = { ENABLE: 0, DISABLE: 1, SET_ZERO: 2 };

  var STATE_CMD = 0x81;
  var STATE_LEN = 32;

  /* 回执帧 0x82: payload = [原CMD][CODE]  CODE: 0=OK 1=参数错 2=CRC错 3=未知命令 */
  var ACK_CMD = 0x82;
  var ACK_CODE_TEXT = { 0: '成功', 1: '参数错误', 2: 'CRC错误', 3: '未知命令' };

  /* 固件日志帧 0x83: payload = 文本字符串 (len 字节) */
  var LOG_CMD = 0x83;

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

  /* "a.b.c.d" -> 4 字节; 非法/为空返回 null */
  function ipToBytes(ip) {
    var oct = String(ip || '').trim().split('.');
    if (oct.length !== 4) return null;
    var p = new Uint8Array(4);
    for (var i = 0; i < 4; i++) {
      var v = parseInt(oct[i], 10);
      if (isNaN(v) || v < 0 || v > 255) return null;
      p[i] = v;
    }
    return p;
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

  /* 0x88: [state][iter][alert][phi i16×100][psi i16×100][ax i16][aligned][maxIter] */
  function parseAlignState(frame) {
    var f = parseFrame(frame);
    if (!f || f.cmd !== ALIGN_STATE_CMD || f.payload.length < ALIGN_STATE_LEN) return null;
    var p = f.payload;
    var dv = new DataView(p.buffer, p.byteOffset, p.byteLength);
    var alert = p[2];
    return {
      state: p[0],
      iter: p[1],
      alert: alert,
      alertText: ALIGN_ALERT_TEXT[alert] || (alert ? ('告警' + alert) : ''),
      phi: dv.getInt16(3, true) / 100,
      psi: dv.getInt16(5, true) / 100,
      ax: dv.getInt16(7, true),
      aligned: !!p[9],
      maxIter: p[10],
      stateText: ALIGN_STATE_TEXT[p[0]] || ('状态' + p[0])
    };
  }

  /* 0x89: 视觉原始 JSON 文本 */
  function parseAlignJson(frame) {
    var f = parseFrame(frame);
    if (!f || f.cmd !== ALIGN_JSON_CMD) return null;
    var text = '';
    for (var i = 0; i < f.payload.length; i++) text += String.fromCharCode(f.payload[i]);
    return text;
  }

  /* 0x8A: AE 距离结果 */
  function parseAeDistance(frame) {
    var f = parseFrame(frame);
    if (!f || f.cmd !== AE_DIST_CMD || f.payload.length < AE_DIST_LEN) return null;
    var p = f.payload;
    var dv = new DataView(p.buffer, p.byteOffset, p.byteLength);
    var cmx = dv.getInt16(0, true);
    var near = p[2];
    var pd = p[3];
    var st = p[4];
    return {
      cm: cmx === 0x7FFF ? null : cmx / 10,
      nearCm: near === 0xFF ? null : near,
      pd: pd === 0xFF ? null : !!pd,
      status: st,
      statusText: AE_ST_TEXT[st] || ('st' + st),
      seq: p[5]
    };
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
    ALIGN_STATE_CMD: ALIGN_STATE_CMD,
    ALIGN_JSON_CMD: ALIGN_JSON_CMD,
    ALIGN_STATE_TEXT: ALIGN_STATE_TEXT,
    parseAlignState: parseAlignState,
    parseAlignJson: parseAlignJson,
    AE_DIST_CMD: AE_DIST_CMD,
    parseAeDistance: parseAeDistance,
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
    /* 启动协同: 可带视觉板 ip ("a.b.c.d"), 母机注入 CoopVision 并 NVS 持久化;
       无 ip 时母机用上次保存的地址 */
    coopStart: function (ip) {
      var p = ipToBytes(ip);
      return buildFrame(CMD.COOP_START, p);
    },
    coopAbort: function () { return buildFrame(CMD.COOP_ABORT, null); },
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
    childCmd: function (speed, spin, key) {
      return buildFrame(CMD.CHILD_CMD, new Uint8Array([
        speed & 0xFF, spin & 0xFF, key & 0xFF
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
    alignEstop: function () { return buildFrame(CMD.ALIGN_ESTOP, null); },

    /* 视觉纠偏标定: 符号/β/速度 (0x24) */
    alignTune: function (signPhi, signPsi, signAx, betaDeg, speedPct) {
      var i8 = function (v) { return (v < 0 ? 256 + (v & 0xFF) : v) & 0xFF; };
      return buildFrame(CMD.ALIGN_TUNE, new Uint8Array([
        i8(signPhi), i8(signPsi), i8(signAx), betaDeg & 0xFF, speedPct & 0xFF
      ]));
    },
    /* 独立纠偏测试: 0x30 启动(可带 K230 ip), 0x32 停止 */
    testStart: function (ip) {
      var p = ipToBytes(ip);
      return buildFrame(CMD.TEST_START, p);
    },
    testStop: function () { return buildFrame(CMD.TEST_STOP, null); }
  };

  global.Protocol = Protocol;
  if (typeof module !== 'undefined' && module.exports) module.exports = Protocol;
})(typeof window !== 'undefined' ? window : globalThis);