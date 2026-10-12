const P = require('../www/js/protocol.js');

function assert(cond, msg){ if(!cond){ console.error('FAIL:', msg); process.exitCode=1; } else console.log('PASS:', msg); }

// 1. CRC16 Modbus 标准向量 "123456789" -> 0x4B37
const ascii = new Uint8Array([...'123456789'].map(c=>c.charCodeAt(0)));
assert(P.crc16(ascii) === 0x4B37, 'CRC16 Modbus 标准向量 = 0x4B37, got 0x'+P.crc16(ascii).toString(16));

// 2. CAN 使能帧: AA 55 05 02 01 00 CRC
const canEn = P.canCmd(1, P.CAN_SUB.ENABLE);
assert(canEn[0]===0xAA && canEn[1]===0x55 && canEn[2]===0x05 && canEn[3]===0x02, 'CAN使能帧头');
assert(canEn[4]===1 && canEn[5]===0, 'CAN使能帧负载 id=1 sub=0');
assert(canEn.length===8, 'CAN使能帧长 = 8, got '+canEn.length);
const crcEn = canEn[6]|(canEn[7]<<8);
assert(P.crc16(canEn.subarray(0,6))===crcEn, 'CAN使能帧 CRC 自校验');

// 3. 设零 sub=2
const canZero = P.canCmd(2, P.CAN_SUB.SET_ZERO);
assert(canZero[4]===2 && canZero[5]===2, 'CAN设零 id=2 sub=2');

// 4. 迈步帧: float32 LE step=80 => 00 00 A0 42
const step = P.step(80);
assert(step[2]===0x13 && step[3]===4, '迈步 CMD=0x13 LEN=4');
assert(step[4]===0x00 && step[5]===0x00 && step[6]===0xA0 && step[7]===0x42, '步距80 float32LE = 00 00 A0 42');
assert(step.length===10, '迈步帧长=10');

// 5. 越障帧
const obs = P.obstacle();
assert(obs[2]===0x11 && obs[3]===0 && obs.length===6, '越障 CMD=0x11 LEN=0');

// 6. 协同
assert(P.coopStart()[2]===0x19, '协同 CMD=0x19');
assert(P.coopSimple()[2]===0x1A, '简化 CMD=0x1A');

// 6b. 避障行走 0x1C
const av=P.avoidRun(1);
assert(av[2]===0x1C && av[3]===1 && av[4]===1, '避障开 CMD=0x1C LEN=1 payload=1');
const av0=P.avoidRun(0);
assert(av0[4]===0, '避障停 payload=0');
assert(P.crc16(av.subarray(0,5))===(av[5]|(av[6]<<8)), '避障帧 CRC 自校验');

// 7. 状态帧解析: 构造 firmware 32字节 payload (web_server.cpp:WebServer_PushLaser)
const payload = new Uint8Array(32);
const dv = new DataView(payload.buffer);
payload[0]=30; payload[1]=85; payload[2]=25; payload[3]=160; payload[4]=0x03; // m1 m2 m3 pb9 pump(前后吸)
dv.setFloat32(5, 1.0, true);   // can1 = 1.0 rad
dv.setFloat32(9, -0.5, true);  // can2 = -0.5 rad
dv.setFloat32(13, 2.5, true);  // imu roll
payload[17]=1;                 // imu online
payload[18]=192; payload[19]=168; payload[20]=4; payload[21]=55; // child IP
dv.setFloat32(22, 0.75, true); // cal_roll_gnd
payload[26]=1;                 // corr mode
dv.setFloat32(27, 123.4, true);// gripper angle
payload[31]=1;                 // gripper online
const frame = P.buildFrame(P.STATE_CMD, payload);
assert(frame[2]===0x81 && frame[3]===32 && frame.length===38, '状态帧头 LEN=32 总长=38');
const st = P.parseState(frame);
assert(st && st.servo.m1===30 && st.servo.pb9===160, '状态: 舵机解析');
assert(st.pump.front===true && st.pump.rear===true, '状态: 泵 = 前后吸');
assert(Math.abs(st.can.m1-1.0)<1e-6 && Math.abs(st.can.m2+0.5)<1e-6, '状态: CAN 弧度');
assert(Math.abs(st.imu.roll-2.5)<1e-6 && st.imu.online===true, '状态: IMU');
assert(Math.abs(st.imu.ground-0.75)<1e-6, '状态: 地面标定滚转');
assert(st.childIp==='192.168.4.55', '状态: 子机 IP');
assert(st.corrMode===1, '状态: 纠偏模式');
assert(Math.abs(st.gripper-123.4)<1e-3 && st.gripperOnline===true, '状态: 夹持器');
assert(P.STATE_LEN===32, 'STATE_LEN = 32');

// 8. 坏 CRC 应被拒绝
const bad = new Uint8Array(frame); bad[6]^=0xFF;
assert(P.parseState(bad)===null, '坏 CRC 被拒绝');

// 9. 抖动/截断
assert(P.parseFrame(new Uint8Array([0xAA,0x55,0x05]))===null, '短帧被拒绝');
assert(P.parseFrame(new Uint8Array([0x00,0x55,0x05,0x00,0,0]))===null, '错误头被拒绝');
