#!/usr/bin/env node
/**
 * 把后端 **sim / mujoco** 停在指定位姿，然后断开（零依赖，直连 WebSocket）。
 *
 * 为什么需要这个原语
 * ------------------
 * 「机器现状 ≠ 页面假设」这类 bug **在纯仿真里也能造出来**，只要机器不在
 * 页面默认假设的那个位姿上。而 `device=sim` 的假固件开机即 HOME
 * （`backend/internal/device/sim.go`），所以"页面刚打开"这个条件**不足以**复现：
 * 必须先把后端**挪到一个非 HOME 的位姿**再刷新页面。
 *
 * 真实触发场景正是如此：上一轮会话把臂停在别处（真机被手动挪过 / 点了 ZERO /
 * 换过一次页面前的操作），刷新后页面以为机械臂在 HOME，而机器实际在别处 ——
 * 于是半透明的"实际臂幽灵"与主臂错开，看起来像重影/阴影。
 *
 * 本工具就是"把那个前提摆出来"的开关，让问题**可复现、可回归**。
 *
 * ⚠️ 本文件**不含任何限位/角度常量**（项目铁律：真值只有 `config/robot.yaml` 一份）。
 *    位姿一律由调用方以 `--joints` 传入。想拿"限位钳位后的零位"，请自己从
 *    `config/robot.yaml` 读，例如：
 *
 *      python -c "import yaml,json;m=yaml.safe_load(open('config/robot.yaml'));\
 *      print(json.dumps({j['id']:min(max(0,j['limits']['min']),j['limits']['max'])\
 *      for j in m['robot']['joints']}))"
 *
 * 用法
 * ----
 *   node tools/park_sim_pose.mjs <wsUrl> --joints '{"base":0,"shoulder":-6.0,...}'
 *   node tools/park_sim_pose.mjs <wsUrl> --joints @file.json
 *
 * 可选：
 *   --http  <base>   健康检查地址（默认由 wsUrl 推导，端口换 8091→8091）
 *   --tol   <deg>    判定"已到位"的关节角容差（默认 0.15°）
 *   --wait  <ms>     最长等待时间（默认 15000）
 *
 * 退出码：0 = 已停好；1 = 没等到；2 = 参数/连接错误
 */
import { readFileSync } from 'node:fs';

const PROTOCOL_VERSION = 1;
const CLIENT_JOINT_COMMAND = 'joint_command';
const CLIENT_STATUS_REQUEST = 'status_request';

function usage(message) {
  if (message) console.error(`[park] ${message}`);
  console.error(
    'usage: node tools/park_sim_pose.mjs <wsUrl> --joints \'{"base":..,"shoulder":..,"elbow":..,"gripper":..}\'',
  );
  process.exit(2);
}

function parseArgs(argv) {
  const positional = [];
  const options = {};
  for (let i = 0; i < argv.length; i += 1) {
    const token = argv[i];
    if (token.startsWith('--')) {
      const key = token.slice(2);
      const value = argv[i + 1];
      if (value === undefined) usage(`选项 ${token} 缺少取值`);
      options[key] = value;
      i += 1;
    } else {
      positional.push(token);
    }
  }
  return { positional, options };
}

const { positional, options } = parseArgs(process.argv.slice(2));
const wsUrl = positional[0];
if (!wsUrl) usage('缺少 wsUrl');

let joints;
if (options.joints === undefined) usage('缺少 --joints');
if (options.joints.startsWith('@')) {
  joints = JSON.parse(readFileSync(options.joints.slice(1), 'utf8'));
} else {
  joints = JSON.parse(options.joints);
}
for (const [id, value] of Object.entries(joints)) {
  if (!Number.isFinite(value)) usage(`关节 ${id} 不是有限数值：${value}`);
}

const tolerance = options.tol === undefined ? 0.15 : Number(options.tol);
const waitMs = options.wait === undefined ? 15000 : Number(options.wait);
const httpBase =
  options.http ??
  `${wsUrl.replace(/^ws/, 'http').replace(/\/ws\/.*$/, '')}`;

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

async function readState() {
  const response = await fetch(`${httpBase}/healthz`);
  if (!response.ok) throw new Error(`healthz HTTP ${response.status}`);
  return (await response.json()).state ?? {};
}

function maxDeviation(state) {
  let worst = 0;
  let worstJoint = null;
  for (const [id, value] of Object.entries(joints)) {
    const got = state[id];
    if (typeof got !== 'number') return { worst: Number.POSITIVE_INFINITY, worstJoint: id };
    const deviation = Math.abs(got - value);
    if (deviation > worst) {
      worst = deviation;
      worstJoint = id;
    }
  }
  return { worst, worstJoint };
}

const socket = new WebSocket(wsUrl);
const fail = (message) => {
  console.error(`[park] ${message}`);
  try {
    socket.close();
  } catch {
    /* 已关闭 */
  }
  process.exit(1);
};

socket.addEventListener('error', () => fail(`无法连接 ${wsUrl}`));
socket.addEventListener('open', () => {
  socket.send(JSON.stringify({ version: PROTOCOL_VERSION, type: CLIENT_JOINT_COMMAND, timestamp: Date.now(), seq: 1, joints }));
  socket.send(JSON.stringify({ version: PROTOCOL_VERSION, type: CLIENT_STATUS_REQUEST, timestamp: Date.now() }));
  console.log(`[park] 已下发 ${JSON.stringify(joints)} → ${wsUrl}`);
});

let settled = false;
const deadline = Date.now() + waitMs;
const poll = setInterval(async () => {
  if (settled) return;
  try {
    const state = await readState();
    const { worst, worstJoint } = maxDeviation(state);
    if (worst <= tolerance) {
      settled = true;
      clearInterval(poll);
      console.log(`[park] ✓ 已停在 ${JSON.stringify(state)}（最大偏差 ${worst.toFixed(4)}° @ ${worstJoint}）`);
      try {
        socket.close();
      } catch {
        /* 已关闭 */
      }
      process.exit(0);
    }
    if (Date.now() > deadline) {
      settled = true;
      clearInterval(poll);
      fail(`超时：期望 ${JSON.stringify(joints)}，实际 ${JSON.stringify(state)}（最大偏差 ${worst.toFixed(4)}° @ ${worstJoint}）`);
    }
  } catch (error) {
    if (Date.now() > deadline) {
      settled = true;
      clearInterval(poll);
      fail(`超时且健康检查失败：${error.message}`);
    }
  }
}, 250);
