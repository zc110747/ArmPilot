/**
 * set_joints.mjs —— 通过关节级 WebSocket 驱动真机（**会动舵机**）
 *
 * 用法
 *   node tools/set_joints.mjs --status                     # 只读，不动臂
 *   node tools/set_joints.mjs --home                       # 回 HOME 位（四关节一起）
 *   node tools/set_joints.mjs shoulder=20 elbow=125        # 只改指定关节
 *   node tools/set_joints.mjs 0 20 125 50                  # 位置式: base shoulder elbow gripper
 *   node tools/set_joints.mjs --settle 1200 shoulder=10    # 自定义到位等待
 *
 * 为什么需要它
 *   「用真机把目标板摆到镜头前」需要一个可脚本化的指令入口。既有工具里
 *   verify_pose.py / verify_serial_e2e.mjs 都是**验收**用途（跑一整套断言），
 *   不能用来"只动一个关节看一眼"。
 *
 * 为什么不用 curl
 *   这是 WebSocket，且必须在**同一次连接**里收 hello（拿限位）→ 等 joint_state
 *   （拿当前姿态）→ 发指令。分两次连接会丢掉上下文。Node 22 内置 WebSocket，零依赖。
 *
 * ⚠️ 安全设计（五条，都不可省）
 *
 *   1. **限位来自 hello，不硬编码**。config/robot.yaml 是唯一真值，本脚本不抄一份。
 *      发送前本地校验，越限直接拒绝并打印范围 —— 比让后端回一条 ERR 更早、更清楚。
 *
 *   2. **只有 device=="serial" 才允许发送**。跑 config.yaml 时 device 是 sim，
 *      发过去只是动虚拟臂；那时"摆位"是假的，会让人以为真机动了。
 *      （实测：hello 里**没有** connected 字段，协议文档 §5.1 写错了 —— 判据用 device。）
 *
 *   3. **"保持不变"的关节必须取最新 joint_state，不能取 homePose**。
 *      实测踩过：hello 与 joint_state 同时到达、hello 在前，若在 hello 分支里就发指令，
 *      cur 会 fallback 到 homePose —— 那不是"保持不动"，是**把臂拉回 home**。
 *      所以发送点挪到 joint_state 之后，并用 lastJoints。
 *
 *   4. **--home 是为"实验可比性"加的**。--settle 的"保持不变"语义会让状态**跨实验累积**：
 *      实测踩过 —— 先动 base=-20、再只发 shoulder=30，后者会把 base 保持在 -20，
 *      于是两张对比图的机位根本不同，读数全部作废。每组实验前先 --home，从已知状态出发。
 *
 *   5. **真机无位置反馈**：joint_state 是固件内部目标值，不代表已物理到位。
 *      所以默认 --settle 900ms 只是「给舵机时间走完」，不是「已确认到位」。
 *
 *   不实现任何轨迹/插值：一次一条 joint_command，避免大角度跳变撞到东西。
 */
const ARGS = process.argv.slice(2);

function num(s) {
  const v = Number(s);
  return Number.isFinite(v) ? v : null;
}

function usage(code = 0) {
  console.log(`用法:
  node tools/set_joints.mjs --status                    只读当前关节状态（不动臂）
  node tools/set_joints.mjs --home                      回 HOME 位（homePose，来自 hello）
  node tools/set_joints.mjs shoulder=20 elbow=125       只改指定关节，其余保持不变
  node tools/set_joints.mjs 0 20 125 50                 位置式 base shoulder elbow gripper
选项:
  --url <ws://...>   默认 ws://127.0.0.1:8090/ws/joint
  --settle <ms>      发完等待，默认 900（给舵机走完的时间，非到位确认）
  --timeout <ms>     整体超时，默认 12000
  --no-verify        跳过本地限位校验（不建议）
  --force            允许非串口链路（sim/mujoco）也发送`);
  process.exit(code);
}

const cfg = {
  url: 'ws://127.0.0.1:8090/ws/joint',
  settle: 900,
  timeout: 12000,
  verify: true,
  statusOnly: false,
  force: false,
  home: false,
};
const assign = {};
const ORDER = ['base', 'shoulder', 'elbow', 'gripper'];
const positional = [];

for (let i = 0; i < ARGS.length; i++) {
  const a = ARGS[i];
  if (a === '--status') { cfg.statusOnly = true; continue; }
  if (a === '--home') { cfg.home = true; continue; }
  if (a === '--no-verify') { cfg.verify = false; continue; }
  if (a === '--force') { cfg.force = true; continue; }
  if (a === '-h' || a === '--help') usage(0);
  if (a === '--url') { cfg.url = ARGS[++i]; continue; }
  if (a === '--settle') { cfg.settle = num(ARGS[++i]) ?? cfg.settle; continue; }
  if (a === '--timeout') { cfg.timeout = num(ARGS[++i]) ?? cfg.timeout; continue; }
  const m = /^([a-z_]+)=(-?[\d.]+)$/i.exec(a);
  if (m) { assign[m[1].toLowerCase()] = Number(m[2]); continue; }
  const n = num(a);
  if (n !== null) { positional.push(n); continue; }
  console.error(`无法解析参数: ${a}`);
  usage(2);
}

if (positional.length) {
  if (positional.length !== ORDER.length) {
    console.error(`位置式需要恰好 ${ORDER.length} 个数（${ORDER.join(' ')}），收到 ${positional.length} 个`);
    usage(2);
  }
  ORDER.forEach((k, i) => { assign[k] = positional[i]; });
}
if (cfg.home && Object.keys(assign).length) {
  console.error('--home 与显式关节赋值不能同时用（哪个优先会变成隐式规则，不如直接报错）');
  usage(2);
}

const ws = new WebSocket(cfg.url);
let settled = false;
let sent = false;
let lastJoints = null;
let limits = {};
let homePose = null;

function finish(code, msg) {
  if (settled) return;
  settled = true;
  clearTimeout(timer);
  if (msg) console.log(msg);
  try { ws.close(); } catch {}
  setTimeout(() => process.exit(code), 60);
}

const timer = setTimeout(() => finish(3, `✗ 超时 ${cfg.timeout}ms 未收到期望消息`), cfg.timeout);

ws.addEventListener('error', () => {
  finish(3, `✗ WebSocket 连接失败。后端在跑吗？（start.bat --real，然后等它打印 [serial] 已连接）`);
});

ws.addEventListener('open', () => {
  if (cfg.statusOnly) {
    ws.send(JSON.stringify({ version: 1, type: 'status_request', timestamp: Date.now() }));
  }
});

ws.addEventListener('message', (ev) => {
  let m;
  try { m = JSON.parse(ev.data); } catch { return; }

  if (m.type === 'hello') {
    limits = Object.fromEntries((m.model?.limits ?? []).map((l) => [l.id, l]));
    homePose = m.model?.homePose ?? null;
    const fmt = (j) => (limits[j] ? `${j}[${limits[j].min.toFixed(2)}..${limits[j].max.toFixed(2)}]` : j);
    console.log(`链路末端 device=${m.device}`);
    console.log(`限位（来自 ${m.model?.source ?? 'config/robot.yaml'}，本脚本不硬编码）:`);
    console.log(`  ${ORDER.map(fmt).join('  ')}`);
    console.log(`  homePose = ${JSON.stringify(homePose)}`);

    if (m.device !== 'serial' && !cfg.force) {
      console.error('');
      console.error(`✗ 链路末端是 "${m.device}"，不是真机 —— 拒绝发送。`);
      console.error('  跑的是 sim/mujoco 配置时，指令只会动虚拟臂；此时"摆位"是假的，');
      console.error('  会让人以为真机动了。改用后端 config.serial.yaml（start.bat --real）。');
      console.error('  确实想发就往后面加 --force。');
      finish(3);
      return;
    }
    // 刻意不在这里发指令 —— 等 joint_state 拿到"此刻在哪"（见文件头安全设计 3）
    return;
  }

  if (m.type === 'joint_state') {
    lastJoints = m.joints;
    if (cfg.statusOnly) {
      console.log(`\n当前关节角（固件内部目标值，非物理到位）: ${JSON.stringify(m.joints)}`);
      finish(0);
      return;
    }
    if (sent) return;      // 这是自己刚发那条指令的回执，不要递归下发
    sent = true;

    if (cfg.verify) {
      const bad = [];
      for (const [k, v] of Object.entries(assign)) {
        const l = limits[k];
        if (!l) { bad.push(`${k}=${v}（未知关节）`); continue; }
        if (v < l.min || v > l.max) bad.push(`${k}=${v} 超出 [${l.min.toFixed(2)}, ${l.max.toFixed(2)}]`);
      }
      if (bad.length) {
        console.error('\n✗ 本地校验拒绝（未发出任何指令）:');
        for (const b of bad) console.error(`    ${b}`);
        console.error('  ↑ 限位真值在 config/robot.yaml。不要为了"让它动"而放宽校验。');
        finish(2);
        return;
      }
    }

    let target = {};
    if (cfg.home) {
      if (!homePose) { finish(3, '✗ hello 没带 homePose，无法回零'); return; }
      for (const k of ORDER) target[k] = homePose[k] ?? 0;
      console.log(`\n当前姿态: ${JSON.stringify(lastJoints)}`);
      console.log(`即将下发（回 HOME）: ${JSON.stringify(target)}`);
    } else {
      // 当前姿态取自 joint_state（此刻在哪），不是 homePose（开机在哪）
      for (const k of ORDER) {
        target[k] = assign[k] !== undefined ? assign[k] : (lastJoints[k] ?? 0);
      }
      const kept = ORDER.filter((k) => assign[k] === undefined);
      console.log(`\n当前姿态: ${JSON.stringify(lastJoints)}`);
      console.log(`即将下发: ${JSON.stringify(target)}`);
      console.log(`  保持不变: ${kept.map((k) => `${k}=${target[k].toFixed(2)}`).join(' ') || '（无）'}`);
    }
    ws.send(JSON.stringify({ version: 1, type: 'joint_command', timestamp: Date.now(), seq: 1, joints: target }));

    setTimeout(() => {
      finish(0, `\n✓ 指令已下发（等待 ${cfg.settle}ms）。真机无回读，此处不代表已物理到位。\n` +
        `  下一步：抓帧看实际效果 —— python tools/capture_texture.py --plate <name>`);
    }, cfg.settle);
    return;
  }

  if (m.type === 'error') {
    console.error(`\n✗ 后端拒绝: [${m.code}] ${m.message}`);
    finish(1);
    return;
  }

  if (m.type === 'device_status' && !m.connected) {
    finish(3, '✗ 设备断开');
  }
});
