/**
 * 启动时自动连接（供「一键启动脚本」使用）。
 *
 * 背景
 * ----
 * 网页默认停在 **MockTransport**（浏览器内仿真）且**不会自动连接** ——
 * 这对开发者手动调 UI 是对的，但一键启动脚本的语义是"起前后端并接上硬件"。
 * 结果就是：脚本跑完、后端也起来了，页面里却是 sim 数据在动，真机纹丝不动，
 * 用户看不出哪里不对（正是我们反复踩的那类"静默失败"）。
 *
 * 解法
 * ----
 * 让 `start.bat` 注入 `VITE_AUTO_CONNECT`（+ `VITE_AUTO_REAL`），本模块在挂载后：
 *   1. 连 WebSocket（默认按 `window.location.hostname` 推导，局域网访问也对）
 *   2. 连接成功后，若要求 real 则切到 Real Robot（**带校验**：链路末端必须是 serial，
 *      否则告警但不阻断 —— sim 降级要看得见，不要静默）
 *
 * 为什么不做成"页面永远自动连 WS"
 * ---------------------------------
 * 那会让 `npm run dev` 的手动调试也被改掉（想试 Mock 就得先断开），
 * 且生产构建会无条件去连一个可能不存在的后端。环境变量把"一键启动"与
 * "手动开发"两种语义分开，各保持正确。
 */
import { useEffect, useRef } from 'react';
import { useRobotStore } from '@/store/robotStore';
import { connectWebSocketTransport, connectMockTransport } from '@/store/transportBridge';
import { getDefaultWsUrl } from '@/components/RobotControl/wsUrl';

/** 解析后的自动连接意图；null = 不自动连接（维持原行为） */
export interface AutoConnectIntent {
  kind: 'ws' | 'mock';
  url?: string;
  real: boolean;
}

/**
 * 从环境变量解析意图。导出以便单测直接覆盖各种取值组合。
 *
 * @param env        形如 `import.meta.env`
 * @param fallbackUrl 未显式给 `VITE_WS_URL` 时用的地址
 */
export function parseAutoConnectIntent(
  env: { VITE_AUTO_CONNECT?: string; VITE_AUTO_REAL?: string; VITE_WS_URL?: string },
  fallbackUrl: string,
): AutoConnectIntent | null {
  const raw = (env.VITE_AUTO_CONNECT ?? '').trim().toLowerCase();
  if (raw === '' || raw === '0' || raw === 'off' || raw === 'none') return null;
  if (raw !== 'ws' && raw !== 'mock') return null;

  const realRaw = (env.VITE_AUTO_REAL ?? '').trim().toLowerCase();
  const real = realRaw === '1' || realRaw === 'true' || realRaw === 'yes';

  if (raw === 'mock') return { kind: 'mock', real: false };
  return { kind: 'ws', url: env.VITE_WS_URL || fallbackUrl, real };
}

/** 本次启动是否要求自动切 Real Robot（与连接状态无关，故不放进 store） */
function isRealRequested(): boolean {
  return parseAutoConnectIntent(import.meta.env, getDefaultWsUrl())?.real === true;
}

/**
 * 在 App 顶层调用一次。重复挂载安全（用 ref 保证只跑一次）。
 *
 * @param enabled 供测试/预览关闭用；默认在浏览器里启用
 */
export function useAutoConnect(enabled = true): void {
  const started = useRef(false);

  // ---- 1. 发起连接（只跑一次） ----------------------------------------------
  useEffect(() => {
    if (!enabled || started.current) return;
    started.current = true;

    const intent = parseAutoConnectIntent(import.meta.env, getDefaultWsUrl());
    if (intent === null) return;

    const store = useRobotStore.getState();
    if (store.connection === 'connected') return;

    void (async () => {
      try {
        if (intent.kind === 'mock') {
          await connectMockTransport();
          useRobotStore
            .getState()
            .pushLog('sys', '（自动连接）已连接 MockTransport —— 浏览器内仿真，不碰硬件');
          return;
        }
        useRobotStore
          .getState()
          .pushLog('sys', `（自动连接）正在连接 WebSocket：${intent.url}`);
        await connectWebSocketTransport({ url: intent.url as string });
        // ⚠️ 这里**只发起连接**：WebSocket 握手是异步的，await 返回时未必已连通。
        //    真正的"已连接"以 store.connection 变 'connected' 为准（见下方第 2 步）。
      } catch (err) {
        useRobotStore
          .getState()
          .pushLog(
            'err',
            `（自动连接）失败：${err instanceof Error ? err.message : String(err)}` +
              ' —— 后端是否已启动？地址是否正确？（Connection 面板可手动连接）',
          );
      }
    })();
  }, [enabled]);

  // ---- 2. 连接成功后按需切 Real Robot ---------------------------------------
  // 用独立 effect + 状态订阅：连接完成是异步事件，不能依赖第 1 步的 await 时序。
  const realRequested = isRealRequested();

  useEffect(() => {
    if (!enabled || !realRequested) return;

    const apply = () => {
      const st = useRobotStore.getState();
      if (st.connection !== 'connected') return;
      if (st.mode === 'real') return;

      // 复用 store 的准入校验：它自己判断链路末端是不是 serial 并给出提示。
      // 末端是 sim 时它只 pushLog('err', ...) 而不改 mode —— 即"告警但不阻断"。
      st.setMode('real');

      if (useRobotStore.getState().mode !== 'real') {
        useRobotStore
          .getState()
          .pushLog(
            'err',
            '（自动连接）未能切到 Real Robot —— 原因见上一条。' +
              '命令仍会下发给当前链路（若后端是 sim，则只动仿真，真机不动）。',
          );
      }
    };

    apply();
    // 握手 + hello 可能稍后才完成，订阅一次 connection 变化兜底
    return useRobotStore.subscribe((state, prev) => {
      if (state.connection !== prev.connection) apply();
    });
  }, [enabled, realRequested]);
}
