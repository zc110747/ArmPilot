/**
 * 一键启动的自动连接意图解析（`useAutoConnect.parseAutoConnectIntent`）。
 *
 * 为什么值得单测：这段逻辑是**环境变量 → 行为**的唯一入口，
 * 而它错的方式很隐蔽 —— 比如把 `VITE_AUTO_CONNECT=1` 误判成"要连"，
 * 或把拼错的取值静默当成默认值，都会让"一键启动"悄悄退回 sim 而不报错。
 * 所以这里把「合法取值」「等价写法」「非法/空值必须返回 null」全部钉死。
 *
 * 注：只测纯函数。真实连接行为由 e2e（浏览器内）覆盖。
 */
import { describe, expect, it } from 'vitest';
import { parseAutoConnectIntent } from '@/hooks/useAutoConnect';

const FALLBACK = 'ws://localhost:8090/ws/joint';

describe('parseAutoConnectIntent · 不自动连接的情形', () => {
  it('未设置 VITE_AUTO_CONNECT → null（维持"等用户点 Connect"的原行为）', () => {
    expect(parseAutoConnectIntent({}, FALLBACK)).toBeNull();
    expect(parseAutoConnectIntent({ VITE_AUTO_REAL: '1' }, FALLBACK)).toBeNull();
  });

  it('空串 / 空白 → null', () => {
    expect(parseAutoConnectIntent({ VITE_AUTO_CONNECT: '' }, FALLBACK)).toBeNull();
    expect(parseAutoConnectIntent({ VITE_AUTO_CONNECT: '   ' }, FALLBACK)).toBeNull();
  });

  it('显式关闭取值（0 / off / none）→ null', () => {
    for (const v of ['0', 'off', 'none', 'OFF', 'None', ' 0 ']) {
      expect(parseAutoConnectIntent({ VITE_AUTO_CONNECT: v }, FALLBACK)).toBeNull();
    }
  });

  it('无法识别的取值 → null（不猜、不静默降级）', () => {
    for (const v of ['1', 'true', 'yes', 'wss', 'websocket', 'mock!', 'ws ']) {
      // 'ws ' 带尾空格会被 trim 成 'ws' —— 那是合法的，单独断言
      const got = parseAutoConnectIntent({ VITE_AUTO_CONNECT: v }, FALLBACK);
      if (v === 'ws ') expect(got?.kind).toBe('ws');
      else expect(got).toBeNull();
    }
  });
});

describe('parseAutoConnectIntent · ws 模式', () => {
  it("VITE_AUTO_CONNECT='ws' → 连 WS，url 用 fallback", () => {
    expect(parseAutoConnectIntent({ VITE_AUTO_CONNECT: 'ws' }, FALLBACK)).toEqual({
      kind: 'ws',
      url: FALLBACK,
      real: false,
    });
  });

  it('大小写与空白不敏感', () => {
    for (const v of ['WS', 'Ws', '  ws  ']) {
      expect(parseAutoConnectIntent({ VITE_AUTO_CONNECT: v }, FALLBACK)?.kind).toBe('ws');
    }
  });

  it('显式 VITE_WS_URL 优先于 fallback（部署到别的后端时用）', () => {
    const got = parseAutoConnectIntent(
      { VITE_AUTO_CONNECT: 'ws', VITE_WS_URL: 'ws://192.168.3.9:8090/ws/joint' },
      FALLBACK,
    );
    expect(got?.url).toBe('ws://192.168.3.9:8090/ws/joint');
  });

  it('空字符串的 VITE_WS_URL 视为未设置，退回 fallback', () => {
    const got = parseAutoConnectIntent(
      { VITE_AUTO_CONNECT: 'ws', VITE_WS_URL: '' },
      FALLBACK,
    );
    expect(got?.url).toBe(FALLBACK);
  });
});

describe('parseAutoConnectIntent · real 开关', () => {
  it('VITE_AUTO_REAL 的等价真值（1 / true / yes）都算开启', () => {
    for (const v of ['1', 'true', 'TRUE', 'yes', 'Yes', ' true ']) {
      expect(parseAutoConnectIntent({ VITE_AUTO_CONNECT: 'ws', VITE_AUTO_REAL: v }, FALLBACK)?.real)
        .toBe(true);
    }
  });

  it('其余取值（含 0 / false / 空）都算关闭', () => {
    for (const v of ['', '0', 'false', 'no', 'on', 'maybe']) {
      expect(parseAutoConnectIntent({ VITE_AUTO_CONNECT: 'ws', VITE_AUTO_REAL: v }, FALLBACK)?.real)
        .toBe(false);
    }
  });

  it('未设置 VITE_AUTO_REAL → real=false（安全默认：不自动驱动机器）', () => {
    expect(parseAutoConnectIntent({ VITE_AUTO_CONNECT: 'ws' }, FALLBACK)?.real).toBe(false);
  });
});

describe('parseAutoConnectIntent · mock 模式', () => {
  it("VITE_AUTO_CONNECT='mock' → 连 Mock，且 real 恒为 false", () => {
    // 连浏览器内 mock 却要求 real 是自相矛盾的：real 只对真机链路有意义。
    const got = parseAutoConnectIntent(
      { VITE_AUTO_CONNECT: 'mock', VITE_AUTO_REAL: '1' },
      FALLBACK,
    );
    expect(got).toEqual({ kind: 'mock', real: false });
  });
});
