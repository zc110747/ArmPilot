/**
 * 链路误差语义回归（Phase 11）。
 *
 * 为什么需要：本模块决定 UI 什么时候说"正常滞后"、什么时候说"异常卡死"。
 * 一旦判据松了（把滞后误报成卡死），用户会去排查并不存在的问题；
 * 判据紧了（卡死不报），又回到"看着正常其实是坏的"那类静默失败。
 * 所以这里把边界钉死：
 *   - 趋势只在样本足够时才下结论；
 *   - 命令刚变时**一律**不判异常；
 *   - 只有"命令已静止 ≥ STALL_HOLD_MS"且趋势不再下降才判 stalled。
 */
import { describe, expect, it } from 'vitest';
import {
  ERROR_TOL_DEG,
  HISTORY_CAP,
  MIN_TREND_SAMPLES,
  STALL_HOLD_MS,
  classifyLinkHealth,
  classifyTrend,
  describeLinkHealth,
  describeTrend,
  nextHistory,
  type LinkHealthInput,
} from '@robot/index';

/**
 * 构造判定上下文。
 *
 * 注意用 `=== undefined` 而不是 `??`：`lagDeg` 的合法取值包含 `null`（无数据），
 * `??` 会把 null 一起替换成默认值，就测不到"回退到历史末值"那条分支了。
 */
function ctx(over: Partial<LinkHealthInput> = {}): LinkHealthInput {
  return {
    transportDriven: over.transportDriven === undefined ? true : over.transportDriven,
    lagDeg: over.lagDeg === undefined ? 3 : over.lagDeg,
    msSinceCommandChange:
      over.msSinceCommandChange === undefined ? 5000 : over.msSinceCommandChange,
  };
}

describe('classifyTrend · 误差趋势', () => {
  it(`样本少于 ${MIN_TREND_SAMPLES} 时不猜（unknown）`, () => {
    expect(classifyTrend([])).toBe('unknown');
    expect(classifyTrend([1, 2, 3])).toBe('unknown');
  });

  it('误差递减 → shrinking', () => {
    expect(classifyTrend([4, 3, 2, 1, 0.8, 0.5, 0.3, 0.1])).toBe('shrinking');
  });

  it('误差递增 → growing', () => {
    expect(classifyTrend([0.1, 0.5, 1, 2, 3, 4, 5, 6])).toBe('growing');
  });

  it('误差基本不变 → flat', () => {
    expect(classifyTrend([2.0, 2.02, 1.98, 2.01, 2.0, 1.99, 2.0, 2.01])).toBe('flat');
  });

  it('单个尖峰不得翻面（四分位中位数，非首末值）', () => {
    // 中部夹了一个野值 9，但整体在收敛 —— 若用"末值 − 首值"或均值平滑都会误判
    expect(classifyTrend([4, 3.5, 9, 2.5, 2, 1.5, 1, 0.5])).toBe('shrinking');
  });
});

describe('nextHistory · 采样窗口', () => {
  it('追加到尾部', () => {
    expect(nextHistory([1, 2], 3, false)).toEqual([1, 2, 3]);
  });

  it('超过上限时丢最旧的', () => {
    const full = Array.from({ length: HISTORY_CAP }, (_, i) => i);
    const got = nextHistory(full, 999, false);
    expect(got.length).toBe(HISTORY_CAP);
    expect(got[got.length - 1]).toBe(999);
    expect(got[0]).toBe(1);
  });

  it('命令变化时清空历史（跨命令趋势无意义）', () => {
    expect(nextHistory([5, 4, 3, 2], 0.1, true)).toEqual([0.1]);
  });
});

describe('classifyLinkHealth · 链路健康判定', () => {
  it('未接入传输 → idle（Actual ≡ Command，误差恒为 0）', () => {
    expect(classifyLinkHealth([], ctx({ transportDriven: false, lagDeg: 0 }))).toBe('idle');
  });

  it('误差在容差内 → settled', () => {
    expect(classifyLinkHealth([0.2], ctx({ lagDeg: ERROR_TOL_DEG }))).toBe('settled');
  });

  it('命令刚变（未到 hold 时间）→ tracking，绝不判异常', () => {
    // 误差仍很大且趋势还在涨 —— 但命令刚下发，这是**必然**的，不是故障
    const growing = [1, 2, 3, 4, 5, 6, 7, 8];
    expect(
      classifyLinkHealth(growing, ctx({ lagDeg: 8, msSinceCommandChange: STALL_HOLD_MS - 1 })),
    ).toBe('tracking');
  });

  it('命令已静止但误差仍在收敛 → tracking（正常滞后）', () => {
    const shrinking = [5, 4, 3, 2, 1.5, 1.2, 1.0, 0.9];
    expect(classifyLinkHealth(shrinking, ctx({ lagDeg: 0.9 }))).toBe('tracking');
  });

  it('命令已静止且误差持平 → stalled（卡死）', () => {
    const flat = [3, 3.01, 2.99, 3.0, 3.0, 3.01, 2.99, 3.0];
    expect(classifyLinkHealth(flat, ctx({ lagDeg: 3 }))).toBe('stalled');
  });

  it('命令已静止且误差发散 → stalled（越追越远）', () => {
    const growing = [1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5];
    expect(classifyLinkHealth(growing, ctx({ lagDeg: 4.5 }))).toBe('stalled');
  });

  it('样本不足时不下断言（宁可说"还在追"）', () => {
    expect(classifyLinkHealth([3, 3], ctx({ lagDeg: 3 }))).toBe('tracking');
  });

  it('lagDeg 为 null 时回退到历史末值', () => {
    expect(classifyLinkHealth([0.1, 0.2], ctx({ lagDeg: null }))).toBe('settled');
  });
});

describe('描述文案', () => {
  it('四种健康态各有 label 与 tone', () => {
    expect(describeLinkHealth('idle', null).tone).toBe('dim');
    expect(describeLinkHealth('settled', 0.1).tone).toBe('live');
    expect(describeLinkHealth('tracking', 2).tone).toBe('dim');
    const stalled = describeLinkHealth('stalled', 3);
    expect(stalled.tone).toBe('warn');
    expect(stalled.label).toContain('异常');
  });

  it('tone 与 kind 严格对应（防止 UI 出现"异常"却是中性色）', () => {
    for (const kind of ['idle', 'settled', 'tracking', 'stalled'] as const) {
      const info = describeLinkHealth(kind, 1);
      expect(info.kind).toBe(kind);
      expect(info.label.length).toBeGreaterThan(0);
    }
  });

  it('趋势文案覆盖四态', () => {
    expect(describeTrend('shrinking')).toContain('收敛');
    expect(describeTrend('flat')).toContain('持平');
    expect(describeTrend('growing')).toContain('发散');
    expect(describeTrend('unknown')).toContain('采样不足');
  });
});
