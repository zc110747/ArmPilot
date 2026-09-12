/**
 * 可注入时钟（spec §二十 传输层）。
 *
 * `MockTransport` 内部有「舵机以有限角速度逼近目标」的连续过程，需要定时器驱动。
 * 但定时器一旦写死成 `setInterval`，单元测试就只能用真实时间等待 —— 慢、且必然 flaky。
 *
 * 因此把时间抽象成 `TimerLike`：
 *   - 生产：`realTimer`（`Date.now` + `setInterval` / `setTimeout`）
 *   - 测试：`FakeTimer`（`tests/helpers/fakeTimer.ts`，`advance(ms)` 精确推进）
 *
 * 这样"延迟 15ms 后开始动""每 tick 最多转 4.8°"这类断言可以**确定性**地验证，
 * 不依赖 CI 机器的负载。
 */

/** 最小定时器抽象：只覆盖传输层用到的四种能力 */
export interface TimerLike {
  /** 当前毫秒时间戳（单调，不要求是 Unix 时间） */
  now(): number;
  /** 周期定时，返回可取消的 handle */
  setInterval(fn: () => void, ms: number): number;
  clearInterval(handle: number): void;
  /** 单次定时，返回可取消的 handle */
  setTimeout(fn: () => void, ms: number): number;
  clearTimeout(handle: number): void;
}

export const realTimer: TimerLike = {
  now: () => Date.now(),
  setInterval: (fn, ms) => setInterval(fn, ms) as unknown as number,
  clearInterval: (handle) => clearInterval(handle),
  setTimeout: (fn, ms) => setTimeout(fn, ms) as unknown as number,
  clearTimeout: (handle) => clearTimeout(handle),
};
