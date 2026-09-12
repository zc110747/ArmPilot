/**
 * FakeTimer —— 可精确推进的虚拟时钟（`TimerLike` 的测试实现）。
 *
 * 让「延迟 15ms 后开始动」「每 tick 最多转 4.8°」这类断言变成**确定性**的，
 * 不用真实等待、不会因 CI 负载而 flaky。
 */
import type { TimerLike } from '@robot/index';

interface Job {
  at: number;
  fn: () => void;
  /** null = 单次定时（setTimeout） */
  interval: number | null;
}

export class FakeTimer implements TimerLike {
  private t: number;
  private seq = 0;
  private readonly jobs = new Map<number, Job>();

  constructor(start = 0) {
    this.t = start;
  }

  now(): number {
    return this.t;
  }

  setInterval(fn: () => void, ms: number): number {
    const id = (this.seq += 1);
    this.jobs.set(id, { at: this.t + Math.max(0, ms), fn, interval: Math.max(1, ms) });
    return id;
  }

  clearInterval(handle: number): void {
    this.jobs.delete(handle);
  }

  setTimeout(fn: () => void, ms: number): number {
    const id = (this.seq += 1);
    this.jobs.set(id, { at: this.t + Math.max(0, ms), fn, interval: null });
    return id;
  }

  clearTimeout(handle: number): void {
    this.jobs.delete(handle);
  }

  /** 推进虚拟时间，按到期顺序执行所有回调（同刻按注册顺序） */
  advance(ms: number): void {
    const end = this.t + ms;
    for (;;) {
      let nextId = -1;
      let nextAt = Number.POSITIVE_INFINITY;
      for (const [id, job] of this.jobs) {
        if (job.at > end) continue;
        if (job.at < nextAt || (job.at === nextAt && id < nextId)) {
          nextAt = job.at;
          nextId = id;
        }
      }
      if (nextId < 0) break;

      const job = this.jobs.get(nextId);
      if (!job) break;
      this.t = job.at;
      if (job.interval === null) {
        this.jobs.delete(nextId);
      } else {
        job.at = this.t + job.interval;
      }
      // 注意：回调里可能 clearInterval 掉自己（Mock 收敛后就会这么做）
      job.fn();
    }
    this.t = end;
  }

  /** 尚未触发的定时任务数 —— 用于断言「收敛后不再空转」 */
  pending(): number {
    return this.jobs.size;
  }
}
