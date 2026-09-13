/**
 * E2E 测试探针（**仅 dev 构建**）。
 *
 * 零依赖 CDP 脚本（`tests/e2e/ui-smoke.mjs`）要验证「真实鼠标拖动末端」，
 * 就必须知道拖动把手在屏幕上的像素坐标，否则只能做弱断言。
 * 这里把「末端 TCP → 屏幕像素」的投影暴露到 `window.__armPilot`。
 *
 * 生产构建下 `import.meta.env.DEV === false`，`RobotScene` 不会渲染本组件；
 * 验收脚本会额外检查产物中不含 `__armPilot` 字样，确保它没进生产包。
 */
import { useEffect } from 'react';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import { useRobotStore } from '@/store/robotStore';

interface ArmPilotProbe {
  /** 末端 TCP 的屏幕像素坐标（视口坐标系，可直接喂给 CDP Input.dispatchMouseEvent） */
  tcpScreen(): { x: number; y: number };
  /** 关键交互状态快照 */
  state(): Record<string, unknown>;
  /** 以编程方式指定末端目标 */
  moveTo(xyz: [number, number, number]): Record<string, unknown>;
}

export function TestProbe() {
  const camera = useThree((s) => s.camera);
  const gl = useThree((s) => s.gl);

  useEffect(() => {
    const projected = new THREE.Vector3();
    const holder = window as unknown as { __armPilot?: ArmPilotProbe };

    holder.__armPilot = {
      tcpScreen() {
        const tcp = useRobotStore.getState().endEffector.position;
        projected.set(tcp[0], tcp[1], tcp[2]).project(camera);
        const rect = gl.domElement.getBoundingClientRect();
        return {
          x: rect.left + ((projected.x + 1) / 2) * rect.width,
          y: rect.top + ((1 - projected.y) / 2) * rect.height,
        };
      },
      state() {
        const s = useRobotStore.getState();
        return {
          target: s.target,
          tcp: s.endEffector.position,
          /**
           * 实际末端位姿（FK of actualJoints）。
           *
           * Phase 12 起 e2e 用它当**独立参照**校验「实际臂幽灵跟的是 actual 而不是 command」：
           * 一边是 store 里的纯数学 FK，一边是 three.js 对象图算出的世界矩阵，
           * 两条代码路径互不依赖 —— 因此这不是自证。
           */
          actualTcp: s.actualEndEffector.position,
          ikStatus: s.ikStatus,
          dragging: s.dragging,
          dragPlane: s.dragPlane,
          commandJoints: s.commandJoints,
          /**
           * 示教轨迹（Phase 13）。e2e 要靠 `lastJoints` 做「回放终点 == 录制终点」
           * 的**逐值**判定 —— 若只读面板里 1 位小数的文本，就分不清
           * "精确落在末帧" 与 "插值差一点点"，而那正是本功能最容易出错的地方。
           */
          teach: {
            recording: s.teachRecording,
            frames: s.teachTrack.frames.length,
            durationMs: s.teachTrack.frames[s.teachTrack.frames.length - 1]?.t ?? 0,
            lastJoints: s.teachTrack.frames[s.teachTrack.frames.length - 1]?.joints ?? null,
          },
        };
      },
      moveTo(xyz) {
        const result = useRobotStore.getState().moveTo(xyz);
        return result.success
          ? { ok: true, branch: result.branch, residual: result.residual }
          : { ok: false, reason: result.reason, joint: result.joint };
      },
    };

    return () => {
      delete holder.__armPilot;
    };
  }, [camera, gl]);

  return null;
}
