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
          ikStatus: s.ikStatus,
          dragging: s.dragging,
          dragPlane: s.dragPlane,
          commandJoints: s.commandJoints,
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
