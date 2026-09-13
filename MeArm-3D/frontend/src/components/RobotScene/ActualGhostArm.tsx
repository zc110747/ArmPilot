/**
 * 实际臂幽灵（Phase 12 · 虚拟/真实同步常驻）。
 *
 * 反向链路（Transport 回推 → `actualJoints`）在 Phase 7 就已经打通，
 * 缺的一直是**渲染**：3D 场景此前只画 `commandJoints`，于是"虚拟臂跟随真机"
 * 这件事在画面上完全看不见 —— 又一次"数据是对的，但你无从判断"。
 *
 * 本组件用**同一份** `buildRobotObject3D` + `applyJointState` 再建一棵半透明的树，
 * 喂 `actualJoints`。刻意复用而不是另写一套渲染：两套实现必然慢慢跑偏，而
 * 「主臂与实际臂用的运动学必须逐值同源」是这类数字孪生的立身之本。
 *
 * 为什么不用 `useFrame` 自己做插值让运动更"顺"：
 * `actualJoints` 本身就是后端周期回推的采样，直接照抄才是诚实的。
 * 再插值等于伪造固件并不具备的平滑度，反而盖住了真实滞后。
 */
import { useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import { useRobotStore } from '@/store/robotStore';
import { applyJointState, buildRobotObject3D, disposeRobotObject3D } from './buildRobotObject3D';

/**
 * dev-only 探针：把幽灵**渲染后**的 TCP 世界坐标暴露出去。
 *
 * 为什么必须是"渲染后"的矩阵而不是再算一次 FK：本组件的存在意义就是
 * "实际臂真的被画出来了"。若 e2e 拿 FK(actualJoints) 当证据，那只证明了数学，
 * 证不了渲染 —— 而 renderer 挂错树、可见性被误关、材质全透明都会让画面空掉
 * 却让 FK 断言全绿。取矩阵是唯一能区分这两者的做法。
 */
const probeTcp = new THREE.Vector3();

export function ActualGhostArm() {
  const model = useRobotStore((s) => s.model);
  const actualJoints = useRobotStore((s) => s.actualJoints);
  const showActualGhost = useRobotStore((s) => s.showActualGhost);
  const showTcp = useRobotStore((s) => s.showTcp);

  // 与主臂同样的"建树后立刻应用一次"处理：否则首帧会短暂停在零位。
  const objects = useMemo(() => {
    const built = buildRobotObject3D(model, { ghost: true });
    applyJointState(built, model, useRobotStore.getState().actualJoints);
    return built;
  }, [model]);

  const holder = useRef(
    globalThis as unknown as { __armPilotGhost?: { tcp: number[]; visible: boolean } },
  );

  useEffect(
    () => () => {
      disposeRobotObject3D(objects);
      // ⚠️ 必须同样用 DEV 守卫：否则 `__armPilotGhost` 这个字面量会残留在生产包里，
      //    触发"产物不得含探针字样"的验收检查（与 TestProbe 同一条纪律）。
      if (import.meta.env.DEV) delete holder.current.__armPilotGhost;
    },
    [objects],
  );

  useEffect(() => {
    applyJointState(objects, model, actualJoints);
  }, [objects, model, actualJoints]);

  useEffect(() => {
    objects.root.visible = showActualGhost;
  }, [objects, showActualGhost]);

  useEffect(() => {
    objects.tcpMarker.visible = showTcp;
  }, [objects, showTcp]);

  // 探针必须在上面几个 effect 之后（effect 按声明顺序执行）：否则读到的是上一帧矩阵。
  useEffect(() => {
    if (!import.meta.env.DEV) return;
    if (!showActualGhost) {
      delete holder.current.__armPilotGhost;
      return;
    }
    objects.root.updateMatrixWorld(true);
    probeTcp.setFromMatrixPosition(objects.tcpMarker.matrixWorld);
    holder.current.__armPilotGhost = {
      tcp: [probeTcp.x, probeTcp.y, probeTcp.z],
      visible: true,
    };
  }, [objects, actualJoints, showActualGhost]);

  return <primitive object={objects.root} />;
}

