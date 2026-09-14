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
import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { useFrame, useThree } from '@react-three/fiber';
import { useRobotStore } from '@/store/robotStore';

/**
 * 逐帧场景快照（**首帧瞬态的专用仪器**）。
 *
 * 为什么需要"逐帧"而不是"看完事之后的稳态"：`Page.captureScreenshot` 拿到的永远是
 * **已经稳定**的那一帧 —— 首次加载时才存在、一动就消失的现象，用截图根本观测不到
 * （实测：等到能截图时现象早已自愈）。要在时序上抓到它，只有在**每一帧**上留证。
 *
 * 记的是**渲染真实用的 `matrixWorld`**，不重算 FK，也不手动 `updateMatrixWorld`：
 * 本记录器在 R3F 的 `useFrame` 阶段执行，先于 `gl.render`，因此读到的正是
 * **上一帧实际画出去**的矩阵。手动更新矩阵会把"矩阵是陈旧的"这类 bug 直接抹掉。
 */
export interface SceneFrameSample {
  /** 帧序号（从 0 起，= 本记录器被调用的次序） */
  f: number;
  /** 场景里名字以 `robot:` 开头的根节点数量 —— **>1 就是同时画了多棵树** */
  roots: number;
  /** 其中 visible 的根节点数量（判定"画出来没有"要看它，不是看 roots） */
  visibleRoots: number;
  /** 每个根：是否半透明（幽灵）、世界矩阵里的平移、以及其下 tcp 标记的世界坐标 */
  arms: Array<{
    ghost: boolean;
    visible: boolean;
    origin: [number, number, number];
    tcp: [number, number, number] | null;
  }>;
  /** 已挂上 `envMap` 的贴图材质数量（0 ⇒ 环境反射那一轮重建还没发生） */
  texturedWithEnv: number;
}

/** 记录上限：60fps 下约 15s，足够覆盖冷启动全过程，又不至于无界增长 */
const FRAME_CAP = 900;

function vec(v: THREE.Vector3): [number, number, number] {
  return [v.x, v.y, v.z];
}

/** 采集一帧。只读，不改动任何对象状态（改了就观测不到 bug 了） */
function sampleScene(scene: THREE.Object3D, frame: number): SceneFrameSample {
  const roots: THREE.Object3D[] = [];
  scene.traverse((object) => {
    if (object.name.startsWith('robot:')) roots.push(object);
  });

  const probe = new THREE.Vector3();
  let texturedWithEnv = 0;
  const arms: SceneFrameSample['arms'] = [];

  for (const root of roots) {
    let ghost = false;
    root.traverse((object) => {
      const mesh = object as THREE.Mesh;
      if (!mesh.isMesh) return;
      const materials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
      for (const material of materials) {
        const standard = material as THREE.MeshStandardMaterial;
        if (standard.map && standard.envMap) texturedWithEnv += 1;
        // 幽灵的判定看 material.transparent（makeGhost 打的就是它），不看名字：
        // 主臂与幽灵的 root.name 完全相同（都是 `robot:<id>`）
        if (standard.transparent) ghost = true;
      }
    });

    const tcpMarker = root.getObjectByName('tcp');
    arms.push({
      ghost,
      visible: root.visible,
      origin: vec(probe.setFromMatrixPosition(root.matrixWorld)),
      tcp: tcpMarker ? vec(new THREE.Vector3().setFromMatrixPosition(tcpMarker.matrixWorld)) : null,
    });
  }

  let visibleRoots = 0;
  for (const arm of arms) if (arm.visible) visibleRoots += 1;

  return { f: frame, roots: roots.length, visibleRoots, arms, texturedWithEnv };
}

interface ArmPilotProbe {
  /** 末端 TCP 的屏幕像素坐标（视口坐标系，可直接喂给 CDP Input.dispatchMouseEvent） */
  tcpScreen(): { x: number; y: number };
  /** 关键交互状态快照 */
  state(): Record<string, unknown>;
  /** 以编程方式指定末端目标 */
  moveTo(xyz: [number, number, number]): Record<string, unknown>;
  /**
   * 把主相机摆到指定位姿（**只动相机，不动任何机器人状态**）。
   *
   * 用途：视觉验收需要一个**确定的**视图才能做「贴图朝向对不对」这类判定
   * （正对板面时上下颠倒 / 左右镜像一眼可辨；斜视图下判不出来）。
   * 与其余探针一样只挂在 dev 构建下。
   */
  setCamera(position: [number, number, number], target: [number, number, number]): void;
}

export function TestProbe() {
  const camera = useThree((s) => s.camera);
  const gl = useThree((s) => s.gl);
  const scene = useThree((s) => s.scene);
  const controls = useThree((s) => s.controls) as unknown as
    | { target: { set(x: number, y: number, z: number): void }; update(): void }
    | null;

  // ---- 首帧时序记录（见 SceneFrameSample 注释）----
  // 数组**只建一次**并把同一个引用挂到 window：CDP 侧可以在任意时刻取走，
  // 拿到的都是活的累积结果，不必与页面同步"开始录制"的时机。
  const frames = useRef<SceneFrameSample[]>([]);
  useEffect(() => {
    const holder = window as unknown as { __armPilotFrames?: SceneFrameSample[] };
    holder.__armPilotFrames = frames.current;
    return () => {
      delete holder.__armPilotFrames;
    };
  }, []);

  useFrame(() => {
    if (!import.meta.env.DEV) return;
    const list = frames.current;
    if (list.length >= FRAME_CAP) return;
    list.push(sampleScene(scene, list.length));
  });

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
      setCamera(position, target) {
        // 本场景是 Z-up，相机 up 必须显式设成 +Z，否则 lookAt 会绕出一个歪斜的滚转角
        camera.up.set(0, 0, 1);
        camera.position.set(...position);
        camera.lookAt(...target);
        // OrbitControls 每帧会按自己的 target 重算球坐标，不同步会把相机拉回去
        if (controls) {
          controls.target.set(...target);
          controls.update();
        }
      },
    };

    return () => {
      delete holder.__armPilot;
    };
  }, [camera, gl, controls]);

  return null;
}
