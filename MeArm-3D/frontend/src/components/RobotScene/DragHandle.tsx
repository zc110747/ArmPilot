/**
 * 末端拖动把手（Phase 6）。
 *
 * 场景元素（都挂在世界系，坐标即 FK 的世界坐标）：
 *   - **把手球**：位置 = `target`（鼠标拖出来的目标点），半透明、可点、hover 提亮。
 *   - **分离连线**：`target` ↔ 实际 TCP 的虚线，只在两者分离时出现。
 *
 * 为什么把手跟随 `target` 而不是 TCP：拖动时把手必须与鼠标**同步**（跟手），
 * 而 TCP 圆点跟随关节。当目标越界时，"够不到"就表现为**二者分离**，
 * 这是工作空间边界最直观的证据 —— 若把手改跟随 TCP，鼠标会与把手脱节，
 * 手感变成"卡住"，且看不出差在哪。
 *
 * 交互：
 *   pointerdown → `resetTarget()` 把起点吸附到真实 TCP → 冻结「平面几何 + 平面模式」
 *                 → 禁用 OrbitControls（否则会边转相机边拖）→ setPointerCapture
 *   pointermove → 射线 ∩ 冻结平面 → moveTo()
 *   pointerup / lostPointerCapture → 恢复 OrbitControls
 */
import { useEffect, useMemo, useRef, useState } from 'react';
import * as THREE from 'three';
import { useFrame, useThree, type ThreeEvent } from '@react-three/fiber';
import { dragTarget, makePlane, type DragPlaneMode, type PlaneSpec } from '@robot/index';
import { useRobotStore } from '@/store/robotStore';

/** 把手球半径（mm）。够大好点中；R3F 对每个被射线命中的对象都派发事件，连杆不会把它挡掉 */
const HANDLE_RADIUS = 15;
/** target 与 TCP 分离超过此值才画连线（mm） */
const SEPARATION_EPS_MM = 0.4;

const COLOR_REACHABLE = '#4c8dff';
const COLOR_REACHABLE_HOVER = '#8fbcff';
const COLOR_OUT_OF_REACH = '#e5534b';
const COLOR_OUT_OF_REACH_HOVER = '#ff8a80';

type ControlsLike = { enabled: boolean } | null;
type PointerCaptureTarget = {
  setPointerCapture?(pointerId: number): void;
  releasePointerCapture?(pointerId: number): void;
};

export function DragHandle() {
  // 本组件**不订阅任何 store 状态**：全部在 useFrame / 事件回调里 `getState()` 读取。
  // 拖动时 moveTo 每帧都在 set 状态，若用 hooks 订阅会让整棵树每帧重渲染。
  const camera = useThree((s) => s.camera);
  const controls = useThree((s) => s.controls) as unknown as ControlsLike;

  const handleRef = useRef<THREE.Mesh>(null);
  const materialRef = useRef<THREE.MeshStandardMaterial>(null);

  // 拖动期间**冻结**的平面：几何 + 模式一起冻，中途改下拉不影响进行中的这一次拖动
  const planeRef = useRef<PlaneSpec | null>(null);
  const modeRef = useRef<DragPlaneMode>('xy');
  const draggingRef = useRef(false);
  const hoveredRef = useRef(false);
  const cameraDir = useMemo(() => new THREE.Vector3(), []);

  const [, forceHoverRender] = useState(0);

  const lineObject = useMemo(() => {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
    const material = new THREE.LineDashedMaterial({
      color: COLOR_OUT_OF_REACH,
      dashSize: 6,
      gapSize: 5,
      transparent: true,
      opacity: 0.9,
    });
    const line = new THREE.Line(geometry, material);
    line.visible = false;
    line.frustumCulled = false;
    return line;
  }, []);

  useEffect(
    () => () => {
      lineObject.geometry.dispose();
      (lineObject.material as THREE.Material).dispose();
    },
    [lineObject],
  );

  // 每帧把把手 / 连线 / 颜色同步到 store —— 不走 React 重渲染
  useFrame(() => {
    const state = useRobotStore.getState();
    const target = state.target;
    const tcp = state.endEffector.position;
    const reachable = state.ikStatus?.ok ?? true;

    const handle = handleRef.current;
    if (handle) handle.position.set(target[0], target[1], target[2]);

    const gap = Math.hypot(target[0] - tcp[0], target[1] - tcp[1], target[2] - tcp[2]);
    const showLine = gap > SEPARATION_EPS_MM;
    lineObject.visible = showLine;
    if (showLine) {
      const position = lineObject.geometry.getAttribute('position') as THREE.BufferAttribute;
      position.setXYZ(0, tcp[0], tcp[1], tcp[2]);
      position.setXYZ(1, target[0], target[1], target[2]);
      position.needsUpdate = true;
      lineObject.geometry.computeBoundingSphere();
      lineObject.computeLineDistances();
    }

    const material = materialRef.current;
    if (material) {
      const hovered = hoveredRef.current;
      material.color.set(
        reachable
          ? hovered
            ? COLOR_REACHABLE_HOVER
            : COLOR_REACHABLE
          : hovered
            ? COLOR_OUT_OF_REACH_HOVER
            : COLOR_OUT_OF_REACH,
      );
      material.opacity = draggingRef.current ? 0.55 : hovered ? 0.42 : 0.26;
    }
  });

  const beginDrag = (event: ThreeEvent<PointerEvent>) => {
    event.stopPropagation();
    const state = useRobotStore.getState();

    // 起点吸附到真实 TCP：保证"抓住的就是末端"，不带上一次的残余目标误差
    state.resetTarget();
    const anchor = useRobotStore.getState().endEffector.position;

    camera.getWorldDirection(cameraDir);
    modeRef.current = state.dragPlane;
    planeRef.current = makePlane(state.dragPlane, anchor, [
      cameraDir.x,
      cameraDir.y,
      cameraDir.z,
    ]);

    draggingRef.current = true;
    state.setDragging(true);
    if (controls) controls.enabled = false;
    document.body.style.cursor = 'grabbing';

    const capture = event.target as unknown as PointerCaptureTarget;
    capture.setPointerCapture?.(event.pointerId);
    forceHoverRender((n) => n + 1);
  };

  const continueDrag = (event: ThreeEvent<PointerEvent>) => {
    if (!draggingRef.current) return;
    const plane = planeRef.current;
    if (!plane) return;
    event.stopPropagation();

    const ray = event.ray;
    const point = dragTarget(
      modeRef.current,
      plane,
      [ray.origin.x, ray.origin.y, ray.origin.z],
      [ray.direction.x, ray.direction.y, ray.direction.z],
    );
    // 射线与冻结平面平行 / 交点在相机背后 → 保持上一目标不动
    if (!point) return;
    useRobotStore.getState().moveTo(point);
  };

  const endDrag = (event?: ThreeEvent<PointerEvent>) => {
    if (!draggingRef.current) return;
    draggingRef.current = false;
    planeRef.current = null;
    if (controls) controls.enabled = true;
    useRobotStore.getState().setDragging(false);
    document.body.style.cursor = hoveredRef.current ? 'grab' : 'default';
    if (event) {
      const capture = event.target as unknown as PointerCaptureTarget;
      capture.releasePointerCapture?.(event.pointerId);
    }
  };

  return (
    <>
      <mesh
        ref={handleRef}
        name="dragHandle"
        onPointerDown={beginDrag}
        onPointerMove={continueDrag}
        onPointerUp={(event) => endDrag(event)}
        onLostPointerCapture={() => endDrag()}
        onPointerOver={(event) => {
          event.stopPropagation();
          hoveredRef.current = true;
          if (!draggingRef.current) document.body.style.cursor = 'grab';
          forceHoverRender((n) => n + 1);
        }}
        onPointerOut={() => {
          hoveredRef.current = false;
          if (!draggingRef.current) document.body.style.cursor = 'default';
          forceHoverRender((n) => n + 1);
        }}
      >
        <sphereGeometry args={[HANDLE_RADIUS, 24, 16]} />
        <meshStandardMaterial
          ref={materialRef}
          color={COLOR_REACHABLE}
          transparent
          opacity={0.26}
          depthWrite={false}
          metalness={0}
          roughness={0.9}
        />
      </mesh>

      <primitive object={lineObject} />
    </>
  );
}
