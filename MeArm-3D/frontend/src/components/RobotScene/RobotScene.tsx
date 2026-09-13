/**
 * 3D 场景（Phase 2）。
 *
 * 坐标系：机器人系 = Three.js 世界系 = **右手系 Z-up**（X 前 / Y 左 / Z 上），1 场景单位 = 1 mm。
 * 这样 FK 解算出的 XYZ 与屏幕上模型的实际世界坐标可以直接逐值比较（见 kinematics/coordinate.ts）。
 *
 * 提供：Grid / World Axis / Robot Axis / Joint Axis / 轨道旋转 / 缩放 / 平移 / 相机复位。
 */
import { useEffect, useRef } from 'react';
import { Canvas, useThree } from '@react-three/fiber';
import { Grid, OrbitControls } from '@react-three/drei';
import { useRobotStore } from '@/store/robotStore';
import { DragHandle } from './DragHandle';
import { ActualGhostArm } from './ActualGhostArm';
import { RobotArm } from './RobotArm';
import { TestProbe } from './TestProbe';

const HOME_CAMERA_POSITION: [number, number, number] = [300, -430, 300];
const HOME_CAMERA_TARGET: [number, number, number] = [0, 0, 110];

/** 只用到 OrbitControls 的这两个成员，避免依赖 drei 的传递依赖类型 */
type OrbitLike = {
  target: { set(x: number, y: number, z: number): void };
  update(): void;
};

/** 相机复位（`cameraResetToken` 自增即触发） */
function CameraRig() {
  const token = useRobotStore((s) => s.cameraResetToken);
  const camera = useThree((s) => s.camera);
  const controls = useThree((s) => s.controls) as unknown as OrbitLike | null;
  const lastToken = useRef(-1);

  useEffect(() => {
    if (token === lastToken.current) return;
    lastToken.current = token;
    camera.up.set(0, 0, 1);
    camera.position.set(...HOME_CAMERA_POSITION);
    camera.lookAt(...HOME_CAMERA_TARGET);
    if (controls) {
      controls.target.set(...HOME_CAMERA_TARGET);
      controls.update();
    }
  }, [token, camera, controls]);

  return null;
}

/** World Axis：世界原点三轴 */
function WorldAxes() {
  const visible = useRobotStore((s) => s.showWorldAxes);
  return <axesHelper args={[130]} visible={visible} />;
}

/** Robot Axis：机器人根坐标系三轴（长度更短以示区分） */
function RobotAxes() {
  const visible = useRobotStore((s) => s.showRobotAxes);
  return <axesHelper args={[70]} position={[0, 0, 1]} visible={visible} />;
}

export function RobotScene() {
  return (
    <Canvas
      dpr={[1, 2]}
      camera={{
        position: HOME_CAMERA_POSITION,
        up: [0, 0, 1],
        fov: 38,
        near: 1,
        far: 6000,
      }}
      gl={{ antialias: true }}
      onCreated={({ camera }) => camera.up.set(0, 0, 1)}
    >
      <color attach="background" args={['#14171c']} />

      <hemisphereLight args={['#8ea6c8', '#1a1d22', 0.55]} />
      <ambientLight intensity={0.35} />
      <directionalLight position={[320, -420, 620]} intensity={1.35} />
      <directionalLight position={[-380, 300, 240]} intensity={0.45} />

      {/* 地面网格：drei Grid 默认位于 XZ 平面，绕 X 转 90° 后落到世界 XY 平面（Z=0） */}
      <Grid
        rotation={[Math.PI / 2, 0, 0]}
        args={[600, 600]}
        cellSize={20}
        cellThickness={0.7}
        cellColor="#333c48"
        sectionSize={100}
        sectionThickness={1.2}
        sectionColor="#4d5867"
        fadeDistance={1400}
        fadeStrength={1.2}
        infiniteGrid
      />

      <WorldAxes />
      <RobotAxes />
      <RobotArm />
      {/* 实际臂幽灵（Phase 12）：跟随 actualJoints，露出部分即滞后量 */}
      <ActualGhostArm />
      <DragHandle />

      <OrbitControls
        makeDefault
        enableDamping
        dampingFactor={0.12}
        target={HOME_CAMERA_TARGET}
        minDistance={60}
        maxDistance={2000}
        enablePan
        enableZoom
        screenSpacePanning
      />
      <CameraRig />
      {/* E2E 探针只在 dev 构建挂载：生产构建下 import.meta.env.DEV 为 false，整段不渲染 */}
      {import.meta.env.DEV ? <TestProbe /> : null}

      {/* 底座参考圆盘，帮助判断 Z=0 地面 */}
      <mesh position={[0, 0, -0.6]}>
        <circleGeometry args={[72, 48]} />
        <meshStandardMaterial color="#1d222a" metalness={0} roughness={1} />
      </mesh>
    </Canvas>
  );
}
