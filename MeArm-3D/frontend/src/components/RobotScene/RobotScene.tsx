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

/**
 * ⚠️ 照片纹理件的环境反射**刻意不在这里做**。
 *
 * `scene.environment` 是**全局**的，而且 three 对「材质没有自带 envMap」的情况会用
 * `scene.environmentIntensity` **覆盖**其 `material.envMapIntensity`
 * （`WebGLRenderer.js`：`m_uniforms.envMapIntensity.value = scene.environmentIntensity`），
 * 所以无法把 IBL 只发给贴图件 —— 实测会把整机非贴图件一并点亮（底座蓝板 ×3.75）。
 *
 * 正确做法见 `plateEnvironment.ts`：环境纹理由 `RobotArm` 建树时**逐材质**挂到
 * 照片纹理件上（`material.envMap`），其余材质完全不受影响（ADR D64）。
 */

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
      {/* 视口背景：中性灰（产品渲染式背景）。
          与 UI 面板的 `--bg #14171c` 刻意不同 —— 面板保持深色、视口用灰底，
          近黑的机件在灰底上轮廓才读得出来（黑件在深底上会糊成一片）。 */}
      <color attach="background" args={['#8b8e93']} />

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

      {/* 底座参考圆盘，帮助判断 Z=0 地面。
          envMapIntensity=0：它是场景物件而非机器人零件，不接收为贴图件准备的环境反射
          （否则会在 IBL 开启时被一并点亮，改变原有的地面色感）。
          色值随背景一起改：它只比背景**暗一档**（而非原来的近黑 #1d222a）——
          灰底上放一块近黑圆盘会变成一个突兀的黑洞，那是"背景换了、地面没换"。 */}
      <mesh position={[0, 0, -0.6]}>
        <circleGeometry args={[72, 48]} />
        <meshStandardMaterial color="#7a7e84" metalness={0} roughness={1} envMapIntensity={0} />
      </mesh>
    </Canvas>
  );
}
