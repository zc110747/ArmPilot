/**
 * 虚拟机械臂（R3F 组件）。
 *
 * 关键点：3D 对象树来自 `buildRobotObject3D()`（与实际验收测试**同一份代码**），
 * 关节运动全部体现为 JointGroup 的旋转，Mesh 位置从不被改动。
 * 每帧（限频）把「真实渲染对象的 TCP 世界坐标」与「纯数学 FK」对比，作为 Phase 3 的运行态证据。
 */
import { useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import { useFrame } from '@react-three/fiber';
import { endEffectorPosition } from '@robot/index';
import { useRobotStore } from '@/store/robotStore';
import { applyJointState, buildRobotObject3D, disposeRobotObject3D } from './buildRobotObject3D';

const ALIGNMENT_CHECK_INTERVAL_SEC = 0.25;

export function RobotArm() {
  const model = useRobotStore((s) => s.model);
  const joints = useRobotStore((s) => s.commandJoints);
  const showJointAxes = useRobotStore((s) => s.showJointAxes);
  const showJointOrigins = useRobotStore((s) => s.showJointOrigins);
  const showTcp = useRobotStore((s) => s.showTcp);
  const setAlignmentError = useRobotStore((s) => s.setAlignmentError);

  // 构树后**立刻**应用一次当前关节状态：否则会出现"首帧仍为零位、随后才跳到目标位姿"
  // 的瞬态（useFrame 的第一次采样可能落在 applyJointState 的 effect 之前）。
  const objects = useMemo(() => {
    const built = buildRobotObject3D(model);
    applyJointState(built, model, useRobotStore.getState().commandJoints);
    return built;
  }, [model]);

  useEffect(() => () => disposeRobotObject3D(objects), [objects]);

  // 关节状态 -> JointGroup 旋转
  useEffect(() => {
    applyJointState(objects, model, joints);
  }, [objects, model, joints]);

  useEffect(() => {
    for (const helper of objects.jointAxisHelpers) helper.visible = showJointAxes;
  }, [objects, showJointAxes]);

  useEffect(() => {
    for (const helper of objects.jointOriginHelpers) helper.visible = showJointOrigins;
  }, [objects, showJointOrigins]);

  useEffect(() => {
    objects.tcpMarker.visible = showTcp;
  }, [objects, showTcp]);

  const latest = useRef({ model, joints });
  latest.current = { model, joints };
  const lastCheckAt = useRef(-1);

  useFrame(({ clock }) => {
    const now = clock.getElapsedTime();
    if (now - lastCheckAt.current < ALIGNMENT_CHECK_INTERVAL_SEC) return;
    lastCheckAt.current = now;

    const { model: m, joints: j } = latest.current;
    objects.root.updateMatrixWorld(true);
    const actual = new THREE.Vector3().setFromMatrixPosition(objects.tcpMarker.matrixWorld);
    const expected = endEffectorPosition(m, j);
    setAlignmentError(
      Math.hypot(actual.x - expected[0], actual.y - expected[1], actual.z - expected[2]),
    );
  });

  return <primitive object={objects.root} />;
}
