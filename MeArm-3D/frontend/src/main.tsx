import * as THREE from 'three';
import { createRoot } from 'react-dom/client';
import App from './App';
import './styles.css';

/**
 * 全项目统一坐标系：右手系 Z-up（X 前 / Y 左 / Z 上）。
 * 必须在创建任何相机 / 轨道控制器之前设置，否则 OrbitControls 会按 Three.js 默认的
 * Y-up 计算轨道，导致「相机上下方向」与机器人 Z 轴不一致。
 * 详见 docs/coordinate-system.md。
 */
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const container = document.getElementById('root');
if (!container) throw new Error('找不到 #root 挂载点');

// 不使用 StrictMode：R3F 的 <primitive> 挂载/卸载在 StrictMode 双调用下会重复
// 触发 three 对象生命周期，且本项目每帧要做 FK 一致性校验，保持单次挂载更可预测。
createRoot(container).render(<App />);
