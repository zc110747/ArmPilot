import path from 'node:path';
import { fileURLToPath } from 'node:url';
import react from '@vitejs/plugin-react';
import { defineConfig } from 'vitest/config';

const dirname = path.dirname(fileURLToPath(import.meta.url));
/** 仓库根（ArmPilot/MeArm-3D），robot.yaml 等共享资源所在处 */
const repoRoot = path.resolve(dirname, '..');

export default defineConfig({
  plugins: [react()],

  resolve: {
    // 注意顺序：'@robot' 必须排在 '@/' 之前，否则 '@' 前缀会先命中 '@robot/...'
    alias: [
      { find: '@robot', replacement: path.resolve(dirname, 'src/robot') },
      // config/robot.yaml 是前后端共享的唯一模型定义，位于仓库根而非前端目录内
      { find: '@config', replacement: path.resolve(repoRoot, 'config') },
      { find: /^@\//, replacement: `${path.resolve(dirname, 'src')}/` },
    ],
  },

  server: {
    port: 5273,
    // 监听所有网卡（0.0.0.0 / ::），否则局域网内其他设备打不开。
    // 注意：只开放 Vite 不够 —— 前端连后端的 WebSocket 地址也必须随之指向
    // 「这台机器」而不是 localhost（见 ConnectionControl.tsx 的 getDefaultWsUrl）。
    host: true,
    // 允许读取仓库根下的 config/robot.yaml（位于 vite root 之外）
    fs: { allow: [dirname, repoRoot] },
  },

  build: {
    outDir: 'dist',
    sourcemap: true,
  },

  test: {
    // 单元测试与验收测试统一放在 frontend/tests 下；
    // ★ Phase 2 步⑤ 起**同时**收集包内测试 `robot-package/<id>/tests/`。
    //
    // 为什么要开这条通道：判据"这条断言换台机器人还成立吗"不成立的断言
    // 属于**包**（如 MeArm 的被动腕 / 平行四连杆耦合事实）。没有这条 include，
    // 它们无处可去，只能继续堆在 Core 里 —— 而那正是"Core 里出现型号名"的来源。
    //
    // ⚠️ 与 Python 侧 `pytest.ini` 的 `testpaths` 是**同一条纪律的两个实现**：
    //    漏掉的话测试**存在但不执行**，`vitest run` 照样全绿。
    //    `tests/unit/packageTestChannel.test.ts` 专门盯住这一点。
    include: ['tests/**/*.test.ts', '../robot-package/*/tests/**/*.test.ts'],
    environment: 'node',
    reporters: ['default'],
  },
});
