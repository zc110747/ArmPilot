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
    // 允许读取仓库根下的 config/robot.yaml（位于 vite root 之外）
    fs: { allow: [dirname, repoRoot] },
  },

  build: {
    outDir: 'dist',
    sourcemap: true,
  },

  test: {
    // 单元测试与验收测试统一放在 frontend/tests 下
    include: ['tests/**/*.test.ts'],
    environment: 'node',
    reporters: ['default'],
  },
});
