/// <reference types="vite/client" />

/**
 * 一键启动脚本（根目录 start.bat）注入的环境变量。
 *
 * 为什么用环境变量而不是写死在代码里：`npm run dev` 是开发者手动跑的，
 * 不该被自动改掉传输方式；而 `start.bat` 是「面向使用的一键入口」，
 * 它的语义就是"起前后端并接上真机"。用 env 让两种用法各自保持正确语义。
 */
interface ImportMetaEnv {
  /** 覆盖默认 WS 地址（见 ConnectionControl.tsx 的 getDefaultWsUrl） */
  readonly VITE_WS_URL?: string;
  /**
   * 启动后自动连接的传输方式。
   *   'ws'   = 自动连 WebSocket（配合 start.bat，连后端而非浏览器内 mock）
   *   'mock' = 自动连 MockTransport（等价于手动点）
   * 未设置 = 维持原行为（不自动连接，等用户点 Connect）
   */
  readonly VITE_AUTO_CONNECT?: string;
  /**
   * 自动连接成功后是否自动切到 Real Robot。
   *   '1' / 'true' = 切（会在链路末端不是 serial 时告警，但不阻断）
   * 仅在 VITE_AUTO_CONNECT 生效时起作用。
   */
  readonly VITE_AUTO_REAL?: string;
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}

/** 通过 `?raw` 导入的文本资源（如 config/robot.yaml） */
declare module '*?raw' {
  const content: string;
  export default content;
}

/**
 * config/robot.yaml 位于 vite root（frontend/）之外，由 vite.config.ts 的 `@config` alias 解析。
 * 这里显式声明，使 tsc 类型检查无需真的去解析该文件。
 */
declare module '@config/robot.yaml?raw' {
  const content: string;
  export default content;
}
