/// <reference types="vite/client" />

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
