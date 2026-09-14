/**
 * 网格注册表：把 `assets/models/**` 下的网格文件**静态登记**为 URL。
 *
 * 与 `textureRegistry.ts` 同一范式、同一纪律（Vite 必须在**构建期**看到 import
 * 才能把资源打进产物；运行时拼路径 `import(url)` 它无法分析）：
 * 用 `import.meta.glob` 一次性登记整个目录，yaml 里写的 `file` 再来查表。
 * 好处：**新增机器人模型只丢文件进 assets/models/ 即可，无需改任何代码。**
 *
 * 与纹理的关键差别 —— **网格比纹理大三个数量级**（本题 13 个 STL 共 15.4 MiB）：
 *
 * 1. 登记的是 **URL**（`?url`）而不是内容。纹理也走 `?url`，但网格必须如此：
 *    若把 15 MB 二进制内联进 JS，首屏包会直接失控。
 * 2. 所以 `eager: true` 在这里**只等价于"把 13 条 URL 字符串静态写下来"**，
 *    不加载任何网格数据 —— 代价可忽略，换取"不需要异步拿 URL"这一简化。
 *
 * 查不到时返回 `undefined`（调用方回退为"这个件不画"）—— 与纹理同取向：
 * 宁可少画一个件，也不能让整个场景崩掉或白屏。
 */
const MODULES = import.meta.glob('../../../../assets/models/**/*.stl', {
  eager: true,
  query: '?url',
  import: 'default',
}) as Record<string, string>;

/** key（相对 assets/models 的路径，正斜杠）→ 构建后的 URL */
const URL_BY_KEY = new Map<string, string>();

/**
 * 用标记定位而不是 `slice(MESH_ROOT.length)`：`import.meta.glob` 返回的 key
 * 是否保留 `../` 前缀属于 Vite 的实现细节，用标记定位对相对 / 绝对两种 key 都成立。
 */
const MARKER = 'assets/models/';

for (const [modulePath, url] of Object.entries(MODULES)) {
  const index = modulePath.lastIndexOf(MARKER);
  if (index < 0) continue;
  URL_BY_KEY.set(modulePath.slice(index + MARKER.length), url);
}

/** 解析网格 key 为可用的 URL；未登记返回 undefined */
export function resolveMeshUrl(key: string): string | undefined {
  return URL_BY_KEY.get(key);
}

/** 已登记的全部网格 key（升序）—— 供自检 / 面板列出可用网格 */
export function listMeshKeys(): string[] {
  return [...URL_BY_KEY.keys()].sort();
}
