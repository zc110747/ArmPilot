/**
 * 照片纹理注册表：把 `assets/textures/**` 下的图片**静态登记**为 URL。
 *
 * 为什么要注册表而不是运行时拼路径：
 * Vite 必须能在**构建期**看到 import 才能把资源打进产物。`robot.yaml` 里的 `texture`
 * 是运行时字符串，`import(url)` 这种动态形式 Vite 无法分析 ⇒ 用 `import.meta.glob`
 * 一次性把整个目录登记进来，yaml 里写的 key 再来查表。
 * 好处：**新增纹理只丢文件进 assets 即可，无需改任何代码。**
 *
 * 查不到时返回 `undefined`（调用方回退纯色）—— 纹理缺失不该让整个场景挂掉，
 * 这跟"探针只准用真实现象做证据"是同一个取向：宁可缺一块装饰，也不能白屏。
 */
const MODULES = import.meta.glob('../../../../assets/textures/**/*.png', {
  eager: true,
  query: '?url',
  import: 'default',
}) as Record<string, string>;

/** key（相对 assets/textures 的路径，正斜杠）→ 构建后的 URL */
const URL_BY_KEY = new Map<string, string>();

/**
 * 取标记之后的子串作为 key，而不是直接 slice 掉 `TEXTURE_ROOT` 的长度：
 * `import.meta.glob` 返回的 key 是否保留 `../` 前缀属于 Vite 的实现细节，
 * 用标记定位对「相对形式」与「绝对形式」两种 key 都成立。
 */
const MARKER = 'assets/textures/';

for (const [modulePath, url] of Object.entries(MODULES)) {
  const index = modulePath.lastIndexOf(MARKER);
  if (index < 0) continue;
  URL_BY_KEY.set(modulePath.slice(index + MARKER.length), url);
}

/** 解析纹理 key 为可用的 URL；未登记返回 undefined */
export function resolveTextureUrl(key: string): string | undefined {
  return URL_BY_KEY.get(key);
}

/** 已登记的全部纹理 key（升序）—— 供自检 / 面板列出可用纹理 */
export function listTextureKeys(): string[] {
  return [...URL_BY_KEY.keys()].sort();
}
