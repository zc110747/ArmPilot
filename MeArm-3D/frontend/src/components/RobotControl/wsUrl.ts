/**
 * WebSocket 地址推导（从 ConnectionControl.tsx 抽出，供自动连接复用）。
 *
 * 为什么不写死 `ws://localhost:8090`：局域网访问时浏览器会把 `localhost`
 * 解析成**访问者自己那台设备**，表现为"页面能开、但一直重连"。
 * 按 `window.location.hostname` 推导即可让局域网访问自动指向正确的机器。
 */
export function getDefaultWsUrl(): string {
  const fromEnv = import.meta.env?.VITE_WS_URL as string | undefined;
  if (fromEnv) return fromEnv;

  // 非浏览器环境（单测 / SSR）：退回字面量默认值
  const loc = globalThis.location;
  if (!loc?.hostname) return 'ws://localhost:8090/ws/joint';

  const proto = loc.protocol === 'https:' ? 'wss:' : 'ws:';
  return `${proto}//${loc.hostname}:8090/ws/joint`;
}
