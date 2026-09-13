/**
 * Link（连杆）——纯几何量。
 *
 * `length` 是**运动学量**：本连杆「近端关节坐标系 → 远端关节坐标系」的距离，
 * 方向为近端坐标系 +Z。它必须等于其子关节 `origin.position` 的模长（由模型校验强制），
 * 以保证「连杆长度」与「关节位置」不会出现两个互相矛盾的真值来源。
 *
 * `geometry` / `details` 只用于 3D 显示，**不参与任何运动学计算**。
 * 全部件由参数化基础体（plate / box / cylinder / sphere / servo）拼装，
 * 后续可替换为 GLB/GLTF/OBJ 而不影响运动学（spec §29 / §30）。
 */
import type { EulerDeg, Vec3 } from './Pose';

/**
 * 倒角薄板：低多边形工程外观的主力件（底盘 / 立柱侧板 / 臂板 / 爪片）。
 * 对应 three.js `RoundedBoxGeometry`。`size = [x, y, z]`（mm），位置为几何中心。
 */
export interface PlateGeometry {
  type: 'plate';
  size: Vec3;
  /** 倒角半径（mm）；缺省用 DEFAULT_PLATE_CORNER_RADIUS，渲染时按最小边自动钳位 */
  cornerRadius?: number;
  position?: Vec3;
  rotation?: EulerDeg;
  color?: string;
  /**
   * 照片纹理 key —— 相对 `assets/textures/` 的路径，如 `mearm/tiles/upper_arm_link.png`。
   *
   * **纯外观字段**：不参与 FK / IK / 标定 / 物理（D55 冻结运动学与物理层）。
   * 由 `textureRegistry` 静态登记为 URL；查不到时静默回退纯色，不会让场景崩掉。
   */
  texture?: string;
  /**
   * 纹理整体水平 / 垂直翻转。
   *
   * 为什么需要它：`make_texture.py` 产出的 tile 是「四边形 → 矩形」的正投影图，
   * 其 u/v 轴指向由**拍照时机位与臂姿**决定（板长边在画面里是竖是横，
   * 决定 `build_tile` 是否 `rotate(-90)`，两种情况 u/v 语义不同）。
   * 这是**外观事实**，不是可推导量 ⇒ 放配置里，不写死在代码。
   * （渲染层另外会自动处理「盒体两个大面的 uv 互为镜像」这一几何必然，见 buildRobotObject3D。）
   */
  textureFlipU?: boolean;
  textureFlipV?: boolean;
}

/**
 * 舵机壳体：壳体 + 两侧安装耳 + 金属输出轴 + 舵盘。
 * **本体局部坐标系约定：`+Z` 为输出轴方向，原点在壳体中心。**
 * 因此「轴朝上」不写 rotation，「轴朝 +Y」写 rotation: [-90, 0, 0]。
 */
export interface ServoGeometry {
  type: 'servo';
  /** 壳体尺寸 [x, y, z]（mm），缺省为常见微型舵机 DEFAULT_SERVO_SIZE */
  size?: Vec3;
  /** 输出轴伸出长度（mm），缺省 DEFAULT_SERVO_SHAFT_LENGTH */
  shaftLength?: number;
  /** 是否绘制两侧安装耳，缺省 true */
  ears?: boolean;
  position?: Vec3;
  rotation?: EulerDeg;
  /** 壳体颜色，缺省 DEFAULT_SERVO_COLOR */
  color?: string;
  /** 输出轴 / 舵盘颜色，缺省 DEFAULT_SERVO_SHAFT_COLOR */
  shaftColor?: string;
}

/**
 * 夹爪单片爪：**平面轮廓挤出件**（照片反解形状，纯外观层）。
 *
 * 为什么它不能由 plate/box 拼出来：实拍（2026-09-13，5 张近景）显示爪片是**激光切割的
 * 单块亚克力**，轮廓上有三类特征量，盒子几何表达不了 —— 根部整圈方齿的齿轮盘、
 * 内侧缘一排锯齿、末端斜切收尖。这些是**形状**而不是纹理（黑件在照片里近纯黑，
 * 没有任何可提取的贴图信息，见 ADR D64），所以必须参数化轮廓而不是贴图。
 *
 * 本体局部坐标系（与渲染层的 pivot 一致）：**原点 = 齿轮盘中心**，
 * `+Z` = 爪伸出方向，`+Y` = **向外**（远离另一片爪）。
 *
 * 三条**由几何自洽给出**的派生关系（渲染层推导，不写死在配置里）：
 * 1. 两齿轮中心距 = `2 × 分度圆半径`（标准啮合 ⇒ 齿顶插入对方齿槽）；
 * 2. 爪指中线相对齿轮中心**内偏** δ = `分度圆半径 − width / 2`；
 * 3. δ 的取值使 `θ=0` 时两爪**内侧缘正好贴合**（不靠额外配平）。
 *
 * ⚠️ 与 `plate` 的 `size` 语义不同：`length` 量的是**齿轮中心 → 爪尖**，
 * 而不是某条边的长度。`jaw_link.length`（运动学量）仍恒为 0，不受影响。
 */
export interface JawGeometry {
  type: 'jaw';
  /** 齿轮中心 → 爪尖（mm），沿局部 +Z */
  length: number;
  /** 板厚（mm），沿局部 Y（= 铰轴方向） */
  thickness?: number;
  /** 爪指根部全宽（mm）—— 决定分度圆半径与闭合间隙，见上 */
  width?: number;
  /** 爪尖全宽（mm） */
  tipWidth?: number;
  /** 齿轮盘**齿顶**圆半径（mm） */
  gearRadius?: number;
  /** 齿轮盘齿数（整圈） */
  gearTeeth?: number;
  /** 齿高（齿顶圆 − 齿根圆，mm） */
  toothDepth?: number;
  /** 内侧缘锯齿数量（0 = 光滑） */
  serrations?: number;
  /** 锯齿齿高（mm） */
  serrationDepth?: number;
  /** 锯齿所在区间，占 `length` 的比例 [起, 止]；缺省 [0.4, 0.9] */
  serrationSpan?: [number, number];
  /** 尖端斜切量（mm）—— 实拍爪尖是**斜切**而非平口 */
  tipSkew?: number;
  /** 外侧缘「脖子」的内缩量（mm）—— 实拍爪指出齿轮盘后先收窄再伸出去 */
  neckInset?: number;
  /** 脖子位置（占 `length` 的比例） */
  neckAt?: number;
  /** 齿轮盘中心装饰镂空半径（0 = 实心） */
  holeRadius?: number;
  position?: Vec3;
  rotation?: EulerDeg;
  color?: string;
}

export type LinkGeometry =
  | { type: 'none'; color?: string }
  | { type: 'box'; size: Vec3; position?: Vec3; rotation?: EulerDeg; color?: string }
  | PlateGeometry
  | {
      type: 'cylinder';
      radius: number;
      height: number;
      radialSegments?: number;
      position?: Vec3;
      rotation?: EulerDeg;
      color?: string;
    }
  | { type: 'sphere'; radius: number; position?: Vec3; rotation?: EulerDeg; color?: string }
  | ServoGeometry
  | JawGeometry;

export interface Link {
  id: string;
  name: string;
  /** 父连杆 id；根连杆为 null / undefined */
  parent?: string | null;
  /** 连杆长度（mm），沿近端关节坐标系 +Z */
  length: number;
  geometry: LinkGeometry;
  /** 附加显示件（舵机 / 螺栓 / 轴销等），挂在同一坐标系下，仅用于 3D 显示 */
  details?: LinkGeometry[];
}

// ---------------------------------------------------------------------------
// 默认值（全部为参数化默认，可在 robot.yaml 中逐项覆盖）
// ---------------------------------------------------------------------------

export const DEFAULT_LINK_COLOR = '#8b93a7';
/** 薄板默认倒角半径（mm）——「边缘适当倒角」的最小成本实现（spec §5） */
export const DEFAULT_PLATE_CORNER_RADIUS = 1.6;
/** 常见微型舵机壳体尺寸（SG90 / MG90S 量级），仅作默认值，非实测 */
export const DEFAULT_SERVO_SIZE: Vec3 = [22.8, 12.2, 22.5];
export const DEFAULT_SERVO_SHAFT_LENGTH = 5;
export const DEFAULT_SERVO_COLOR = '#1b1f28';
export const DEFAULT_SERVO_SHAFT_COLOR = '#c9ced8';
/**
 * 单片爪的缺省参数（mm）—— `JawGeometry` 里省略任一项时取此值。
 *
 * 量级来自 **2026-09-13 五张实拍近景**的比例反解（见 ADR D66）：
 * 齿轮直径 / 爪长 = 24 / 34 = 0.71，实拍 246 / 359 px = 0.69；
 * 齿轮直径 / 爪指宽 = 24 / 9 = 2.67，实拍 246 / 69 px = 3.6（同一量级）。
 * 绝对尺度锚在**已有配置**上（`length` 34 沿用改前的爪片长度），
 * 不用照片推绝对尺寸 —— 斜视角 + 遮挡下的像素测量不足以定 mm。
 */
export const DEFAULT_JAW_SPEC = {
  length: 34,
  thickness: 4.5,
  width: 9,
  tipWidth: 3,
  gearRadius: 12,
  gearTeeth: 16,
  toothDepth: 3,
  serrations: 6,
  serrationDepth: 1.2,
  serrationSpan: [0.4, 0.9] as [number, number],
  tipSkew: 2.6,
  neckInset: 1.6,
  neckAt: 0.32,
  holeRadius: 3.4,
} as const;

export function isRootLink(link: Link): boolean {
  return link.parent === null || link.parent === undefined;
}

/** 附加显示件（缺省空数组，避免调用方到处写 `?? []`） */
export function linkDetails(link: Link): LinkGeometry[] {
  return link.details ?? [];
}

/**
 * 显示几何相对近端关节坐标系的平移偏移。
 * 省略 `position` 时按「连杆中段」放置（0, 0, length/2），这是最符合直觉的默认值。
 */
export function geometryPosition(geometry: LinkGeometry, linkLength = 0): Vec3 {
  if (geometry.type === 'none') return [0, 0, 0];
  if (geometry.position) return [...geometry.position];
  return [0, 0, linkLength / 2];
}

/** 显示几何相对近端关节坐标系的旋转（缺省零） */
export function geometryRotation(geometry: LinkGeometry): EulerDeg {
  if (geometry.type === 'none') return [0, 0, 0];
  return geometry.rotation ? [...geometry.rotation] : [0, 0, 0];
}

export function geometryColor(geometry: LinkGeometry): string {
  return geometry.color ?? DEFAULT_LINK_COLOR;
}

/** 薄板倒角半径：按最小边自动钳位，防止半径大于半边长导致几何自交 */
export function plateCornerRadius(geometry: PlateGeometry): number {
  const requested = geometry.cornerRadius ?? DEFAULT_PLATE_CORNER_RADIUS;
  const halfMin = Math.min(...geometry.size) / 2;
  if (!(halfMin > 0)) return 0;
  return Math.max(0, Math.min(requested, halfMin - 0.01));
}

/** 薄板照片纹理规格（补齐缺省，渲染层不再关心 `?? false`） */
export interface PlateTextureSpec {
  /** 相对 `assets/textures/` 的 key */
  key: string;
  flipU: boolean;
  flipV: boolean;
}

/** 薄板是否配了照片纹理；未配返回 null */
export function plateTexture(geometry: PlateGeometry): PlateTextureSpec | null {
  if (!geometry.texture) return null;
  return {
    key: geometry.texture,
    flipU: geometry.textureFlipU ?? false,
    flipV: geometry.textureFlipV ?? false,
  };
}

/** 舵机渲染参数（补齐全部默认值，渲染层不再关心缺省逻辑） */
export interface ServoRenderSpec {
  size: Vec3;
  shaftLength: number;
  ears: boolean;
  bodyColor: string;
  shaftColor: string;
}

export function servoRenderSpec(geometry: ServoGeometry): ServoRenderSpec {
  return {
    size: geometry.size ? [...geometry.size] : [...DEFAULT_SERVO_SIZE],
    shaftLength: geometry.shaftLength ?? DEFAULT_SERVO_SHAFT_LENGTH,
    ears: geometry.ears ?? true,
    bodyColor: geometry.color ?? DEFAULT_SERVO_COLOR,
    shaftColor: geometry.shaftColor ?? DEFAULT_SERVO_SHAFT_COLOR,
  };
}

/**
 * 单片爪的渲染参数 —— 缺省补齐**与三条自洽关系的推导**都在这里一次算完，
 * 渲染层只消费结果，不再关心 `?? default` 与「哪个量该等于哪个量」。
 */
export interface JawRenderSpec {
  /** 齿轮中心 → 爪尖 */
  length: number;
  thickness: number;
  /** 爪指根部全宽 */
  width: number;
  tipWidth: number;
  /** 齿顶圆半径 */
  gearRadius: number;
  /** 分度圆半径 */
  pitchRadius: number;
  /** 齿根圆半径 */
  gearRootRadius: number;
  gearTeeth: number;
  /** 爪指中线相对**齿轮中心**的内偏（mm）—— 使 θ=0 时两爪内侧缘贴合 */
  fingerInset: number;
  /** 齿轮中心相对**掌心中线**的偏移（mm）= 分度圆半径 —— 使两齿轮标准啮合 */
  hubOffsetY: number;
  serrations: number;
  serrationDepth: number;
  /** 锯齿区间（占 length 的比例） */
  serrationSpan: [number, number];
  tipSkew: number;
  /** 外侧缘脖子内缩量 */
  neckInset: number;
  /** 脖子位置（占 length 比例） */
  neckAt: number;
  holeRadius: number;
}

export function jawRenderSpec(link: Link | undefined): JawRenderSpec {
  const geometry = link?.geometry;
  const jaw = geometry?.type === 'jaw' ? geometry : undefined;

  const width = jaw?.width ?? DEFAULT_JAW_SPEC.width;
  const gearRadius = jaw?.gearRadius ?? DEFAULT_JAW_SPEC.gearRadius;
  const toothDepth = jaw?.toothDepth ?? DEFAULT_JAW_SPEC.toothDepth;
  const pitchRadius = gearRadius - toothDepth / 2;

  return {
    length: jaw?.length ?? DEFAULT_JAW_SPEC.length,
    thickness: jaw?.thickness ?? DEFAULT_JAW_SPEC.thickness,
    width,
    tipWidth: jaw?.tipWidth ?? DEFAULT_JAW_SPEC.tipWidth,
    gearRadius,
    pitchRadius,
    // 齿根圆由「齿顶 + 齿根关于分度圆对称」定：两齿轮中心距 2·rp 时齿顶刚好落到对方齿根
    gearRootRadius: 2 * pitchRadius - gearRadius,
    gearTeeth: jaw?.gearTeeth ?? DEFAULT_JAW_SPEC.gearTeeth,
    // ★ 闭合约束：两爪各自内偏 (rp − w/2)，则 θ=0 时两内侧缘在中线处贴合
    fingerInset: pitchRadius - width / 2,
    // ★ 啮合约束：齿轮中心间距 = 2·rp
    hubOffsetY: pitchRadius,
    serrations: jaw?.serrations ?? DEFAULT_JAW_SPEC.serrations,
    serrationDepth: jaw?.serrationDepth ?? DEFAULT_JAW_SPEC.serrationDepth,
    serrationSpan: jaw?.serrationSpan
      ? [jaw.serrationSpan[0], jaw.serrationSpan[1]]
      : [DEFAULT_JAW_SPEC.serrationSpan[0], DEFAULT_JAW_SPEC.serrationSpan[1]],
    tipSkew: jaw?.tipSkew ?? DEFAULT_JAW_SPEC.tipSkew,
    neckInset: jaw?.neckInset ?? DEFAULT_JAW_SPEC.neckInset,
    neckAt: jaw?.neckAt ?? DEFAULT_JAW_SPEC.neckAt,
    holeRadius: jaw?.holeRadius ?? DEFAULT_JAW_SPEC.holeRadius,
  };
}
