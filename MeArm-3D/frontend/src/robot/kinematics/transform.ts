/**
 * 4x4 齐次变换矩阵（列主序，与 Three.js `Matrix4.elements` 布局一致）。
 *
 * 为什么自己写而不是直接用 Three.js：机器人运动学层（FK/IK）必须**不依赖渲染库**，
 * 才能在 node 中独立、快速地做验收测试，也便于将来在 Go 后端 / 固件侧镜像同一套算法。
 *
 * 约定：
 *   - 矩阵乘法 `a·b` 表示“先施加 b，再施加 a”（与数学习惯一致）
 *   - 欧拉角使用 intrinsic XYZ，即 R = Rx·Ry·Rz，与 Three.js `Euler` order `'XYZ'` 完全一致
 *     （这是**缺省**约定 `'xyz'`；`'rpy'` = fixed-axis XYZ = Rz·Ry·Rx，见 `mat4Rpy`，
 *      用于原样承载 URDF `<origin rpy>` 的数值。调用方一律走 `mat4EulerByConvention`）
 *   - 旋转轴按右手定则，角度为弧度（对外 API 用 degree，转换集中在本文件与 coordinate.ts）
 */
import type { EulerDeg, RotationConvention, Vec3 } from '../model/Pose';
import { degToRad } from '../model/Pose';

/** 列主序 4x4 矩阵，长度 16；m[col * 4 + row] */
export type Mat4 = number[];

export function mat4Identity(): Mat4 {
  return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

export function mat4Translation(position: Vec3): Mat4 {
  const m = mat4Identity();
  m[12] = position[0];
  m[13] = position[1];
  m[14] = position[2];
  return m;
}

/** 矩阵乘法 out = a · b */
export function mat4Multiply(a: Mat4, b: Mat4): Mat4 {
  const out = new Array<number>(16);
  for (let col = 0; col < 4; col += 1) {
    for (let row = 0; row < 4; row += 1) {
      let sum = 0;
      for (let k = 0; k < 4; k += 1) {
        sum += a[k * 4 + row]! * b[col * 4 + k]!;
      }
      out[col * 4 + row] = sum;
    }
  }
  return out;
}

export function mat4MultiplyAll(...mats: Mat4[]): Mat4 {
  return mats.reduce((acc, m) => mat4Multiply(acc, m), mat4Identity());
}

/** 绕任意轴（右手定则）旋转 angleRad。轴会自动归一化；零向量返回单位阵。 */
export function mat4AxisAngle(axis: Vec3, angleRad: number): Mat4 {
  const len = Math.hypot(axis[0], axis[1], axis[2]);
  if (len === 0) return mat4Identity();

  const x = axis[0] / len;
  const y = axis[1] / len;
  const z = axis[2] / len;
  const c = Math.cos(angleRad);
  const s = Math.sin(angleRad);
  const t = 1 - c;

  const m = mat4Identity();
  m[0] = t * x * x + c;
  m[1] = t * x * y + s * z;
  m[2] = t * x * z - s * y;

  m[4] = t * x * y - s * z;
  m[5] = t * y * y + c;
  m[6] = t * y * z + s * x;

  m[8] = t * x * z + s * y;
  m[9] = t * y * z - s * x;
  m[10] = t * z * z + c;

  return m;
}

/** intrinsic XYZ 欧拉角（degree）→ 旋转矩阵，R = Rx·Ry·Rz（与 Three.js 'XYZ' 一致） */
export function mat4EulerXYZ(rotationDeg: EulerDeg): Mat4 {
  const [rx, ry, rz] = rotationDeg;
  if (rx === 0 && ry === 0 && rz === 0) return mat4Identity();
  const a = Math.cos(degToRad(rx));
  const b = Math.sin(degToRad(rx));
  const c = Math.cos(degToRad(ry));
  const d = Math.sin(degToRad(ry));
  const e = Math.cos(degToRad(rz));
  const f = Math.sin(degToRad(rz));

  const m = mat4Identity();
  m[0] = c * e;
  m[4] = -c * f;
  m[8] = d;

  m[1] = a * f + b * e * d;
  m[5] = a * e - b * f * d;
  m[9] = -b * c;

  m[2] = b * f - a * e * d;
  m[6] = b * e + a * f * d;
  m[10] = a * c;

  return m;
}

/**
 * fixed-axis（extrinsic）XYZ 欧拉角（degree）→ 旋转矩阵，`R = Rz · Ry · Rx`。
 *
 * 这就是 **URDF `<origin rpy="roll pitch yaw">`** 的约定，也等价于"先绕 X 转 roll、
 * 再绕**固定** Y 转 pitch、最后绕**固定** Z 转 yaw"。
 *
 * ⚠️ 与 `mat4EulerXYZ`（intrinsic，`R = Rx · Ry · Rz`）**不是同一种参数化**：
 *   extrinsic XYZ(r,p,y) ≡ intrinsic ZYX(y,p,r)。除少数特例外两者数值不等，
 *   所以「用哪个」必须由配置显式声明，不能靠猜。
 */
export function mat4Rpy(rotationDeg: EulerDeg): Mat4 {
  const [rx, ry, rz] = rotationDeg;
  if (rx === 0 && ry === 0 && rz === 0) return mat4Identity();
  const a = Math.cos(degToRad(rx));
  const b = Math.sin(degToRad(rx));
  const c = Math.cos(degToRad(ry));
  const d = Math.sin(degToRad(ry));
  const e = Math.cos(degToRad(rz));
  const f = Math.sin(degToRad(rz));

  // R = Rz·Ry·Rx，按列主序写入（m[col*4+row]）
  const m = mat4Identity();
  m[0] = e * c; //  R00
  m[1] = f * c; //  R10
  m[2] = -d; //     R20

  m[4] = -f * a + e * d * b; //  R01
  m[5] = e * a + f * d * b; //   R11
  m[6] = c * b; //               R21

  m[8] = f * b + e * d * a; //   R02
  m[9] = -e * b + f * d * a; //  R12
  m[10] = c * a; //              R22

  return m;
}

/**
 * 按**显式约定**把欧拉角三元组变成旋转矩阵。
 *
 * 这是唯一应当被 FK / 渲染层调用的入口 —— 直接调 `mat4EulerXYZ` 等于把约定
 * 硬编码成 intrinsic，那么任何 `'rpy'` 来源的配置都会被**静默**按错误约定解读。
 * 缺省参数保证既有调用语义不变。
 */
export function mat4EulerByConvention(
  rotationDeg: EulerDeg,
  convention: RotationConvention = 'xyz',
): Mat4 {
  return convention === 'rpy' ? mat4Rpy(rotationDeg) : mat4EulerXYZ(rotationDeg);
}

/**
 * 取旋转矩阵的 3x3 部分（**行主序** 9 个数）。
 *
 * 用途：跨实现比对姿态。欧拉角三元组在万向锁附近会跳变、且同一旋转有多组等价解，
 * 直接比三元组会得到"其实一致却判为不一致"的假失败；比矩阵没有这个问题。
 */
export function mat4RotationMatrix3(m: Mat4): number[] {
  return [m[0]!, m[4]!, m[8]!, m[1]!, m[5]!, m[9]!, m[2]!, m[6]!, m[10]!];
}

/** 两个旋转的两个 3x3 之间的最大逐元素绝对差（行主序输入） */
export function rotationMatrixMaxAbsDiff(a: readonly number[], b: readonly number[]): number {
  let worst = 0;
  for (let i = 0; i < 9; i += 1) worst = Math.max(worst, Math.abs((a[i] ?? 0) - (b[i] ?? 0)));
  return worst;
}

export function mat4TransformPoint(m: Mat4, p: Vec3): Vec3 {  return [
    m[0]! * p[0] + m[4]! * p[1] + m[8]! * p[2] + m[12]!,
    m[1]! * p[0] + m[5]! * p[1] + m[9]! * p[2] + m[13]!,
    m[2]! * p[0] + m[6]! * p[1] + m[10]! * p[2] + m[14]!,
  ];
}

export function mat4GetPosition(m: Mat4): Vec3 {
  return [m[12]!, m[13]!, m[14]!];
}

/**
 * 从旋转矩阵提取 intrinsic XYZ 欧拉角（degree）。
 * 与 Three.js `Euler.setFromRotationMatrix(..., 'XYZ')` 使用同一分支逻辑，
 * 因此结果可与 Three.js 逐值对比（见 tests/acceptance/fk-three-alignment.test.ts）。
 */
export function mat4GetEulerXYZ(m: Mat4): EulerDeg {
  // 按行主序命名：m11=m[0] m12=m[4] m13=m[8] / m21=m[1] ... m33=m[10]
  const m11 = m[0]!;
  const m12 = m[4]!;
  const m13 = m[8]!;
  const m23 = m[9]!;
  const m22 = m[5]!;
  const m32 = m[6]!;
  const m33 = m[10]!;

  const y = Math.asin(Math.max(-1, Math.min(1, m13)));
  let x: number;
  let z: number;

  if (Math.abs(m13) < 0.9999999) {
    x = Math.atan2(-m23, m33);
    z = Math.atan2(-m12, m11);
  } else {
    // 万向锁：把 z 归零
    x = Math.atan2(m32, m22);
    z = 0;
  }

  const toDeg = 180 / Math.PI;
  return [x * toDeg, y * toDeg, z * toDeg];
}

/** 矩阵 -> 位姿（位置 mm / 欧拉角 degree） */
export function mat4ToTransform(m: Mat4): { position: Vec3; rotation: EulerDeg } {
  return { position: mat4GetPosition(m), rotation: mat4GetEulerXYZ(m) };
}
