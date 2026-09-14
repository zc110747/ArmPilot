# 板件纹理资产

```
raw/     你用相机 / 手机拍的原图（**不要手动裁切、旋转、调色**）
tiles/   robot-package/mearm-v1/tools/make_texture.py 生成的贴图（已按真实尺寸归一化）
```

## 怎么用

1. 读 `docs/texture-capture-guide.md` —— 拍摄清单（拍哪块板 / 大面朝向 / 四项硬性要求 / 自查清单）
2. 文件名权威清单：`python robot-package/mearm-v1/tools/make_texture.py --list`
3. 照片放进 `raw/<name>.jpg`
4. `python robot-package/mearm-v1/tools/make_texture.py --all`
5. **打开 `.workbuddy/analysis/texture/<name>_corners.png` 复核红框是否贴合板的四条边**
   —— 不贴合就用 `--corners` 手动指定，**不要硬调阈值凑**
6. `tiles/<name>.png` 即最终贴图，随后由前端按面贴到对应板件的大面上

## 为什么原图不要预处理

透视校正、裁切、归一化全部在 `robot-package/mearm-v1/tools/make_texture.py` 里做，依据是照片里板的
**四个角的几何位置**。手动裁切会丢掉边界信息，反而让自动找角变难；
手动调色则会破坏各板件之间的一致性（贴到同一个模型上会显得像拼凑的）。

## 当前状态

尚未采集任何照片。最早的 4 张（`upper_arm_link` / `forearm_link` / `base_link` /
`column_link`）就足够验证整条链路并看出八成效果。
