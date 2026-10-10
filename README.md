# Chenjiahao · 车载多传感器融合状态估计

从零构建可复现的车载 GNSS/IMU/LiDAR/Camera 融合状态估计研究。研究顺序：**数据验证 → 固定噪声因子图 → 退化与传统鲁棒基线 → 可靠性预测 → 动态融合 → 跨序列实验**。Mamba 是候选方法，不提前承诺其必然有效。

## 当前进度（2026-10-10）

| 阶段 | 状态 | 已验证内容 |
| --- | --- | --- |
| Stage 0A 数据完整性 | 已完成 | 4 个序列帧数/编号一致；必要标定文件存在 |
| Stage 0B 时间分析 | 已完成初步诊断 | 采样时间单调；0027 前几帧有超过 50 ms 的跨传感器偏差，待制定时间关联方案 |
| Stage 0C OXTS/标定检查 | 已完成初步诊断 | 13 个本地单测通过、4 个序列成功解析，未触发程序物理异常告警；RGB 双目基线 0.532332 m |
| Stage 1A WGS84/ECEF/ENU | 已提交，待本地验证 | 无第三方依赖的米制 ENU 转换、4 序列 OXTS 同源运动一致性检查 |\n| Stage 1B GNSS/IMU 因子图 | 待开始 | 坐标/时间验证通过后，再设计 GTSAM 状态向量、惯性约束和固定噪声 GNSS 因子 |
| Stage 2 四传感器基线 | 待开始 | 独立的 LiDAR/双目里程计和统一因子图 |
| Stage 3–5 鲁棒性、学习与论文实验 | 待开始 | 根据前一阶段证据决定是否引入 GRU/Mamba |

本地 KITTI Raw 目录：`dataset/kitti/2011_10_03/`，包含 `0027`（4544 帧）、`0034`（4663 帧）、`0042`（1170 帧）、`0047`（837 帧），每段采用 `2011_10_03_drive_XXXX_sync` 命名。**数据集不上传至 GitHub。**

## 当前必要目录

```text
chenjiahao/
├── dataset/              # 本地 KITTI 数据，Git 忽略
├── src/
│   └── coordinates.py     # WGS84 → ECEF → ENU，单位米
├── scripts/              # 可执行的数据验证工具
│   ├── audit_kitti.py
│   ├── analyze_timing.py
│   ├── audit_oxts.py
│   └── validate_coordinates.py
├── tests/                # 对应单元测试
│   ├── test_audit_kitti.py
│   ├── test_analyze_timing.py
│   ├── test_audit_oxts.py
│   └── test_coordinates.py
├── reports/              # 检查报告，本地生成，Git 忽略
├── .gitignore
└── README.md
```

## 当前验证命令（项目根目录执行）

```bash
python3 -m unittest discover -s tests -v
python3 scripts/audit_kitti.py --sequence dataset/kitti/2011_10_03/2011_10_03_drive_0027_sync --report reports/0027_audit.json
python3 scripts/analyze_timing.py --root dataset/kitti/2011_10_03 --report reports/timing_analysis.json
python3 scripts/audit_oxts.py --root dataset/kitti/2011_10_03 --report reports/oxts_audit.json
python3 scripts/validate_coordinates.py --root dataset/kitti/2011_10_03 --report reports/coordinates_summary.json
```

阈值触发的 `REVIEW` 表示需分析，不表示数据损坏。完成脚本测试不等于完成最终测量模型与定位准确性验证。

## 实验可信度约束

1. KITTI Raw `*_sync` 的 OXTS 约为 10 Hz，**不能当作完整 100 Hz 原始 IMU**；因子图使用实际时间间隔，时间关联策略必须留档。
2. OXTS 位置、速度、姿态属于同一导航源：用 OXTS 测量做状态估计并以同源 OXTS 轨迹评价，**不属于独立真实精度验证**。合成 GNSS 退化实验须明确其评价限制。
3. 训练、验证、最终测试序列在建模前预先登记；已接触的 `0027/0034/0042/0047` 不自动被视为完全盲测数据。所有基线采用相同评估协议。
4. 坐标系、测量噪声、标定和时间偏差均通过单元测试及数值检查验证，不能依赖未经证明的固定时间步长或测量独立性假设。

## 代码与文件管理制度

- **一项功能一个正式实现**；优先修改已有模块，不创建 `_new`、`_final`、`_fix2` 等版本叠加脚本。实验版本由 Git commit 和配置记录。
- 所有核心代码遵循 `src/` 负责可复用逻辑、`scripts/` 负责可执行入口、`tests/` 负责回归测试。只在需要时新增目录，不预先创建空目录。
- 修改前检查调用方、数据格式、参数和测试；修改后运行相关测试。对外提供能直接覆盖使用的完整文件。
- 不提交 `__pycache__`、测试缓存、临时文件、原始数据、模型权重和生成报告；由 `.gitignore` 管理。旧的可复现审计脚本与测试仍有用，**不得只为减文件数量就删除**。
- 确认没有任何调用与复现价值的文件才删除，并同步修正导入路径、测试、文档。大型实验结果应由参数、随机种子、代码提交号和指标文件追溯。

## 下一步

验证 Stage 1A：先 `git pull origin main`，执行 `python3 -m unittest discover -s tests -v`，再执行 `python3 scripts/validate_coordinates.py --root dataset/kitti/2011_10_03 --report reports/coordinates_summary.json`。预期 18 个单测（尚需本机验证）。审计输出四段轨迹的 ENU 终点、3D 行驶距离、相邻位置差分与 OXTS 自带速度的一致性。**坐标系原点为各序列首帧 OXTS，坐标单位为米；未经验证的高度基准暂按椭球高近似。** 与 OXTS 速度的一致性检验不是独立定位精度评价。验证通过后再进入 Stage 1B GTSAM 固定噪声 GNSS/IMU 因子图。

参考：[KITTI Raw](https://www.cvlibs.net/datasets/kitti/raw_data.php) · [KITTI Odometry](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)
