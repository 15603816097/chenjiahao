# Chenjiahao · 车载多传感器融合状态估计

从零构建可复现的车载 GNSS/IMU/LiDAR/Camera 融合状态估计研究。研究顺序：**数据验证 → 固定噪声因子图 → 退化与传统鲁棒基线 → 可靠性预测 → 动态融合 → 跨序列实验**。Mamba 是候选方法，不提前承诺其必然有效。

## 当前进度（2026-10-10）

| 阶段 | 状态 | 已验证内容 |
| --- | --- | --- |
| Stage 0A 数据完整性 | 已完成 | 4 个序列帧数/编号一致；必要标定文件存在 |
| Stage 0B 时间分析 | 已完成初步诊断 | 采样时间单调；0027 前几帧有超过 50 ms 的跨传感器偏差，待制定时间关联方案 |
| Stage 0C OXTS/标定检查 | 已完成初步诊断 | 13 个本地单测通过、4 个序列成功解析，未触发程序物理异常告警；RGB 双目基线 0.532332 m |
| Stage 1A WGS84/ECEF/ENU | 已通过 | 18 项本地测试通过；四序列 ENU 验证完成，同源速度差分一致性正常 |
| Stage 1B GNSS/IMU 因子图 | 40 帧试运行完成，误差待诊断 | 0027: 121 因子；目标函数 106.014→67.647；OXTS 输入位置拟合 RMSE 6.372 m，不能作为独立 ATE |
| Stage 2 四传感器基线 | 待开始 | 独立的 LiDAR/双目里程计和统一因子图 |
| Stage 3–5 鲁棒性、学习与论文实验 | 待开始 | 根据前一阶段证据决定是否引入 GRU/Mamba |

本地 KITTI Raw 目录：`dataset/kitti/2011_10_03/`，包含 `0027`（4544 帧）、`0034`（4663 帧）、`0042`（1170 帧）、`0047`（837 帧），每段采用 `2011_10_03_drive_XXXX_sync` 命名。**数据集不上传至 GitHub。**

## 当前必要目录

```text
chenjiahao/
├── dataset/              # 本地 KITTI 数据，Git 忽略
├── src/
│   ├── coordinates.py     # WGS84 → ECEF → ENU，单位米
│   └── gnss_imu.py        # 固定噪声 GTSAM GNSS/IMU 试运行实现
├── scripts/              # 可执行的数据验证工具
│   ├── audit_kitti.py
│   ├── analyze_timing.py
│   ├── audit_oxts.py
│   ├── validate_coordinates.py
│   └── run_gnss_imu.py
├── tests/                # 对应单元测试
│   ├── test_audit_kitti.py
│   ├── test_analyze_timing.py
│   ├── test_audit_oxts.py
│   ├── test_coordinates.py
│   └── test_gnss_imu.py
├── reports/              # 检查报告，本地生成，Git 忽略
├── requirements.txt      # Stage 1B GTSAM 与 NumPy 最小依赖
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

## Stage 1B：固定噪声 GNSS/IMU 工程试运行

当前使用 KITTI Raw 的 OXTS 同步（约 10 Hz）数据；这**不是**完整高频原始 IMU，也不是独立 GNSS 与独立参考真值的融合精度实验。`GPSFactor` 输入来自 OXTS 经纬度；IMU 因子使用同源 OXTS 的 `ax/ay/az` 与 `wx/wy/wz`。只有第一帧的 OXTS 姿态、速度用于初始化；后续姿态/速度不会用于因子图初始化或观测。其用途仅为验证因子图连接、单位、重力符号和数值稳定性。

推荐 Python 3.11 独立环境（后续考虑 Mamba 兼容），不要污染 base：

```bash
conda create -n chenjiahao python=3.11 -y
conda activate chenjiahao
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
python scripts/run_gnss_imu.py --sequence 0027 --frames 40 --iterations 30
```

结果写入 `reports/gnss_imu_stage1b.json`（被 Git 忽略）。报告包含因子图目标函数初始值与优化值、因子数量、局部 ENU 终点和**OXTS 测量残差**。该残差绝不可标注为独立定位 ATE，不能用于 ISJ 论文的真实精度提升声明。首次试运行先处理 40 帧，待通过后再讨论扩展序列与稀疏积分的替代方案。

数值/坐标约定：世界 ENU，车体前/左/上（与 KITTI IMU 坐标一致），第一帧 `yaw,pitch,roll` 使用 GTSAM 的 `Rot3.Ypr`；预积分使用 ENU 的 `MakeSharedU(g)`，重力方向为 `(0,0,-g)`；每一相邻帧使用真实时间差，用当前同步 OXTS IMU 近似零阶保持（ZOH）。噪声暂是工程固定值，未经标定与公允基线验证。

参考：[KITTI Raw 字段](https://github.com/pratikac/kitti/blob/master/readme.raw.txt) · [GTSAM IMU 因子](https://borglab.github.io/gtsam/imufactor/) · [GTSAM 重力约定](https://borglab.github.io/gtsam/preintegrationparams/)。

## Stage 1B 真实数据诊断（2026-10-11）

第一次真实 0027 前 40 帧试验已运行：初始因子图目标函数 106.014，优化后 67.647，121 个因子（4 个初始先验/首帧 GPS 因子 + 39×3 个运动、偏置、位置因子）；OXTS 输入位置拟合 RMSE 6.372 m。数值优化成功，**但 GNSS 位置拟合偏差仍较大，Stage 1B 尚不能视为物理/精度验收通过**。没有独立真值和高频 IMU，不能将其包装成论文精度提升。

为定位问题，已在 `src/gnss_imu.py` 现有函数内增加 OXTS 位置拟合的 E/N/U 分轴 RMSE、分轴均值、首末帧误差、95 分位、最大误差帧、优化前拟合 RMSE、输入与优化终点。避免另建 `_debug` 脚本。

```bash
git pull origin main
conda activate chenjiahao
python -m unittest discover -s tests -v
python scripts/run_gnss_imu.py --sequence 0027 --frames 40 --iterations 30
```

结果会覆盖本地 `reports/gnss_imu_stage1b.json`。不要急于调协方差使 RMSE 变小；先判断误差是否主要来自 ENU 高度/运动约束、初值、10 Hz IMU ZOH 积分、或者测量相关性。阶段下一步需设计未使用同源 OXTS 位置作为独立真值的评估方案，并在能获得高频原始 IMU 时升级预积分。

## 下一步

Stage 1A 已由本地 18 个单元测试与四段真实数据的坐标诊断验证。审计输出四段轨迹的 ENU 终点、3D 行驶距离、相邻位置差分与 OXTS 自带速度的一致性。**坐标系原点为各序列首帧 OXTS，坐标单位为米；未经验证的高度基准暂按椭球高近似。** 与 OXTS 速度的一致性检验不是独立定位精度评价。验证通过后再进入 Stage 1B GTSAM 固定噪声 GNSS/IMU 因子图。

参考：[KITTI Raw](https://www.cvlibs.net/datasets/kitti/raw_data.php) · [KITTI Odometry](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)
