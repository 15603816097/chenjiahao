# Chenjiahao · 车载多传感器融合状态估计

基于 KITTI Raw 开展可复现的 GNSS/IMU/LiDAR/Camera 状态估计研究。**先保证数据、坐标系与测量模型正确，再开展鲁棒因子图及 Mamba 动态可靠性研究。**

## 当前状态（2026-10-11）

| 阶段 | 状态 | 验证结果 |
|---|---|---|
| Stage 0A 数据完整性 | 已通过 | 4 段 KITTI Raw 同步数据结构与标定文件检查通过 |
| Stage 0B 时间分析 | 初步完成 | OXTS ~10 Hz；0027 前几帧与相机/LiDAR 相差 >50 ms，需保留真实时间戳 |
| Stage 0C OXTS/标定审计 | 已通过初步检查 | 4 序列 OXTS 30 字段解析正常，RGB 双目基线 0.532332 m |
| Stage 1A WGS84 → ENU | 已通过 | 本地18个测试通过；4段轨迹和 OXTS 速度自洽性检查通过 |
| **Stage 1B 固定噪声 GTSAM** | **数值试运行成功，物理一致性待查** | 本地22个单测通过；0027 前40帧因子图运行成功；输入位置拟合 RMSE 6.372 m |
| Stage 2 四传感器 | 未开始 | 需独立 LiDAR/双目里程计 |
| Stage 3–5 退化/鲁棒性/Mamba/论文 | 未开始 | 先建立可信固定噪声基线 |

## 项目文件结构

```text
chenjiahao/
├── dataset/kitti/2011_10_03/   # 本地数据；Git 忽略
├── src/
│   ├── coordinates.py          # WGS84/ECEF/ENU 坐标转换
│   └── gnss_imu.py             # 固定噪声 GTSAM 试运行与误差诊断
├── scripts/
│   ├── audit_kitti.py          # Stage 0A 文件完整性
│   ├── analyze_timing.py       # Stage 0B 时间戳
│   ├── audit_oxts.py           # Stage 0C OXTS 物理量与标定
│   ├── validate_coordinates.py # Stage 1A ENU/轨迹验证
│   └── run_gnss_imu.py         # Stage 1B GTSAM 运行入口
├── tests/                      # 5 个对应的回归测试文件
├── reports/                    # 本地生成，不进入 Git
├── requirements.txt
├── .gitignore
└── README.md
```

这 5 个脚本各有唯一用途：**Stage 0 的审计程序仍是数据质量和论文可复现性证据，不属于废弃文件**。删除文件前检查依赖与实验复现价值；只有确实无用途的才删除。避免出现 `_new`、`_final`、`_fix` 等重复版本，历史版本由 Git 记录。

## 环境与运行

Linux / Python 3.11，GTSAM 4.3.0。建议独立 Conda 环境：

```bash
conda create -n chenjiahao python=3.11 -y
conda activate chenjiahao
python -m pip install -r requirements.txt
```

在项目根目录：

```bash
python -m unittest discover -s tests -v
python scripts/audit_kitti.py --sequence dataset/kitti/2011_10_03/2011_10_03_drive_0027_sync --report reports/0027_audit.json
python scripts/analyze_timing.py --root dataset/kitti/2011_10_03 --report reports/timing_analysis.json
python scripts/audit_oxts.py --root dataset/kitti/2011_10_03 --report reports/oxts_audit.json
python scripts/validate_coordinates.py --root dataset/kitti/2011_10_03 --report reports/coordinates_summary.json
python scripts/run_gnss_imu.py --sequence 0027 --frames 40 --iterations 30
```

模型与测试脚本只使用必要的包。不要上传 KITTI 原始数据、`__pycache__`、临时报告、模型权重等文件，已通过 `.gitignore` 排除。

## Stage 1B 已观测问题

0027 前 40 帧使用约 10 Hz 同步 OXTS 进行工程验证（非正式定位精度实验）：

- 因子图：121 因子；初始目标函数 106.0144，优化后 67.6474。
- 相对**输入 OXTS 位置**的三维拟合 RMSE：**6.3716 m**（E=4.7338、N=1.4698、U=4.0035 m）。
- 前半段位置拟合 RMSE 1.7987 m，后半段 8.8295 m；第 39 帧最大偏差 13.8492 m。
- IMU 加速度转换到 ENU（加上重力）与 OXTS 速度导数比较，三维差异 RMSE 1.3208 m/s²；主要来自 Up（RMSE 1.2900，均值 +0.7141 m/s²）。
- IMU 角速度与 OXTS 姿态导数的差异 RMSE 0.1055 rad/s。
- 这些数字只指示**同源数据的一致性与工程数值问题**，不等于独立定位误差，也不能据此断定错误来自某一传感器或坐标轴。

最新 `src/gnss_imu.py` 提供 X/Y/Z 陀螺仪分轴残差、分时段加速度残差，并把最终目标函数分解成**位置因子代价**和**IMU/先验/零偏因子合计代价**。根据用户本地 40 帧结果，使用位置因子的公式计算，约 `67.4208 / 67.6474 ≈ 99.67%` 的最终代价来自 OXTS 位置因子。这提示当前运动约束与位置测量之间的置信度或物理模型不匹配，不能直接断言 IMU 本身坏了。

下一步比较相同 20 帧长度的**起始窗口**与**后段窗口**，检查前期运动不一致是否造成后续积累；不更改噪声参数，也不新增临时调试文件。运行：

```bash
git pull origin main
conda activate chenjiahao
python -m unittest discover -s tests -v
python scripts/run_gnss_imu.py --sequence 0027 --start-frame 0 --frames 20 --report reports/gnss_imu_0027_first20.json
python scripts/run_gnss_imu.py --sequence 0027 --start-frame 20 --frames 20 --report reports/gnss_imu_0027_later20.json
python scripts/run_gnss_imu.py --sequence 0027 --start-frame 0 --frames 40 --report reports/gnss_imu_stage1b.json
```

每个窗口以该窗口第一个 OXTS 点为局部 ENU 原点、首帧姿态/速度为初值。窗口各自重设初值，因此**比较仅用于定位瞬态问题，不是公平的定位精度排行榜**。上述噪声参数均是工程值、未经真实传感器噪声标定；10 Hz OXTS ZOH 积分不代表高频原始 IMU 实验。下一步优先获取未同步的高频 OXTS/IMU 或独立来源 GNSS，建立真正可靠的状态估计评价协议。

## 科研约束

1. KITTI Raw `*_sync` OXTS 约10 Hz，**不是完整100 Hz原始 IMU**。目前 ZOH 预积分是临时工程近似，不得声称高频 IMU 精度。
2. OXTS 经纬度、速度、姿态均来自同一 OXTS 导航系统。首帧速度/姿态仅用于 GTSAM 初值，后续姿态/速度仅用于**离线诊断**；输入 OXTS 位置与同源参考轨迹的拟合不能当作独立 ATE。
3. ENU 为世界坐标、单位米；重力取 `(0,0,-9.81)`。机体坐标为前/左/上，姿态根据 KITTI OXTS 角度定义实现，需用运动学与标定证据继续核验。
4. `0027/0034/0042/0047` 已用于开发验证，不应自动视为从未使用的最终盲测数据；后续实验需登记训练/验证/测试划分及独立评价方案。
5. 修改源文件前检查调用方、测试与数据结构；修改后运行测试。保留版本控制，避免无用文件和重复代码。

参考：[KITTI Raw](https://www.cvlibs.net/datasets/kitti/raw_data.php) · [KITTI OXTS 字段说明](https://github.com/pratikac/kitti/blob/master/readme.raw.txt) · [GTSAM PreintegrationParams](https://borglab.github.io/gtsam/preintegrationparams/)。
