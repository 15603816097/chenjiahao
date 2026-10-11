# Chenjiahao · 车载多传感器融合状态估计

基于 KITTI Raw 开展可复现的 GNSS/IMU/LiDAR/Camera 状态估计研究。**先保证数据、坐标系与测量模型正确，再开展鲁棒因子图及 Mamba 动态可靠性研究。**

## 当前状态（2026-10-11）

| 阶段 | 状态 | 验证结果 |
|---|---|---|
| Stage 0A 数据完整性 | 已通过 | 4 段 KITTI Raw 同步数据结构与标定文件检查通过 |
| Stage 0B 时间分析 | 初步完成 | OXTS ~10 Hz；0027 前几帧与相机/LiDAR 相差 >50 ms，需保留真实时间戳 |
| Stage 0C OXTS/标定审计 | 已通过初步检查 | 4 序列 OXTS 30 字段解析正常，RGB 双目基线 0.532332 m |
| Stage 1A WGS84 → ENU | 已通过 | 本地18个测试通过；4段轨迹和 OXTS 速度自洽性检查通过 |
| **Stage 1B 固定噪声 GTSAM** | **数值试运行成功，物理一致性待查** | 本地23个单测通过；0–19帧输入位置拟合2.0269m，20–39帧0.0923m，连续0–39帧6.3716m；物理一致性待确认 |
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

**2026-10-11 已完成等长20帧分段实验（所有窗口单独以首帧 OXTS 初始化）：**

| 窗口 | OXTS位置拟合RMSE | IMU vs OXTS dv/dt RMSE | gyro vs OXTS dR/dt RMSE |
|---|---:|---:|---:|
| 0–19帧 | 2.0269 m | 1.8673 m/s² | 0.1479 rad/s |
| 20–39帧 | 0.0923 m | 0.2975 m/s² | 0.0308 rad/s |
| 0–39帧 | 6.3716 m | 1.3208 m/s² | 0.1055 rad/s |

**诊断结论：** 开头约2秒的数据与当前机体/姿态/稀疏IMU模型的相容性明显较差；后20帧独立初始化后可很好拟合输入OXTS位置，不能直接归因于IMU硬件启动故障。跨窗口连续优化可能受早期约束不一致和过高的IMU相对权重影响；这是待验证假设。不同窗口重新初始化且时长不等，0.0923m仅是输入测量拟合，不是独立定位精度。

下一步只运行**同样40帧长度、不同起始位置**的对照（不改源码、噪声或优化方法）：

```bash
conda activate chenjiahao
cd ~/chenjiahao
python scripts/run_gnss_imu.py --sequence 0027 --start-frame 20 --frames 40 --iterations 30 --report reports/gnss_imu_0027_20_59.json
```

将该结果与已经执行的0–39帧40帧窗口比较，核查是否仅为开头暂态导致的模型失配。实验若仍异常，应优先转向原始高频IMU和合理的协方差标定，而不是反复修改噪声强行贴合OXTS轨迹。完整KITTI unsynced/extract包非常大，采购/下载前需要先确认是否可仅取得OXTS资料。

## 科研约束

1. KITTI Raw `*_sync` OXTS 约10 Hz，**不是完整100 Hz原始 IMU**。目前 ZOH 预积分是临时工程近似，不得声称高频 IMU 精度。
2. OXTS 经纬度、速度、姿态均来自同一 OXTS 导航系统。首帧速度/姿态仅用于 GTSAM 初值，后续姿态/速度仅用于**离线诊断**；输入 OXTS 位置与同源参考轨迹的拟合不能当作独立 ATE。
3. ENU 为世界坐标、单位米；重力取 `(0,0,-9.81)`。机体坐标为前/左/上，姿态根据 KITTI OXTS 角度定义实现，需用运动学与标定证据继续核验。
4. `0027/0034/0042/0047` 已用于开发验证，不应自动视为从未使用的最终盲测数据；后续实验需登记训练/验证/测试划分及独立评价方案。
5. 修改源文件前检查调用方、测试与数据结构；修改后运行测试。保留版本控制，避免无用文件和重复代码。

参考：[KITTI Raw](https://www.cvlibs.net/datasets/kitti/raw_data.php) · [KITTI OXTS 字段说明](https://github.com/pratikac/kitti/blob/master/readme.raw.txt) · [GTSAM PreintegrationParams](https://borglab.github.io/gtsam/preintegrationparams/)。
