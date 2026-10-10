# Chenjiahao — Robust Multi-Sensor Vehicle State Estimation

从零重建车载多传感器融合状态估计研究工程。目标是可信、可复现的实验结论；Mamba 和动态协方差暂不作为已证实创新。

## 研究原则

1. 先核验数据与评价协议，再实现固定噪声因子图；不从深度学习开始。
2. 原始输入、模拟退化输入、参考轨迹分别保存；严禁使用参考轨迹参与在线状态估计。
3. 区分 KITTI Raw `*_sync` 中按约 10 Hz 同步的 OXTS 数据与完整原始高频 IMU 记录；不得声称同步 OXTS 具有原始 100 Hz 数据的预积分精度。
4. 论文中的 OXTS 轨迹若与 GNSS 测量同源，必须标注其为 OXTS reference；基于它的合成 GNSS 退化实验不等同于独立真实定位验证。
5. 所有方法共享相同输入、初值、坐标系、数据划分及评价指标。开发/验证/测试序列预先登记，不能在最终测试集上反复调参。
6. 每一阶段必须有可执行脚本、自动测试、实验配置、日志和可复现的结果。

## Stage 0：本地数据审计

本仓库**不存放** KITTI 大型原始数据。先检查本地 `2011_10_03_drive_0027_sync` 目录。

```bash
git clone https://github.com/15603816097/chenjiahao.git
cd chenjiahao
python3 scripts/audit_kitti.py --sequence /absolute/path/to/2011_10_03_drive_0027_sync --report reports/0027_audit.json
python3 -m unittest discover -s tests -v
```

期望该序列包含 `oxts/data/*.txt`、`velodyne_points/data/*.bin`、`image_02/data/*.png`、`image_03/data/*.png`，各子目录同级有 `timestamps.txt`；日期父目录包含三个标定文件。脚本报告真实帧数（不硬编码 4544）、时间单调性/间隔、帧 ID 对齐、文件样本有效性及标定文件存在性。

## 实施路线（尚未承诺创新结果）

- Stage 0：数据审计、时钟/标定核验、坐标系说明、数据划分协议。
- Stage 1：独立 GNSS 轨迹/IMU 检查、GTSAM 的正确运动状态与 IMU 预积分、GPS/IMU 固定协方差基线。
- Stage 2：独立 LiDAR 里程计与立体视觉里程计，再做四传感器固定因子图。
- Stage 3：预注册退化场景；对比固定噪声、鲁棒核与非学习自适应方法。
- Stage 4：如实验显示有必要，再训练 GRU/Mamba 可靠性模型、预测不确定性与安全的噪声映射。
- Stage 5：独立序列泛化、强基线、消融、统计检验、论文撰写。

## 现在的验收关口

Stage 0 通过：检查脚本在本地真实序列上退出码为 0，所有必需流存在、帧 ID 一致、时间递增、标定齐全；同时记录 OXTS 与输入 GNSS 的来源关系。

数据来源与格式参考：
- KITTI Raw: https://www.cvlibs.net/datasets/kitti/raw_data.php
- KITTI Odometry: https://www.cvlibs.net/datasets/kitti/eval_odometry.php

**注意：**旧项目 DFG 保留用于历史回溯，不直接复制旧算法与实验结果作为新论文成果。

## Stage 0B：时序对齐诊断（在 Stage 0 通过之后执行）

```bash
git pull
python3 -m unittest discover -s tests -v
python3 scripts/analyze_timing.py --root dataset/kitti/2011_10_03 --report reports/timing_analysis.json
```

该脚本输出每种传感器相对 OXTS 的时间偏差（signed = sensor - OXTS）的中位数、95 分位绝对偏差、最大绝对偏差、时间间隔及疑似大间隔。超过 50 ms 标记 `REVIEW`，这只是诊断阈值，不等同于数据损坏。

**验收原则：** 未确认相机曝光、LiDAR 采集与 OXTS 测量的时标关系前，不采用简单的按帧号等时测量假设，不自动平移/插值。后续明确关联策略，并通过不同策略的敏感性实验检查结果。

**后续数据层工作：** 对 OXTS 30 字段、经纬度合法范围、姿态及速度单位、四序列轨迹、标定矩阵进行物理一致性检查；随后设计 Stage 1 GNSS/IMU 固定协方差基线。

## Stage 0C：OXTS 数据与标定的物理一致性检查

已完成 Stage 0A/B 后执行：

```bash
git pull origin main
python3 -m unittest discover -s tests -v
python3 scripts/audit_oxts.py --root dataset/kitti/2011_10_03 --report reports/oxts_audit.json
```

输出四个 KITTI sync 序列的 OXTS 30 字段解析、位置范围、车辆速度/加速度/角速度统计、导航状态/卫星数分布、OXTS 位置变化与同源 OXTS 速度的一致性检查，以及 IMU↔LiDAR、LiDAR↔Camera 旋转矩阵和双目投影参数的校验。

审计会拒绝损坏的 30 字段测量、NaN/Inf、非法经纬度、缺失/非法标定。物理异常阈值仅产生 `REVIEW`，需人工解释；不可把任意速度阈值当成数据集官方验证结论。

**关键研究约束：** `lat/lon` 与 `vn/ve` 均来自同一 OXTS 定位导航系统；相互比较只是自洽性检查，不是独立真值验证。固定因子图阶段必须显式区分输入测量和用于评价的 OXTS 参考，并标注来源相关性。

**后续：** 看 Stage 0C 的四序列统计决定是否修正传感器单位、坐标系和测量时间关联；之后再进入 Stage 1 GNSS/IMU 基线（此时才需要安装 GTSAM）。

OXTS 字段与同步说明参见 [KITTI Raw 格式说明](https://github.com/yanii/kitti-pcl/blob/master/KITTI_README.TXT)、[KITTI 官方 Raw Data 页面](https://www.cvlibs.net/datasets/kitti/raw_data.php)。
