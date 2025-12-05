---
layout:     post
title:       "开源SLAM方案评估"
subtitle:    ""
description: "视觉/激光/融合SLAM方案简述"
date:        2024-12-17
author:      "LETTER"
image:       ""
tags:
    - SLAM
categories:  ["SLAM" ]
---

# 工作室/团队

- [HKUST Aerial Robotics Group](https://github.com/HKUST-Aerial-Robotics)

- [Mechatronics and Robotic Systems (MaRS) Laboratory](https://github.com/hku-mars)

- [武汉大学多源智能导航实验室](https://github.com/i2Nav-WHU)

# 视觉SLAM

此处视觉SLAM为VIO/VO，一般只包含定位部分，不带有回环建图，除ORB_SLAM3完整集成回环和地图优化（和“建图”不一样），其他不带有回环功能，部分有其他算法实现回环再订阅里程计实现优化后位姿输出，实际不对VIO本身进行优化。

| 算法                                                         | 方案            | 传感器                         | 备注                                                         |
| ------------------------------------------------------------ | --------------- | ------------------------------ | ------------------------------------------------------------ |
| [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)         | orb+优化+回环   | 单目，双目，单目+IMU，双目+IMU | 鲁棒性好，集成多地图模块，精度高，代码重，可修改性一般，适用于大规模场景 |
| [vins-fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) | 光流+优化       | 双目，单目+IMU，双目+IMU       | 代码框架清晰，光流在面对高速场景下稳定性较特征法高，算力消耗和orb类似，作为纯VIO来说比ORB稳定 |
| [open-vins](https://github.com/rpng/open_vins)               | 光流/特征+msckf | 单目，双目，单目+IMU，双目+IMU | 轻量，持续更新集成新算法，算力消耗为vins的1/5不到，小场景下精度和vins类似甚至更高，大场景下稳定性没有vins好，适用于室内低算力平台 |
| [SVO2](https://github.com/uzh-rpg/rpg_svo_pro_open)          | 半直接法+优化   |    单目+IMU，双目+IMU    | 代码框架较为复杂不考虑                                     |
| [MAC-VO](https://github.com/MAC-VO/MAC-VO) | （光流+poseNN）+3d-3d BA | 双目 | 帧率低，运行量大，鲁棒性好可以在动态环境中运行 |

[Visual-Inertial SLAM Comparison](https://joshi-bharat.github.io/projects/visual_slam_comparison/)

Takeaways

- Inclusion of IMU data enhances the performance highly
  - IMU can propagate state for a few seconds
- Most algorithms fail at similar segments of the trajectory
  - Challenging scenarios
- Descriptor Matching Performs generally better than KLT Tracker
  - Illumination changes
- Direct Methods sometimes can not solve the optimization problem
  - Illumination changes, no photometric calibration
- Optimization based methods perform slightly better than filtering

# 激光SLAM

激光SLAM这几年相对更新不多，不同方案在标定良好的硬件平台上差异不大，部分corner case需要根据实际情况修改，不能依赖现有开源解决。

| 算法                                                         | 方案            | 传感器                         | 备注                                                         |
| ------------------------------------------------------------ | --------------- | ------------------------------ | ------------------------------------------------------------ |
| [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)         | 特征提取+图优化  | 多线激光+IMU | 算力消耗较大，包含回环检测 |
| [fast_lio2](https://github.com/hku-mars/FAST_LIO)    | 不提取特征+ikdtree+ieskf      | 多线激光+IMU      |  轻量，算力消耗低，面对运动剧烈场景稳定性好|
| [faster-lio](https://github.com/gaoxiang12/faster-lio)    | 不提取特征+iVox +ieskf      | 多线激光+IMU      |  工程及效率改进，狭窄场景下iVox更稳定，部分空旷大场景下iVox没有ikdtree稳定|
| [lightning-lm](https://github.com/gaoxiang12/lightning-lm) | 不提取特征+iVox +改进ieskf（包含aa） | 多线激光+IMU，odom，GPS | 基于faster-lio改进优化，包含了AA-FasterLIO，改进并简化了ieskf，增加了地图管理，回环和其他传感器的融合，不启用AA情况下（默认不启用），ieskf的速度已经较原版有较为明显提升，和faster-lio相比，完整实现了SLAM相关的功能，而不只是一个LIO前端 |
| [rko_lio](https://github.com/PRBonn/rko_lio) | 不提取特征+vdb+icp  | 多线激光+IMU      | 代码架构清晰简单，地图架构优秀，IMU作为去畸变和匹配先验使用，无法充分发挥IMU的作用，并未表现出比fast系列更优的效果。基于点云匹配的方案在运动较大场景下（如无人机），普遍没有以IMU为核心的滤波方案稳定。 |

# 融合SLAM

融合算法包含，视觉、激光、GPS和IMU融合，融合不一定会增加稳定性和精度，部分场景下，多传感器融合会导致效果下降，同时硬件上的同步和标定也会带来更多的问题。

| 算法                                                         | 方案            | 传感器                         | 备注                                                         |
| ------------------------------------------------------------ | --------------- | ------------------------------ | ------------------------------------------------------------ |
| [r3live](https://github.com/hku-mars/r3liveM)         |    |视觉+激光+IMU  | 计算资源占用高，实时性不佳，适合离线融合建图 |
| [FAST-LIVO](https://github.com/hku-mars/FAST-LIVO)    |     | 视觉+激光+IMU | 基于fast-lio的工作衍生，实时性较r3live好，不退化的场景下，并没有比fast-lio精度高，部分场景精度反而下降，只支持视觉-激光频率一致的数据|
| [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)    |     | 视觉+激光+IMU | 基于fast-livo迭代工作，主要改进点为使用voxel map统一视觉激光地图，效率和稳定性较fast-livo好（论文说的），实测确实会好点，但内存占用过高 |
| [GVINS](https://github.com/HKUST-Aerial-Robotics/GVINS)         |  光流+图优化 | 视觉+gps+IMU |基于vins-mono的工作衍生 |
| [IC-GVINS](https://github.com/i2Nav-WHU/IC-GVINS)               | 光流+图优化| 视觉+gps+IMU | 基于vins-mono，增加地球自转补偿的IMU预积分方法，基于IMU的特征特征预测及后端两步优化 |
| [LE-VINS](https://github.com/i2Nav-WHU/LE-VINS)               |  光流+图优化 | 视觉+激光+IMU | 基于IC-GVINS的工作衍生，使用激光雷达提供深度信息 |

# 基于网络

## 3R系列

| 算法                                                         | 方案            | 传感器                         | 备注                                                         |
| ------------------------------------------------------------ | --------------- | ------------------------------ | ------------------------------------------------------------ |
| [dust3r](https://github.com/naver/dust3r) |    |RGB only  | 开创性工作 |
| [mast3r](https://github.com/naver/mast3r) | |RGB only | dust3r改进 |
| [pow3r](https://github.com/naver/pow3r) | |RGB (K, depth, relative pose可选) | 提供更多的先验可选项，存在先验时，精度能有明显增加，作为插件满足稠密建图，稀疏补全，定位等下游任务 |

# 数据集

| 数据集名称                                                   | 机构               | 年份 |    平台    |         环境         | 激光 | 视觉 | IMU  | 真值 |
| :----------------------------------------------------------- | :----------------- | :--: | :--------: | :------------------: | :--: | :--: | :--: | :--: |
| [KITTI](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) | 卡尔斯鲁厄理工学院 | 2013 |    车载    |         城市         |  有  |  有  |  有  |  有  |
| [TUM](https://cvg.cit.tum.de/data/datasets)                  | 慕尼黑工业大学     | 2012 |    手持    | 室内/室外/挑战性场景 |  无  |  有  |  有  |  有  |
| [EuRoC MAV](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) | 苏黎世联邦理工     | 2016 |   无人机   |         室内         |  无  |  有  |  有  |  有  |
| M2DGR                                                        | 上海交通大学       | 2021 | 地面机器人 |      室内+室外       |  有  |  有  |  有  |  有  |
| TartanAir                                                    | 卡耐基梅隆大学     | 2020 | 仿真无人机 |         仿真         |  有  |  有  |  有  |  有  |
| NCLT                                                         | 密歇根大学         | 2016 | 地面机器人 |         城市         |  有  |  有  |  有  |  有  |
| Hilti SLAM                                                   | 牛津/UZH           | 2022 |    手持    |      室内+室外       |  有  |  有  |  有  |  有  |
| 4Seasons                                                     | 慕尼黑工业大学     | 2020 |    车载    |         室外         |  有  |  有  |  有  |  有  |
| VECtor                                                       | 上海科技大学       | 2022 |    手持    |      室内+室外       |  有  |  有  |  有  |  有  |
| UrbanLoco                                                    | 新加坡国立大学     | 2019 |    车载    |         城市         |  有  |  有  |  有  |  有  |
| ICL-NUIM                                                     | 帝国理工           | 2014 |    手持    |         室内         |  无  |  有  |  无  |  有  |
| ADVIO                                                        | Aalto大学          | 2018 |    手持    |         城市         |  无  |  有  |  有  |  有  |