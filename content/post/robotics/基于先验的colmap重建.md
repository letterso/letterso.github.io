---
layout:     post
title:       "基于先验的colmap重建"
subtitle:    ""
description: "结合SLAM构建的关键帧和位姿信息使用colmap重建"
date:        2025-11-14
author:      "LETTER"
image:       ""
publishDate: 2025-11-14
tags:
    - SLAM
    - 三维重建
categories:  ["robotics" ]
---

## 安装

- linux
	
	```bash
	sudo apt install colmap
	```
	
- window
	
	可以直接下载编译好的包https://github.com/colmap/colmap/releases。

## 文件格式

> 输出默认为bin，可以使用export model as txt输出txt格式结果

```bash
+── images # 原始图像
│   +── image1.jpg
│   +── image2.jpg
│   +── ...
+── sparse # 稀疏重建结果
│   +── cameras.txt/bin		# 相机内参
│   +── images.txt/bin		# 图像姿态Pose以及关键点Keypoints
│   +── points3D.txt/bin	# 3D点信息
│   +── project 			# 项目文件
+── stereo # 稠密重建结果
+── database # 用于存储所有提取到的信息
```

- cameras.txt格式
  ```bash
  # Camera list with one line of data per camera:
  #   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]
  # Number of cameras: 3
  1 SIMPLE_PINHOLE 3072 2304 2559.81 1536 1152
  2 PINHOLE 3072 2304 2560.56 2560.56 1536 1152
  3 SIMPLE_RADIAL 3072 2304 2559.69 1536 1152 -0.0218531
  ```
- images.txt格式
  ```bash
  # Image list with two lines of data per image:
  #   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
  #   POINTS2D[] as (X, Y, POINT3D_ID)
  # Number of images: 2, mean observations per image: 2
  1 0.851773 0.0165051 0.503764 -0.142941 -0.737434 1.02973 3.74354 1 P1180141.JPG
  2362.39 248.498 58396 1784.7 268.254 59027 1784.7 268.254 -1
  2 0.851773 0.0165051 0.503764 -0.142941 -0.737434 1.02973 3.74354 1 P1180142.JPG
  1190.83 663.957 23056 1258.77 640.354 59070
  ```
- points3D.txt格式
  ```bash
  # 3D point list with one line of data per point:
  #   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)
  # Number of points: 3, mean track length: 3.3334
  63390 1.67241 0.292931 0.609726 115 121 122 1.33927 16 6542 15 7345 6 6714 14 7227
  63376 2.01848 0.108877 -0.0260841 102 209 250 1.73449 16 6519 15 7322 14 7212 8 3991
  63371 1.71102 0.28566 0.53475 245 251 249 0.612829 118 4140 117 4473
  ```

## 基于先验重建

绕过COLMAP从零开始的全自动SfM（Structure from Motion）流程，而是直接为其提供一个包含初始位姿和内参的“项目骨架”，然后利用COLMAP强大的特征匹配和光束法平差（Bundle Adjustment）功能来优化这些初始值和重建三维点云。

### 核心流程概述

1. **准备数据**：将图像、相机内参、相机外参（位姿）转换为COLMAP兼容的文本格式。
2. **创建项目结构**：手动创建一个COLMAP项目文件夹，并放入转换后的`cameras.txt`, `images.txt`和空的`points3D.txt`。
3. **特征提取与匹配**：运行COLMAP的特征提取和匹配命令，为后续步骤生成特征点对应关系。
4. **点云三角化**：使用`point_triangulator`命令，根据你提供的固定位姿和特征匹配，生成初始的三维稀疏点云。
5. **光束法平差（Bundle Adjustment）**：使用`bundle_adjuster`命令，以三角化后的点云和位姿为初始值，进行联合优化，得到更精确的位姿和三维点。

### 创建项目目录和数据文件

```bash
my_project/
├── images/             # 存放所有图像文件 (e.g., 0001.png, 0002.png, ...)
└── sparse/
    └── 0/              # 手动创建初始模型
        ├── cameras.txt
        ├── images.txt
        └── points3D.txt
```

- `cameras.txt` - 相机内参文件

-  `images.txt` - 图像位姿文件
   
   POINTS2D这行必须为空
   
   ```txt
   # Image list with two lines of data per image:
   #   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
   #   POINTS2D[]
   # Number of images: 2, mean observations per image: 0
   1 1.0 0.0 0.0 0.0 0.0 0.0 0.0 1 0001.png
   
   2 0.9659 0.0 0.2588 0.0 -0.1 0.0 -0.5 1 0002.png
   ```
   
- `points3D.txt` - 三维点文件
   
   只需创建一个空文件即可

### COLMAP处理

1. 创建数据库并提取特征
   ```bash
   # 创建一个数据库文件
   colmap database_creator --database_path my_project/database.db
   
   # 提取特征
   colmap feature_extractor \
       --database_path my_project/database.db \
       --image_path my_project/images \
       --ImageReader.single_camera 1 \
       --ImageReader.camera_model PINHOLE \
       --ImageReader.camera_params "525.0,525.0,319.5,239.5"
   ```
   `--ImageReader.single_camera 1`: 如果所有图像共享相同的内参，这个选项可以简化流程。
   `--ImageReader.camera_model` 和 `--ImageReader.camera_params`: 确保这里提供的内参与你写入`cameras.txt`的完全一致。
2. 特征匹配
   ```bash
   colmap exhaustive_matcher \
       --database_path my_project/database.db
   ```
   `exhaustive_matcher`会对所有图像对进行匹配，如果图像序列性很强，也可以使用`sequential_matcher`来提高效率。
3. 点云三角化
   > triangulated文件夹需提前手动创建

   这一步是核心，colmap会读取你提供的位姿（从`images.txt`导入到数据库中）和特征匹配结果，来生成三维点云，但**不会改变**你提供的位姿。
   ```bash
   colmap point_triangulator \
       --database_path my_project/database.db \
       --image_path my_project/images \
       --input_path my_project/sparse/0 \
       --output_path my_project/sparse/triangulated
   ```
   `--input_path`: 指向我们手动创建的包含`cameras.txt`和`images.txt`的文件夹。
   `--output_path`: 指定一个新的输出文件夹，COLMAP会在这里生成包含三角化后三维点的新`cameras.txt`, `images.txt`, `points3D.txt`。
4. 光束法平差（Bundle Adjustment）
   > optimized文件夹需提前手动创建
   
   同时微调相机位姿和三维点的位置，以最小化重投影误差。
   ```bash
   colmap bundle_adjuster \
       --input_path my_project/sparse/triangulated \
       --output_path my_project/sparse/optimized
   ```
   可调整参数
   ```bash
   --BundleAdjustment.refine_focal_length arg (=1)		# 若相机提前标定，固定焦距设置为0
   --BundleAdjustment.refine_principal_point arg (=0)
   --BundleAdjustment.refine_extra_params arg (=1) 	# 若不需要优化相机位姿，则固定为0
   ```

## 参考
[colmap](https://github.com/colmap/colmap)

https://zhuanlan.zhihu.com/p/594202644