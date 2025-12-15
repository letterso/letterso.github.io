---
layout:     post
title:       "ros2点云格式转换"
subtitle:    ""
description: "ROS2 pcl::PCLPointCloud和sensor_msgs"
date:        2025-04-23
author:      "LETTER"
image:       ""
tags:
    - ros
categories:  ["ROS" ]
---


ROS2环境下推荐使用[pcl_conversions](https://github.com/ros-perception/perception_pcl/tree/ros2/pcl_conversions)实现pcl点云和sensor_msgs的转换。

- sensor_msgs::msg::PCLPointCloud2-> pcl::PCLPointCloud2

```cpp
template<typename T> void pcl::fromROSMsg(const sensor_msgs::msg::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud);
template<typename T> void pcl::moveFromROSMsg(const sensor_msgs::msg::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud); // 移动语义
```

- pcl::PCLPointCloud2->sensor_msgs::msg::PCLPointCloud2

```cpp
template<typename T> void toROSMsg(const pcl::PointCloud<T> &pcl_cloud, sensor_msgs::msg::PointCloud2 &cloud);
```

- pcl::PCLPointCloud2->sensor_msgs::msg::Image

```cpp
template<typename T> void toROSMsg (const pcl::PointCloud<T> &pcl_cloud, sensor_msgs::msg::Image& msg);
inline void moveToROSMsg(sensor_msgs::msg::PointCloud2 &cloud, sensor_msgs::msg::Image &image); // 移动语义
```

