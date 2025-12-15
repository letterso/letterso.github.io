---
layout:     post
title:       "rosbag"
subtitle:    ""
description: "ros1和ros2 bag操作和转换"
date:        2025-04-30
author:      "LETTER"
image:       ""
tags:
    - ros
categories:  ["ROS" ]
---

# ROS2 bag

`ros2 bag`与ros1的`rosbag`有少量区别

```bash
Commands:
  convert  Given an input bag, write out a new bag with different settings
  info     Print information about a bag to the screen
  list     Print information about available plugins to the screen
  play     Play back ROS data from a bag
  record   Record ROS data to a bag
  reindex  Reconstruct metadata file for a bag
```

- info

  ```bash
  positional arguments:
    bag_path              Bag to open
  
  options:
    -h, --help            show this help message and exit
    -s {my_test_plugin,mcap,sqlite3,my_read_only_test_plugin}, --storage {my_test_plugin,mcap,sqlite3,my_read_only_test_plugin}
                          Storage implementation of bag. By default attempts to detect automatically - use this argument to override.
  ```

- play

  - read-ahead-queue-size 默认1000，预加载入内存，用于平滑播放速度，若数据集包含大量图像/点云数据，可加大该值，优化播放效果，但不能过大否则会占用高内存
  - qos-profile-overrides-path 用于修改发布的qos

  ```bash
  usage: ros2 bag play [-h] [-s {sqlite3,my_test_plugin,mcap,my_read_only_test_plugin}] [--read-ahead-queue-size READ_AHEAD_QUEUE_SIZE] [-r RATE]
                       [--topics TOPICS [TOPICS ...]] [--qos-profile-overrides-path QOS_PROFILE_OVERRIDES_PATH] [-l] [--remap REMAP [REMAP ...]]
                       [--storage-config-file STORAGE_CONFIG_FILE] [--clock [CLOCK]] [-d DELAY] [--disable-keyboard-controls] [-p]
                       [--start-offset START_OFFSET] [--wait-for-all-acked TIMEOUT] [--disable-loan-message]
                       [--log-level {debug,info,warn,error,fatal}]
                       bag_path
  
  Play back ROS data from a bag
  
  positional arguments:
    bag_path              Bag to open
  
  options:
    -h, --help            show this help message and exit
    -s {sqlite3,my_test_plugin,mcap,my_read_only_test_plugin}, --storage {sqlite3,my_test_plugin,mcap,my_read_only_test_plugin}
                          Storage implementation of bag. By default attempts to detect automatically - use this argument to override.
    --read-ahead-queue-size READ_AHEAD_QUEUE_SIZE
                          size of message queue rosbag tries to hold in memory to help deterministic playback. Larger size will result in larger
                          memory needs but might prevent delay of message playback.
    -r RATE, --rate RATE  rate at which to play back messages. Valid range > 0.0.
    --topics TOPICS [TOPICS ...]
                          topics to replay, separated by space. If none specified, all topics will be replayed.
    --qos-profile-overrides-path QOS_PROFILE_OVERRIDES_PATH
                          Path to a yaml file defining overrides of the QoS profile for specific topics.
    -l, --loop            enables loop playback when playing a bagfile: it starts back at the beginning on reaching the end and plays
                          indefinitely.
    --remap REMAP [REMAP ...], -m REMAP [REMAP ...]
                          list of topics to be remapped: in the form "old_topic1:=new_topic1 old_topic2:=new_topic2 etc."
    --storage-config-file STORAGE_CONFIG_FILE
                          Path to a yaml file defining storage specific configurations. For the default storage plugin settings are specified
                          through syntax:read: pragmas: ["<setting_name>" = <setting_value>]Note that applicable settings are limited to read-only
                          for ros2 bag play.For a list of sqlite3 settings, refer to sqlite3 documentation
    --clock [CLOCK]       Publish to /clock at a specific frequency in Hz, to act as a ROS Time Source. Value must be positive. Defaults to not
                          publishing.
    -d DELAY, --delay DELAY
                          Sleep duration before play (each loop), in seconds. Negative durations invalid.
    --disable-keyboard-controls
                          disables keyboard controls for playback
    -p, --start-paused    Start the playback player in a paused state.
    --start-offset START_OFFSET
                          Start the playback player this many seconds into the bag file.
                          相当于rosbag的-s，用于指定起始播放偏移时间
    --wait-for-all-acked TIMEOUT
                          Wait until all published messages are acknowledged by all subscribers or until the timeout elapses in millisecond before
                          play is terminated. Especially for the case of sending message with big size in a short time. Negative timeout is
                          invalid. 0 means wait forever until all published messages are acknowledged by all subscribers. Note that this option is
                          valid only if the publisher's QOS profile is RELIABLE.
    --disable-loan-message
                          Disable to publish as loaned message. By default, if loaned message can be used, messages are published as loaned
                          message. It can help to reduce the number of data copies, so there is a greater benefit for sending big data.
    --log-level {debug,info,warn,error,fatal}
                          Logging level.
  ```

- qos设置

  ```yaml
  # reliability_override.yaml
  /livox/imu:
    reliability: best_effort
    depth: 5
    history: keep_last
  /livox/lidar:
    reliability: best_effort
    depth: 5
    history: keep_last
  /camera/color/image_raw:
    reliability: best_effort
    depth: 5
    history: keep_last
  ```

  覆盖默认QOS设置

  ```bash
  $ros2 bag play bag_file --qos-profile-overrides-path /home/giaiac/Datasets/qos_overrides.yaml
  ```

# ROS1 bag和ROS2 bag转换

- [rosbag2_bag_v2](https://github.com/ros2/rosbag2_bag_v2)：支持`dashing`版本，高版本不支持
- [rosbags](https://gitlab.com/ternaris/rosbags)：全版本支持，推荐

## rosbag2_bag_v2

`dashing`版本可以使用rosbag2_bag_v2可读取ros1 bag，更高版本不支持，同时对自定义消息支持不是太友好

```bash
ros2 bag play -s rosbag_v2 ro1.bag
```

## rosbags

高版本可使用rosbags转换，当前默认转换为sqlite3数据集，转换时可以一并转换自定义消息格式，但发布时需要加载定义了该消息的工作空间环境。

> 版本0.10.9已经支持转换为mcap格式，但通过rosbags转换的mcaps格式包存储空间并没有减少

命令行使用

```bash
usage: rosbags-convert [-h] --src [PATH ...] --dst DST
                       [--dst-storage {sqlite3,mcap}]
                       [--dst-version DST_VERSION]
                       [--compress {none,bz2,lz4,zstd}]
                       [--compress-mode {file,message,storage}]
                       [--src-typestore {empty,latest,ros1_noetic,ros2_dashing,ros2_eloquent,ros2_foxy,ros2_galactic,ros2_humble,ros2_iron,ros2_jazzy} | --src-typestore-ref SRC_TYPESTORE_REF  --dst-typestore {copy,empty,latest,ros1_noetic,ros2_dashing,ros2_eloquent,ros2_foxy,ros2_galactic,ros2_humble,ros2_iron,ros2_jazzy} | --dst-typestore-ref DST_TYPESTORE_REF]
                       [--exclude-topic [TOPIC ...]]
                       [--include-topic [TOPIC ...]]
                       [--exclude-msgtype [MSGTYPE ...]]
                       [--include-msgtype [MSGTYPE ...]]
```

例子

```bash
Convert bag from rosbag1 to rosbag2:
        rosbags-convert --src example.bag --dst ros2_bagdir

    Convert bag from rosbag1 to rosbag2, using per file compression for destination:
        rosbags-convert --src example.bag --dst ros2_bagdir --compress zstd

    Convert bag from rosbag1 to rosbag2, upgrade types to iron:
        rosbags-convert --src example.bag --dst ros2_bagdir --dst-typestore ros2_iron

    Convert bag from legacy rosbag2 (with humble types) to rosbag1:
        rosbags-convert --src ros2_bagdir --dst dst.bag --src_typestore ros2_humble

    Copy only image topics:
        rosbags-convert --src src.bag --dst dst.bag --include-topic sensor_msgs/msg/Image
```

rosbags支持下面定义的消息类型

- EMPTY : Only builtin messages (Duration, Time)
- LATEST : Alias for lastest ROS2 LTS
- ROS1_NOETIC
- ROS2_DASHING 
- ROS2_ELOQUENT 
- ROS2_FOXY 
- ROS2_GALACTIC 
- ROS2_HUMBLE 
- ROS2_IRON 
- ROS2_JAZZY

### 自定义消息

若需脱离ROS2环境实现自定义消息的读取和转换，需要增加消息类型到对应的`typestore`，下面将livox定义的自定义消息添加到`ROS2_HUMBLE`的`typestore`中

```python
from pathlib import Path
from rosbags.typesys import get_types_from_msg

LIVOX_POINT = """
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""

LIVOX_MSG = """
Header header
uint64 timebase
uint32 point_num
uint8  lidar_id
uint8[3]  rsvd
CustomPoint[] points
"""

# Plain dictionary to hold message definitions.
add_types = {}

# Add definitions from one msg file to the dict.
add_types.update(get_types_from_msg(LIVOX_POINT, 'livox_ros_driver/msg/CustomPoint'))
add_types.update(get_types_from_msg(LIVOX_MSG, 'livox_ros_driver/msg/CustomMsg'))

from rosbags.typesys import Stores, get_typestore

typestore = get_typestore(Stores.ROS1_NOETIC)
typestore.register(add_types)
```

命令行使用

```bash
$ rosbags-convert --src ros1_file.bag --dst <ros2_bag_folder> --src-typestore-ref mystore.typestore # 指定typestore,若需指定自定义typestore，需加入typestore所在路径到pythonpath，当前目录则使用export PYTHONPATH=.
```

### MCAP

参考[Conversion of ROS 1 Bags to MCAP Files (ROS 2 Default Bag Format)](https://www.phonethk.com/posts/conversion-of-ros1-bags-to-ros2/)

ROS2默认类型已修改为mcap，该类型可降低包大小一半以上，且通用性更强，可使用`ros2 bag convert`进行转换，但rqt_bag（humble）暂不支持mcap格式。

编写`convert.yaml`

```bash
output_bags:
  - uri: ros2_output
    storage_id: mcap
    all: true
```

转换

```bash
$ ros2 bag convert -i ros2_input.db3 -o convert.yaml
```

## 问题记录

通過rosbags转换后的bag存在接收频率异常，原因有可能为

- 接收的qos设置不正确
- ros1中包含嵌套定义的自定义消息类型

具体可参考

- [rostopic hz异常](https://blog.csdn.net/qq_39167050/article/details/143145263)
- [用了那么久ROS 2，你会对DDS调优吗？](https://fishros.org.cn/forum/topic/1456/用了那么久ros-2-你会对dds调优吗)