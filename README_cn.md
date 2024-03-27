[English](./README.md) | 简体中文

# 功能介绍

hobot_mot package实现多目标跟踪（MOT）功能，用于检测框的跟踪、ID分配。

MOT采用基于IOU的跟踪算法，根据位置信息对目标进行跟踪。

算法主要分3个步骤：

1) prediction 利用卡尔曼滤波器预测轨迹在当前帧的位置；

2) match 利用预测位置和观测位置得到匹配结果；

3) update 根据匹配结果对轨迹集合进行更新

# 编译

## 依赖库

- hobot：ai_middleware_v1.0.0
- hobotlog：1.0.4
- jsoncpp：1.8.4
- iou_based_mot
- ipc_tracking
- feat_based_mot

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

 支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### Ubuntu板端编译

1. 编译环境确认 
   - 板端已安装X3 Ubuntu系统。
   - 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
   - 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`
2. 编译

编译命令：`colcon build --packages-select hobot_mot`

### Docker交叉编译

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

```
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

colcon build --packages-select hobot_mot \
   --merge-install \
   --cmake-force-configure \
   --cmake-args \
   --no-warn-unused-cli \
   -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
```

## 注意事项

# 使用介绍

## package说明 

hobot_mot package被编译成库，用户在算法检测package中直接使用hobot_mot package的接口实现MOT功能。

MOT功能所需要的配置在配置文件中指定，包含4类配置：

iou2_method_param.json：基于IOU简单匹配的目标跟踪策略，用于人体、人头、人脸检测框的MOT配置。

iou2_euclid_method_param.json：基于IOU欧式距离的目标跟踪策略，用于人手检测框的MOT配置。

iou_method_param.json：基于IOU匈牙利匹配的目标跟踪策略，**暂不支持**。

reid_method_param.json：基于人体特征的目标跟踪策略，**暂不支持**。

**hobot_mot使用方法**

1. 根据需要跟踪的检测框类型，选择使用的配置文件。
2. 使用配置文件创建HobotMot对象。
3. 使用创建的HobotMot对象的DoProcess接口处理每一帧的检测框结果、时间戳和检测框对应图片分辨率等数据，输出跟踪后的检测框（分配ID）和消失的ID列表。

## 参数

|         字段          | 描述                                               | 范围           |                            默认值                            |
| :-------------------: | -------------------------------------------------- | -------------- | :----------------------------------------------------------: |
|      match_type       | 匹配模式                                           | IOU和Euclidean | iou2_method_param.json配置文件中为IOU，iou2_euclid_method_param.json配置文件中为Euclidean |
|     tracker_type      | MOT工作模式，目前仅支持IOU based MOT               | IOU_2.0        |                           IOU_2.0                            |
|   use_kalman_filter   | 是否使用卡尔曼滤波器预测框，1为使用                | 0/1            |                              1                               |
|  missing_time_thres   | 目标不可见帧数阈值，超过则置为InVisible            | >=0            |                              2                               |
|  vanish_frame_count   | 目标消失帧数阈值，超过则置为Deleted                | >=0            |                              50                              |
|       time_gap        | 帧间隔时间                                         | >=0            |                              40                              |
|       iou_thres       | iou阈值，超过则进入匹配流程                        | 0-1.0          |                             0.2                              |
|    euclidean_thres    | 欧式距离阈值，小于则进入匹配流程                   | >=0            |                             200                              |
|   use_location_gain   | 是否计算目标检测框和其他检测框的最小距离，1为使用  | 0/1            |                              1                               |
| max_trajectory_number | 状态机保存最大track数                              | >=0            |                              3                               |
|       min_score       | 目标检测框得分阈值，低于此阈值的检测框不被用于跟踪 | 0-1.0          |                             0.9                              |
| ignore_overlap_thres  | 筛选遮挡过大的box阈值,超过则不跟踪                 | 0-1.0          |                             0.9                              |

## 注意事项

使用HobotMot时，需要保证输入给DoProcess接口的数据时序，推荐的做法是在检测package中将算法模型的输出结果进行排序后输出/使用。

为了保证MOT效果，如果输入的检测框置信度小于阈值（配置文件中的min_score配置项），此检测框将被过滤，输出的跟踪后的对应检测框的state_为INVALID，表示此检测框不可用，需要丢弃此检测结果。

用户可以根据实际使用场景调整配置文件中的检测框得分阈值min_score。
