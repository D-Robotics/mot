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

## tros_mot_node

`tros_mot_node` 是一个独立的 ROS2 节点，订阅 `ai_msgs::msg::PerceptionTargets` 消息，通过 HobotMot 跟踪算法为目标分配和更新 track ID，并发布新的 `ai_msgs::msg::PerceptionTargets` 消息。该节点将 MOT 功能从检测节点中解耦，可以配合任意检测源使用。

### 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `mot_config_path` | string | `config/iou2_method_param.json` | MOT 配置文件路径（相对于包共享目录，或绝对路径） |
| `sub_topic` | string | `/hobot_dnn_detection` | 输入 PerceptionTargets 话题 |
| `pub_topic` | string | `/tros_mot_targets` | 输出 PerceptionTargets 话题 |
| `frame_width` | int | 960 | MOT 处理的图像宽度 |
| `frame_height` | int | 544 | MOT 处理的图像高度 |

### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `sub_topic` | `ai_msgs::msg::PerceptionTargets` | 包含待跟踪目标的检测结果 |

### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `pub_topic` | `ai_msgs::msg::PerceptionTargets` | 包含已分配 track ID 的跟踪结果 |

输出消息中包含：
- `targets` — 经过 MOT 跟踪并分配 `track_id` 的目标，保留原始的 attributes/points/captures
- `disappeared_targets` — 消失的目标（超过 `vanish_frame_count` 配置的帧数），包含 `track_id` 和 `type`

### 示例：跟踪人体检测框

以下示例展示如何使用 `tros_mot_node` 对 `mono2d_body_detection` 输出的人体检测框进行跟踪：

**步骤1：启动人体检测**（输出检测结果到 `/hobot_dnn_detection`）

```bash
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py
```

**步骤2：启动 tros_mot_node**（订阅检测结果，输出跟踪结果）

```bash
ros2 run hobot_mot tros_mot_node --ros-args \
  -p sub_topic:=/hobot_dnn_detection \
  -p pub_topic:=/tros_mot_targets \
  -p mot_config_path:=config/iou2_method_param.json \
  -p frame_width:=960 \
  -p frame_height:=544
```

**步骤3：查看跟踪结果**

```bash
# 查看已分配 ID 的跟踪目标
ros2 topic echo /tros_mot_targets

# 查看话题信息
ros2 topic info /tros_mot_targets
```

**输出消息示例：**

```
header:
  stamp: ...
  frame_id: "default"
targets:
  - type: "person"
    track_id: 1
    rois:
      - type: "body"
        rect: {x_offset: 100, y_offset: 50, width: 80, height: 200}
    attributes:
      - type: "height_cm"
        value: 170.0
  - type: "person"
    track_id: 2
    rois:
      - type: "body"
        rect: {x_offset: 400, y_offset: 60, width: 75, height: 190}
disappeared_targets:
  - type: "person"
    track_id: 0
```

每个目标的 `track_id` 由 MOT 跟踪器分配，同一人在不同帧间保持相同的 track_id。当被跟踪的人消失超过配置的 `vanish_frame_count` 帧数后，会出现在 `disappeared_targets` 中。

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

参数通过修改配置文件（`config/iou2_method_param.json` 或 `config/iou2_euclid_method_param.json`，按所选匹配模式）中对应 JSON 字段配置。创建 `HobotMot` 对象时传入该配置文件路径即可生效（参考前文"使用方法"）。

### 参数总览

| 字段 | 描述 | 范围 | 默认值 |
| :--- | :--- | :--- | :--- |
| `match_type` | 匹配模式 | IOU / Euclidean | IOU（iou2_method_param.json）/ Euclidean（iou2_euclid_method_param.json） |
| `tracker_type` | MOT 工作模式，目前仅支持 IOU based MOT | IOU_2.0 | IOU_2.0 |
| `use_kalman_filter` | 是否使用卡尔曼滤波器预测框，1 为使用 | 0/1 | 1 |
| `missing_time_thres` | 目标不可见帧数阈值，超过则置为 InVisible | >=0 | 2 |
| `vanish_frame_count` | 目标消失帧数阈值，超过则置为 Deleted | >=0 | 50 |
| `time_gap` | 帧间隔时间 | >=0 | 40 |
| `iou_thres` | iou 阈值，超过则进入匹配流程 | 0-1.0 | 0.2 |
| `euclidean_thres` | 欧式距离阈值，小于则进入匹配流程 | >=0 | 200 |
| `use_location_gain` | 是否计算目标检测框和其他检测框的最小距离，1 为使用 | 0/1 | 1 |
| `max_trajectory_number` | 状态机保存最大 track 数 | >=0 | 3 |
| `min_score` | 目标检测框得分阈值，低于此阈值的检测框不被用于跟踪 | 0-1.0 | 0.9 |
| `ignore_overlap_thres` | 筛选遮挡过大的 box 阈值，超过则不跟踪 | 0-1.0 | 0.9 |

### 参数详解

- **`match_type`**
  - 影响：决定前后帧检测框如何关联。IOU 按重叠度匹配，适合框位移小的场景；Euclidean 按中心点欧氏距离匹配，适合目标移动较快/框位移大的场景。选错会导致跟踪 ID 频繁切换或误匹配。
  - 配置方法：选择对应的配置文件（`iou2_method_param.json` 或 `iou2_euclid_method_param.json`），将 `match_type` 字段改为 `"IOU"` 或 `"Euclidean"`。

- **`tracker_type`**
  - 影响：当前仅 `IOU_2.0` 可用，无实际可调项。
  - 配置方法：修改配置文件中 `tracker_type` 字段（保持 `IOU_2.0`）。

- **`use_kalman_filter`**
  - 影响：开启后基于历史轨迹预测下一帧位置，提升遮挡/丢帧时的关联稳定性；关闭则只用当前检测框匹配，目标短暂遮挡时易丢 ID。
  - 配置方法：修改配置文件中 `use_kalman_filter` 字段为 `0` 或 `1`。

- **`missing_time_thres`**
  - 影响：值越大，目标短暂遮挡时容忍越久（保持 Visible 越久），不易提前判不可见；值越小则更敏感，遮挡即转 InVisible。
  - 配置方法：修改配置文件中 `missing_time_thres` 字段。

- **`vanish_frame_count`**
  - 影响：值越大，目标消失后保留 track 越久（便于重新出现时复用 ID）；值越小则更快释放资源、更早删除。
  - 配置方法：修改配置文件中 `vanish_frame_count` 字段。

- **`time_gap`**
  - 影响：影响卡尔曼预测的时间步长（单位与帧率相关）。配置与实际帧间隔不符会影响预测位置精度，导致匹配偏差。需与检测帧率匹配。
  - 配置方法：修改配置文件中 `time_gap` 字段，按实际检测帧间隔设置。

- **`iou_thres`**
  - 影响：IOU 模式下，前后帧框 IoU 超过该值才视为同一目标。值越小越宽松（更易匹配，可能误关联）；值越大越严格（可能漏关联导致 ID 切换）。
  - 配置方法：修改 `iou2_method_param.json` 中 `iou_thres` 字段。

- **`euclidean_thres`**
  - 影响：Euclidean 模式下，前后帧框中心距小于该值才视为同一目标。值越大越宽松（远距离框也匹配，易误关联）；值越小越严格（目标移动快时易漏关联）。单位与检测框坐标系相关。
  - 配置方法：修改 `iou2_euclid_method_param.json` 中 `euclidean_thres` 字段。

- **`use_location_gain`**
  - 影响：开启后引入位置增益辅助匹配，提升密集多目标场景的区分度；关闭则仅按主匹配准则。
  - 配置方法：修改配置文件中 `use_location_gain` 字段为 `0` 或 `1`。

- **`max_trajectory_number`**
  - 影响：限制同时维护的 track 数量上限。值过小会过早删除有效 track；值过大增加内存与计算开销。按场景目标数设置。
  - 配置方法：修改配置文件中 `max_trajectory_number` 字段。

- **`min_score`**
  - 影响：低置信度框被过滤（state_ 置 INVALID 并丢弃）。值越大跟踪质量越高但可能漏目标；值越小越易引入误检导致 ID 抖动。
  - 配置方法：修改配置文件中 `min_score` 字段。

- **`ignore_overlap_thres`**
  - 影响：与其他框重叠度超过该值的框被视为遮挡过大，不跟踪。值越大越宽容（保留更多被遮挡框）；值越小越严格（更易丢弃被遮挡目标）。
  - 配置方法：修改配置文件中 `ignore_overlap_thres` 字段。

## 注意事项

使用HobotMot时，需要保证输入给DoProcess接口的数据时序，推荐的做法是在检测package中将算法模型的输出结果进行排序后输出/使用。

为了保证MOT效果，如果输入的检测框置信度小于阈值（配置文件中的min_score配置项），此检测框将被过滤，输出的跟踪后的对应检测框的state_为INVALID，表示此检测框不可用，需要丢弃此检测结果。

用户可以根据实际使用场景调整配置文件中的检测框得分阈值min_score。
