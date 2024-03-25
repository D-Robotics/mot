English| [简体中文](./README_cn.md)

# Function Introduction

The hobot_mot package implements Multiple Object Tracking (MOT) function for detecting box tracking and ID assignment.

MOT uses an IOU-based tracking algorithm to track objects based on their spatial information.

The algorithm mainly consists of 3 steps:

1) Prediction: Utilize Kalman filter to predict the position of the trajectory in the current frame;

2) Match: Obtain matching results based on the predicted positions and observed positions;

3) Update: Update the trajectory set according to the matching results.

# Compilation

## Dependencies

- hobot: ai_middleware_v1.0.0
- hobotlog: 1.0.4
- jsoncpp: 1.8.4
- iou_based_mot
- ipc_tracking
- feat_based_mot

## Development Environment

- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0 / Linaro GCC 9.3.0

## Compilation

Support compilation on X3 Ubuntu system and cross-compilation using Docker on PC.

### Compilation on Ubuntu based platform

1. Confirm the compilation environment
   - The X3 Ubuntu system is installed on the board.
   - The current compilation terminal has set the TogetherROS environment variable: `source PATH/setup.bash`. Here, PATH is the installation path of TogetherROS.
   - ROS2 compilation tool colcon is installed, installation command: `pip install -U colcon-common-extensions`

2. Compilation

Compilation command: `colcon build --packages-select hobot_mot`

### Docker Cross-Compilation

1. Confirm the compilation environment
   - Compile in Docker and TogetherROS is already installed in Docker. For instructions on Docker installation, cross-compilation, TogetherROS compilation, and deployment, please refer to the README.md in the robot development platform robot_dev_config repo.

4. Compilation

   - Compilation command:

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

## Notes

# User Guide

## Package Description

The hobot_mot package is compiled into a library, and users can directly use the interfaces of hobot_mot package in the algorithm detection package to implement the MOT (Multiple Object Tracking) function.

The configurations required for MOT are specified in the configuration files, including four types of configurations:

iou2_method_param.json: Target tracking strategy based on simple matching using IOU, used for MOT configuration in human body, head, and face detection boxes.

iou2_euclid_method_param.json: Target tracking strategy based on IOU Euclidean distance, used for MOT configuration in hand detection boxes.

iou_method_param.json: Target tracking strategy based on IOU Hungarian matching, **currently not supported**.

reid_method_param.json: Target tracking strategy based on human feature, **currently not supported**.

**Usage of hobot_mot**

1. Select the configuration file based on the type of detection boxes to be tracked.
2. Create a HobotMot object using the configuration file.
3. Use the DoProcess interface of the created HobotMot object to process the detection box results, timestamps, image resolutions corresponding to detection boxes, etc., for each frame, and output the tracked detection boxes (assigned IDs) and the list of disappeared IDs.

## Parameters

|         Field          | Description                                     | Range          |                         Default                          |
| :-------------------:  | -----------------------------------------------  | -------------- | :------------------------------------------------------: |
|      match_type        | Matching mode                                   | IOU and Euclidean | IOU in iou2_method_param.json, Euclidean in iou2_euclid_method_param.json |
|     tracker_type       | MOT working mode, currently only supports IOU based MOT | IOU 2.0   |                        IOU 2.0                          |
|   use_kalman_filter    | Whether to use Kalman filter to predict boxes, 1 for yes | 0/1            |                           1                              |
|  missing_time_thres   | Threshold of consecutive frames where the target is invisible, beyond which it is set as 'InVisible'            | >=0            |                              2                               |
|  vanish_frame_count   | Threshold of consecutive frames where the target disappears, beyond which it is set as 'Deleted'                | >=0            |                              50                              |
|       time_gap        | Time interval between frames                                                              | >=0            |                              40                              |
|       iou_thres       | Threshold of IoU, above which it enters the matching process                                | 0-1.0          |                             0.2                              |
|    euclidean_thres    | Threshold of Euclidean distance, below which it enters the matching process                  | >=0            |                             200                              |
|   use_location_gain   | Whether to calculate the minimum distance between the target detection box and other detection boxes, 1 for using  | 0/1            |                              1                               |
| max_trajectory_number | Maximum number of tracks saved in the state machine                                         | >=0            |                              3                               |
|       min_score       | Threshold of the score of target detection box, detection boxes with scores lower than this threshold are not used for tracking | 0-1.0          |                             0.9                              |
| ignore_overlap_thres  | Threshold for filtering boxes with excessive overlap, beyond which they are not tracked          | 0-1.0          |                             0.9                              |

## Notes

When using HobotMot, it is necessary to ensure the temporal order of the data input to the DoProcess interface. It is recommended to sort the output of the algorithm model in the detection package before outputting/using it.

To ensure the effectiveness of MOT, if the confidence score of an input detection box is lower than the threshold (min_score in the configuration file), this detection box will be filtered out. The state of the corresponding tracked detection box will be set to INVALID, indicating that this detection box is not usable and should be discarded.

Users can adjust the detection box score threshold min_score in the configuration file based on actual usage scenarios.
