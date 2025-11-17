# Welcome to HandPose-ROS
This is Handpose ROS Integration Project.  
This system reconstructs hand joint coordinate frames from 21 hand landmarks using google `mediapipe`.  
It includes both a standalone script version and a ROS 2 version.

## DEMO
<!-- https://github.com/user-attachments/assets/956f45fd-a752-4233-a894-6325916b71ae -->
https://github.com/user-attachments/assets/b2a561e7-b902-4bd1-a802-426902151187

## Summary
This package integrates **MediaPipe Hands** with **ROS 2 (Humble)**.  
It detects hand landmarks from a camera stream, scales them into canonical and world coordinates, builds per-joint coordinate frames, and publishes them into the ROS TF system for downstream robotics applications (e.g., teleoperation, grasp planning).

#### License
- Apache 2.0
---

## Detail Logic
### ⚠️ Note  
In this project, I didn't use depth camera.
<span style="background-color: #fff59d">**So Z value of wrist is always zero.**</span>  
So if you want to know Z value, make sure use depth module and take the wrist distance from camera. And add depth value to all joint's points

### 1. Landmark extraction
- MediaPipe Hands is used to get 21 hand landmarks per hand.
- Landmarks are published as normalized coordinates `(x, y, z)`.

#### 1-1. Canonical conversion
- Normalized coordinates are multiplied by image width/height to obtain **canonical pixel space coordinates**.

#### 1-2. World absolute scaling
- To approximate metric scale, the wrist–index MCP distance is assumed to be `0.08 m (80 mm)` (based on author’s hand).
- This scaling is applied uniformly, so that the hand size remains constant regardless of the screen position.
- The resulting transforms are suffixed with `world_abs`.

### 2. Coordinate frame generation (ROS TF)
- Each landmark is just a point, so local coordinate systems must be defined.

#### 2-1. Wrist frame
- Palm direction → **Y axis**  
- Middle finger MCP direction → **X axis**  
- Z axis is defined as the cross product, forming a right-handed system.

#### 2-2. Finger frames
- Each finger has joints: `MCP → PIP → DIP → TIP`.
- For each joint:
  - Project onto wrist XZ plane to determine **Y axis**.
  - Joint-to-joint vector defines **X axis**.
  - **Z axis** is set by cross product.

#### 2-3. Thumb special case
- For thumb MCP, an additional ~60° rotation about X axis is applied to better align with human thumb kinematics.

#### 2-4. TF broadcasting
- All transforms are published into ROS TF tree.

---

## System
#### OS
Ubuntu 22.04.6 LTS

#### Software
- [ROS2 humble](https://docs.ros.org/en/humble/index.html)
- [Hand landmarks detection](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker)

#### System Requirements
- Ubuntu 22.04 (ROS2 humble)
- Realsense camera or Leap Motion (TBD)

---

## Installation
#### virtual environment
[Using Python Packages with ROS 2](https://docs.ros.org/en/humble/How-To-Guides/Using-Python-Packages.html#installing-via-a-virtual-environment)
```bash
source {PATH_OF_YOUR_VIRTUAL_ENV}/bin/activate
```

#### python modules
```bash
  $ pip install -r requirements.txt
```

## Execute
#### Clone repositories
`HandPose` packages is required for run this project.
```bash
cd ~
git clone https://github.com/DaeyunJang/Mediapipe-Hand-ROS2.git
```

#### Build and source
```bash
  # activate python environment
  source {PATH_OF_YOUR_VIRTUAL_ENV}/bin/activate

  # build dependent package
  cd ~/Mediapipe-Hand-ROS2
  colcon build
  ./install/setup.bash
```

#### Launch
```bash
  ros2 launch handpose_ros handpose_launch.py
```

# ROS configurations

## Interfaces
`handpose_interfaces/Hands`
| Field            | Type                  | Description             |
| ---------------- | --------------------- | ----------------------- |
| hands            | array\<HandLandmarks> | Multiple detected hands |

`handpose_interfaces/HandLandmarks`
| Field            | Type     | Description                 |
| ---------------- | -------- | --------------------------- |
| id               | int      | Hand index                  |
| label            | string   | `left` / `right`            |
| score            | float    | Detection confidence        |
| handed\_index    | int      | Mediapipe internal index    |
| width            | int      | Input image width           |
| height           | int      | Input image height          |
| landmarks\_norm  | float\[] | Normalized landmarks        |
| landmarks\_canon | float\[] | Canonical (pixel) landmarks |
| landmarks\_world | float\[] | World (metric) landmarks    |


## Topic
| Topic                                    | Msg type                    | Description                                          |
| ---------------------------------------- | --------------------------- | ---------------------------------------------------- |
| `/hands/detections`                      | `handpose_interfaces/Hands` | Detected hands (landmarks + metadata)                |
| `/hands/points`                          | `sensor_msgs/PointCloud2`   | All hand landmarks as PointCloud                     |
| `/hands/points/hand_left`                | `sensor_msgs/PointCloud2`   | Left hand landmarks                                  |
| `/hands/points/hand_right`               | `sensor_msgs/PointCloud2`   | Right hand landmarks                                 |
| `/mp_overlay_image`                      | `sensor_msgs/Image`         | Debug overlay image                                  |
| `/tf`                                    | `tf2_msgs/TFMessage`        | TF transforms of wrist & joints                      |
| `hand_{label}_{finger}_{joint}_{suffix}` | `tf2_msgs/TFMessage`        | Per-joint TFs (e.g. `hand_left_index_mcp_world_abs`) |


## Parameters
check `handpose_ros/config/params.yaml`

`mediapipe_hands_node`
| Parameter                  | Type   | Default value                    | Description                    |
| -------------------------- | ------ | -------------------------------- | ------------------------------ |
| `image_topic`              | string | `/camera/camera/color/image_raw` | Input RGB image topic          |
| `max_num_hands`            | int    | 2                                | Max hands to detect            |
| `min_detection_confidence` | float  | 0.95                             | Detection confidence threshold |
| `min_tracking_confidence`  | float  | 0.95                             | Tracking confidence threshold  |
| `draw`                     | bool   | False                            | Overlay debug image            |
| `flip_image`               | bool   | True                             | Flip input image horizontally  |
| `use_pointcloud`           | bool   | True                             | Publish PointCloud2            |

`handpose_tf_broadcaster`
| Parameter                                  | Type   | Default value                              | Description                    |
| ------------------------------------------ | ------ | ------------------------------------------ | ------------------------------ |
| `hands_topic`                              | string | `hands/detections`                         | Input landmark topic           |
| `use_depth`                                | bool   | False                                      | Use depth info                 |
| `depth_topic`                              | string | `/camera/aligned_depth_to_color/image_raw` | Depth image topic              |
| `camera_info_topic`                        | string | `/camera/camera/color/camera_info`         | Camera info topic              |
| `camera_frame`                             | string | `camera_color_optical_frame`               | TF frame name                  |
| `tf.norm.enable`                           | bool   | False                                      | Enable normalized TF           |
| `tf.canonical.enable`                      | bool   | False                                      | Enable canonical TF            |
| `tf.canonical_norm.enable`                 | bool   | True                                       | Enable canonical normalized TF |
| `tf.canonical_norm.scale`                  | float  | `1/1280`                                   | Scale factor                   |
| `tf.world_absolute_scale.enable`           | bool   | True                                       | Enable world absolute scaling  |
| `tf.world_absolute_scale.target_length`    | float  | 0.06                                       | Wrist–MCP reference length (m) |
| `tf.world_absolute_scale.finger_name`      | string | `index`                                    | Reference finger               |
| `tf.world_absolute_scale.joint_name`       | string | `mcp`                                      | Reference joint                |
| `tf.world_absolute_scale.eps`              | float  | 1e-6                                       | Numerical stability            |
| `tf.world_absolute_scale.EMA_smooth_alpha` | float  | 0.3                                        | Exponential smoothing alpha    |
| `tf.world_absolute_scale.suffix`           | string | `world_abs`                                | TF suffix                      |
| `tf.world_absolute_scale.max_scale_step`   | float  | 0.0                                        | Per-frame clamp                |

