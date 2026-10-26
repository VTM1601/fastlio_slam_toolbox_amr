# fastlio_slam_toolbox_amr

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E)
![License](https://img.shields.io/badge/license-Apache--2.0-blue)

[English](README.md) | **Tiếng Việt**

Dựng map 2D và định vị cho AMR dùng **Livox MID-360**.
[FAST-LIO2](https://github.com/hku-mars/FAST_LIO) làm odometry, còn
[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) dựng map 2D và định vị trên map đó.
Không cần odometry bánh xe. Đã chạy thử trên AMR thật.

<p align="center">
  <img src="docs/media/mapping_rviz.gif" width="720" alt="Dựng map tầng văn phòng trên RViz">
</p>

<p align="center">
  <img src="docs/media/amr_base.jpg" height="320" alt="AMR chạy thử, MID-360 gắn phía trên">
  <img src="docs/media/amr_driving.gif" height="320" alt="AMR chạy trong lúc dựng map">
  <img src="docs/media/map.png" height="320" alt="Map occupancy grid lưu từ slam_toolbox">
  <br>
  <sub>Robot chạy thử · lúc dựng map · map đã lưu (5 cm mỗi ô)</sub>
</p>

## Cách hoạt động

1. **FAST-LIO2** publish odometry (`odom → base_link`) và point cloud đã bù chuyển động.
2. Point cloud được cắt trong dải cao 0.05 m đến 1.0 m so với `base_link` rồi ép thành
   scan 360°, `/livox/scan`.
3. **slam_toolbox** dựng map từ `/livox/scan`, publish `/map` và `map → odom`.
   nav2 dùng thẳng `/map` được.

## Yêu cầu

- Ubuntu 22.04 và ROS 2 Humble
- Livox MID-360
- `base_link` phải là gốc của URDF, vì FAST-LIO2 publish frame cha của nó

## Cài đặt

### 1. Công cụ

```bash
sudo apt install python3-vcstool python3-rosdep python3-colcon-common-extensions
sudo rosdep init   # chỉ chạy lần đầu
rosdep update
```

### 2. Livox-SDK2

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j"$(nproc)"
sudo make install
```

### 3. Workspace

Gồm package này, FAST-LIO2 và driver Livox:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/VTM1601/fastlio_slam_toolbox_amr.git
vcs import --recursive . < fastlio_slam_toolbox_amr/dependencies.repos

# livox_ros_driver2 có cả file ROS 1 và ROS 2: chọn ROS 2
cd livox_ros_driver2
cp package_ROS2.xml package.xml
cp -r launch_ROS2 launch
```

### 4. Build

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
source install/setup.bash
```

### 5. Mạng lidar

Đặt IP tĩnh cho cổng Ethernet của máy tính, cùng dải với lidar (ví dụ `192.168.1.50`).
Sau đó sửa IP máy tính (`host_net_info`) và IP lidar (`lidar_configs` → `ip`) trong
`src/livox_ros_driver2/config/MID360_config.json`. IP lidar là `192.168.1.1XX`, với `XX` là
hai số cuối của số serial.

## Sử dụng

**1. Chạy driver lidar** và bringup của robot (robot_state_publisher, động cơ):

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**2. Dựng map**

```bash
ros2 launch fastlio_slam_toolbox_amr mapping.launch.py rviz:=true \
  lidar_pose:="0 0 0.357 0 0 0"   # vị trí lidar trong base_link: x y z roll pitch yaw (m, rad)
```

Lúc khởi động, để robot đứng yên vài giây rồi mới chạy chậm.
Nên chạy lại qua những chỗ đã đi để slam_toolbox đóng vòng (loop closure).

**3. Lưu map** khi đang dựng map:

```bash
ros2 run fastlio_slam_toolbox_amr save_map ~/maps/office
# ghi office.pgm/.yaml (occupancy grid) và office.posegraph/.data (dùng để định vị)
```

**4. Định vị** trên map đã lưu:

```bash
ros2 launch fastlio_slam_toolbox_amr localization.launch.py \
  map_file:=~/maps/office rviz:=true
```

Mặc định robot được coi là xuất phát đúng chỗ bắt đầu dựng map. Nếu xuất phát ở chỗ khác,
bấm **2D Pose Estimate** trên RViz, hoặc truyền `start_pose:="x y yaw"`.

**5. Điều hướng** bằng nav2, **không** chạy `map_server` và AMCL vì slam_toolbox đã publish
`/map` và `map → odom`.

## Tham số chính

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `lidar_pose` | `0 0 0.357 -0.0048 0.0132 0` | Vị trí gắn lidar trong `base_link`: `x y z roll pitch yaw` |
| `min_height` / `max_height` | `0.05` / `1.0` | Dải chiều cao (m so với `base_link`) dùng để tạo scan 2D |
| `map_file` | (bắt buộc) | Map lưu bằng `save_map`, dùng khi định vị |
| `start_pose` | `0 0 0` | Vị trí ban đầu `x y yaw` trên map, dùng khi định vị |
| `rviz` | `false` | Mở RViz |

Xem tất cả tham số: `ros2 launch fastlio_slam_toolbox_amr mapping.launch.py --show-args`.
Tham số của FAST-LIO2 và slam_toolbox nằm trong [`config/`](config).

## Ghi công

Package này chỉ kết nối các công trình có sẵn:

- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO), bản ROS 2
  [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) (GPL-2.0)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) (LGPL-2.1)
- [pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan) (BSD)
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) (MIT)

Các package trên được cài riêng và giữ license của chúng. Repo này (launch file, config và
script) dùng license [Apache-2.0](LICENSE).
