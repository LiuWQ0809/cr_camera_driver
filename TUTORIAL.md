# CR Camera Driver Tutorial / 使用教程

This tutorial is for running the ROS2 multi-camera node on Jetson Orin (JetPack 6 + ROS2 Humble) under:

- Workspace root: `/home/nvidia/yanbo/`
- Package folder: `/home/nvidia/yanbo/cr_camera_driver/`

If your device uses a different folder layout, adjust paths accordingly.

---

## 0) What you get / 你会得到什么

**EN**
- A ROS2 node: `cr_camera_driver/cr_camera_node`
- Captures from 5 V4L2 devices (default `/dev/video0..4`)
- Optionally uses NVIDIA VPI(VIC+CUDA) to convert `YUYV/UYVY -> RGB/BGR`
- Publishes `sensor_msgs/msg/Image` to topics configured in `config/cameras.yaml`

**中文**
- 一个 ROS2 节点：`cr_camera_driver/cr_camera_node`
- 采集 5 路 V4L2 设备（默认 `/dev/video0..4`）
- 可选使用 NVIDIA VPI（VIC+CUDA）做 `YUYV/UYVY -> RGB/BGR` 硬件转换
- 按 `config/cameras.yaml` 配置发布 `sensor_msgs/msg/Image` 到对应 topic

---

## 1) Prerequisites / 前置条件

### 1.1 System check / 系统检查

```bash
cat /etc/nv_tegra_release
echo $ROS_DISTRO
ros2 --version
```

Expected / 期望：
- JetPack 6.x (L4T R36.x)
- ROS2 Humble

### 1.2 Tools / 工具

```bash
sudo apt update
sudo apt install -y v4l-utils gstreamer1.0-tools
```

---

## 2) Camera driver init (GMSL2) / GMSL2 驱动初始化

This repo assumes your kernel modules + driver package live in a folder like:

- `/home/nvidia/wkp/SG8A_ORIN_GMSL2-F_V2_AGX_Orin_YUV_JP6.2_L4TR36.4.3`
or
- `/home/nvidia/yanbo/SG8A_ORIN_GMSL2-F_V2_AGX_Orin_YUV_JP6.2_L4TR36.4.3`

It must contain a `ko/` subfolder with:
- `max9295.ko`, `max9296.ko`, `sgx-yuv-gmsl2.ko`

### 2.1 Run init script / 运行初始化脚本

**EN**
Run this once after boot or whenever cameras stop working.

**中文**
建议每次开机后、或相机异常时执行一次。

```bash
cd /home/nvidia/yanbo/cr_camera_driver

# If your driver folder is not in the default locations, set it explicitly:
# export GMSL2_DRIVER_DIR=/path/to/SG8A_ORIN_GMSL2-F_V2_AGX_Orin_YUV_JP6.2_L4TR36.4.3

./scripts/init_cameras.sh --no-gstreamer
```

If you see permission prompts, run with sudo (or ensure your user can sudo):

```bash
sudo -E ./scripts/init_cameras.sh --no-gstreamer
```

### 2.2 Validate devices / 验证设备节点

```bash
ls -la /dev/video*
for d in 0 1 2 3 4; do
  v4l2-ctl -d /dev/video$d --get-fmt-video || true
done
```

---

## 3) Build (colcon) / 编译

This package is intended to be built from the workspace root `/home/nvidia/yanbo/`.

```bash
cd /home/nvidia/yanbo
source /opt/ros/humble/setup.bash
colcon build --packages-select cr_camera_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 4) Run the node / 运行节点

### Option A (recommended): one-click script / 一键脚本（推荐）

```bash
cd /home/nvidia/yanbo/cr_camera_driver
./scripts/start_cameras.sh --high-performance
```

What it does / 脚本做什么：
- (optional) set MAXN clocks (`nvpmodel`, `jetson_clocks`)
- init GMSL2 modules (`init_cameras.sh --no-gstreamer`)
- `colcon build` from workspace root
- launch: `ros2 run cr_camera_driver cr_camera_node`

### Option B: manual / 手动方式

```bash
cd /home/nvidia/yanbo
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run cr_camera_driver cr_camera_node
```

### Option C: launch file / launch 启动

```bash
cd /home/nvidia/yanbo
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cr_camera_driver cr_cameras.launch.py
```

---

## 5) Verify ROS2 topics / 验证 ROS2 输出

### 5.1 List topics / 查看 topic 列表

```bash
ros2 topic list | grep ^/cr/camera/
```

### 5.2 Check rate / 检查帧率

```bash
ros2 topic hz /cr/camera/bgr/front_right_960_768
```

Notes / 说明：
- Cameras capture at ~30fps, but publish is controlled by `frame_skip_ratio`.
- The node waits for per-camera time calibration (~10s) before publishing. If you see “no messages”, wait a bit.

### 5.3 View images / 查看图像

```bash
# GUI
rqt_image_view

# Or built-in tools (if installed)
ros2 run image_tools showimage --ros-args -r image:=/cr/camera/bgr/front_right_960_768
```

---

## 6) Configuration (cameras.yaml) / 配置说明

Config file / 配置文件：
- Source: `/home/nvidia/yanbo/cr_camera_driver/config/cameras.yaml`
- Installed share path (after build): `$(ros2 pkg prefix cr_camera_driver)/share/cr_camera_driver/config/cameras.yaml`

Key fields / 关键字段：

### 6.1 Device mapping / 设备映射

Ensure your 5 cameras map to the right nodes. Default is:

```yaml
device: "/dev/video0"
...
device: "/dev/video4"
```

默认映射（与 `config/cameras.yaml` 保持一致）：
- `/dev/video0` → 前右
- `/dev/video1` → 前左
- `/dev/video2` → 左侧
- `/dev/video3` → 右侧
- `/dev/video4` → 后视

If your `/dev/video*` numbering differs, update each camera `device:` accordingly.

### 6.2 Pixel format (important!) / 像素格式（非常重要）

On Orin 130 we observed cameras output `YUYV`:

```bash
v4l2-ctl -d /dev/video0 --get-fmt-video
```

Set the global pixel format to match:

```yaml
global_settings:
  pixel_format: "YUYV"   # or "UYVY"
```

Wrong setting symptoms / 配置错误表现：
- green/purple images
- weird color shifts

### 6.3 Performance knobs / 性能参数

```yaml
global_settings:
  frame_skip_ratio: 3     # 1=30fps, 2=15fps, 3=10fps
  num_buffers: 2          # usually 2
  use_vic_converter: true # hardware conversion via VPI(VIC+CUDA)
```

---

## 7) Troubleshooting / 常见问题排查

### 7.1 “Green image” or wrong colors / 画面发绿/颜色不对

1) Check device pixel format:
```bash
v4l2-ctl -d /dev/video0 --get-fmt-video
```

2) Set YAML `global_settings.pixel_format` to `YUYV` or `UYVY` accordingly.

### 7.2 Missing cameras / 相机数量不对

```bash
ls -la /dev/video*
dmesg | tail -200
lsmod | grep -E "max9295|max9296|sgx"
```

Re-run init:
```bash
cd /home/nvidia/yanbo/cr_camera_driver
sudo -E ./scripts/init_cameras.sh --no-gstreamer
```

### 7.3 Node publishes nothing / 没有 topic 数据

- Wait ~10–15 seconds for time calibration.
- Confirm at least one topic is `enabled: true` in YAML.
- Confirm `use_vic_converter` can initialize (VPI installed); else set it to `false` temporarily.

### 7.4 `/dev/video*` permission denied

```bash
groups
sudo usermod -aG video $USER
newgrp video
```

---

## 8) Optional: systemd auto-init service / 可选：开机自启动初始化服务

**EN**
The repo includes `camera-init.service` and installer script. It writes the correct working directory + ExecStart for the current checkout path.

**中文**
项目提供开机自启动初始化服务安装脚本，会自动把 service 的路径写成当前项目路径。

```bash
cd /home/nvidia/yanbo/cr_camera_driver
sudo -E ./install_camera_service.sh

sudo systemctl status camera-init
sudo journalctl -u camera-init -n 200 --no-pager
```

---

## 9) Quick reference / 快速命令汇总

```bash
# init cameras
cd /home/nvidia/yanbo/cr_camera_driver
sudo -E ./scripts/init_cameras.sh --no-gstreamer

# build
cd /home/nvidia/yanbo
source /opt/ros/humble/setup.bash
colcon build --packages-select cr_camera_driver --cmake-args -DCMAKE_BUILD_TYPE=Release

# run
source install/setup.bash
ros2 run cr_camera_driver cr_camera_node

# check topics
ros2 topic list | grep ^/cr/camera/
```
