#!/bin/bash

# 快速测试脚本 - 验证系统是否正常工作

red_print(){
    echo -e "\e[1;31m$1\e[0m"
}
green_print(){
    echo -e "\e[1;32m$1\e[0m"
}

green_print "========================================="
green_print "CR Camera System Quick Test"
green_print "========================================="

# Determine workspace install path (assumes repo lives one level under the colcon workspace root)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
WORKSPACE_DIR="$(dirname "$PROJECT_DIR")"

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

# 在后台启动摄像头节点
green_print "Starting camera node in background..."
ros2 run cr_camera_driver cr_camera_node &
NODE_PID=$!

# 等待节点启动
sleep 5

# 检查topics
green_print "Checking published topics..."
ros2 topic list | grep "/cr/camera"

# 检查topic数据
green_print "Checking topic data (5 seconds)..."
timeout 5 ros2 topic hz /cr/camera/rgb/front_right_full

# 清理
green_print "Cleaning up..."
kill $NODE_PID 2>/dev/null

green_print "========================================="
green_print "Quick test completed!"
green_print "If you saw topics and frame rates, the system is working."
green_print "Use './scripts/launch_cameras.sh' to start the full system."
green_print "========================================="
