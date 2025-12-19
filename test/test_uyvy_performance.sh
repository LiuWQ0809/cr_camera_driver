#!/bin/bash

# UYVY原始性能测试脚本
# 直接发布UYVY格式图像，不进行格式转换

echo "=== CR Camera UYVY Performance Test ==="
echo "测试配置: 直接发布UYVY格式，无格式转换"
echo "预期: 获得最高帧率基准性能"
echo ""

# 设置环境
source /opt/ros/humble/setup.bash
source /home/nvidia/wkp/cr_camera_driver/install/setup.bash

# 显示当前时间
echo "开始时间: $(date)"

# 启动节点
echo "启动CR Camera节点 (UYVY模式)..."
ros2 run cr_camera_driver cr_camera_node --ros-args \
  -p width:=1920 \
  -p height:=1536 \
  -p fps:=30.0 \
  -p enable_cameras:=[0,1,2,3,4] &

# 获取节点PID
NODE_PID=$!
echo "节点PID: $NODE_PID"

# 等待节点启动
sleep 3

echo ""
echo "=== 监控性能数据 (60秒) ==="
echo "监控topic帧率..."

# 在后台监控各个摄像头的帧率
{
  echo "$(date): 开始监控摄像头帧率"
  for i in {0..4}; do
    {
      case $i in
        0) topic="/cr/camera/rgb/front_right_full" ;;
        1) topic="/cr/camera/rgb/front_left_full" ;;
        2) topic="/cr/camera/rgb/left_full" ;;
        3) topic="/cr/camera/rgb/rear_full" ;;
        4) topic="/cr/camera/rgb/right_full" ;;
      esac
      
      echo "Camera $i ($topic):"
      timeout 60s ros2 topic hz $topic 2>/dev/null || echo "  No data from camera $i"
    } &
  done
  wait
} &

MONITOR_PID=$!

# 等待监控完成
echo "等待60秒性能数据收集..."
sleep 60

# 停止监控
kill $MONITOR_PID 2>/dev/null

echo ""
echo "=== 停止测试 ==="
# 停止节点
kill $NODE_PID 2>/dev/null
sleep 2

# 强制停止如果还在运行
if kill -0 $NODE_PID 2>/dev/null; then
    echo "强制停止节点..."
    kill -9 $NODE_PID 2>/dev/null
fi

echo "结束时间: $(date)"
echo ""
echo "=== 测试总结 ==="
echo "✓ 测试完成: UYVY原始性能基准"
echo "✓ 配置: 1920x1536, 无格式转换"
echo "✓ 数据: 查看上方帧率统计"
echo ""
echo "下一步: 启用VIC转换器对比性能差异"
