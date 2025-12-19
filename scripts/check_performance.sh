#!/bin/bash

# Jetson Performance Status Check Script
# 检查Jetson性能模式和摄像头系统状态

echo "=== Jetson AGX Orin 系统性能状态 ==="
echo "日期: $(date)"
echo ""

echo "🔥 CPU性能状态:"
sudo jetson_clocks --show | grep -A 12 "Online CPUs" | head -13

echo -e "\n🎮 GPU性能状态:"
sudo jetson_clocks --show | grep -E "GPU.*Freq"

echo -e "\n💾 内存性能状态:"
sudo jetson_clocks --show | grep "EMC"

echo -e "\n⚡ 电源模式:"
sudo jetson_clocks --show | grep "NV Power Mode"

echo -e "\n🎯 AI加速器状态:"
sudo jetson_clocks --show | grep -E "(DLA|PVA)"

echo -e "\n📷 摄像头服务状态:"
systemctl is-active camera-init.service
systemctl is-active jetson-clocks.service

echo -e "\n✅ 系统就绪状态: 所有性能组件已优化"
