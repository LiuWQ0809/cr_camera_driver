#!/bin/bash

# V4L2摄像头帧率查询脚本
# 检查所有摄像头的帧率设置和能力

echo "=== V4L2摄像头帧率诊断 ==="
echo "时间: $(date)"
echo ""

# 检查可用的视频设备
echo "🔍 检查可用的视频设备:"
ls -la /dev/video* 2>/dev/null || echo "❌ 没有找到视频设备"
echo ""

# 对每个摄像头进行详细查询
for i in {0..4}; do
    device="/dev/video$i"
    
    if [ -e "$device" ]; then
        echo "📹 ========== Camera $i ($device) =========="
        
        # 检查设备信息
        echo "📋 设备信息:"
        v4l2-ctl -d $device --info 2>/dev/null || echo "❌ 无法获取设备信息"
        echo ""
        
        # 检查当前格式和帧率设置
        echo "⚙️  当前格式设置:"
        v4l2-ctl -d $device --get-fmt-video 2>/dev/null || echo "❌ 无法获取格式信息"
        echo ""
        
        # 检查当前帧率参数
        echo "🎬 当前帧率设置:"
        v4l2-ctl -d $device --get-parm 2>/dev/null || echo "❌ 无法获取帧率参数"
        echo ""
        
        # 列出支持的格式和分辨率
        echo "📐 支持的格式和分辨率:"
        v4l2-ctl -d $device --list-formats-ext 2>/dev/null || echo "❌ 无法列出支持的格式"
        echo ""
        
        # 检查支持的帧率
        echo "🎯 针对1920x1536 UYVY格式的帧率:"
        v4l2-ctl -d $device --list-framerates-ext 2>/dev/null | grep -A5 -B5 "1920x1536" || echo "❌ 没有找到1920x1536的帧率信息"
        echo ""
        
        # 测试设置30fps
        echo "🧪 测试设置30fps:"
        v4l2-ctl -d $device --set-parm=30 2>/dev/null && echo "✅ 设置成功" || echo "❌ 设置失败"
        
        # 验证设置后的帧率
        echo "✅ 设置后的帧率:"
        v4l2-ctl -d $device --get-parm 2>/dev/null || echo "❌ 无法验证"
        
        echo "================================================"
        echo ""
    else
        echo "⚠️  Camera $i: 设备 $device 不存在"
    fi
done

echo ""
echo "=== 摄像头驱动状态 ==="
echo "加载的摄像头相关模块:"
lsmod | grep -E "(gmsl|max92|sgx)" || echo "❌ 没有找到GMSL相关驱动模块"

echo ""
echo "=== 系统资源使用 ==="
echo "内存使用:"
free -h

echo ""
echo "CPU使用:"
top -bn1 | head -5

echo ""
echo "=== 诊断完成 ==="
echo "如果帧率不是30fps，可能的原因："
echo "1. 摄像头硬件限制"
echo "2. 驱动配置问题"
echo "3. 系统资源不足"
echo "4. 多摄像头带宽限制"
