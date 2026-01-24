#!/bin/bash
<<COMMENT
#
# 多摄像头自动显示脚本
# 自动点亮并显示全部5路SG3S-ISX031C-GMSL2摄像头
# 2025-08-26
#
COMMENT

clear

red_print(){
        echo -e "\e[1;31m$1\e[0m"
}
green_print(){
        echo -e "\e[1;32m$1\e[0m"
}

red_print "Multi-Camera Display Script for AGX Orin & Jetson_Linux_R36.4.3"
green_print "Automatically configuring 5 cameras with SG3S-ISX031C-GMSL2 (GMSL2/3G)"

# 进入ko目录加载驱动
cd $PWD/ko

# 加载MAX9295串行器驱动
if [ "`sudo lsmod | grep max9295`" == "" ];then
	green_print "Loading max9295.ko..."
	sudo insmod max9295.ko
fi	

# 加载MAX9296解串器驱动
if [ "`sudo lsmod | grep max9296`" == "" ];then
	green_print "Loading max9296.ko..."
	sudo insmod max9296.ko
fi

# 加载GMSL2主驱动，全部配置为3G模式
if [ "`sudo lsmod | grep sgx_yuv_gmsl2`" == "" ];then
	green_print "Loading sgx-yuv-gmsl2.ko with 3G mode for all cameras..."
	sudo insmod sgx-yuv-gmsl2.ko enable_3G=1,1,1,1
else
	green_print "sgx-yuv-gmsl2 already loaded"
fi

# 设置显示环境（远程终端使用）
export DISPLAY=:1

# 摄像头类型：3 = SG3S-ISX031C-GMSL2
yuv_cam_type=3

green_print "Configuring and displaying all 5 cameras..."

# 依次配置并启动每路摄像头显示
for port in 0 1 2 3 4; do
    green_print "========== Configuring Camera Port ${port} =========="
    
    # 配置摄像头参数（对应类型3的配置）
    green_print "Setting camera parameters for /dev/video${port}..."
    v4l2-ctl -d /dev/video${port} -c sensor_mode=1,trig_pin=0xffff0007
    
    if [ $? -eq 0 ]; then
        green_print "✓ Camera ${port} configured successfully"
        
        # 启动显示窗口（后台运行）
        green_print "Starting display window for camera ${port}..."
        gst-launch-1.0 v4l2src device=/dev/video${port} ! \
            video/x-raw,format=UYVY,width=1920,height=1536 ! \
            fpsdisplaysink video-sink=xvimagesink sync=false &
        
        # 记录进程ID
        echo $! >> /tmp/camera_pids.txt
        
        green_print "✓ Camera ${port} display started (PID: $!)"
    else
        red_print "✗ Failed to configure camera ${port}"
    fi
    
    # 延迟以避免资源冲突
    sleep 2
done

green_print "=========================================="
green_print "All 5 cameras started!"
green_print "Press Ctrl+C to stop all camera displays"
green_print "Or run: pkill gst-launch-1.0"
green_print "=========================================="

# 等待用户中断
trap 'echo "Stopping all cameras..."; pkill gst-launch-1.0; rm -f /tmp/camera_pids.txt; exit' INT

# 保持脚本运行
wait
