#!/bin/bash

# 摄像头帧率监控脚本

red_print(){
    echo -e "\e[1;31m$1\e[0m"
}
green_print(){
    echo -e "\e[1;32m$1\e[0m"
}

echo "=========================================="
echo "         Camera Frame Rate Monitor        "
echo "=========================================="

while true; do
    clear
    echo "Camera Frame Rate Status - $(date)"
    echo "=========================================="
    
    for port in 0 1 2 3 4; do
        echo "Camera ${port} (/dev/video${port}):"
        
        # 检查设备格式和帧率
        fmt_info=$(v4l2-ctl -d /dev/video${port} --get-fmt-video 2>/dev/null)
        if [ $? -eq 0 ]; then
            echo "  Format: $(echo "$fmt_info" | grep "Pixel Format")"
            echo "  Resolution: $(echo "$fmt_info" | grep "Width/Height")"
            
            # 检查帧率控制
            frame_rate=$(v4l2-ctl -d /dev/video${port} -C frame_rate 2>/dev/null | grep frame_rate)
            if [ ! -z "$frame_rate" ]; then
                fps_value=$(echo $frame_rate | awk '{print $2/1000000}')
                green_print "  ✓ Frame Rate: ${fps_value} fps"
            else
                echo "  Frame Rate: Not available"
            fi
        else
            red_print "  ✗ Device not accessible"
        fi
        echo ""
    done
    
    echo "Press Ctrl+C to exit"
    sleep 3
done
