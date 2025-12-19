#!/bin/bash

# CR Camera Test Script
# 专门用于测试摄像头的脚本

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

red_print(){
    echo -e "\e[1;31m$1\e[0m"
}
green_print(){
    echo -e "\e[1;32m$1\e[0m"
}
yellow_print(){
    echo -e "\e[1;33m$1\e[0m"
}

usage() {
    echo "CR Camera Test Script"
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -h, --help              Show this help message"
    echo "  -a, --all               Test all cameras (3s each)"
    echo "  -l, --all-long          Test all cameras (10s each)"
    echo "  -s, --single PORT       Test single camera port (0-4)"
    echo "  -c, --continuous PORT   Test single camera continuously"
    echo "  -q, --quick             Quick verification without GStreamer"
    echo "  --no-init               Skip camera initialization"
    echo ""
    echo "Examples:"
    echo "  $0 -a                   # Quick test all cameras"
    echo "  $0 -l                   # Long test all cameras"
    echo "  $0 -s 0                 # Test camera 0 for 3 seconds"
    echo "  $0 -c 1                 # Test camera 1 continuously"
    echo "  $0 -q                   # Quick verification only"
}

# 默认参数
TEST_MODE=""
TARGET_PORT=""
SKIP_INIT=false
QUICK_MODE=false

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            usage
            exit 0
            ;;
        -a|--all)
            TEST_MODE="all"
            shift
            ;;
        -l|--all-long)
            TEST_MODE="all-long"
            shift
            ;;
        -s|--single)
            TEST_MODE="single"
            TARGET_PORT="$2"
            shift 2
            ;;
        -c|--continuous)
            TEST_MODE="continuous"
            TARGET_PORT="$2"
            shift 2
            ;;
        -q|--quick)
            QUICK_MODE=true
            shift
            ;;
        --no-init)
            SKIP_INIT=true
            shift
            ;;
        *)
            red_print "Error: Unknown option $1"
            usage
            exit 1
            ;;
    esac
done

test_single_camera_gstreamer() {
    local port=$1
    local duration=${2:-3}
    local continuous=${3:-false}
    
    export DISPLAY=:1
    
    if [ ! -c "/dev/video${port}" ]; then
        red_print "✗ Camera ${port}: Device not available"
        return 1
    fi
    
    green_print "Testing camera ${port} with GStreamer..."
    
    if [ "$continuous" = true ]; then
        green_print "Running continuous test for camera ${port}"
        green_print "Press Ctrl+C to stop the test"
        green_print "Command: gst-launch-1.0 v4l2src device=/dev/video${port} ! xvimagesink"
        gst-launch-1.0 v4l2src device=/dev/video${port} ! xvimagesink
    else
        green_print "Running ${duration}s test for camera ${port}"
        green_print "Command: gst-launch-1.0 v4l2src device=/dev/video${port} ! xvimagesink"
        timeout ${duration}s gst-launch-1.0 v4l2src device=/dev/video${port} ! xvimagesink 2>/dev/null
        
        if [ $? -eq 124 ]; then
            green_print "✓ Camera ${port}: Test completed successfully"
            return 0
        elif [ $? -eq 0 ]; then
            green_print "✓ Camera ${port}: Test successful"
            return 0
        else
            red_print "✗ Camera ${port}: Test failed"
            return 1
        fi
    fi
}

quick_verify_cameras() {
    green_print "========================================="
    green_print "Quick Camera Verification"
    green_print "========================================="
    
    local all_ok=true
    
    for port in 0 1 2 3 4; do
        if [ -c "/dev/video${port}" ]; then
            format_info=$(v4l2-ctl -d /dev/video${port} --get-fmt-video 2>/dev/null)
            if [ $? -eq 0 ]; then
                green_print "✓ Camera ${port}: Available and accessible"
            else
                red_print "✗ Camera ${port}: Not accessible"
                all_ok=false
            fi
        else
            red_print "✗ Camera ${port}: Device node not found"
            all_ok=false
        fi
    done
    
    if [ "$all_ok" = true ]; then
        green_print "All cameras are ready for testing!"
        return 0
    else
        red_print "Some cameras have issues"
        return 1
    fi
}

test_all_cameras() {
    local duration=${1:-3}
    
    green_print "========================================="
    green_print "Testing All Cameras (${duration}s each)"
    green_print "========================================="
    
    export DISPLAY=:1
    green_print "Set DISPLAY=:1 for remote display"
    
    local failed_cameras=()
    
    for port in 0 1 2 3 4; do
        yellow_print "Testing camera ${port}..."
        test_single_camera_gstreamer $port $duration false
        if [ $? -ne 0 ]; then
            failed_cameras+=($port)
        fi
        sleep 1
    done
    
    green_print "========================================="
    green_print "Test Summary:"
    if [ ${#failed_cameras[@]} -eq 0 ]; then
        green_print "✓ All cameras passed the test!"
    else
        red_print "✗ Failed cameras: ${failed_cameras[*]}"
        yellow_print "You may want to check hardware connections or drivers"
    fi
    green_print "========================================="
}

main() {
    green_print "========================================="
    green_print "CR Camera Test Tool"
    green_print "========================================="
    
    # 初始化摄像头（如果需要）
    if [ "$SKIP_INIT" = false ]; then
        green_print "Initializing cameras..."
        "$SCRIPT_DIR/init_cameras.sh" --no-gstreamer
        if [ $? -ne 0 ]; then
            red_print "Camera initialization failed!"
            exit 1
        fi
    fi
    
    # 快速验证模式
    if [ "$QUICK_MODE" = true ]; then
        quick_verify_cameras
        exit $?
    fi
    
    # 根据测试模式执行相应操作
    case "$TEST_MODE" in
        "all")
            test_all_cameras 3
            ;;
        "all-long")
            test_all_cameras 10
            ;;
        "single")
            if [[ -z "$TARGET_PORT" ]]; then
                red_print "Error: Port number required for single camera test"
                usage
                exit 1
            fi
            test_single_camera_gstreamer "$TARGET_PORT" 3 false
            ;;
        "continuous")
            if [[ -z "$TARGET_PORT" ]]; then
                red_print "Error: Port number required for continuous test"
                usage
                exit 1
            fi
            test_single_camera_gstreamer "$TARGET_PORT" 0 true
            ;;
        *)
            yellow_print "No test mode specified. Running quick verification..."
            quick_verify_cameras
            echo ""
            yellow_print "Use --help to see available test options"
            ;;
    esac
}

# 执行主函数
main "$@"
