#!/bin/bash

# CR Camera System Launch Script
# 启动摄像头系统和ROS2节点

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"

red_print(){
    echo -e "\e[1;31m$1\e[0m"
}
green_print(){
    echo -e "\e[1;32m$1\e[0m"
}

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  -h, --help          Show this help message"
    echo "  -c, --cameras LIST  Specify which cameras to enable (default: 0,1,2,3,4)"
    echo "  -f, --fps FPS       Set frame rate (default: 30)"
    echo "  -r, --resolution WxH Set resolution (default: 1920x1536)"
    echo "  --no-init           Skip camera initialization"
    echo "  --test-init         Test camera initialization with GStreamer"
    echo "  --test-all          Test all cameras before starting ROS2 node"
    echo "  --test-single PORT  Test single camera before starting ROS2 node"
    echo ""
    echo "Examples:"
    echo "  $0                           # Start all cameras with default settings"
    echo "  $0 -c 0,1,2                  # Start only cameras 0, 1, 2"
    echo "  $0 -f 15                     # Start with 15 FPS"
    echo "  $0 -r 1280x720               # Start with 720p resolution"
    echo "  $0 --no-init                 # Skip camera initialization"
    echo "  $0 --test-all                # Test all cameras with GStreamer before ROS2"
    echo "  $0 --test-single 0           # Test camera 0 before starting ROS2"
}

# 默认参数
CAMERAS="0,1,2,3,4"
FPS=30
WIDTH=1920
HEIGHT=1536
SKIP_INIT=false
TEST_INIT=false
TEST_ALL=false
TEST_SINGLE_PORT=""

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            usage
            exit 0
            ;;
        -c|--cameras)
            CAMERAS="$2"
            shift 2
            ;;
        -f|--fps)
            FPS="$2"
            shift 2
            ;;
        -r|--resolution)
            if [[ "$2" =~ ^([0-9]+)x([0-9]+)$ ]]; then
                WIDTH="${BASH_REMATCH[1]}"
                HEIGHT="${BASH_REMATCH[2]}"
            else
                red_print "Error: Invalid resolution format. Use WIDTHxHEIGHT (e.g., 1920x1080)"
                exit 1
            fi
            shift 2
            ;;
        --no-init)
            SKIP_INIT=true
            shift
            ;;
        --test-init)
            TEST_INIT=true
            shift
            ;;
        --test-all)
            TEST_ALL=true
            shift
            ;;
        --test-single)
            TEST_SINGLE_PORT="$2"
            shift 2
            ;;
        *)
            red_print "Error: Unknown option $1"
            usage
            exit 1
            ;;
    esac
done

# 转换摄像头列表为ROS参数格式
IFS=',' read -ra CAMERA_ARRAY <<< "$CAMERAS"
CAMERA_PARAMS="["
for i in "${!CAMERA_ARRAY[@]}"; do
    if [ $i -gt 0 ]; then
        CAMERA_PARAMS="${CAMERA_PARAMS},"
    fi
    CAMERA_PARAMS="${CAMERA_PARAMS}${CAMERA_ARRAY[i]}"
done
CAMERA_PARAMS="${CAMERA_PARAMS}]"

cleanup() {
    green_print "Shutting down camera system..."
    pkill -f cr_camera_node
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

main() {
    green_print "========================================="
    green_print "CR Camera System Launcher"
    green_print "========================================="
    green_print "Configuration:"
    green_print "  Cameras: $CAMERAS"
    green_print "  FPS: $FPS"
    green_print "  Resolution: ${WIDTH}x${HEIGHT}"
    green_print "  Skip init: $SKIP_INIT"
    green_print "  Test init: $TEST_INIT"
    green_print "  Test all: $TEST_ALL"
    if [[ -n "$TEST_SINGLE_PORT" ]]; then
        green_print "  Test single: $TEST_SINGLE_PORT"
    fi
    green_print "========================================="
    
    # 初始化摄像头（如果需要）
    if [ "$SKIP_INIT" = false ]; then
        green_print "Initializing cameras..."
        
        if [ "$TEST_INIT" = true ]; then
            # 使用测试模式初始化
            source "$SCRIPT_DIR/init_cameras.sh" --test-all
        elif [ "$TEST_ALL" = true ]; then
            # 测试所有摄像头
            source "$SCRIPT_DIR/init_cameras.sh" --test-all-long
        elif [[ -n "$TEST_SINGLE_PORT" ]]; then
            # 测试单个摄像头
            green_print "Testing single camera $TEST_SINGLE_PORT before initialization..."
            "$SCRIPT_DIR/init_cameras.sh" --test-single "$TEST_SINGLE_PORT"
            if [ $? -eq 0 ]; then
                green_print "Camera $TEST_SINGLE_PORT test successful, proceeding with normal initialization..."
                source "$SCRIPT_DIR/init_cameras.sh" --no-gstreamer
            else
                red_print "Camera $TEST_SINGLE_PORT test failed!"
                exit 1
            fi
        else
            # 正常初始化
            source "$SCRIPT_DIR/init_cameras.sh" --no-gstreamer
        fi
        
        if [ $? -ne 0 ]; then
            red_print "Camera initialization failed!"
            exit 1
        fi
        sleep 2
    else
        green_print "Skipping camera initialization"
    fi
    
    # 设置ROS2环境
    green_print "Setting up ROS2 environment..."
    source /opt/ros/humble/setup.bash
    
    # # 编译包（如果需要）
    # if [ ! -f "$PACKAGE_DIR/install/cr_camera_driver/lib/cr_camera_driver/cr_camera_node" ]; then
    #     green_print "Building cr_camera_driver package..."
    #     cd "$PACKAGE_DIR/.."
    #     colcon build --packages-select cr_camera_driver
    #     if [ $? -ne 0 ]; then
    #         red_print "Package build failed!"
    #         exit 1
    #     fi
    # fi
    
    # 设置包环境
    if [ -f "$PACKAGE_DIR/../install/setup.bash" ]; then
        source "$PACKAGE_DIR/../install/setup.bash"
    fi
    
    # 启动ROS2节点
    green_print "Starting CR camera node..."
    green_print "Publishing topics:"
    for cam in "${CAMERA_ARRAY[@]}"; do
        case $cam in
            0) green_print "  Camera $cam -> /cr/camera/rgb/front_right_full" ;;
            1) green_print "  Camera $cam -> /cr/camera/rgb/front_left_full" ;;
            2) green_print "  Camera $cam -> /cr/camera/rgb/left_full" ;;
            3) green_print "  Camera $cam -> /cr/camera/rgb/rear_full" ;;
            4) green_print "  Camera $cam -> /cr/camera/rgb/right_full" ;;
        esac
    done
    green_print "========================================="
    
    # 运行节点
    ros2 run cr_camera_driver cr_camera_node \
        --ros-args \
        -p width:=$WIDTH \
        -p height:=$HEIGHT \
        -p fps:=${FPS}.0 \
        -p enable_cameras:="$CAMERA_PARAMS"
}

# 执行主函数
main "$@"
