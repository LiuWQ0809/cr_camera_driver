#!/bin/bash

# CR Camera Driver 一键启动脚本
# 包含相机初始化和ROS2节点启动，无需GStreamer图像显示

set -e

# 脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ROS 日志级别（默认仅显示错误）
ROS_LOG_LEVEL="ERROR"

# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# 显示帮助信息
show_help() {
    cat << EOF
CR Camera Driver 一键启动脚本

用法: $0 [选项]

选项:
    -h, --help              显示此帮助信息
    --no-init              跳过相机初始化（如果之前已初始化）
    --high-performance     启用高性能模式（推荐）
    --skip-build           跳过编译步骤
    --config-only          仅运行配置，不启动相机节点
    --verbose              显示详细日志
    --ros-log-level LEVEL  设置ROS节点日志级别: DEBUG|INFO|WARN|ERROR|FATAL (默认: ERROR)

示例:
    $0                     完整启动（推荐）
    $0 --no-init           跳过初始化直接启动
    $0 --config-only       仅配置系统不启动节点
    $0 --high-performance  启用最高性能模式

EOF
}

# 默认参数
SKIP_INIT=false
HIGH_PERFORMANCE=false
SKIP_BUILD=false
CONFIG_ONLY=false
VERBOSE=false

to_upper() {
    echo "$1" | tr '[:lower:]' '[:upper:]'
}

to_lower() {
    echo "$1" | tr '[:upper:]' '[:lower:]'
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        --no-init)
            SKIP_INIT=true
            shift
            ;;
        --high-performance)
            HIGH_PERFORMANCE=true
            shift
            ;;
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        --config-only)
            CONFIG_ONLY=true
            shift
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        --ros-log-level)
            if [ -z "$2" ]; then
                log_error "--ros-log-level 需要一个参数"
                exit 1
            fi
            ROS_LOG_LEVEL="$(to_upper "$2")"
            case "$ROS_LOG_LEVEL" in
                DEBUG|INFO|WARN|ERROR|FATAL)
                    ;;
                *)
                    log_error "无效的ROS日志级别: $2"
                    exit 1
                    ;;
            esac
            shift 2
            ;;
        *)
            log_error "未知参数: $1"
            show_help
            exit 1
            ;;
    esac
done

ROS_LOG_LEVEL_LOWER="$(to_lower "$ROS_LOG_LEVEL")"

# 设置高性能模式
setup_high_performance() {
    if [ "$HIGH_PERFORMANCE" = true ]; then
        log_step "设置高性能模式..."
        
        # 设置MAXN性能模式
        log_info "设置NV Power Mode为MAXN..."
        sudo nvpmodel -m 0
        
        # 锁定CPU/GPU频率到最高
        log_info "锁定CPU/GPU频率..."
        sudo jetson_clocks
        
        # 设置风扇为最高转速
        log_info "设置风扇为最高转速..."
        sudo jetson_clocks --fan 255
        
        log_info "高性能模式设置完成"
        
        if [ "$VERBOSE" = true ]; then
            log_info "当前频率设置："
            sudo jetson_clocks --show | head -15
        fi
    fi
}

build_driver() {
    if [ "$SKIP_BUILD" = true ]; then
        log_info "跳过编译步骤（使用了 --skip-build）"
        return 0
    fi

    local builder="$SCRIPT_DIR/build_driver.sh"
    if [ ! -x "$builder" ]; then
        log_error "编译脚本未找到或不可执行，请检查: $builder"
        exit 1
    fi

    log_step "编译CR Camera Driver..."
    local builder_args=(--workspace-root "$PROJECT_DIR")
    if [ "$VERBOSE" = true ]; then
        builder_args+=(--verbose)
    fi

    "$builder" "${builder_args[@]}"
}

# 验证相机设备
verify_cameras() {
    log_step "验证相机设备..."
    
    local cameras_found=0
    for i in {0..4}; do
        if [ -c "/dev/video$i" ]; then
            cameras_found=$((cameras_found + 1))
            if [ "$VERBOSE" = true ]; then
                log_info "找到相机设备: /dev/video$i"
            fi
        fi
    done
    
    log_info "检测到 $cameras_found 个相机设备"
    
    if [ $cameras_found -eq 0 ]; then
        log_error "未检测到任何相机设备，请检查硬件连接和驱动"
        exit 1
    fi
}

init_cameras() {
    if [ "$SKIP_INIT" = true ]; then
        log_info "跳过相机初始化（使用了 --no-init）"
        return 0
    fi

    local init_script="$SCRIPT_DIR/init_cameras.sh"
    if [ ! -f "$init_script" ]; then
        log_error "未找到初始化脚本: $init_script"
        exit 1
    fi

    log_step "初始化相机..."
    bash "$init_script"
}

# 启动相机节点
start_camera_node() {
    if [ "$CONFIG_ONLY" = false ]; then
        log_step "启动CR Camera Driver节点..."
        
        cd "$PROJECT_DIR"
        source /opt/ros/humble/setup.bash
        source install/setup.bash

        log_info "========================================"
        log_info "CR Camera Driver 已启动"
        log_info "按 Ctrl+C 停止"
        log_info "========================================"

        local ros_cmd=(ros2 run cr_camera_driver cr_camera_node)
        if [ -n "$ROS_LOG_LEVEL_LOWER" ]; then
            ros_cmd+=(--ros-args --log-level "$ROS_LOG_LEVEL_LOWER")
        fi

        "${ros_cmd[@]}"
    else
        log_info "配置完成，节点未启动（使用了 --config-only）"
        log_info "要启动节点，请运行："
        log_info "  cd $PROJECT_DIR/.."
        log_info "  source /opt/ros/humble/setup.bash"
        log_info "  source install/setup.bash"
        log_info "  ros2 run cr_camera_driver cr_camera_node --ros-args --log-level $ROS_LOG_LEVEL_LOWER"
    fi
}

# 清理函数
cleanup() {
    log_info ""
    log_info "正在停止相机节点..."
    # 节点会自动清理，这里只需要显示消息
}

# 设置信号处理
trap cleanup EXIT INT TERM

# 主函数
main() {
    log_info "========================================"
    log_info "CR Camera Driver 一键启动脚本"
    log_info "========================================"
    
    setup_high_performance
    # build_driver
    init_cameras
    verify_cameras
    start_camera_node
}

# 运行主函数
main "$@"
