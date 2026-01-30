#!/bin/bash

# CR Camera Service Initialization Script
# 用于系统服务的摄像头初始化脚本
# 只包含核心初始化功能：检查驱动、加载驱动、配置摄像头

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 相对于脚本目录的驱动路径
GMSL2_DRIVER_DIR="$SCRIPT_DIR/../mod/SG8A_ORIN_GMSL2-F_V2_AGX_Orin_YUV_JP6.2_L4TR36.4.3"

# 日志函数
log_info() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') [INFO] $1"
    logger -t camera_init "[INFO] $1"
}

log_warn() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') [WARN] $1"
    logger -t camera_init "[WARN] $1"
}

log_error() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') [ERROR] $1"
    logger -t camera_init "[ERROR] $1"
}

# 检查摄像头驱动目录
check_camera_driver() {
    log_info "检查GMSL2驱动目录..."
    
    if [ ! -d "$GMSL2_DRIVER_DIR" ]; then
        log_error "GMSL2驱动目录不存在: $GMSL2_DRIVER_DIR"
        return 1
    fi
    
    if [ ! -d "$GMSL2_DRIVER_DIR/ko" ]; then
        log_error "驱动文件目录不存在: $GMSL2_DRIVER_DIR/ko"
        return 1
    fi
    
    # 检查关键驱动文件是否存在
    local required_modules=("max9295.ko" "max9296.ko" "sgx-yuv-gmsl2.ko")
    for module in "${required_modules[@]}"; do
        if [ ! -f "$GMSL2_DRIVER_DIR/ko/$module" ]; then
            log_error "驱动文件不存在: $GMSL2_DRIVER_DIR/ko/$module"
            return 1
        fi
    done
    
    log_info "✓ GMSL2驱动目录检查通过"
    return 0
}

# 加载摄像头驱动
load_drivers() {
    log_info "加载GMSL2摄像头驱动..."
    
    cd "$GMSL2_DRIVER_DIR/ko" || {
        log_error "无法进入驱动目录: $GMSL2_DRIVER_DIR/ko"
        return 1
    }
    
    # 检查并加载MAX9295串行器驱动
    if lsmod | grep -q "^max9295 "; then
        log_info "max9295驱动已加载"
    else
        log_info "加载max9295.ko..."
        if insmod max9295.ko; then
            log_info "✓ max9295驱动加载成功"
        else
            log_error "✗ max9295驱动加载失败"
            return 1
        fi
    fi
    
    # 检查并加载MAX9296解串器驱动
    if lsmod | grep -q "^max9296 "; then
        log_info "max9296驱动已加载"
    else
        log_info "加载max9296.ko..."
        if insmod max9296.ko; then
            log_info "✓ max9296驱动加载成功"
        else
            log_error "✗ max9296驱动加载失败"
            return 1
        fi
    fi
    
    # 检查并加载GMSL2主驱动
    if lsmod | grep -q "^sgx_yuv_gmsl2 "; then
        log_info "sgx-yuv-gmsl2驱动已加载"
    else
        log_info "加载sgx-yuv-gmsl2.ko (启用3G模式)..."
        if insmod sgx-yuv-gmsl2.ko enable_3G=1,1,1,1; then
            log_info "✓ sgx-yuv-gmsl2驱动加载成功"
        else
            log_error "✗ sgx-yuv-gmsl2驱动加载失败"
            return 1
        fi
    fi
    
    # 等待设备节点出现
    log_info "等待摄像头设备节点出现..."
    local max_wait=10
    local wait_count=0
    
    while [ $wait_count -lt $max_wait ]; do
        local device_count=0
        for port in 0 1 2 3; do
            if [ -c "/dev/video${port}" ]; then
                ((device_count++))
            fi
        done
        
        if [ $device_count -ge 1 ]; then
            log_info "✓ 检测到 $device_count 个摄像头设备节点"
            break
        fi
        
        log_info "等待设备节点... ($((wait_count + 1))/$max_wait)"
        sleep 1
        ((wait_count++))
    done
    
    if [ $wait_count -eq $max_wait ]; then
        log_warn "等待设备节点超时，但继续配置..."
    fi
    
    return 0
}

# 配置摄像头
configure_cameras() {
    log_info "配置摄像头 (类型: SG3S-ISX031C-GMSL2)..."
    
    local configured_count=0
    local total_ports=5
    
    # 配置每路摄像头
    for port in 0 1 2 3; do
        if [ -c "/dev/video${port}" ]; then
            log_info "配置摄像头端口 $port..."
            
            # 使用v4l2-ctl配置摄像头，增加重试机制
            local retry_count=0
            local max_retries=3
            local config_success=false
            
            while [ $retry_count -lt $max_retries ] && [ "$config_success" = false ]; do
                if v4l2-ctl -d /dev/video${port} -c sensor_mode=1,trig_pin=0xffff0007 2>/dev/null; then
                    # 验证配置是否真正生效
                    local sensor_mode=$(v4l2-ctl -d /dev/video${port} --get-ctrl=sensor_mode 2>/dev/null | cut -d: -f2 | tr -d ' ')
                    if [ "$sensor_mode" = "1" ]; then
                        log_info "✓ 摄像头 $port 配置成功 (sensor_mode=$sensor_mode)"
                        config_success=true
                        ((configured_count++))
                    else
                        log_warn "⚠ 摄像头 $port 配置部分生效 (sensor_mode=$sensor_mode), 重试..."
                        ((retry_count++))
                        sleep 0.5
                    fi
                else
                    log_warn "✗ 摄像头 $port 配置命令失败, 重试 $((retry_count + 1))/$max_retries"
                    ((retry_count++))
                    sleep 0.5
                fi
            done
            
            if [ "$config_success" = false ]; then
                log_warn "✗ 摄像头 $port 配置最终失败"
            fi
        else
            log_warn "摄像头端口 $port: 设备节点不存在"
        fi
        
        sleep 0.5
    done
    
    log_info "摄像头配置完成: $configured_count/$total_ports 个端口配置成功"
    
    # 如果至少有一个摄像头配置成功，认为服务启动成功
    if [ $configured_count -gt 0 ]; then
        return 0
    else
        log_error "没有摄像头配置成功"
        return 1
    fi
}

# 验证摄像头状态
verify_cameras() {
    log_info "验证摄像头状态..."
    
    local available_count=0
    
    for port in 0 1 2 3 4; do
        if [ -c "/dev/video${port}" ]; then
            # 尝试获取格式信息来验证摄像头是否可访问
            if v4l2-ctl -d /dev/video${port} --get-fmt-video >/dev/null 2>&1; then
                log_info "✓ 摄像头 $port: 可用"
                ((available_count++))
            else
                log_warn "✗ 摄像头 $port: 不可访问"
            fi
        else
            log_warn "✗ 摄像头 $port: 设备节点未找到"
        fi
    done
    
    log_info "摄像头验证完成: $available_count/4 个摄像头可用"
    return 0
}

# 主函数
main() {
    log_info "开始CR摄像头系统初始化..."
    
    # 检查摄像头驱动
    if ! check_camera_driver; then
        log_error "摄像头驱动检查失败"
        exit 1
    fi
    
    # 加载驱动
    if ! load_drivers; then
        log_error "摄像头驱动加载失败"
        exit 1
    fi
    
    # 配置摄像头
    if ! configure_cameras; then
        log_error "摄像头配置失败"
        exit 1
    fi
    
    # 验证摄像头状态
    verify_cameras
    
    log_info "CR摄像头系统初始化完成"
    return 0
}

# 执行主函数
main "$@"
