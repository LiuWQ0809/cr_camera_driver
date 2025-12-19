#!/bin/bash

# CR Camera Initialization Script (no cam_geac dependency)
# Run local init_demo if available, then configure sensor_mode/trig_pin.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEAC_INIT_DIR="$SCRIPT_DIR/../geac_init"
INIT_DEMO="$GEAC_INIT_DIR/demo/init_demo"
CFG_FILE="$GEAC_INIT_DIR/cfg/hb_j5dev.json"

log_info() {
    echo "[INFO] $1"
}

log_warn() {
    echo "[WARN] $1"
}

log_error() {
    echo "[ERROR] $1"
}

compute_camera_mask() {
    if [ ! -f "$CFG_FILE" ]; then
        return 1
    fi

    local ports
    ports=$(grep -o '"port_[0-9]\+"' "$CFG_FILE" | sed 's/"port_//; s/"//g' | sort -u | tr '\n' ' ')
    if [ -z "$ports" ]; then
        return 1
    fi

    local mask=0
    local port
    for port in $ports; do
        mask=$((mask + (1 << port)))
    done

    echo "$mask"
    return 0
}

run_init_demo() {
    if [ ! -x "$INIT_DEMO" ]; then
        return 1
    fi

    local mask
    if ! mask=$(compute_camera_mask); then
        log_warn "无法解析 $CFG_FILE 中的端口信息，跳过 init_demo"
        return 1
    fi

    log_info "运行 init_demo 初始化 (mask=$mask)..."
    local lib_path="$GEAC_INIT_DIR/lib:$GEAC_INIT_DIR/lib/sensorlib"
    (cd "$GEAC_INIT_DIR" && sudo -E env LD_LIBRARY_PATH="$lib_path${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ./demo/init_demo "$mask")
    return 0
}

log_info "加载 i2c-mux-pca954x 模块..."
if ! sudo modprobe i2c-mux-pca954x; then
    log_warn "i2c-mux-pca954x 加载失败（可能已加载或模块不存在）"
fi

init_demo_ok=false
if run_init_demo; then
    init_demo_ok=true
else
    log_warn "init_demo 未运行或失败，继续尝试 v4l2-ctl 初始化"
fi

log_info "配置相机 sensor_mode 和 trig_pin..."
configured_count=0
for port in 0 1 2 3 4; do
    if [ -c "/dev/video${port}" ]; then
        if sudo v4l2-ctl -d /dev/video${port} -c sensor_mode=1,trig_pin=0xffff0007; then
            sensor_mode=$(sudo v4l2-ctl -d /dev/video${port} --get-ctrl=sensor_mode 2>/dev/null | cut -d: -f2 | tr -d ' ')
            if [ "$sensor_mode" = "1" ]; then
                log_info "Camera ${port} 配置成功 (sensor_mode=$sensor_mode)"
                configured_count=$((configured_count + 1))
            else
                log_warn "Camera ${port} sensor_mode 未生效 (sensor_mode=$sensor_mode)"
            fi
        else
            log_warn "Camera ${port} 配置失败"
        fi
    else
        log_warn "Camera ${port} 设备节点不存在"
    fi
done

if [ "$configured_count" -eq 0 ]; then
    if [ "$init_demo_ok" = true ]; then
        log_warn "v4l2-ctl 未成功配置，但 init_demo 已完成，继续启动"
    else
        log_error "未成功配置任何相机"
        exit 1
    fi
fi

log_info "设置 /dev/video* 设备权限..."
if ls /dev/video* 1>/dev/null 2>&1; then
    sudo chmod 666 /dev/video* || log_warn "chmod /dev/video* 失败"
fi

log_info "相机初始化完成"
