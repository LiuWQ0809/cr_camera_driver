#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DRIVER_DIR="${GMSL2_DRIVER_DIR:-$SCRIPT_DIR}"
KERNEL_RELEASE="$(uname -r)"

KO_DIR="$DRIVER_DIR/ko"
DTB_DIR="$DRIVER_DIR/dtb/SGX_YUV_GMSL2"
BOOT_DIR="$DRIVER_DIR/boot"

MODULE_BASE="/lib/modules/${KERNEL_RELEASE}/updates"
TEGRA_CAMERA_DEST="${MODULE_BASE}/drivers/media/platform/tegra/camera"
NVCSI_DEST="${MODULE_BASE}/drivers/video/tegra/host/nvcsi"
I2C_DEST="${MODULE_BASE}/drivers/media/i2c"

MODULE_FILES=("tegra-camera.ko" "nvhost-nvcsi-t194.ko" "max9295.ko" "max9296.ko" "sgx-yuv-gmsl2.ko")
MODULE_NAMES=("max9295" "max9296" "sgx-yuv-gmsl2")

log_info() {
    echo "[INFO] $1"
}

log_error() {
    echo "[ERROR] $1"
}

if [ ! -d "$KO_DIR" ]; then
    log_error "未找到 ko 目录: $KO_DIR"
    log_error "可通过设置 GMSL2_DRIVER_DIR 指定驱动目录"
    exit 1
fi

for module_file in "${MODULE_FILES[@]}"; do
    if [ ! -f "$KO_DIR/$module_file" ]; then
        log_error "未找到模块文件: $KO_DIR/$module_file"
        exit 1
    fi
done

# update ko
log_info "复制模块到标准路径..."
sudo mkdir -p "$TEGRA_CAMERA_DEST" "$NVCSI_DEST" "$I2C_DEST"
sudo install -m 0644 "$KO_DIR/tegra-camera.ko" "$TEGRA_CAMERA_DEST/tegra-camera.ko"
sudo install -m 0644 "$KO_DIR/nvhost-nvcsi-t194.ko" "$NVCSI_DEST/nvhost-nvcsi-t194.ko"
sudo install -m 0644 "$KO_DIR/max9295.ko" "$I2C_DEST/max9295.ko"
sudo install -m 0644 "$KO_DIR/max9296.ko" "$I2C_DEST/max9296.ko"
sudo install -m 0644 "$KO_DIR/sgx-yuv-gmsl2.ko" "$I2C_DEST/sgx-yuv-gmsl2.ko"

log_info "写入 /etc/modprobe.d/gmsl2.conf ..."
sudo tee /etc/modprobe.d/gmsl2.conf >/dev/null <<'EOF'
options max9295
options max9296
options sgx-yuv-gmsl2 enable_3G=1,1,1,1
EOF

log_info "写入 /etc/modules-load.d/gmsl2.conf ..."
sudo tee /etc/modules-load.d/gmsl2.conf >/dev/null <<'EOF'
max9295
max9296
sgx-yuv-gmsl2
EOF

log_info "运行 depmod 更新模块依赖..."
sudo depmod -a "$KERNEL_RELEASE"

# add dtbo
log_info "更新 dtbo..."
sudo cp "$DTB_DIR/tegra234-camera-yuv-gmsl2x8-overlay.dtbo" /boot/

# upgrade Image
log_info "更新 Image..."
sudo cp "$BOOT_DIR/Image" /boot/Image

sync

log_info "完成。可执行: sudo modprobe ${MODULE_NAMES[*]} (或重启生效)"
