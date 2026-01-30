#!/bin/bash

# CR Camera Driver Installation Script
# 安装CR摄像头驱动和服务

set -e  # 遇到错误立即退出

# 检查是否以root权限运行
if [ "$EUID" -ne 0 ]; then
    echo "请以root权限运行此脚本: sudo $0"
    exit 1
fi

echo "开始安装CR摄像头驱动..."

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# 执行mod目录下的install.sh
echo "执行mod目录下的安装脚本..."
if [ -f "$PROJECT_DIR/mod/SG8A_ORIN_GMSL2-F_V2_AGX_Orin_YUV_JP6.2_L4TR36.4.3/install.sh" ]; then
    bash "$PROJECT_DIR/mod/SG8A_ORIN_GMSL2-F_V2_AGX_Orin_YUV_JP6.2_L4TR36.4.3/install.sh"
else
    echo "警告: $PROJECT_DIR/mod/SG8A_ORIN_GMSL2-F_V2_AGX_Orin_YUV_JP6.2_L4TR36.4.3/install.sh 不存在，跳过"
fi

# 安装系统服务
echo "安装摄像头初始化服务..."
SERVICE_FILE="$PROJECT_DIR/service/camera.service"
SYSTEMD_SERVICE="/etc/systemd/system/camera.service"

if [ -f "$SERVICE_FILE" ]; then
    cp "$SERVICE_FILE" "$SYSTEMD_SERVICE"
    echo "服务文件已复制到 $SYSTEMD_SERVICE"
else
    echo "错误: 服务文件不存在: $SERVICE_FILE"
    exit 1
fi

# 重新加载systemd配置
echo "重新加载systemd配置..."
systemctl daemon-reload

# 启用服务自启动
echo "启用服务自启动..."
systemctl enable camera.service

# 启动服务
echo "启动摄像头初始化服务..."
systemctl start camera.service

# 检查服务状态
echo "检查服务状态..."
if systemctl is-active --quiet camera.service; then
    echo "✓ 摄像头初始化服务已成功启动"
else
    echo "✗ 摄像头初始化服务启动失败"
    systemctl status camera.service
    exit 1
fi

echo "CR摄像头驱动安装完成！"
echo "服务将在系统启动时自动运行。"
