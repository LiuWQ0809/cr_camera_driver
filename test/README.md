# 测试文件说明

本目录包含了所有与VIC硬件加速和图像格式转换相关的测试文件。

## 测试文件列表

### 基础转换测试
- **test_direct_conversion.cpp/.** - 测试VIC直接UYVY→RGBA8转换
- **test_twostep_performance.cpp** - 测试两步转换（UYVY→NV12→RGBA8）性能
- **test_optimized_conversion.cpp** - 优化版转换测试

### 格式转换兼容性测试
- **test_rgba_rgb_conversion.cpp/.** - 测试RGBA8→RGB8转换在不同后端的支持
- **test_gpu_conversion.cpp/.** - 测试CUDA后端的格式转换能力

### 硬件后端测试
- **test_vic_performance.cpp/.** - VIC硬件性能基准测试
- **test_pva_conversion.cpp/.** - 测试PVA后端的UYVY→RGBA8转换支持
- **test_pva_formats.cpp/.** - 测试PVA支持的各种格式转换

### 混合架构测试
- **test_hybrid_performance.cpp** - VIC+CUDA混合架构性能测试

## 编译和运行

所有测试都可以使用以下命令编译：

```bash
# 编译单个测试文件
cd /home/nvidia/wkp/cr_camera_driver
g++ -std=c++17 -I/opt/nvidia/vpi3/include -I./include \
    src/vic_converter.cpp test/[测试文件名].cpp \
    -o test/[输出名] \
    -L/opt/nvidia/vpi3/lib/aarch64-linux-gnu -lnvvpi -lpthread

# 运行测试
./test/[可执行文件名]
```

## 主要发现

### VIC硬件能力
- ✅ 支持UYVY→RGBA8直接转换（比两步转换快）
- ❌ 不支持RGBA8→RGB8转换

### CUDA后端能力  
- ✅ 支持RGBA8→RGB8转换
- ✅ 支持RGBA8→BGR8转换
- ✅ 支持BGRA8→RGB8转换

### PVA硬件能力
- ❌ 不支持任何图像格式转换
- ℹ️ 主要用于复杂视觉算法（立体匹配、光流等）

### 最优方案
**VIC+CUDA混合架构**：
1. UYVY→RGBA8（VIC硬件加速）
2. RGBA8→RGB8（CUDA硬件加速）

这种方案完全避免了CPU转换，实现了端到端的硬件加速。

## 性能对比

| 方案 | UYVY→RGBA8 | RGBA8→RGB8 | 总耗时 | FPS |
|------|------------|-------------|--------|-----|
| 两步转换(VIC+CPU) | 10.0ms | 15.7ms | 27.3ms | 36.6 |
| 混合架构(VIC+CUDA) | 10.0ms | ~3-5ms | ~15ms | ~66.7 |

混合架构相比之前方案提升约80%的性能。
