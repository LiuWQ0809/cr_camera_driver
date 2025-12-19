# Makefile for VIC Camera Driver Tests
# 编译所有测试文件的便捷工具

CXX = g++
CXXFLAGS = -std=c++17 -I/opt/nvidia/vpi3/include -I../src
LDFLAGS = -L/opt/nvidia/vpi3/lib/aarch64-linux-gnu -lnvvpi -lpthread

# 源文件路径
VIC_SRC = ../src/vic_converter.cpp

# 测试目标
TESTS = test_direct_conversion \
        test_gpu_conversion \
        test_pva_conversion \
        test_pva_formats \
        test_rgba_rgb_conversion \
        test_vic_performance \
        test_hybrid_performance

# 默认目标：编译所有测试
all: $(TESTS)

# 基础转换测试
test_direct_conversion: test_direct_conversion.cpp $(VIC_SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# GPU转换测试
test_gpu_conversion: test_gpu_conversion.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# PVA转换测试
test_pva_conversion: test_pva_conversion.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# PVA格式测试
test_pva_formats: test_pva_formats.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# RGBA→RGB转换测试
test_rgba_rgb_conversion: test_rgba_rgb_conversion.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# VIC性能测试
test_vic_performance: test_vic_performance.cpp $(VIC_SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# 混合架构性能测试
test_hybrid_performance: test_hybrid_performance.cpp $(VIC_SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# 清理目标
clean:
	rm -f $(TESTS)

# 运行所有测试
run-all: all
	@echo "=== 运行所有VIC转换测试 ==="
	@for test in $(TESTS); do \
		echo ""; \
		echo ">>> 运行 $$test <<<"; \
		./$$test || echo "$$test 测试失败"; \
		echo ""; \
	done

# 运行基础测试
run-basic: test_direct_conversion test_vic_performance
	@echo "=== 运行基础VIC测试 ==="
	./test_direct_conversion
	./test_vic_performance

# 运行格式兼容性测试
run-format: test_rgba_rgb_conversion test_gpu_conversion test_pva_formats
	@echo "=== 运行格式转换兼容性测试 ==="
	./test_rgba_rgb_conversion
	./test_gpu_conversion
	./test_pva_formats

# 运行性能测试
run-performance: test_vic_performance test_hybrid_performance
	@echo "=== 运行性能基准测试 ==="
	./test_vic_performance
	./test_hybrid_performance

# 帮助信息
help:
	@echo "VIC Camera Driver 测试编译工具"
	@echo ""
	@echo "可用目标："
	@echo "  all              - 编译所有测试"
	@echo "  clean            - 清理编译文件"
	@echo "  run-all          - 编译并运行所有测试"
	@echo "  run-basic        - 运行基础VIC测试"
	@echo "  run-format       - 运行格式兼容性测试"
	@echo "  run-performance  - 运行性能基准测试"
	@echo "  help             - 显示此帮助信息"
	@echo ""
	@echo "单个测试目标："
	@for test in $(TESTS); do \
		echo "  $$test"; \
	done

.PHONY: all clean run-all run-basic run-format run-performance help
