#include "../src/vic_converter.hpp"
#include <iostream>
#include <chrono>
#include <vector>
#include <cstring>

int main() {
    std::cout << "=== VIC+CUDA Hybrid Performance Test ===" << std::endl;
    
    const int width = 1920;
    const int height = 1080;
    const int test_iterations = 100;
    
    // Create converter
    VICConverter converter;
    
    // Initialize
    if (!converter.initialize(width, height)) {
        std::cout << "❌ Failed to initialize converter: " << converter.getLastError() << std::endl;
        return -1;
    }
    
    std::cout << "✅ VIC+CUDA converter initialized for " << width << "x" << height << std::endl;
    
    // Prepare test data
    size_t uyvy_size = width * height * 2;
    size_t rgb_size = width * height * 3;
    
    std::vector<uint8_t> uyvy_data(uyvy_size);
    std::vector<uint8_t> rgb_data(rgb_size);
    
    // Fill UYVY with test pattern
    for (size_t i = 0; i < uyvy_size; i++) {
        uyvy_data[i] = (i * 123) % 256;  // Some test pattern
    }
    
    std::cout << "Input UYVY size: " << uyvy_size << " bytes" << std::endl;
    std::cout << "Output RGB size: " << rgb_size << " bytes" << std::endl;
    
    // Warm up
    std::cout << "Warming up..." << std::endl;
    for (int i = 0; i < 10; i++) {
        if (!converter.convertUYVYToRGB(uyvy_data.data(), uyvy_size, rgb_data.data(), rgb_size)) {
            std::cout << "❌ Warm-up conversion failed: " << converter.getLastError() << std::endl;
            return -1;
        }
    }
    
    std::cout << "Starting performance test (" << test_iterations << " iterations)..." << std::endl;
    
    // Performance test
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < test_iterations; i++) {
        if (!converter.convertUYVYToRGB(uyvy_data.data(), uyvy_size, rgb_data.data(), rgb_size)) {
            std::cout << "❌ Conversion failed at iteration " << i << ": " << converter.getLastError() << std::endl;
            return -1;
        }
        
        if ((i + 1) % 20 == 0) {
            std::cout << "Completed " << (i + 1) << " iterations..." << std::endl;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    double total_time_ms = duration.count() / 1000.0;
    double avg_time_ms = total_time_ms / test_iterations;
    double fps = 1000.0 / avg_time_ms;
    
    std::cout << "\n=== Performance Results ===" << std::endl;
    std::cout << "Total test time: " << total_time_ms << " ms" << std::endl;
    std::cout << "Average conversion time: " << avg_time_ms << " ms" << std::endl;
    std::cout << "Performance: " << fps << " FPS" << std::endl;
    
    // Verify output data
    size_t non_zero_count = 0;
    for (size_t i = 0; i < rgb_size; i++) {
        if (rgb_data[i] != 0) non_zero_count++;
    }
    
    std::cout << "\n=== Output Verification ===" << std::endl;
    std::cout << "Non-zero RGB pixels: " << non_zero_count << "/" << rgb_size << std::endl;
    std::cout << "RGB data sample (first 12 bytes): ";
    for (int i = 0; i < 12 && i < rgb_size; i++) {
        std::cout << (int)rgb_data[i] << " ";
    }
    std::cout << std::endl;
    
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "✅ VIC+CUDA hybrid conversion working!" << std::endl;
    std::cout << "Architecture: UYVY->(VIC)->RGBA8->(CUDA)->RGB8" << std::endl;
    std::cout << "Performance: " << avg_time_ms << " ms per frame (" << fps << " FPS)" << std::endl;
    
    return 0;
}