#include "vic_converter.hpp"
#include <iostream>
#include <cstring>
#include <chrono>
#include <vector>
#include <numeric>
#include <algorithm>

// NVIDIA VPI includes
#include <vpi/VPI.h>
#include <vpi/algo/ConvertImageFormat.h>

class VICPerformanceTester {
private:
    VPIStream stream_;
    VPIImage input_image_;
    VPIImage output_image_;
    int width_, height_;
    bool initialized_;
    
public:
    VICPerformanceTester() : stream_(nullptr), input_image_(nullptr), output_image_(nullptr), 
                            width_(0), height_(0), initialized_(false) {}
    
    ~VICPerformanceTester() {
        cleanup();
    }
    
    bool initialize(int width, int height) {
        width_ = width;
        height_ = height;
        
        // Create VPI stream with VIC backend
        VPIStatus status = vpiStreamCreate(VPI_BACKEND_VIC, &stream_);
        if (status != VPI_SUCCESS) {
            std::cout << "❌ Failed to create VPI stream: " << vpiStatusGetName(status) << std::endl;
            return false;
        }
        
        // Create input image (UYVY format)
        status = vpiImageCreate(width_, height_, VPI_IMAGE_FORMAT_UYVY, 0, &input_image_);
        if (status != VPI_SUCCESS) {
            std::cout << "❌ Failed to create input image: " << vpiStatusGetName(status) << std::endl;
            return false;
        }
        
        // Create output image (RGBA8 format)
        status = vpiImageCreate(width_, height_, VPI_IMAGE_FORMAT_RGBA8, 0, &output_image_);
        if (status != VPI_SUCCESS) {
            std::cout << "❌ Failed to create output image: " << vpiStatusGetName(status) << std::endl;
            return false;
        }
        
        initialized_ = true;
        std::cout << "✅ Performance tester initialized for " << width_ << "x" << height_ << std::endl;
        return true;
    }
    
    void testConversionPerformance(const uint8_t* uyvy_data, uint8_t* rgb_data, int iterations = 10) {
        if (!initialized_) {
            std::cout << "❌ Tester not initialized" << std::endl;
            return;
        }
        
        std::vector<double> total_times;
        std::vector<double> vic_times;
        std::vector<double> cpu_times;
        std::vector<double> copy_in_times;
        std::vector<double> copy_out_times;
        
        std::cout << "\n🚀 Running performance test (" << iterations << " iterations)..." << std::endl;
        std::cout << "📊 Testing UYVY→RGBA8→RGB conversion pipeline" << std::endl;
        
        for (int i = 0; i < iterations; i++) {
            auto start_total = std::chrono::high_resolution_clock::now();
            
            // Step 1: Copy UYVY data to VPI image
            auto start_copy_in = std::chrono::high_resolution_clock::now();
            VPIImageData input_data;
            VPIStatus status = vpiImageLockData(input_image_, VPI_LOCK_WRITE, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &input_data);
            if (status != VPI_SUCCESS) continue;
            
            void* input_ptr = input_data.buffer.pitch.planes[0].data;
            int input_pitch = input_data.buffer.pitch.planes[0].pitchBytes;
            
            for (int y = 0; y < height_; y++) {
                memcpy((uint8_t*)input_ptr + y * input_pitch, uyvy_data + y * width_ * 2, width_ * 2);
            }
            
            vpiImageUnlock(input_image_);
            auto end_copy_in = std::chrono::high_resolution_clock::now();
            
            // Step 2: VIC hardware conversion UYVY→RGBA8
            auto start_vic = std::chrono::high_resolution_clock::now();
            status = vpiSubmitConvertImageFormat(stream_, VPI_BACKEND_VIC, input_image_, output_image_, nullptr);
            if (status != VPI_SUCCESS) continue;
            
            status = vpiStreamSync(stream_);
            if (status != VPI_SUCCESS) continue;
            auto end_vic = std::chrono::high_resolution_clock::now();
            
            // Step 3: Copy RGBA data and convert to RGB
            auto start_copy_out = std::chrono::high_resolution_clock::now();
            VPIImageData output_data;
            status = vpiImageLockData(output_image_, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &output_data);
            if (status != VPI_SUCCESS) continue;
            
            void* output_ptr = output_data.buffer.pitch.planes[0].data;
            int output_pitch = output_data.buffer.pitch.planes[0].pitchBytes;
            auto end_copy_out = std::chrono::high_resolution_clock::now();
            
            // Step 4: CPU conversion RGBA→RGB
            auto start_cpu = std::chrono::high_resolution_clock::now();
            for (int y = 0; y < height_; y++) {
                const uint8_t* rgba_row = (uint8_t*)output_ptr + y * output_pitch;
                uint8_t* rgb_row = rgb_data + y * width_ * 3;
                
                for (int x = 0; x < width_; x++) {
                    rgb_row[x * 3 + 0] = rgba_row[x * 4 + 0]; // R
                    rgb_row[x * 3 + 1] = rgba_row[x * 4 + 1]; // G
                    rgb_row[x * 3 + 2] = rgba_row[x * 4 + 2]; // B
                }
            }
            auto end_cpu = std::chrono::high_resolution_clock::now();
            
            vpiImageUnlock(output_image_);
            auto end_total = std::chrono::high_resolution_clock::now();
            
            // Calculate times in milliseconds
            double copy_in_time = std::chrono::duration<double, std::milli>(end_copy_in - start_copy_in).count();
            double vic_time = std::chrono::duration<double, std::milli>(end_vic - start_vic).count();
            double copy_out_time = std::chrono::duration<double, std::milli>(end_copy_out - start_copy_out).count();
            double cpu_time = std::chrono::duration<double, std::milli>(end_cpu - start_cpu).count();
            double total_time = std::chrono::duration<double, std::milli>(end_total - start_total).count();
            
            copy_in_times.push_back(copy_in_time);
            vic_times.push_back(vic_time);
            copy_out_times.push_back(copy_out_time);
            cpu_times.push_back(cpu_time);
            total_times.push_back(total_time);
            
            if (i % 5 == 0 || i == iterations - 1) {
                std::cout << "Iteration " << (i+1) << ": Total=" << total_time << "ms "
                         << "(Copy-in=" << copy_in_time << "ms, VIC=" << vic_time << "ms, "
                         << "Copy-out=" << copy_out_time << "ms, CPU=" << cpu_time << "ms)" << std::endl;
            }
        }
        
        // Calculate averages
        auto avg = [](const std::vector<double>& v) {
            return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
        };
        
        std::cout << "\n📈 Performance Analysis Results:" << std::endl;
        std::cout << "=================================" << std::endl;
        std::cout << "🎯 Total Processing Time:    " << avg(total_times) << " ms (avg)" << std::endl;
        std::cout << "   ├─ Copy UYVY to VPI:      " << avg(copy_in_times) << " ms (" 
                  << (avg(copy_in_times)/avg(total_times)*100) << "%)" << std::endl;
        std::cout << "   ├─ VIC UYVY→RGBA8:        " << avg(vic_times) << " ms (" 
                  << (avg(vic_times)/avg(total_times)*100) << "%)" << std::endl;
        std::cout << "   ├─ Copy RGBA from VPI:    " << avg(copy_out_times) << " ms (" 
                  << (avg(copy_out_times)/avg(total_times)*100) << "%)" << std::endl;
        std::cout << "   └─ CPU RGBA→RGB:          " << avg(cpu_times) << " ms (" 
                  << (avg(cpu_times)/avg(total_times)*100) << "%)" << std::endl;
        std::cout << std::endl;
        
        double hardware_time = avg(vic_times);
        double software_time = avg(copy_in_times) + avg(copy_out_times) + avg(cpu_times);
        
        std::cout << "⚡ Hardware vs Software:" << std::endl;
        std::cout << "   • VIC Hardware Time:      " << hardware_time << " ms (" 
                  << (hardware_time/avg(total_times)*100) << "%)" << std::endl;
        std::cout << "   • CPU Software Time:      " << software_time << " ms (" 
                  << (software_time/avg(total_times)*100) << "%)" << std::endl;
        std::cout << std::endl;
        
        std::cout << "🔥 Bottleneck Analysis:" << std::endl;
        std::vector<std::pair<std::string, double>> components;
        components.push_back({"VIC Hardware", avg(vic_times)});
        components.push_back({"CPU RGBA→RGB", avg(cpu_times)});
        components.push_back({"Copy In", avg(copy_in_times)});
        components.push_back({"Copy Out", avg(copy_out_times)});
        
        std::sort(components.begin(), components.end(), 
                  [](const std::pair<std::string, double>& a, const std::pair<std::string, double>& b) { 
                      return a.second > b.second; 
                  });
        
        for (int i = 0; i < components.size(); i++) {
            std::string prefix = (i == 0) ? "   🚨 " : "   📊 ";
            std::cout << prefix << components[i].first << ": " << components[i].second << " ms" << std::endl;
        }
    }
    
private:
    void cleanup() {
        if (output_image_) vpiImageDestroy(output_image_);
        if (input_image_) vpiImageDestroy(input_image_);
        if (stream_) vpiStreamDestroy(stream_);
        initialized_ = false;
    }
};

int main() {
    const int width = 1920;
    const int height = 1536;
    const int iterations = 20;
    
    // Create test data
    size_t uyvy_size = width * height * 2;
    size_t rgb_size = width * height * 3;
    
    auto uyvy_data = std::make_unique<uint8_t[]>(uyvy_size);
    auto rgb_data = std::make_unique<uint8_t[]>(rgb_size);
    
    // Fill with test pattern
    for (size_t i = 0; i < uyvy_size; i++) {
        uyvy_data[i] = i % 256;
    }
    
    VICPerformanceTester tester;
    if (!tester.initialize(width, height)) {
        std::cout << "❌ Failed to initialize performance tester" << std::endl;
        return 1;
    }
    
    tester.testConversionPerformance(uyvy_data.get(), rgb_data.get(), iterations);
    
    return 0;
}
