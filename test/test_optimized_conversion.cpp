#include <iostream>
#include <cstring>
#include <chrono>
#include <vector>
#include <numeric>
#include <algorithm>

// NVIDIA VPI includes
#include <vpi/VPI.h>
#include <vpi/algo/ConvertImageFormat.h>

class OptimizedVICConverter {
private:
    VPIStream vic_stream_;
    VPIStream cuda_stream_;
    VPIImage input_image_;
    VPIImage rgba_image_;
    VPIImage output_image_;
    int width_, height_;
    bool initialized_;
    
public:
    OptimizedVICConverter() : vic_stream_(nullptr), cuda_stream_(nullptr), 
                             input_image_(nullptr), rgba_image_(nullptr), output_image_(nullptr),
                             width_(0), height_(0), initialized_(false) {}
    
    ~OptimizedVICConverter() {
        cleanup();
    }
    
    bool initialize(int width, int height) {
        width_ = width;
        height_ = height;
        
        // Create VIC stream for UYVY->RGBA8 conversion
        VPIStatus status = vpiStreamCreate(VPI_BACKEND_VIC, &vic_stream_);
        if (status != VPI_SUCCESS) {
            std::cout << "❌ Failed to create VIC stream: " << vpiStatusGetName(status) << std::endl;
            return false;
        }
        
        // Create CUDA stream for RGBA8->RGB8 conversion
        status = vpiStreamCreate(VPI_BACKEND_CUDA, &cuda_stream_);
        if (status != VPI_SUCCESS) {
            std::cout << "❌ Failed to create CUDA stream: " << vpiStatusGetName(status) << std::endl;
            return false;
        }
        
        // Create input image (UYVY format)
        status = vpiImageCreate(width_, height_, VPI_IMAGE_FORMAT_UYVY, 0, &input_image_);
        if (status != VPI_SUCCESS) {
            std::cout << "❌ Failed to create input image: " << vpiStatusGetName(status) << std::endl;
            return false;
        }
        
        // Create intermediate image (RGBA8 format)
        status = vpiImageCreate(width_, height_, VPI_IMAGE_FORMAT_RGBA8, 0, &rgba_image_);
        if (status != VPI_SUCCESS) {
            std::cout << "❌ Failed to create RGBA image: " << vpiStatusGetName(status) << std::endl;
            return false;
        }
        
        // Create output image (RGB8 format)
        status = vpiImageCreate(width_, height_, VPI_IMAGE_FORMAT_RGB8, 0, &output_image_);
        if (status != VPI_SUCCESS) {
            std::cout << "❌ Failed to create output image: " << vpiStatusGetName(status) << std::endl;
            return false;
        }
        
        initialized_ = true;
        std::cout << "✅ Optimized converter initialized (VIC + CUDA backends)" << std::endl;
        return true;
    }
    
    void testConversionPerformance(const uint8_t* uyvy_data, uint8_t* rgb_data, int iterations = 10) {
        if (!initialized_) {
            std::cout << "❌ Converter not initialized" << std::endl;
            return;
        }
        
        std::vector<double> total_times;
        std::vector<double> vic_times;
        std::vector<double> cuda_times;
        std::vector<double> copy_in_times;
        std::vector<double> copy_out_times;
        
        std::cout << "\n🚀 Testing optimized conversion pipeline (" << iterations << " iterations)..." << std::endl;
        std::cout << "📊 VIC (UYVY→RGBA8) + CUDA (RGBA8→RGB8) pipeline" << std::endl;
        
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
            status = vpiSubmitConvertImageFormat(vic_stream_, VPI_BACKEND_VIC, input_image_, rgba_image_, nullptr);
            if (status != VPI_SUCCESS) continue;
            
            status = vpiStreamSync(vic_stream_);
            if (status != VPI_SUCCESS) continue;
            auto end_vic = std::chrono::high_resolution_clock::now();
            
            // Step 3: CUDA GPU conversion RGBA8→RGB8
            auto start_cuda = std::chrono::high_resolution_clock::now();
            status = vpiSubmitConvertImageFormat(cuda_stream_, VPI_BACKEND_CUDA, rgba_image_, output_image_, nullptr);
            if (status != VPI_SUCCESS) continue;
            
            status = vpiStreamSync(cuda_stream_);
            if (status != VPI_SUCCESS) continue;
            auto end_cuda = std::chrono::high_resolution_clock::now();
            
            // Step 4: Copy RGB data
            auto start_copy_out = std::chrono::high_resolution_clock::now();
            VPIImageData output_data;
            status = vpiImageLockData(output_image_, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &output_data);
            if (status != VPI_SUCCESS) continue;
            
            void* output_ptr = output_data.buffer.pitch.planes[0].data;
            int output_pitch = output_data.buffer.pitch.planes[0].pitchBytes;
            
            // Direct RGB copy (no CPU conversion needed!)
            for (int y = 0; y < height_; y++) {
                memcpy(rgb_data + y * width_ * 3, (uint8_t*)output_ptr + y * output_pitch, width_ * 3);
            }
            
            vpiImageUnlock(output_image_);
            auto end_copy_out = std::chrono::high_resolution_clock::now();
            auto end_total = std::chrono::high_resolution_clock::now();
            
            // Calculate times in milliseconds
            double copy_in_time = std::chrono::duration<double, std::milli>(end_copy_in - start_copy_in).count();
            double vic_time = std::chrono::duration<double, std::milli>(end_vic - start_vic).count();
            double cuda_time = std::chrono::duration<double, std::milli>(end_cuda - start_cuda).count();
            double copy_out_time = std::chrono::duration<double, std::milli>(end_copy_out - start_copy_out).count();
            double total_time = std::chrono::duration<double, std::milli>(end_total - start_total).count();
            
            copy_in_times.push_back(copy_in_time);
            vic_times.push_back(vic_time);
            cuda_times.push_back(cuda_time);
            copy_out_times.push_back(copy_out_time);
            total_times.push_back(total_time);
            
            if (i % 5 == 0 || i == iterations - 1) {
                std::cout << "Iteration " << (i+1) << ": Total=" << total_time << "ms "
                         << "(Copy-in=" << copy_in_time << "ms, VIC=" << vic_time << "ms, "
                         << "CUDA=" << cuda_time << "ms, Copy-out=" << copy_out_time << "ms)" << std::endl;
            }
        }
        
        // Calculate averages
        auto avg = [](const std::vector<double>& v) {
            return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
        };
        
        std::cout << "\n📈 Optimized Performance Analysis:" << std::endl;
        std::cout << "==================================" << std::endl;
        std::cout << "🎯 Total Processing Time:    " << avg(total_times) << " ms (avg)" << std::endl;
        std::cout << "   ├─ Copy UYVY to VPI:      " << avg(copy_in_times) << " ms (" 
                  << (avg(copy_in_times)/avg(total_times)*100) << "%)" << std::endl;
        std::cout << "   ├─ VIC UYVY→RGBA8:        " << avg(vic_times) << " ms (" 
                  << (avg(vic_times)/avg(total_times)*100) << "%)" << std::endl;
        std::cout << "   ├─ CUDA RGBA8→RGB8:       " << avg(cuda_times) << " ms (" 
                  << (avg(cuda_times)/avg(total_times)*100) << "%)" << std::endl;
        std::cout << "   └─ Copy RGB from VPI:     " << avg(copy_out_times) << " ms (" 
                  << (avg(copy_out_times)/avg(total_times)*100) << "%)" << std::endl;
        std::cout << std::endl;
        
        double hardware_time = avg(vic_times) + avg(cuda_times);
        double memory_time = avg(copy_in_times) + avg(copy_out_times);
        
        std::cout << "⚡ Hardware vs Memory:" << std::endl;
        std::cout << "   • Hardware Time (VIC+CUDA): " << hardware_time << " ms (" 
                  << (hardware_time/avg(total_times)*100) << "%)" << std::endl;
        std::cout << "   • Memory Copy Time:          " << memory_time << " ms (" 
                  << (memory_time/avg(total_times)*100) << "%)" << std::endl;
        std::cout << std::endl;
        
        std::cout << "🎉 Performance Benefits:" << std::endl;
        std::cout << "   • ✅ NO CPU bottleneck (eliminated RGBA→RGB CPU conversion)" << std::endl;
        std::cout << "   • ✅ Full hardware acceleration (VIC + CUDA)" << std::endl;
        std::cout << "   • ✅ Direct RGB8 output (no format conversion needed)" << std::endl;
    }
    
private:
    void cleanup() {
        if (output_image_) vpiImageDestroy(output_image_);
        if (rgba_image_) vpiImageDestroy(rgba_image_);
        if (input_image_) vpiImageDestroy(input_image_);
        if (cuda_stream_) vpiStreamDestroy(cuda_stream_);
        if (vic_stream_) vpiStreamDestroy(vic_stream_);
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
    
    OptimizedVICConverter converter;
    if (!converter.initialize(width, height)) {
        std::cout << "❌ Failed to initialize optimized converter" << std::endl;
        return 1;
    }
    
    converter.testConversionPerformance(uyvy_data.get(), rgb_data.get(), iterations);
    
    return 0;
}
