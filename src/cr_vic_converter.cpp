#include "cr_camera_driver/cr_vic_converter.hpp"
#include <iostream>
#include <cstring>
#include <linux/videodev2.h>

// NVIDIA VPI includes
#include <vpi/VPI.h>
#include <vpi/algo/ConvertImageFormat.h>

struct VICConverter::VPIConverterImpl {
    VPIStream vic_stream;    // VIC stream for YUV422->RGBA conversion
    VPIStream gpu_stream;    // GPU stream for RGBA->RGB conversion
    VPIImage input_image;    // YUV422 input (UYVY or YUYV)
    VPIImage rgba_image;     // RGBA intermediate
    VPIImage rgb_image;      // RGB8 output
    bool resources_created;
    
    VPIConverterImpl() : vic_stream(nullptr), gpu_stream(nullptr), input_image(nullptr), 
                        rgba_image(nullptr), rgb_image(nullptr), resources_created(false) {}
    
    ~VPIConverterImpl() {
        if (resources_created) {
            if (rgb_image) vpiImageDestroy(rgb_image);
            if (rgba_image) vpiImageDestroy(rgba_image);
            if (input_image) vpiImageDestroy(input_image);
            if (gpu_stream) vpiStreamDestroy(gpu_stream);
            if (vic_stream) vpiStreamDestroy(vic_stream);
        }
    }
};

VICConverter::VICConverter() 
    : pImpl(std::make_unique<VPIConverterImpl>())
    , width_(0)
    , height_(0)
    , initialized_(false)
    , v4l2_pixel_format_(0)
    , total_conversions_(0)
    , total_vic_time_ms_(0.0)
    , total_gpu_time_ms_(0.0)  // GPU时间统计
    , total_conversion_time_ms_(0.0)
    , last_stats_time_(std::chrono::steady_clock::now()) {
}

VICConverter::~VICConverter() {
    cleanup();
}

bool VICConverter::initialize(int input_width, int input_height, int v4l2_pixel_format) {
    if (initialized_) {
        last_error_ = "VPI converter already initialized";
        return false;
    }
    
    v4l2_pixel_format_ = v4l2_pixel_format;
    width_ = input_width;
    height_ = input_height;
    
    try {
        // Create VIC stream for YUV422->RGBA conversion
        VPIStatus status = vpiStreamCreate(VPI_BACKEND_VIC, &pImpl->vic_stream);
        if (status != VPI_SUCCESS) {
            last_error_ = "Failed to create VIC stream: " + std::string(vpiStatusGetName(status));
            return false;
        }
        
        // Create GPU stream for RGBA->RGB conversion
        status = vpiStreamCreate(VPI_BACKEND_CUDA, &pImpl->gpu_stream);
        if (status != VPI_SUCCESS) {
            last_error_ = "Failed to create GPU stream: " + std::string(vpiStatusGetName(status));
            return false;
        }
        
        // Create input image (YUV422: UYVY or YUYV)
        VPIImageFormat vpi_input_format = VPI_IMAGE_FORMAT_UYVY;
        input_format_str_ = "UYVY";
        if (v4l2_pixel_format_ == V4L2_PIX_FMT_YUYV) {
            vpi_input_format = VPI_IMAGE_FORMAT_YUYV;
            input_format_str_ = "YUYV";
        } else if (v4l2_pixel_format_ != V4L2_PIX_FMT_UYVY) {
            last_error_ = "Unsupported V4L2 pixel format (only UYVY/YUYV supported)";
            return false;
        }

        status = vpiImageCreate(width_, height_, vpi_input_format, 0, &pImpl->input_image);
        if (status != VPI_SUCCESS) {
            last_error_ = "Failed to create input VPI image: " + std::string(vpiStatusGetName(status));
            return false;
        }
        
        // Create intermediate RGBA image for VIC conversion
        status = vpiImageCreate(width_, height_, VPI_IMAGE_FORMAT_RGBA8, 0, &pImpl->rgba_image);
        if (status != VPI_SUCCESS) {
            last_error_ = "Failed to create RGBA intermediate image: " + std::string(vpiStatusGetName(status));
            return false;
        }
        
        // Create RGB8 output image
        status = vpiImageCreate(width_, height_, VPI_IMAGE_FORMAT_RGB8, 0, &pImpl->rgb_image);
        if (status != VPI_SUCCESS) {
            last_error_ = "Failed to create RGB output image: " + std::string(vpiStatusGetName(status));
            return false;
        }
        
        pImpl->resources_created = true;
        initialized_ = true;
        
        size_t input_size = width_ * height_ * 2;  // YUV422 is 2 bytes per pixel
        size_t output_size = width_ * height_ * 3; // RGB is 3 bytes per pixel
        
        std::cout << "VIC converter (VPI) initialized successfully for " << width_ << "x" << height_ 
                  << " using TWO-STEP conversion: " << input_format_str_ << "->RGBA (VIC) + RGBA->RGB (GPU)"
                  << " (Input: " << input_size << " bytes, Output: " << output_size << " bytes)" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        last_error_ = "Exception during VPI initialization: " + std::string(e.what());
        cleanup();
        return false;
    }
}

bool VICConverter::convertUYVYToRGB(const uint8_t* uyvy_data, size_t uyvy_size, 
                                   uint8_t* rgb_data, size_t rgb_size) {
    // 多线程安全：锁定VIC硬件资源
    std::lock_guard<std::mutex> lock(conversion_mutex_);
    
    // 开始计时
    auto conversion_start = std::chrono::steady_clock::now();
    
    if (!initialized_ || !pImpl->resources_created) {
        last_error_ = "VPI converter not initialized";
        return false;
    }
    
    if (!uyvy_data || !rgb_data) {
        last_error_ = "Invalid input/output data pointers";
        return false;
    }
    
    size_t expected_uyvy_size = width_ * height_ * 2; // UYVY is 2 bytes per pixel
    size_t expected_rgb_size = width_ * height_ * 3;  // RGB is 3 bytes per pixel
    
    if (uyvy_size < expected_uyvy_size) {
        last_error_ = "UYVY data size too small: " + std::to_string(uyvy_size) + 
                     " < " + std::to_string(expected_uyvy_size);
        return false;
    }
    
    if (rgb_size < expected_rgb_size) {
        last_error_ = "RGB buffer size too small: " + std::to_string(rgb_size) + 
                     " < " + std::to_string(expected_rgb_size);
        return false;
    }
    
    try {
        // Lock input image for writing
        VPIImageData input_data;
        VPIStatus status = vpiImageLockData(pImpl->input_image, VPI_LOCK_WRITE, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &input_data);
        if (status != VPI_SUCCESS) {
            last_error_ = "Failed to lock input image: " + std::string(vpiStatusGetName(status));
            return false;
        }
        
        // Copy UYVY data to input image
        void* input_ptr = input_data.buffer.pitch.planes[0].data;
        int input_pitch = input_data.buffer.pitch.planes[0].pitchBytes;
        
        for (int y = 0; y < height_; y++) {
            memcpy((uint8_t*)input_ptr + y * input_pitch, uyvy_data + y * width_ * 2, width_ * 2);
        }
        
        // Unlock input image
        status = vpiImageUnlock(pImpl->input_image);
        if (status != VPI_SUCCESS) {
            last_error_ = "Failed to unlock input image: " + std::string(vpiStatusGetName(status));
            return false;
        }
        
        // 第一步：UYVY to RGBA using VIC hardware acceleration
        auto vic_start = std::chrono::steady_clock::now();
        
        status = vpiSubmitConvertImageFormat(pImpl->vic_stream, VPI_BACKEND_VIC, 
                                           pImpl->input_image, pImpl->rgba_image, nullptr);
        if (status != VPI_SUCCESS) {
            last_error_ = "Failed to submit UYVY->RGBA conversion (VIC): " + std::string(vpiStatusGetName(status));
            return false;
        }
        
        // Wait for VIC conversion to complete
        status = vpiStreamSync(pImpl->vic_stream);
        if (status != VPI_SUCCESS) {
            last_error_ = "Failed to sync VIC stream: " + std::string(vpiStatusGetName(status));
            return false;
        }
        
        auto vic_end = std::chrono::steady_clock::now();
        double vic_time_ms = std::chrono::duration<double, std::milli>(vic_end - vic_start).count();
        
        // 第二步：RGBA to RGB conversion using GPU backend
        auto gpu_start = std::chrono::steady_clock::now();
        
        status = vpiSubmitConvertImageFormat(pImpl->gpu_stream, VPI_BACKEND_CUDA, 
                                           pImpl->rgba_image, pImpl->rgb_image, nullptr);
        if (status != VPI_SUCCESS) {
            last_error_ = "Failed to submit RGBA->RGB conversion (GPU): " + std::string(vpiStatusGetName(status));
            return false;
        }
        
        // Wait for GPU conversion to complete
        status = vpiStreamSync(pImpl->gpu_stream);
        if (status != VPI_SUCCESS) {
            last_error_ = "Failed to sync GPU stream: " + std::string(vpiStatusGetName(status));
            return false;
        }
        
        auto gpu_end = std::chrono::steady_clock::now();
        double gpu_time_ms = std::chrono::duration<double, std::milli>(gpu_end - gpu_start).count();
        
        // Copy RGB result to output buffer
        VPIImageData rgb_output_data;
        status = vpiImageLockData(pImpl->rgb_image, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &rgb_output_data);
        if (status != VPI_SUCCESS) {
            last_error_ = "Failed to lock RGB output image: " + std::string(vpiStatusGetName(status));
            return false;
        }
        
        // Copy RGB data to output buffer
        const uint8_t* rgb_data_ptr = (const uint8_t*)rgb_output_data.buffer.pitch.planes[0].data;
        int rgb_pitch = rgb_output_data.buffer.pitch.planes[0].pitchBytes;
        
        for (int y = 0; y < height_; y++) {
            memcpy(rgb_data + y * width_ * 3, rgb_data_ptr + y * rgb_pitch, width_ * 3);
        }
        
        // Unlock RGB image
        status = vpiImageUnlock(pImpl->rgb_image);
        if (status != VPI_SUCCESS) {
            last_error_ = "Failed to unlock RGB image: " + std::string(vpiStatusGetName(status));
            return false;
        }
        
        // 计算总转换时间并更新统计
        auto conversion_end = std::chrono::steady_clock::now();
        double total_time_ms = std::chrono::duration<double, std::milli>(conversion_end - conversion_start).count();
        
        // 更新性能统计
        total_conversions_++;
        total_vic_time_ms_ += vic_time_ms;
        total_gpu_time_ms_ += gpu_time_ms;  // RGBA->RGB转换时间 (现在使用GPU)
        total_conversion_time_ms_ += total_time_ms;
        
        return true;
        
    } catch (const std::exception& e) {
        last_error_ = "Exception during VPI conversion: " + std::string(e.what());
        return false;
    }
}

size_t VICConverter::getRGBBufferSize() const {
    if (!initialized_) {
        return 0;
    }
    return width_ * height_ * 3; // RGB is 3 bytes per pixel
}

bool VICConverter::isInitialized() const {
    return initialized_;
}

std::string VICConverter::getLastError() const {
    return last_error_;
}

std::string VICConverter::getConversionStats() const {
    if (total_conversions_ == 0) {
        return "No conversions performed yet";
    }
    
    double avg_vic_time = total_vic_time_ms_ / total_conversions_;
    double avg_gpu_time = total_gpu_time_ms_ / total_conversions_;  // 现在存储的是GPU时间
    double avg_total_time = total_conversion_time_ms_ / total_conversions_;
    
    auto current_time = std::chrono::steady_clock::now();
    double elapsed_seconds = std::chrono::duration<double>(current_time - last_stats_time_).count();
    double fps = total_conversions_ / elapsed_seconds;
    
    std::string stats = "VIC Conversion Performance Stats:\n";
    stats += "  Total conversions: " + std::to_string(total_conversions_) + "\n";
    stats += "  Average VIC time: " + std::to_string(avg_vic_time) + " ms\n";
    stats += "  Average GPU time: " + std::to_string(avg_gpu_time) + " ms\n";
    stats += "  Average total time: " + std::to_string(avg_total_time) + " ms\n";
    stats += "  Conversion FPS: " + std::to_string(fps) + "\n";
    stats += "  VIC efficiency: " + std::to_string(avg_vic_time / avg_total_time * 100) + "%\n";
    stats += "  GPU efficiency: " + std::to_string(avg_gpu_time / avg_total_time * 100) + "%";
    
    return stats;
}

void VICConverter::resetStats() {
    total_conversions_ = 0;
    total_vic_time_ms_ = 0.0;
    total_gpu_time_ms_ = 0.0;
    total_conversion_time_ms_ = 0.0;
    last_stats_time_ = std::chrono::steady_clock::now();
}

void VICConverter::cleanup() {
    // Cleanup is handled by VPIConverterImpl destructor
    initialized_ = false;
    width_ = 0;
    height_ = 0;
}
