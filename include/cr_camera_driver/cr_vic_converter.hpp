#ifndef CR_VIC_CONVERTER_HPP
#define CR_VIC_CONVERTER_HPP

#include <memory>
#include <string>
#include <chrono>
#include <mutex>
#include <vector>

/**
 * @brief VPI-based VIC Hardware-accelerated image format converter
 * 
 * This class provides hardware-accelerated image format conversion
 * using NVIDIA's Vision Programming Interface (VPI) with VIC engine on Jetson platforms.
 * Thread-safe for multi-threaded usage.
 */
class VICConverter {
public:
    /**
     * @brief Constructor
     */
    VICConverter();
    
    /**
     * @brief Destructor
     */
    ~VICConverter();
    
    /**
     * @brief Initialize the VPI-based VIC converter
     * 
     * @param input_width Input image width
     * @param input_height Input image height
     * @param v4l2_pixel_format Input pixel format (e.g. V4L2_PIX_FMT_YUYV / V4L2_PIX_FMT_UYVY)
     * @return true if initialization successful, false otherwise
     */
    bool initialize(int input_width, int input_height, int v4l2_pixel_format);
    
    /**
     * @brief Convert UYVY format to RGB format using VPI+VIC hardware
     * 
     * @param uyvy_data Input UYVY data pointer
     * @param uyvy_size Input UYVY data size in bytes
     * @param rgb_data Output RGB data pointer (must be pre-allocated)
     * @param rgb_size Output RGB data size in bytes
     * @return true if conversion successful, false otherwise
     */
    bool convertUYVYToRGB(const uint8_t* uyvy_data, size_t uyvy_size, 
                         uint8_t* rgb_data, size_t rgb_size);
    
    /**
     * @brief Convert UYVY to multiple formats and sizes in one operation
     * 
     * @param uyvy_data Input UYVY data pointer
     * @param uyvy_size Input UYVY data size in bytes
     * @param outputs Vector of output specifications (format, size, buffer)
     * @return true if conversion successful, false otherwise
     */
    struct ConversionOutput {
        std::string format;    // "RGB" or "BGR"
        int width, height;     // Output dimensions
        uint8_t* buffer;       // Output buffer pointer
        size_t buffer_size;    // Buffer size in bytes
    };
    
    bool convertUYVYToMultipleFormats(const uint8_t* uyvy_data, size_t uyvy_size,
                                     const std::vector<ConversionOutput>& outputs);
    
    /**
     * @brief Get the required output RGB buffer size
     * 
     * @return Required RGB buffer size in bytes
     */
    size_t getRGBBufferSize() const;
    
    /**
     * @brief Check if VPI converter is initialized
     * 
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const;
    
    /**
     * @brief Get last error message
     * 
     * @return Last error message string
     */
    std::string getLastError() const;
    
    /**
     * @brief 获取格式转换性能统计
     * 
     * @return 转换统计信息的字符串
     */
    std::string getConversionStats() const;
    
    /**
     * @brief 重置性能统计计数器
     */
    void resetStats();

private:
    // Private implementation details
    struct VPIConverterImpl;
    std::unique_ptr<VPIConverterImpl> pImpl;
    
    // Image dimensions
    int width_;
    int height_;
    bool initialized_;
    std::string last_error_;
    int v4l2_pixel_format_;
    std::string input_format_str_;
    
    // Thread safety
    mutable std::mutex conversion_mutex_;
    
    // Performance statistics
    mutable uint64_t total_conversions_;
    mutable double total_vic_time_ms_;
    mutable double total_gpu_time_ms_;  // PVA转换时间统计 (重用变量名)
    mutable double total_conversion_time_ms_;
    mutable std::chrono::steady_clock::time_point last_stats_time_;
    
    /**
     * @brief Setup VPI context and stream
     * @return true if successful
     */
    bool setupVPIContext();
    
    /**
     * @brief Create VPI images for input/output
     * @return true if successful
     */
    bool createVPIImages();
    
    /**
     * @brief Cleanup VPI resources
     */
    void cleanup();
};

#endif // CR_VIC_CONVERTER_HPP
