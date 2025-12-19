#ifndef CR_CAMERA_DRIVER_CR_V4L2_CAMERA_HPP
#define CR_CAMERA_DRIVER_CR_V4L2_CAMERA_HPP

#include <string>
#include <vector>
#include <functional>
#include <chrono>
#include <mutex>
#include <memory>
#include <linux/videodev2.h>
#include <rclcpp/rclcpp.hpp>

namespace cr_camera_driver {

struct V4L2Buffer {
    void* start;
    size_t length;
    struct v4l2_buffer v4l2_buf;
};

struct CameraConfig {
    std::string device_path;
    int width;
    int height;
    int pixel_format;  // V4L2_PIX_FMT_UYVY等
    int field;         // V4L2_FIELD_NONE等
    int colorspace;    // V4L2_COLORSPACE_SRGB等
    int num_buffers;   // 缓冲区数量，通常为2
};

class V4L2Camera {
public:
    using FrameCallback = std::function<void(int cam_id, const void* buffer_data, 
                                           size_t buffer_size, int buffer_index, 
                                           int64_t corrected_wall_timestamp_ns)>;
    
    using TimeCalibrationCallback = std::function<void(int cam_id)>;

    explicit V4L2Camera(rclcpp::Node* node);
    ~V4L2Camera();

    // 禁用拷贝构造和拷贝赋值
    V4L2Camera(const V4L2Camera&) = delete;
    V4L2Camera& operator=(const V4L2Camera&) = delete;

    // 初始化相机
    bool initializeCamera(int cam_id, const CameraConfig& config);
    
    // 设置帧回调函数
    void setFrameCallback(FrameCallback callback);
    
    // 设置时间校准完成回调函数
    void setTimeCalibrationCallback(TimeCalibrationCallback callback);
    
    // 开始捕获
    bool startCapture(int cam_id);
    
    // 停止捕获
    bool stopCapture(int cam_id);
    
    // 处理单个相机的帧数据
    bool processFrame(int cam_id);
    
    // 获取文件描述符（用于select/poll）
    int getFileDescriptor(int cam_id) const;
    
    // 检查相机是否已初始化
    bool isCameraInitialized(int cam_id) const;
    
    // 获取相机配置
    const CameraConfig& getCameraConfig(int cam_id) const;
    
    // 清理资源
    void cleanup();
    
    // 获取帧率统计信息：返回(fps, total_frames)
    std::pair<double, int> getFrameStats(int cam_id) const;
    
    // 获取计算出的时间差值（线程安全）
    double getCalculatedTimeDiff(int cam_id) const;
    
    // 检查时间差值是否已计算
    bool isTimeDiffCalculated(int cam_id) const;

private:
    struct TimeDiffStats {
        std::vector<double> time_diffs;  // 存储前10帧的时间差值（毫秒）：V4L2时间 - 系统世界时间
        std::chrono::steady_clock::time_point last_stats_time;
        int total_frame_count = 0;
        int skip_frames = 5;  // 启动后跳过前5帧
        double calculated_diff_ms = 0.0;  // 计算出的时间差值（最大值+0.5ms）：用于将V4L2时间转换为系统世界时间
        bool diff_calculated = false;     // 是否已经计算出差值
        std::unique_ptr<std::mutex> diff_mutex;    // 保护差值计算的互斥锁
        
        TimeDiffStats() : last_stats_time(std::chrono::steady_clock::now()),
                         diff_mutex(std::make_unique<std::mutex>()) {
            time_diffs.reserve(10);  // 预分配10个元素的空间
        }
        
        // 移动构造函数
        TimeDiffStats(TimeDiffStats&& other) noexcept
            : time_diffs(std::move(other.time_diffs)),
              last_stats_time(std::move(other.last_stats_time)),
              total_frame_count(other.total_frame_count),
              skip_frames(other.skip_frames),
              calculated_diff_ms(other.calculated_diff_ms),
              diff_calculated(other.diff_calculated),
              diff_mutex(std::move(other.diff_mutex)) {}
        
        // 移动赋值运算符
        TimeDiffStats& operator=(TimeDiffStats&& other) noexcept {
            if (this != &other) {
                time_diffs = std::move(other.time_diffs);
                last_stats_time = std::move(other.last_stats_time);
                total_frame_count = other.total_frame_count;
                skip_frames = other.skip_frames;
                calculated_diff_ms = other.calculated_diff_ms;
                diff_calculated = other.diff_calculated;
                diff_mutex = std::move(other.diff_mutex);
            }
            return *this;
        }
        
        // 禁用拷贝构造和拷贝赋值
        TimeDiffStats(const TimeDiffStats&) = delete;
        TimeDiffStats& operator=(const TimeDiffStats&) = delete;
    };

    struct V4L2Stats {
        int capture_count = 0;
        int frames_since_stats = 0;
        std::chrono::steady_clock::time_point last_stats_time;
        TimeDiffStats time_diff_stats;  // 添加时间差统计
        
        V4L2Stats() : last_stats_time(std::chrono::steady_clock::now()) {}
    };

    struct CameraData {
        int fd = -1;
        bool initialized = false;
        bool capturing = false;
        CameraConfig config;
        std::vector<V4L2Buffer> buffers;
        int current_buffer_index = 0;
        V4L2Stats stats;  // V4L2统计信息
    };

    rclcpp::Node* node_;
    std::vector<CameraData> cameras_;
    FrameCallback frame_callback_;
    TimeCalibrationCallback time_calibration_callback_;

    // 内部辅助函数
    bool openDevice(int cam_id, const std::string& device_path);
    bool setFormat(int cam_id, const CameraConfig& config);
    bool requestBuffers(int cam_id, int num_buffers);
    bool mapBuffers(int cam_id);
    bool queueBuffer(int cam_id, int buffer_index);
    bool dequeueBuffer(int cam_id, struct v4l2_buffer& buf);
    void unmapBuffers(int cam_id);
    void closeDevice(int cam_id);
    
    // 检查V4L2操作结果
    bool checkV4L2Result(int result, const std::string& operation, int cam_id) const;
    
    // 更新V4L2帧率统计
    void updateV4L2Stats(int cam_id);
    
    // 更新时间差统计
    void updateTimeDiffStats(int cam_id, double time_diff_ms);
    
    // 获取像素格式字符串表示
    std::string getPixelFormatString(int format) const;
};

} // namespace cr_camera_driver

#endif // CR_CAMERA_DRIVER_CR_V4L2_CAMERA_HPP
