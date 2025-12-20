#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <chrono>
#include <memory>
#include <vector>
#include "cr_camera_driver/cr_vic_converter.hpp"
#include "cr_camera_driver/cr_yaml_config.hpp"  // 包含ResizeTopicConfig定义
#include "cr_camera_driver/cr_h265_encoder.hpp"

namespace cr_camera_driver {

struct CameraStats {
    int frame_count;            // V4L2接收的总帧数
    int real_image_count;       // 进入图像处理流程的总帧数（Deal Total）
    int skip_count;             // 用于帧抽取计数
    std::chrono::steady_clock::time_point last_stats_time;
    int frames_since_stats;     // 自上次统计以来处理的帧数（用于Deal Rate计算）
    bool publish_enabled;       // 是否启用发布
    bool time_calibrated;       // 时间是否已校正
    
    CameraStats() : frame_count(0), real_image_count(0), skip_count(0),
                   last_stats_time(std::chrono::steady_clock::now()), 
                   frames_since_stats(0), publish_enabled(true), time_calibrated(false) {
    }
};

// 新的topic级别统计结构
struct TopicStats {
    int published_count;           // 发布的消息数量
    int frames_since_stats;        // 自上次统计以来的帧数
    std::chrono::steady_clock::time_point last_stats_time;
    std::string topic_name;        // topic名称
    int cam_id;                    // 所属相机ID
    std::string format;            // 图像格式 (RGB/BGR)
    int width, height;             // 图像尺寸
    
    // 时延统计字段
    std::vector<double> latency_samples;  // 时延样本(毫秒)：publish时间 - 图像数据时间
    double min_latency_ms = 0.0;          // 最小时延(毫秒)
    double max_latency_ms = 0.0;          // 最大时延(毫秒)
    double avg_latency_ms = 0.0;          // 平均时延(毫秒)
    
    TopicStats() : published_count(0), frames_since_stats(0),
                   last_stats_time(std::chrono::steady_clock::now()),
                   cam_id(-1), width(0), height(0) {
        latency_samples.reserve(1000);  // 预分配空间
    }
                   
    TopicStats(const std::string& name, int camera_id, const std::string& fmt, int w, int h)
        : published_count(0), frames_since_stats(0),
          last_stats_time(std::chrono::steady_clock::now()),
          topic_name(name), cam_id(camera_id), format(fmt), width(w), height(h) {
        latency_samples.reserve(1000);  // 预分配空间
    }
};

struct H265StreamState {
    YamlCameraConfig::H265StreamConfig config;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher;
    std::unique_ptr<JetsonH265Encoder> encoder;
};

class ImagePublisher {
public:
    ImagePublisher(rclcpp::Node* node, int width, int height, int frame_skip_ratio = 2);
    ~ImagePublisher() = default;

        // 初始化发布器 - 支持发布配置
    void initializePublisher(int cam_id, const std::string& topic_name, 
                            bool enable_publishing);
    
    // 新接口：支持resize_topics配置
    void initializePublisher(int cam_id, const std::vector<cr_camera_driver::ResizeTopicConfig>& resize_topics);
    void initializeH265Stream(int cam_id, const YamlCameraConfig::H265StreamConfig& config, int cam_width, int cam_height);
    
    // 发布图像 - 自动应用帧率控制
    void publishImage(int cam_id, const void* buffer_data, int buffer_index, 
                     bool use_vic_converter = false, int64_t corrected_wall_timestamp_ns = 0);
    
    // 设置相机时间校正状态
    void setCameraTimeCalibrated(int cam_id, bool calibrated);
                     
    // 设置相机发布配置
    void setCameraPublishConfig(int cam_id, bool publish_enabled);
    
    // 检查是否应该发布帧（基于frame_skip_ratio控制）
    bool shouldPublishFrame(int cam_id) const;
    
    // 设置VIC转换器
    void setVICConverter(std::unique_ptr<VICConverter> converter);
    
    // 获取统计信息
    const CameraStats& getCameraStats(int cam_id) const;
    
    // 获取图像处理统计信息：返回(deal_fps, total_dealt)
    std::pair<double, int> getPublishStats(int cam_id) const;
    
    // 获取VIC转换器性能统计
    std::string getVICStats() const;
    
    // 新的topic统计方法
    void getTopicStats(std::vector<TopicStats>& topic_stats) const;
    std::string getTopicStatsTable() const;
    
    // 重置topic时延统计
    void resetTopicLatencyStats();
    
    // 重置帧率统计（用于定期统计报告）
    void resetFrameRateStats();

private:
    // 发布UYVY格式图像
    void publishUYVYImage(int cam_id, const void* buffer_data, int buffer_index, 
                         std::chrono::high_resolution_clock::time_point start_time,
                         int64_t corrected_wall_timestamp_ns = 0);
    
    // 发布RGB格式图像
    void publishRGBImage(int cam_id, const void* buffer_data, int buffer_index, 
                        std::chrono::high_resolution_clock::time_point start_time,
                        int64_t corrected_wall_timestamp_ns = 0);
    
    // 批处理发布图像 - 一次处理该相机的所有topic
    void publishImageBatch(int cam_id, const void* buffer_data, int buffer_index,
                          std::chrono::high_resolution_clock::time_point start_time,
                          int64_t corrected_wall_timestamp_ns = 0);
    void publishH265Encoded(int cam_id, int64_t corrected_wall_timestamp_ns = 0);
    
    // 更新统计信息
    void updateStats(int cam_id);
    
    // 更新topic统计信息
    void updateTopicStats(const std::string& topic_key, int64_t image_timestamp_ns = 0);

private:
    rclcpp::Node* node_;
    int width_;
    int height_;
    int frame_skip_ratio_;  // 帧抽取比率
    
    // 发布器映射 - 使用字符串key支持多个topic
    std::map<int, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> topic_publishers_;  // 新的多topic支持
    
    // 统计信息映射
    std::map<int, CameraStats> camera_stats_;
    std::map<std::string, TopicStats> topic_stats_;  // 新的topic级别统计
    std::map<int, H265StreamState> h265_streams_;
    
    // VIC硬件转换器
    std::unique_ptr<VICConverter> vic_converter_;
    std::vector<uint8_t> rgb_buffer_;  // RGB输出缓冲区
};

} // namespace cr_camera_driver
