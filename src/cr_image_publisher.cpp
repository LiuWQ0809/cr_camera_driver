#include "cr_camera_driver/cr_image_publisher.hpp"
#include <cv_bridge/cv_bridge.h>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <opencv2/opencv.hpp>

namespace cr_camera_driver {

ImagePublisher::ImagePublisher(rclcpp::Node* node, int width, int height, int frame_skip_ratio)
    : node_(node), width_(width), height_(height), frame_skip_ratio_(frame_skip_ratio) {
}

void ImagePublisher::setVICConverter(std::unique_ptr<VICConverter> converter) {
    vic_converter_ = std::move(converter);
    if (vic_converter_ && vic_converter_->isInitialized()) {
        // 预分配RGB缓冲区
        rgb_buffer_.resize(vic_converter_->getRGBBufferSize());
    }
}

void ImagePublisher::initializePublisher(int cam_id, const std::string& topic_name, 
                                        bool publish_enabled) {
    publishers_[cam_id] = node_->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
    camera_stats_[cam_id] = CameraStats();
    camera_stats_[cam_id].publish_enabled = publish_enabled;
    
    RCLCPP_INFO(node_->get_logger(), 
                "Publisher initialized for Camera %d -> %s (publish: %s, skip_ratio: %d)", 
                cam_id, topic_name.c_str(), publish_enabled ? "YES" : "NO", frame_skip_ratio_);
}

void ImagePublisher::initializeH265Stream(int cam_id, const YamlCameraConfig::H265StreamConfig& config, int cam_width, int cam_height) {
    if (!config.enabled || config.topic.empty()) {
        return;
    }

    H265StreamState state;
    state.config = config;
    state.publisher = node_->create_publisher<sensor_msgs::msg::CompressedImage>(config.topic, 5);
    state.encoder = std::make_unique<JetsonH265Encoder>(cam_width, cam_height, config.bitrate, config.fps, config.group_len);
    if (!state.encoder->initialize()) {
        RCLCPP_WARN(node_->get_logger(), "Camera %d H265 encoder init failed: %s", cam_id, state.encoder->getLastError().c_str());
        return;
    }

    h265_streams_[cam_id] = std::move(state);
    RCLCPP_INFO(node_->get_logger(), "H265 stream enabled for camera %d -> %s", cam_id, config.topic.c_str());
}

void ImagePublisher::publishImage(int cam_id, const void* buffer_data, int buffer_index, 
                                 bool use_vic_converter, int64_t corrected_wall_timestamp_ns) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    auto& stats = camera_stats_[cam_id];
    
    // 帧计数统计（所有接收到的帧）
    stats.frame_count++;
    
    // 检查是否启用发布
    if (!stats.publish_enabled) {
        return;  // 不发布此相机的图像
    }
    
    // 检查时间是否已经校正，如果没有校正则不发布
    if (!stats.time_calibrated) {
        return;  // 时间未校正，跳过发布
    }
    
    // 基于frame_skip_ratio的帧率控制
    if (!shouldPublishFrame(cam_id)) {
        return;  // 帧率限制，跳过此帧
    }
    
    // Deal Rate统计：只统计真正进行图像处理的帧
    updateStats(cam_id);
    
    // 使用新的批处理发布方式
    if (use_vic_converter && vic_converter_ && vic_converter_->isInitialized()) {
        publishImageBatch(cam_id, buffer_data, buffer_index, start_time, corrected_wall_timestamp_ns);
    } else {
        publishUYVYImage(cam_id, buffer_data, buffer_index, start_time, corrected_wall_timestamp_ns);
        // 如果没有使用VIC，但也启用了H265，我们需要在这里处理
        publishH265Encoded(cam_id, buffer_data, corrected_wall_timestamp_ns);
    }
}

void ImagePublisher::publishUYVYImage(int cam_id, const void* buffer_data, int buffer_index,
                                     std::chrono::high_resolution_clock::time_point start_time,
                                     int64_t corrected_wall_timestamp_ns) {
    // 移除未使用的参数警告
    (void)buffer_index;
    (void)start_time;
    
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    
    // 使用校正后的时间戳或当前时间
    if (corrected_wall_timestamp_ns > 0) {
        // 将纳秒时间戳转换为ROS时间
        msg->header.stamp.sec = corrected_wall_timestamp_ns / 1000000000LL;
        msg->header.stamp.nanosec = corrected_wall_timestamp_ns % 1000000000LL;
    } else {
        msg->header.stamp = node_->now();
    }
    
    msg->header.frame_id = "camera_" + std::to_string(cam_id);
    msg->height = height_;
    msg->width = width_;
    msg->encoding = "yuv422";  // 使用UYVY格式，无需转换
    msg->is_bigendian = false;
    msg->step = width_ * 2;  // UYVY是2字节每像素
    msg->data.resize(width_ * height_ * 2);
    
    try {
        // 直接复制UYVY数据，无需颜色转换
        memcpy(msg->data.data(), buffer_data, width_ * height_ * 2);
        
        // 为所有UYVY主题发布图像，使用新的多主题发布器系统
        for (const auto& [key, publisher] : topic_publishers_) {
            // 检查这个发布器是否属于当前相机
            if (key.find(std::to_string(cam_id) + "_") == 0) {
                // 为每个主题创建独立的消息
                auto topic_msg = std::make_unique<sensor_msgs::msg::Image>(*msg);
                publisher->publish(std::move(topic_msg));
                
                // 更新topic统计
                updateTopicStats(key, corrected_wall_timestamp_ns);
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error publishing UYVY image for camera %d: %s", cam_id, e.what());
    }
}

void ImagePublisher::publishRGBImage(int cam_id, const void* buffer_data, int buffer_index,
                                    std::chrono::high_resolution_clock::time_point start_time,
                                    int64_t corrected_wall_timestamp_ns) {
    // 移除未使用的参数警告
    (void)buffer_index;
    (void)start_time;
    
    // 使用VIC转换器将UYVY转换为RGB
    bool conversion_success = vic_converter_->convertUYVYToRGB(
        static_cast<const uint8_t*>(buffer_data), 
        width_ * height_ * 2,  // UYVY输入大小
        rgb_buffer_.data(), 
        rgb_buffer_.size()     // RGB输出大小
    );
    
    if (!conversion_success) {
        RCLCPP_ERROR(node_->get_logger(), "VIC conversion failed for camera %d: %s", 
                     cam_id, vic_converter_->getLastError().c_str());
        // 回退到UYVY格式
        publishUYVYImage(cam_id, buffer_data, buffer_index, start_time, corrected_wall_timestamp_ns);
        return;
    }
    
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    
    // 使用校正后的时间戳或当前时间
    if (corrected_wall_timestamp_ns > 0) {
        // 将纳秒时间戳转换为ROS时间
        msg->header.stamp.sec = corrected_wall_timestamp_ns / 1000000000LL;
        msg->header.stamp.nanosec = corrected_wall_timestamp_ns % 1000000000LL;
    } else {
        msg->header.stamp = node_->now();
    }
    
    msg->header.frame_id = "camera_" + std::to_string(cam_id);
    msg->height = height_;
    msg->width = width_;
    msg->encoding = "rgb8";  // RGB格式
    msg->is_bigendian = false;
    msg->step = width_ * 3;  // RGB是3字节每像素
    msg->data.resize(width_ * height_ * 3);
    
    try {
        // 复制RGB数据
        memcpy(msg->data.data(), rgb_buffer_.data(), width_ * height_ * 3);
        
        // 为所有RGB主题发布图像，使用新的多主题发布器系统
        for (const auto& [key, publisher] : topic_publishers_) {
            // 检查这个发布器是否属于当前相机并且是RGB格式
            if (key.find(std::to_string(cam_id) + "_") == 0) {
                // 为每个主题创建独立的消息
                auto topic_msg = std::make_unique<sensor_msgs::msg::Image>(*msg);
                publisher->publish(std::move(topic_msg));
                
                // 更新topic统计
                updateTopicStats(key, corrected_wall_timestamp_ns);
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error publishing RGB image for camera %d: %s", cam_id, e.what());
    }
}

void ImagePublisher::publishImageBatch(int cam_id, const void* buffer_data, int buffer_index,
                                      std::chrono::high_resolution_clock::time_point start_time,
                                      int64_t corrected_wall_timestamp_ns) {
    // 移除未使用的参数警告
    (void)buffer_index;
    (void)start_time;
    
    // 第一步：VIC硬件转换 UYVY -> RGB (full resolution)
    bool conversion_success = vic_converter_->convertUYVYToRGB(
        static_cast<const uint8_t*>(buffer_data), 
        width_ * height_ * 2,  // UYVY输入大小
        rgb_buffer_.data(), 
        rgb_buffer_.size()     // RGB输出大小
    );
    
    if (!conversion_success) {
        RCLCPP_ERROR(node_->get_logger(), "VIC conversion failed for camera %d: %s", 
                     cam_id, vic_converter_->getLastError().c_str());
        // 回退到UYVY格式
        publishUYVYImage(cam_id, buffer_data, buffer_index, start_time);
        return;
    }
    
    // 第二步：为该相机的每个topic生成对应的图像数据并发布
    for (const auto& [key, publisher] : topic_publishers_) {
        // 检查这个发布器是否属于当前相机
        if (key.find(std::to_string(cam_id) + "_") == 0) {
            // 获取topic配置信息
            auto topic_stats_it = topic_stats_.find(key);
            if (topic_stats_it == topic_stats_.end()) {
                continue;
            }
            
            const auto& topic_stats = topic_stats_it->second;
            
            // 创建输出图像消息
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            
            // 使用校正后的时间戳或当前时间
            if (corrected_wall_timestamp_ns > 0) {
                // 将纳秒时间戳转换为ROS时间
                msg->header.stamp.sec = corrected_wall_timestamp_ns / 1000000000LL;
                msg->header.stamp.nanosec = corrected_wall_timestamp_ns % 1000000000LL;
            } else {
                msg->header.stamp = node_->now();
            }
            
            msg->header.frame_id = "camera_" + std::to_string(cam_id);
            
            try {
                // 根据topic配置生成对应的图像数据
                if (topic_stats.format == "RGB") {
                    // RGB格式处理
                    if (topic_stats.width == width_ && topic_stats.height == height_) {
                        // Full resolution RGB - 直接使用VIC转换结果
                        msg->height = height_;
                        msg->width = width_;
                        msg->encoding = "rgb8";
                        msg->is_bigendian = false;
                        msg->step = width_ * 3;
                        msg->data.resize(width_ * height_ * 3);
                        memcpy(msg->data.data(), rgb_buffer_.data(), width_ * height_ * 3);
                    } else {
                        // Resized RGB - 使用OpenCV resize
                        cv::Mat full_rgb(height_, width_, CV_8UC3, rgb_buffer_.data());
                        cv::Mat resized_rgb;
                        cv::resize(full_rgb, resized_rgb, cv::Size(topic_stats.width, topic_stats.height));
                        
                        msg->height = topic_stats.height;
                        msg->width = topic_stats.width;
                        msg->encoding = "rgb8";
                        msg->is_bigendian = false;
                        msg->step = topic_stats.width * 3;
                        msg->data.resize(topic_stats.width * topic_stats.height * 3);
                        memcpy(msg->data.data(), resized_rgb.data, topic_stats.width * topic_stats.height * 3);
                    }
                } else if (topic_stats.format == "BGR") {
                    // BGR格式处理
                    if (topic_stats.width == width_ && topic_stats.height == height_) {
                        // Full resolution BGR - 从RGB转换
                        cv::Mat rgb_mat(height_, width_, CV_8UC3, rgb_buffer_.data());
                        cv::Mat bgr_mat;
                        cv::cvtColor(rgb_mat, bgr_mat, cv::COLOR_RGB2BGR);
                        
                        msg->height = height_;
                        msg->width = width_;
                        msg->encoding = "bgr8";
                        msg->is_bigendian = false;
                        msg->step = width_ * 3;
                        msg->data.resize(width_ * height_ * 3);
                        memcpy(msg->data.data(), bgr_mat.data, width_ * height_ * 3);
                    } else {
                        // Resized BGR - RGB->resize->BGR
                        cv::Mat full_rgb(height_, width_, CV_8UC3, rgb_buffer_.data());
                        cv::Mat resized_rgb;
                        cv::resize(full_rgb, resized_rgb, cv::Size(topic_stats.width, topic_stats.height));
                        cv::Mat resized_bgr;
                        cv::cvtColor(resized_rgb, resized_bgr, cv::COLOR_RGB2BGR);
                        
                        msg->height = topic_stats.height;
                        msg->width = topic_stats.width;
                        msg->encoding = "bgr8";
                        msg->is_bigendian = false;
                        msg->step = topic_stats.width * 3;
                        msg->data.resize(topic_stats.width * topic_stats.height * 3);
                        memcpy(msg->data.data(), resized_bgr.data, topic_stats.width * topic_stats.height * 3);
                    }
                }
                
                // 发布图像
                publisher->publish(std::move(msg));
                
                // 更新topic统计
                updateTopicStats(key, corrected_wall_timestamp_ns);
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Error processing topic %s for camera %d: %s", 
                           key.c_str(), cam_id, e.what());
            }
        }
    }

    publishH265Encoded(cam_id, buffer_data, corrected_wall_timestamp_ns);
}

void ImagePublisher::publishH265Encoded(int cam_id, const void* buffer_data, int64_t corrected_wall_timestamp_ns) {
    auto it = h265_streams_.find(cam_id);
    if (it == h265_streams_.end()) {
        return;
    }
    auto& state = it->second;
    if (!state.encoder || !state.publisher) {
        return;
    }

    const uint8_t* rgb_ptr = nullptr;
    size_t rgb_size = 0;
    cv::Mat temp_rgb;

    // 如果rgb_buffer_不为空且VIC转换器已初始化，优先使用VIC转换后的数据
    if (vic_converter_ && vic_converter_->isInitialized() && !rgb_buffer_.empty()) {
        rgb_ptr = rgb_buffer_.data();
        rgb_size = rgb_buffer_.size();
    } else {
        // 否则使用OpenCV进行转换作为回退
        try {
            cv::Mat uyvy(height_, width_, CV_8UC2, const_cast<void*>(buffer_data));
            cv::cvtColor(uyvy, temp_rgb, cv::COLOR_YUV2RGB_UYVY);
            rgb_ptr = temp_rgb.data;
            rgb_size = temp_rgb.total() * temp_rgb.elemSize();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "H265 fallback conversion failed for camera %d: %s", cam_id, e.what());
            return;
        }
    }

    if (!rgb_ptr || rgb_size == 0) {
        return;
    }

    std::vector<uint8_t> encoded;
    if (!state.encoder->encodeRGBFrame(rgb_ptr, rgb_size, encoded)) {
        RCLCPP_WARN(node_->get_logger(), "Camera %d H265 encode failed: %s", cam_id, state.encoder->getLastError().c_str());
        return;
    }

    if (encoded.empty()) {
        return;
    }

    auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
    if (corrected_wall_timestamp_ns > 0) {
        msg->header.stamp.sec = corrected_wall_timestamp_ns / 1000000000LL;
        msg->header.stamp.nanosec = corrected_wall_timestamp_ns % 1000000000LL;
    } else {
        msg->header.stamp = node_->now();
    }

    msg->header.frame_id = "camera_" + std::to_string(cam_id);
    msg->format = "h265";
    msg->data = std::move(encoded);
    state.publisher->publish(std::move(msg));
}

void ImagePublisher::updateStats(int cam_id) {
    auto& stats = camera_stats_[cam_id];
    
    // 如果这是第一次进入处理流程，重置统计开始时间
    if (stats.real_image_count == 0) {
        stats.last_stats_time = std::chrono::steady_clock::now();
        stats.frames_since_stats = 0;
    }
    
    // 图像处理计数统计（Deal Rate相关）
    stats.real_image_count++;
    stats.frames_since_stats++;
    
    // 不在此处打印统计信息，统一由主节点打印表格
}

const CameraStats& ImagePublisher::getCameraStats(int cam_id) const {
    static CameraStats default_stats;
    auto it = camera_stats_.find(cam_id);
    return (it != camera_stats_.end()) ? it->second : default_stats;
}

std::pair<double, int> ImagePublisher::getPublishStats(int cam_id) const {
    auto it = camera_stats_.find(cam_id);
    if (it == camera_stats_.end()) {
        return {0.0, 0};
    }
    
    const auto& stats = it->second;
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        current_time - stats.last_stats_time);
    
    double fps = 0.0;
    if (elapsed.count() > 0) {
        fps = stats.frames_since_stats / static_cast<double>(elapsed.count());
    }
    
    // 调试信息：打印详细的统计计算
    // RCLCPP_DEBUG(node_->get_logger(), 
    //              "Cam %d: frames_since_stats=%d, elapsed=%ld sec, fps=%.1f", 
    //              cam_id, stats.frames_since_stats, elapsed.count(), fps);
    
    return {fps, stats.real_image_count};
}

std::string ImagePublisher::getVICStats() const {
    if (!vic_converter_) {
        return "";
    }
    return vic_converter_->getConversionStats();
}

void ImagePublisher::initializePublisher(int cam_id, const std::vector<cr_camera_driver::ResizeTopicConfig>& resize_topics) {
    // 为这个相机创建所有启用的topic的发布器
    for (const auto& topic_config : resize_topics) {
        if (topic_config.enabled) {
            auto publisher = node_->create_publisher<sensor_msgs::msg::Image>(topic_config.topic, 10);
            
            // 保存发布器信息（使用新的多topic结构）
            std::string key = std::to_string(cam_id) + "_" + topic_config.topic;
            topic_publishers_[key] = publisher;
            
            // 创建topic统计
            topic_stats_[key] = TopicStats(topic_config.topic, cam_id, topic_config.format, 
                                          topic_config.width, topic_config.height);
            
            RCLCPP_INFO(node_->get_logger(), "Initialized publisher for camera %d: %s (%dx%d, %s)", 
                       cam_id, topic_config.topic.c_str(), 
                       topic_config.width, topic_config.height, topic_config.format.c_str());
        }
    }
    
    // 初始化相机统计
    if (camera_stats_.find(cam_id) == camera_stats_.end()) {
        camera_stats_[cam_id] = CameraStats();
        camera_stats_[cam_id].publish_enabled = true;
    }
}

void ImagePublisher::setCameraPublishConfig(int cam_id, bool publish_enabled) {
    auto it = camera_stats_.find(cam_id);
    if (it != camera_stats_.end()) {
        it->second.publish_enabled = publish_enabled;
    }
}

void ImagePublisher::setCameraTimeCalibrated(int cam_id, bool calibrated) {
    auto it = camera_stats_.find(cam_id);
    if (it != camera_stats_.end()) {
        it->second.time_calibrated = calibrated;
    }
}

bool ImagePublisher::shouldPublishFrame(int cam_id) const {
    auto it = camera_stats_.find(cam_id);
    if (it == camera_stats_.end()) {
        return false;
    }
    
    const auto& stats = it->second;
    
    // 使用frame_skip_ratio进行帧抽取控制
    // skip_count从0开始，每frame_skip_ratio帧发布一次
    return (stats.frame_count % frame_skip_ratio_) == 0;
}

// 更新topic统计
void ImagePublisher::updateTopicStats(const std::string& topic_key, int64_t image_timestamp_ns) {
    auto it = topic_stats_.find(topic_key);
    if (it != topic_stats_.end()) {
        it->second.published_count++;
        it->second.frames_since_stats++;
        
        // 计算时延：publish时间 - 图像数据时间
        if (image_timestamp_ns > 0) {
            auto publish_time = std::chrono::system_clock::now();
            auto publish_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                publish_time.time_since_epoch()).count();
            
            double latency_ms = (publish_time_ns - image_timestamp_ns) / 1000000.0;
            it->second.latency_samples.push_back(latency_ms);
        }
    }
}

// 重置topic时延统计
void ImagePublisher::resetTopicLatencyStats() {
    for (auto& [key, stats] : topic_stats_) {
        stats.latency_samples.clear();
        stats.frames_since_stats = 0;
        stats.last_stats_time = std::chrono::steady_clock::now();
    }
}

// 重置帧率统计（用于定期统计报告）
void ImagePublisher::resetFrameRateStats() {
    for (auto& [cam_id, stats] : camera_stats_) {
        // 只重置帧率统计的时间基准和计数，保持累计发布数量
        stats.frames_since_stats = 0;
        stats.last_stats_time = std::chrono::steady_clock::now();
    }
}

// 获取topic统计数据
void ImagePublisher::getTopicStats(std::vector<TopicStats>& topic_stats) const {
    topic_stats.clear();
    for (const auto& [key, stats] : topic_stats_) {
        topic_stats.push_back(stats);
    }
}

// 生成topic统计表格
std::string ImagePublisher::getTopicStatsTable() const {
    std::ostringstream table;
    
    table << "\n╔═══════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╗\n";
    table << "║                                              Topic Publishing Statistics                                                    ║\n";
    table << "╠════════╦══════════════════════════════════════════════╦═══════╦═══════════╦═══════╦═══════╦══════════════════════════════════╣\n";
    table << "║Cam ID  ║                 Topic Name                   ║Format ║   Size    ║  Rate ║ Total ║        Latency (ms)              ║\n";
    table << "║        ║                                              ║       ║           ║       ║       ║    Min   Max   Avg              ║\n";
    table << "╠════════╬══════════════════════════════════════════════╬═══════╬═══════════╬═══════╬═══════╬══════════════════════════════════╣\n";
    
    // 按相机ID排序显示
    std::vector<std::pair<std::string, TopicStats>> sorted_stats;
    for (const auto& [key, stats] : topic_stats_) {
        sorted_stats.emplace_back(key, stats);
    }
    
    // 按相机ID排序
    std::sort(sorted_stats.begin(), sorted_stats.end(), 
              [](const auto& a, const auto& b) {
                  return a.second.cam_id < b.second.cam_id;
              });
    
    for (const auto& [key, stats] : sorted_stats) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
            now - stats.last_stats_time);
        
        double fps = 0.0;
        if (elapsed.count() > 0 && stats.frames_since_stats > 0) {
            fps = stats.frames_since_stats / elapsed.count();
        }
        
        // 计算时延统计
        double min_latency = 0.0, max_latency = 0.0, avg_latency = 0.0;
        if (!stats.latency_samples.empty()) {
            min_latency = *std::min_element(stats.latency_samples.begin(), stats.latency_samples.end());
            max_latency = *std::max_element(stats.latency_samples.begin(), stats.latency_samples.end());
            double sum = 0.0;
            for (double latency : stats.latency_samples) {
                sum += latency;
            }
            avg_latency = sum / stats.latency_samples.size();
        }
        
        // 格式化topic名称，显示更多字符
        std::string display_topic = stats.topic_name;
        if (display_topic.length() > 46) {
            // 显示前20个字符 + "..." + 最后23个字符
            display_topic = display_topic.substr(0, 20) + "..." + 
                           display_topic.substr(display_topic.length() - 23);
        }
        
        // 格式化尺寸信息
        std::string size_str = std::to_string(stats.width) + "x" + std::to_string(stats.height);
        
        table << "║   " << std::setw(2) << stats.cam_id << "   ║ " 
              << std::setw(46) << std::left << display_topic << " ║ "
              << std::setw(5) << std::left << stats.format << " ║ "
              << std::setw(9) << std::left << size_str << " ║ "
              << std::setw(5) << std::right << std::fixed << std::setprecision(1) << fps << " ║ "
              << std::setw(5) << std::right << stats.published_count << " ║ "
              << std::setw(5) << std::right << std::fixed << std::setprecision(1) << min_latency << " "
              << std::setw(5) << std::right << std::fixed << std::setprecision(1) << max_latency << " "
              << std::setw(5) << std::right << std::fixed << std::setprecision(1) << avg_latency << "              ║\n";
    }
    
    table << "╚════════╩══════════════════════════════════════════════╩═══════╩═══════════╩═══════╩═══════╩══════════════════════════════════╝";
    
    return table.str();
}

} // namespace cr_camera_driver
