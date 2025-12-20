#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <exception>
#include <sensor_msgs/msg/image.hpp>
#include "cr_camera_driver/cr_image_publisher.hpp"
#include "cr_camera_driver/cr_v4l2_camera.hpp"
#include "cr_camera_driver/cr_vic_converter.hpp"
#include "cr_camera_driver/cr_yaml_config.hpp"

#include <linux/videodev2.h>
#include <vector>
#include <map>
#include <thread>
#include <atomic>
#include <sys/select.h>
#include <chrono>
#include <cstring>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <sstream>
#include <iomanip>

// 帧处理任务结构
struct FrameProcessTask {
  int cam_id;
  const void* buffer_data;
  int buffer_index;
  std::chrono::high_resolution_clock::time_point capture_time;
  
  FrameProcessTask(int id, const void* data, int idx) 
    : cam_id(id), buffer_data(data), buffer_index(idx), 
      capture_time(std::chrono::high_resolution_clock::now()) {}
};

// 线程池类
class ThreadPool {
private:
  std::vector<std::thread> workers_;
  std::queue<std::function<void()>> tasks_;
  std::mutex queue_mutex_;
  std::condition_variable condition_;
  std::atomic<bool> stop_;

public:
  ThreadPool(size_t threads) : stop_(false) {
    for(size_t i = 0; i < threads; ++i) {
      workers_.emplace_back([this] {
        for(;;) {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock(this->queue_mutex_);
            this->condition_.wait(lock, [this]{ return this->stop_ || !this->tasks_.empty(); });
            if(this->stop_ && this->tasks_.empty()) return;
            task = std::move(this->tasks_.front());
            this->tasks_.pop();
          }
          task();
        }
      });
    }
  }

  template<class F, class... Args>
  auto enqueue(F&& f, Args&&... args) -> std::future<typename std::result_of<F(Args...)>::type> {
    using return_type = typename std::result_of<F(Args...)>::type;
    auto task = std::make_shared<std::packaged_task<return_type()>>(
      std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );
    std::future<return_type> res = task->get_future();
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      if(stop_) throw std::runtime_error("enqueue on stopped ThreadPool");
      tasks_.emplace([task](){ (*task)(); });
    }
    condition_.notify_one();
    return res;
  }

  ~ThreadPool() {
    stop_ = true;
    condition_.notify_all();
    for(std::thread &worker: workers_) worker.join();
  }
};

class CRCameraNode : public rclcpp::Node {
public:
  CRCameraNode() : Node("cr_camera_node") {
    
    // 声明配置文件参数
    std::string default_config_file;
    try {
      default_config_file = ament_index_cpp::get_package_share_directory("cr_camera_driver") + "/config/cameras.yaml";
    } catch (const std::exception& e) {
      default_config_file = "/home/nvidia/yanbo/cr_camera_driver/config/cameras.yaml";
      RCLCPP_WARN(this->get_logger(),
                  "Failed to resolve cr_camera_driver share directory: %s; using fallback config: %s",
                  e.what(), default_config_file.c_str());
    }
    this->declare_parameter("config_file", default_config_file);
    
    // 加载配置
    std::string config_file = this->get_parameter("config_file").as_string();
    if (!loadConfiguration(config_file)) {
      throw std::runtime_error("Failed to load camera configuration");
    }
    
    RCLCPP_INFO(this->get_logger(), "CR Camera Driver started");
    RCLCPP_INFO(this->get_logger(), "Loaded configuration with %zu cameras", camera_configs_.size());
    
    // 获取全局设置
    auto global_settings = config_manager_.getGlobalSettings();
    RCLCPP_INFO(this->get_logger(), "Global settings: VIC=%s, Buffers=%d, SkipRatio=%d", 
                global_settings.use_vic_converter ? "ENABLED" : "DISABLED",
                global_settings.num_buffers,
                global_settings.frame_skip_ratio);
    
    // 初始化模块
    initializeModules(global_settings.use_vic_converter);
    
    // 初始化摄像头
    for (const auto& config : camera_configs_) {
      if (config.enabled) {
        initializeCamera(config);
      }
    }
    
    // 根据有启用topic发布的相机数量创建线程池
    size_t thread_pool_size = countCamerasWithEnabledTopics();
    if (thread_pool_size > 0) {
      thread_pool_ = std::make_unique<ThreadPool>(thread_pool_size);
      RCLCPP_INFO(this->get_logger(), "Thread pool created with %zu worker threads (one per camera with enabled topics)", thread_pool_size);
    } else {
      RCLCPP_WARN(this->get_logger(), "No cameras have enabled topics, thread pool not created");
    }
    
    // 启动异步捕获线程
    startCaptureLoop();
    
    // 创建定时器，按配置的间隔打印统计表格
    stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(global_settings.stats_interval),
      std::bind(&CRCameraNode::printStatsTable, this));
  }
  
  ~CRCameraNode() {
    stopCaptureLoop();
    cleanup();
  }

private:
  // 加载配置文件
  bool loadConfiguration(const std::string& config_file) {
    if (!config_manager_.loadConfig(config_file)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load config: %s", config_manager_.getLastError().c_str());
      return false;
    }
    
    camera_configs_ = config_manager_.getCameraConfigs();
    
    RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully from: %s", config_file.c_str());
    for (const auto& config : camera_configs_) {
      RCLCPP_INFO(this->get_logger(), "Camera %d: %s (%dx%d) %s", 
                  config.id, config.device.c_str(),
                  config.width, config.height, config.enabled ? "ENABLED" : "DISABLED");
    }
    
    return true;
  }
  
  // 统计有启用topic的相机数量
  size_t countCamerasWithEnabledTopics() const {
    size_t count = 0;
    for (const auto& config : camera_configs_) {
      if (!config.enabled) {
        continue; // 相机本身未启用，跳过
      }
      
      // 检查是否有任何topic启用了
      bool has_enabled_topic = false;
      for (const auto& topic_config : config.resize_topics) {
        if (topic_config.enabled) {
          has_enabled_topic = true;
          break;
        }
      }
      
      if (has_enabled_topic) {
        count++;
      }
    }
    return count;
  }
  
  // 初始化模块
  void initializeModules(bool use_vic) {
    // 获取全局设置
    auto global_settings = config_manager_.getGlobalSettings();
    
    // 初始化V4L2相机模块
    v4l2_camera_ = std::make_unique<cr_camera_driver::V4L2Camera>(this);
    
    // 初始化图像发布模块（从第一个相机配置获取尺寸）
    if (!camera_configs_.empty()) {
      const auto& first_config = camera_configs_[0];
      image_publisher_ = std::make_unique<cr_camera_driver::ImagePublisher>(
        this, first_config.width, first_config.height, global_settings.frame_skip_ratio);
    }
    
    // 设置帧回调函数 - 使用线程池异步处理
    v4l2_camera_->setFrameCallback(
      [this](int cam_id, const void* buffer_data, size_t buffer_size, int buffer_index, 
              int64_t corrected_wall_timestamp_ns) {
        (void)buffer_size; // 移除未使用参数警告
        if (thread_pool_) {
          // 将帧处理任务提交到线程池
          thread_pool_->enqueue([this, cam_id, buffer_data, buffer_index, corrected_wall_timestamp_ns]() {
            processFrameAsync(cam_id, buffer_data, buffer_index, corrected_wall_timestamp_ns);
          });
        } else {
          // 回退到同步处理
          image_publisher_->publishImage(cam_id, buffer_data, buffer_index, use_vic_converter_, corrected_wall_timestamp_ns);
        }
      });
    
    // 设置时间校准回调函数
    v4l2_camera_->setTimeCalibrationCallback(
      [this](int cam_id) {
        image_publisher_->setCameraTimeCalibrated(cam_id, true);
        RCLCPP_INFO(this->get_logger(), "Camera %d time calibration completed", cam_id);
      });
    
    // 如果启用VIC转换器，则初始化
    if (use_vic && !camera_configs_.empty()) {
      const auto& first_config = camera_configs_[0];
      vic_converter_ = std::make_unique<VICConverter>();
      if (vic_converter_->initialize(first_config.width, first_config.height, first_config.pixel_format)) {
        image_publisher_->setVICConverter(std::move(vic_converter_));
        use_vic_converter_ = true;
        RCLCPP_INFO(this->get_logger(), "VIC converter initialized successfully");
      } else {
        RCLCPP_WARN(this->get_logger(), "VIC converter initialization failed, using UYVY format");
        vic_converter_.reset();
        use_vic_converter_ = false;
      }
    } else {
      use_vic_converter_ = false;
    }
  }
  
  // 初始化单个相机
  void initializeCamera(const cr_camera_driver::YamlCameraConfig& config) {
    cr_camera_driver::CameraConfig cam_config;
    cam_config.device_path = config.device;
    cam_config.width = config.width;
    cam_config.height = config.height;
    cam_config.pixel_format = config.pixel_format;
    cam_config.field = config.field_type;
    cam_config.colorspace = config.colorspace;
    cam_config.num_buffers = config.num_buffers;
    
    if (v4l2_camera_->initializeCamera(config.id, cam_config)) {
      // 初始化图像发布器，传递resize topics配置
      image_publisher_->initializePublisher(config.id, config.resize_topics);
      image_publisher_->initializeH265Stream(config.id, config.h265_stream, config.width, config.height);
      
      initialized_cameras_.push_back(config.id);
      RCLCPP_INFO(this->get_logger(), "Camera %d initialized successfully", config.id);
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to initialize camera %d: %s", 
                  config.id, config.device.c_str());
    }
  }
  
  // 启动捕获循环
  void startCaptureLoop() {
    if (initialized_cameras_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No cameras initialized, cannot start capture loop");
      return;
    }
    
    // 启动所有相机的捕获
    for (int cam_id : initialized_cameras_) {
      if (!v4l2_camera_->startCapture(cam_id)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start capture for camera %d", cam_id);
      }
    }
    
    capture_active_ = true;
    capture_thread_ = std::thread(&CRCameraNode::captureLoop, this);
    
    RCLCPP_INFO(this->get_logger(), "Camera capture started with %zu cameras", initialized_cameras_.size());
  }
  
  // 停止捕获循环
  void stopCaptureLoop() {
    capture_active_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    
    // 停止所有相机的捕获
    for (int cam_id : initialized_cameras_) {
      v4l2_camera_->stopCapture(cam_id);
    }
  }
  
  // 主捕获循环 - 使用事件驱动模式
  void captureLoop() {
    RCLCPP_INFO(this->get_logger(), "Starting event-driven capture loop for %zu cameras", initialized_cameras_.size());
    
    while (capture_active_ && rclcpp::ok()) {
      // 使用select等待任何摄像头有数据可读
      fd_set readfds;
      FD_ZERO(&readfds);
      int max_fd = 0;
      
      for (int cam_id : initialized_cameras_) {
        int fd = v4l2_camera_->getFileDescriptor(cam_id);
        if (fd >= 0) {
          FD_SET(fd, &readfds);
          max_fd = std::max(max_fd, fd);
        }
      }
      
      if (max_fd == 0) {
        RCLCPP_WARN(this->get_logger(), "No valid cameras available for capture");
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30fps fallback
        continue;
      }
      
      // 设置超时时间，适配30fps的帧间隔（33ms）
      struct timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 50000; // 50ms timeout，大于33ms帧间隔
      
      int ready = select(max_fd + 1, &readfds, nullptr, nullptr, &timeout);
      
      if (ready < 0) {
        RCLCPP_ERROR(this->get_logger(), "select() error: %s", strerror(errno));
        break;
      } else if (ready == 0) {
        // 超时，继续下一次循环
        continue;
      }
      
      // 检查哪些摄像头有数据可读并处理
      for (int cam_id : initialized_cameras_) {
        int fd = v4l2_camera_->getFileDescriptor(cam_id);
        if (fd >= 0 && FD_ISSET(fd, &readfds)) {
          if (!v4l2_camera_->processFrame(cam_id)) {
            RCLCPP_DEBUG(this->get_logger(), "Failed to process frame for camera %d", cam_id);
          }
        }
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "Capture loop ended");
  }
  
  // 异步帧处理方法 - 在工作线程中执行
  void processFrameAsync(int cam_id, const void* buffer_data, int buffer_index, 
                        int64_t corrected_wall_timestamp_ns) {
    // 执行图像发布处理
    image_publisher_->publishImage(cam_id, buffer_data, buffer_index, use_vic_converter_, corrected_wall_timestamp_ns);
  }
  
  // 清理资源
  void cleanup() {
    if (v4l2_camera_) {
      v4l2_camera_->cleanup();
    }
  }
  
  // 打印帧率统计表格
  void printStatsTable() {
    if (initialized_cameras_.empty()) {
      return;
    }
    
    // 构建相机级别统计表格
    std::ostringstream table;
    table << "\n╔════════════════════════════════════════════════════════════════╗\n";
    table << "║                         Camera Frame Rate Stats                ║\n";
    table << "╠════════╦═════════════╦═════════════╦═════════════╦═════════════╣\n";
    table << "║ Cam ID ║ V4L2 Capture║   Deal Rate ║   V4L2 Total║   Deal Total║\n";
    table << "╠════════╬═════════════╬═════════════╬═════════════╬═════════════╣\n";
    
    for (int cam_id : initialized_cameras_) {
      // 获取V4L2统计
      double v4l2_fps = 0.0;
      int v4l2_total = 0;
      if (v4l2_camera_) {
        auto v4l2_stats = v4l2_camera_->getFrameStats(cam_id);
        v4l2_fps = v4l2_stats.first;
        v4l2_total = v4l2_stats.second;
      }
      
      // 获取图像处理统计（Deal Rate）
      double deal_fps = 0.0;
      int deal_total = 0;
      if (image_publisher_) {
        auto deal_stats = image_publisher_->getPublishStats(cam_id);
        deal_fps = deal_stats.first;
        deal_total = deal_stats.second;
      }
      
      table << "║   " << std::setw(2) << cam_id << "   ║   " 
            << std::setw(7) << std::fixed << std::setprecision(1) << v4l2_fps << "   ║   "
            << std::setw(7) << std::fixed << std::setprecision(1) << deal_fps << "   ║   "
            << std::setw(7) << v4l2_total << "   ║   "
            << std::setw(7) << deal_total << "   ║\n";
    }
    
    table << "╚════════╩═════════════╩═════════════╩═════════════╩═════════════╝";
    
    // 一次性输出整个表格
    RCLCPP_INFO(this->get_logger(), "%s", table.str().c_str());
    
    // 如果启用了VIC转换器，打印格式转换统计
    if (use_vic_converter_ && image_publisher_) {
      auto vic_stats = image_publisher_->getVICStats();
      if (!vic_stats.empty()) {
        RCLCPP_INFO(this->get_logger(), "\n%s", vic_stats.c_str());
      }
    }
    
    // 显示详细的topic统计表格
    if (image_publisher_) {
      auto topic_stats_table = image_publisher_->getTopicStatsTable();
      if (!topic_stats_table.empty()) {
        RCLCPP_INFO(this->get_logger(), "%s", topic_stats_table.c_str());
        // 重置时延统计以便下一个周期的统计
        image_publisher_->resetTopicLatencyStats();
      }
    }
  }
  
  // 成员变量
  cr_camera_driver::YamlConfigManager config_manager_;
  std::vector<cr_camera_driver::YamlCameraConfig> camera_configs_;
  std::vector<int> initialized_cameras_;
  
  // 模块实例
  std::unique_ptr<cr_camera_driver::V4L2Camera> v4l2_camera_;
  std::unique_ptr<cr_camera_driver::ImagePublisher> image_publisher_;
  std::unique_ptr<VICConverter> vic_converter_;
  
  // 捕获线程
  std::thread capture_thread_;
  std::atomic<bool> capture_active_;
  
  // 多线程处理
  std::unique_ptr<ThreadPool> thread_pool_;
  
  // 统计定时器
  rclcpp::TimerBase::SharedPtr stats_timer_;
  
  // 状态标志
  bool use_vic_converter_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<CRCameraNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("cr_camera_node"), "Exception: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
