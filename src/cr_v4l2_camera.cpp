#include "cr_camera_driver/cr_v4l2_camera.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cstring>
#include <errno.h>
#include <algorithm>
#include <string>

namespace cr_camera_driver {

V4L2Camera::V4L2Camera(rclcpp::Node* node) : node_(node), frame_skip_ratio_(1) {
    // 预分配5个相机的空间
    cameras_.resize(5);
}

V4L2Camera::~V4L2Camera() {
    cleanup();
}

bool V4L2Camera::initializeCamera(int cam_id, const CameraConfig& config) {
    if (cam_id < 0 || cam_id >= static_cast<int>(cameras_.size())) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid camera ID: %d", cam_id);
        return false;
    }

    auto& camera = cameras_[cam_id];
    
    // 如果已经初始化，先清理
    if (camera.initialized) {
        stopCapture(cam_id);
        closeDevice(cam_id);
    }

    camera.config = config;

    // 打开设备
    if (!openDevice(cam_id, config.device_path)) {
        return false;
    }

    // 设置格式
    if (!setFormat(cam_id, config)) {
        closeDevice(cam_id);
        return false;
    }

    // 申请缓冲区
    if (!requestBuffers(cam_id, config.num_buffers)) {
        closeDevice(cam_id);
        return false;
    }

    // 映射缓冲区
    if (!mapBuffers(cam_id)) {
        closeDevice(cam_id);
        return false;
    }

    camera.initialized = true;
    RCLCPP_INFO(node_->get_logger(), 
                "Camera %d initialized: %s (%dx%d, %s, %d buffers)", 
                cam_id, config.device_path.c_str(), config.width, config.height,
                getPixelFormatString(config.pixel_format).c_str(), config.num_buffers);

    return true;
}

void V4L2Camera::setFrameCallback(FrameCallback callback) {
    frame_callback_ = callback;
}

void V4L2Camera::setTimeCalibrationCallback(TimeCalibrationCallback callback) {
    time_calibration_callback_ = callback;
}

void V4L2Camera::setFrameSkipRatio(int ratio) {
    if (ratio < 1) {
        ratio = 1;
    }
    frame_skip_ratio_ = ratio;
    RCLCPP_INFO(node_->get_logger(), "V4L2 frame skip ratio set to %d", frame_skip_ratio_);
}

bool V4L2Camera::startCapture(int cam_id) {
    if (cam_id < 0 || cam_id >= static_cast<int>(cameras_.size())) {
        return false;
    }

    auto& camera = cameras_[cam_id];
    if (!camera.initialized || camera.capturing) {
        return false;
    }

    // 将所有缓冲区加入队列
    for (size_t i = 0; i < camera.buffers.size(); ++i) {
        if (!queueBuffer(cam_id, i)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to queue buffer %zu for camera %d", i, cam_id);
            return false;
        }
    }

    // 开始流传输
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (!checkV4L2Result(ioctl(camera.fd, VIDIOC_STREAMON, &type), "VIDIOC_STREAMON", cam_id)) {
        return false;
    }

    camera.capturing = true;
    camera.current_buffer_index = 0;
    
    RCLCPP_INFO(node_->get_logger(), "Camera %d capture started", cam_id);
    return true;
}

bool V4L2Camera::stopCapture(int cam_id) {
    if (cam_id < 0 || cam_id >= static_cast<int>(cameras_.size())) {
        return false;
    }

    auto& camera = cameras_[cam_id];
    if (!camera.capturing) {
        return true;  // 已经停止
    }

    // 停止流传输
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (!checkV4L2Result(ioctl(camera.fd, VIDIOC_STREAMOFF, &type), "VIDIOC_STREAMOFF", cam_id)) {
        return false;
    }

    camera.capturing = false;
    RCLCPP_INFO(node_->get_logger(), "Camera %d capture stopped", cam_id);
    return true;
}

bool V4L2Camera::processFrame(int cam_id) {
    if (cam_id < 0 || cam_id >= static_cast<int>(cameras_.size())) {
        return false;
    }

    auto& camera = cameras_[cam_id];
    if (!camera.capturing) {
        return false;
    }

    struct v4l2_buffer buf;
    if (!dequeueBuffer(cam_id, buf)) {
        return false;
    }

    // 获取当前系统世界时间（wall clock time）
    auto system_wall_time = std::chrono::system_clock::now();
    auto system_wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        system_wall_time.time_since_epoch()).count();

    // 获取V4L2缓冲区时间戳（monotonic时间）
    auto v4l2_timestamp_ns = static_cast<int64_t>(buf.timestamp.tv_sec) * 1000000000LL + 
                            static_cast<int64_t>(buf.timestamp.tv_usec) * 1000LL;

    // 计算时间差值（V4L2时间 - 系统世界时间），转换为毫秒
    double time_diff_ms = (v4l2_timestamp_ns - system_wall_ns) / 1000000.0;

    // 更新时间差统计
    updateTimeDiffStats(cam_id, time_diff_ms);

    // 计算校正后的系统世界时间戳
    int64_t corrected_system_wall_ns = v4l2_timestamp_ns;
    {
        std::lock_guard<std::mutex> lock(*camera.stats.time_diff_stats.diff_mutex);
        if (camera.stats.time_diff_stats.diff_calculated) {
            // 从V4L2时间戳减去计算出的差值，得到校正后的系统世界时间
            corrected_system_wall_ns = v4l2_timestamp_ns - 
                static_cast<int64_t>(camera.stats.time_diff_stats.calculated_diff_ms * 1000000.0);
        }
    }

    // 更新V4L2帧率统计
    updateV4L2Stats(cam_id);

    // 采集入口帧抽取：只对进入后续pipeline的帧调用回调
    if (frame_skip_ratio_ > 1 && (camera.stats.capture_count % frame_skip_ratio_) != 0) {
        if (!queueBuffer(cam_id, buf.index)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to requeue buffer %d for camera %d", 
                         buf.index, cam_id);
            return false;
        }
        camera.current_buffer_index = buf.index;
        return true;
    }

    // 调用帧回调函数，传递校正后的世界时间戳
    if (frame_callback_) {
        frame_callback_(cam_id, camera.buffers[buf.index].start, 
                       buf.bytesused, buf.index, corrected_system_wall_ns);
    }

    // 重新将缓冲区加入队列
    if (!queueBuffer(cam_id, buf.index)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to requeue buffer %d for camera %d", 
                     buf.index, cam_id);
        return false;
    }

    camera.current_buffer_index = buf.index;
    return true;
}

int V4L2Camera::getFileDescriptor(int cam_id) const {
    if (cam_id < 0 || cam_id >= static_cast<int>(cameras_.size())) {
        return -1;
    }
    return cameras_[cam_id].fd;
}

bool V4L2Camera::isCameraInitialized(int cam_id) const {
    if (cam_id < 0 || cam_id >= static_cast<int>(cameras_.size())) {
        return false;
    }
    return cameras_[cam_id].initialized;
}

const CameraConfig& V4L2Camera::getCameraConfig(int cam_id) const {
    static CameraConfig default_config;
    if (cam_id < 0 || cam_id >= static_cast<int>(cameras_.size())) {
        return default_config;
    }
    return cameras_[cam_id].config;
}

void V4L2Camera::cleanup() {
    for (int i = 0; i < static_cast<int>(cameras_.size()); ++i) {
        if (cameras_[i].initialized) {
            stopCapture(i);
            closeDevice(i);
        }
    }
}

bool V4L2Camera::openDevice(int cam_id, const std::string& device_path) {
    auto& camera = cameras_[cam_id];
    
    camera.fd = open(device_path.c_str(), O_RDWR | O_NONBLOCK);
    if (camera.fd == -1) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open camera %d device %s: %s", 
                     cam_id, device_path.c_str(), strerror(errno));
        return false;
    }

    // 检查设备能力
    struct v4l2_capability cap;
    if (!checkV4L2Result(ioctl(camera.fd, VIDIOC_QUERYCAP, &cap), "VIDIOC_QUERYCAP", cam_id)) {
        close(camera.fd);
        camera.fd = -1;
        return false;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        RCLCPP_ERROR(node_->get_logger(), "Camera %d device %s does not support video capture", 
                     cam_id, device_path.c_str());
        close(camera.fd);
        camera.fd = -1;
        return false;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        RCLCPP_ERROR(node_->get_logger(), "Camera %d device %s does not support streaming", 
                     cam_id, device_path.c_str());
        close(camera.fd);
        camera.fd = -1;
        return false;
    }

    return true;
}

bool V4L2Camera::setFormat(int cam_id, const CameraConfig& config) {
    auto& camera = cameras_[cam_id];
    
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = config.width;
    fmt.fmt.pix.height = config.height;
    fmt.fmt.pix.pixelformat = config.pixel_format;
    fmt.fmt.pix.field = config.field;
    fmt.fmt.pix.colorspace = config.colorspace;

    if (!checkV4L2Result(ioctl(camera.fd, VIDIOC_S_FMT, &fmt), "VIDIOC_S_FMT", cam_id)) {
        return false;
    }

    // 验证设置的格式
    if (fmt.fmt.pix.width != static_cast<__u32>(config.width) || 
        fmt.fmt.pix.height != static_cast<__u32>(config.height) ||
        fmt.fmt.pix.pixelformat != static_cast<__u32>(config.pixel_format)) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "Camera %d format mismatch - requested: %dx%d %s, got: %dx%d %s", 
                     cam_id, config.width, config.height, 
                     getPixelFormatString(config.pixel_format).c_str(),
                     fmt.fmt.pix.width, fmt.fmt.pix.height,
                     getPixelFormatString(fmt.fmt.pix.pixelformat).c_str());
        return false;
    }

    return true;
}

bool V4L2Camera::requestBuffers(int cam_id, int num_buffers) {
    auto& camera = cameras_[cam_id];
    
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = num_buffers;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (!checkV4L2Result(ioctl(camera.fd, VIDIOC_REQBUFS, &req), "VIDIOC_REQBUFS", cam_id)) {
        return false;
    }

    if (req.count < static_cast<__u32>(num_buffers)) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "Camera %d insufficient buffer memory - requested: %d, got: %d", 
                     cam_id, num_buffers, req.count);
        return false;
    }

    camera.buffers.resize(req.count);
    return true;
}

bool V4L2Camera::mapBuffers(int cam_id) {
    auto& camera = cameras_[cam_id];
    
    for (size_t i = 0; i < camera.buffers.size(); ++i) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (!checkV4L2Result(ioctl(camera.fd, VIDIOC_QUERYBUF, &buf), "VIDIOC_QUERYBUF", cam_id)) {
            unmapBuffers(cam_id);
            return false;
        }

        camera.buffers[i].length = buf.length;
        camera.buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, 
                                      MAP_SHARED, camera.fd, buf.m.offset);

        if (camera.buffers[i].start == MAP_FAILED) {
            RCLCPP_ERROR(node_->get_logger(), "Camera %d failed to map buffer %zu: %s", 
                         cam_id, i, strerror(errno));
            unmapBuffers(cam_id);
            return false;
        }
    }

    return true;
}

bool V4L2Camera::queueBuffer(int cam_id, int buffer_index) {
    auto& camera = cameras_[cam_id];
    
    if (buffer_index < 0 || buffer_index >= static_cast<int>(camera.buffers.size())) {
        return false;
    }

    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = buffer_index;

    return checkV4L2Result(ioctl(camera.fd, VIDIOC_QBUF, &buf), "VIDIOC_QBUF", cam_id);
}

bool V4L2Camera::dequeueBuffer(int cam_id, struct v4l2_buffer& buf) {
    auto& camera = cameras_[cam_id];
    
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    return checkV4L2Result(ioctl(camera.fd, VIDIOC_DQBUF, &buf), "VIDIOC_DQBUF", cam_id);
}

void V4L2Camera::unmapBuffers(int cam_id) {
    auto& camera = cameras_[cam_id];
    
    for (auto& buffer : camera.buffers) {
        if (buffer.start != nullptr && buffer.start != MAP_FAILED) {
            munmap(buffer.start, buffer.length);
            buffer.start = nullptr;
        }
    }
    camera.buffers.clear();
}

void V4L2Camera::closeDevice(int cam_id) {
    auto& camera = cameras_[cam_id];
    
    if (camera.capturing) {
        stopCapture(cam_id);
    }
    
    unmapBuffers(cam_id);
    
    if (camera.fd != -1) {
        close(camera.fd);
        camera.fd = -1;
    }
    
    camera.initialized = false;
}

bool V4L2Camera::checkV4L2Result(int result, const std::string& operation, int cam_id) const {
    if (result == -1) {
        RCLCPP_ERROR(node_->get_logger(), "Camera %d %s failed: %s", 
                     cam_id, operation.c_str(), strerror(errno));
        return false;
    }
    return true;
}

void V4L2Camera::updateV4L2Stats(int cam_id) {
    auto& camera = cameras_[cam_id];
    auto& stats = camera.stats;
    
    stats.capture_count++;
    stats.frames_since_stats++;
    
    // 不在此处打印统计信息，统一由主节点打印表格
}

void V4L2Camera::updateTimeDiffStats(int cam_id, double time_diff_ms) {
    auto& camera = cameras_[cam_id];
    auto& time_stats = camera.stats.time_diff_stats;
    
    time_stats.total_frame_count++;
    
    // 跳过前5帧
    if (time_stats.total_frame_count <= time_stats.skip_frames) {
        return;
    }
    
    // 如果还没收集到10帧，继续收集
    if (time_stats.time_diffs.size() < 10) {
        time_stats.time_diffs.push_back(time_diff_ms);
    }
    
    // 检查是否该打印10秒统计了
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        current_time - time_stats.last_stats_time);
    
    if (elapsed.count() >= 10 && time_stats.time_diffs.size() == 10) {
        // 计算最终使用的差值：最大值 + 0.5ms
        double max_diff = *std::max_element(time_stats.time_diffs.begin(), time_stats.time_diffs.end());
        double calculated_diff = max_diff + 0.5;
        
        // 线程安全地更新计算出的差值
        {
            std::lock_guard<std::mutex> lock(*time_stats.diff_mutex);
            time_stats.calculated_diff_ms = calculated_diff;
            time_stats.diff_calculated = true;
        }
        
        // 通知主节点时间校准已完成
        if (time_calibration_callback_) {
            time_calibration_callback_(cam_id);
        }
        
        // 重置统计数据，开始下一个10秒周期
        time_stats.time_diffs.clear();
        time_stats.last_stats_time = current_time;
    }
}

std::pair<double, int> V4L2Camera::getFrameStats(int cam_id) const {
    if (cam_id < 0 || cam_id >= static_cast<int>(cameras_.size())) {
        return {0.0, 0};
    }
    
    const auto& camera = cameras_[cam_id];
    if (!camera.initialized) {
        return {0.0, 0};
    }
    
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        current_time - camera.stats.last_stats_time);
    
    double fps = 0.0;
    if (elapsed.count() > 0) {
        fps = camera.stats.frames_since_stats / static_cast<double>(elapsed.count());
    }
    
    return {fps, camera.stats.capture_count};
}

double V4L2Camera::getCalculatedTimeDiff(int cam_id) const {
    if (cam_id < 0 || cam_id >= static_cast<int>(cameras_.size())) {
        return 0.0;
    }
    
    const auto& time_stats = cameras_[cam_id].stats.time_diff_stats;
    std::lock_guard<std::mutex> lock(*time_stats.diff_mutex);
    return time_stats.calculated_diff_ms;
}

bool V4L2Camera::isTimeDiffCalculated(int cam_id) const {
    if (cam_id < 0 || cam_id >= static_cast<int>(cameras_.size())) {
        return false;
    }
    
    const auto& time_stats = cameras_[cam_id].stats.time_diff_stats;
    std::lock_guard<std::mutex> lock(*time_stats.diff_mutex);
    return time_stats.diff_calculated;
}

std::string V4L2Camera::getPixelFormatString(int format) const {
    switch (format) {
        case V4L2_PIX_FMT_UYVY: return "UYVY";
        case V4L2_PIX_FMT_YUYV: return "YUYV";
        case V4L2_PIX_FMT_RGB24: return "RGB24";
        case V4L2_PIX_FMT_BGR24: return "BGR24";
        case V4L2_PIX_FMT_MJPEG: return "MJPEG";
        default: 
            char buf[5];
            buf[0] = (format >> 0) & 0xFF;
            buf[1] = (format >> 8) & 0xFF;
            buf[2] = (format >> 16) & 0xFF;
            buf[3] = (format >> 24) & 0xFF;
            buf[4] = '\0';
            return std::string(buf);
    }
}

} // namespace cr_camera_driver
